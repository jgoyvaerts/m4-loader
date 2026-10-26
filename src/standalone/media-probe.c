/* SPDX-License-Identifier: MIT */
/* R36 media lane: disabled audio/AVD nodes and one guarded register probe.
 *
 *   m4_media_probe_run()  Single 32-bit reads, each in its own j616s_usb_call
 *       guard. A synchronous data abort is recorded (ESR/FAR) and ends only
 *       that block's reads; nothing is retried, written or waited for. PMGR
 *       power state is READ first; the AVD block is read only when AVD_SYS and
 *       its parent FAB0_SOC already report ACTIVE. There is no PMGR write, so
 *       nothing to reverse: m1n1's pmgr_adt_power_disable() skips the virtual
 *       AVD gates, and pmgr_set_mode() clears auto-enable bits on every parent.
 *   m4_media_fdt_write()  RAM/FDT edit at publication: S0b audio nodes (AOP,
 *       ASC mailbox, dart-aop, two ADMACs) and AVD stage-1 nodes, all
 *       status = "disabled" with T6040-only compatibles; no-map reservations
 *       for the AOP's DRAM segments read from the ADT; /chosen/m4,media-probe.
 *
 * Size matters: the loader image has little layout headroom (see README), so
 * this file is -Os, table driven, and keeps a sticky error instead of checks.
 *
 * /chosen/m4,media-probe wire format (big endian u64 words):
 *   [0] magic "M4MEDPR1"  [1] version  [2] total bytes  [3] nodes-emitted bits
 *   [4] record count R  [5] segment count S  [6] gate bits  [7] 0
 *   R x { id, state, address, value, esr, far }
 *   S x { name (8 ASCII bytes), phys, size, flags }
 */
#pragma GCC optimize("Os")
#include "adt.h"
#include "libfdt.h"
#include "utils.h"
#include "ram.h"
#include "j616s_usb.h"
#include "media-probe.h"
#include <stdint.h>
#include <string.h>

#define ARMIO UINT64_C(0x200000000)

/* ADT address/size pairs; physical = ADT + ARMIO. */
static const uint64_t aop_reg[] = {0x310600000, 0x88000, 0x310050000, 0x4000,
                                   0x310c00000, 0x2f0000, 0x308328000, 0x24};
static const uint64_t dart_reg[] = {0x310f00000, 0xc000};
static const uint64_t leap_reg[] = {0x313300000, 0x34000};
static const uint64_t base_reg[] = {0x313800000, 0x34000};
static const uint64_t avd_reg[] = {0x80000000, 0x1404000};
static const uint64_t avd_dart_reg[] = {0x81020000, 0xc000};
static const uint64_t pmgr_reg0[] = {0x302800000, 0x1e0000};
static const uint64_t arm_io_ranges[] = {0, ARMIO};
/* R51 lane U, right-hand USB-C port (ATC2): ADT reg values (physical = ADT + ARMIO); every one is checked by usb_gate(). */
static const uint64_t pmgr_reg1[] = {0x308300000, 0x80000};
static const uint64_t usb_drd_reg0[] = {0x192280000, 0x11800};
static const uint64_t usb_drd_reg3[] = {0x192a84000, 0x4000};   /* pipehandler */
static const uint64_t usb_drd_reg4[] = {0x192800000, 0x4000};   /* USB2 event bank = atc-phy2 reg[1] */
static const uint64_t atcphy_reg[] = {0x192a90000, 0x4000, 0x192800000, 0x4000};
static const uint64_t usb_dart_reg[] = {0x192f00000, 0xc000, 0x192f80000, 0xc000};
static const uint64_t spmi_a1_reg[] = {0x309198000, 0x4000};
static const uint32_t usb_common_dp_gate[] = {0x11e};   /* atc-phy2 clock-gates[3] = ATC2_COMMON_DP */
static const uint32_t hpm2_reg[] = {0xc, 3, 0, 4, 0, 0};
static const uint32_t hpm2_irq[] = {11, 17, 19};
static const uint32_t usb_drd_irq0[] = {1683};
static const uint32_t usb_dart_irq[] = {1687};
static const uint32_t spmi_a1_irq1[] = {868};

/* ---------------------------------------------------------------- ADT gates */
static int has(const char *path, const char *key, const void *want, uint32_t bytes)
{
    uint32_t len;
    int node = adt_path_offset(adt, path);
    const void *p = node < 0 ? NULL : adt_getprop(adt, node, key, &len);
    return p && len >= bytes && !memcmp(p, want, bytes);
}
/* the same at a byte offset inside the property (reg[n], interrupts[n]) */
static int has_at(const char *path, const char *key, uint32_t at, const void *want, uint32_t bytes)
{
    uint32_t len;
    int node = adt_path_offset(adt, path);
    const uint8_t *p = node < 0 ? NULL : adt_getprop(adt, node, key, &len);
    return p && len >= at + bytes && !memcmp(p + at, want, bytes);
}
#define HAS(path, key, lit) has(path, key, lit, sizeof(lit))
#define HASARR(path, key, arr) has(path, key, arr, sizeof(arr))

/* pmgr `devices`: 48-byte entries, 32 header bytes then a 16-byte name.
 * Word 4 = PS register offset (group 0), word 6 high half = device id. */
static int pmgr_has(const char *name, uint32_t offset, uint32_t id, uint32_t flags, uint32_t parent)
{
    uint32_t len;
    int node = adt_path_offset(adt, "/arm-io/pmgr");
    const uint8_t *p = node < 0 ? NULL : adt_getprop(adt, node, "devices", &len);
    if (!p || len % 48) return 0;
    for (uint32_t off = 0; off + 48 <= len; off += 48) {
        const uint32_t *e = (const uint32_t *)(p + off);
        if (strncmp((const char *)p + off + 32, name, 16)) continue;
        return e[4] == offset && (e[6] >> 16) == id && (!flags || (e[0] == flags && e[1] == parent));
    }
    return 0;
}

/* word 1 = first parent word (two 16-bit ids packed), compared only when `parents` is non-zero */
static int pmgr_ps(const char *name, uint32_t offset, uint32_t id, uint32_t parents)
{
    uint32_t len;
    int node = adt_path_offset(adt, "/arm-io/pmgr");
    const uint8_t *p = node < 0 ? NULL : adt_getprop(adt, node, "devices", &len);
    if (!p || len % 48) return 0;
    for (uint32_t off = 0; off + 48 <= len; off += 48) {
        const uint32_t *e = (const uint32_t *)(p + off);
        if (strncmp((const char *)p + off + 32, name, 16)) continue;
        if (parents == 0xffffffffu) return e[4] == offset && (e[6] >> 16) == id && !e[0] && !e[1];   /* no flags, no parent */
        return e[4] == offset && (e[6] >> 16) == id && (!parents || e[1] == parents);
    }
    return 0;
}

static int base_gate(void)
{
    return (HAS("/chosen", "firmware-version", "mBoot-18000.161.10") ||
            HAS("/chosen", "firmware-version", "mBoot-18000.161.10.701.1")) &&
           HASARR("/arm-io", "ranges", arm_io_ranges);
}
static int aop_gate(void)
{
    return base_gate() && HASARR("/arm-io/aop", "reg", aop_reg) &&
           HAS("/arm-io/aop", "compatible", "iop,ascwrap-v6") && HAS("/arm-io/aop", "role", "AOP");
}
/* The right-hand port group: every address, interrupt, PS record and the SPMI/PD identity the nodes below carry. */
static int usb_gate(void)
{
    return base_gate() && has_at("/arm-io/pmgr", "reg", 16, pmgr_reg1, sizeof(pmgr_reg1)) &&
           pmgr_ps("NUB_SPMI_A1", 0x3000070, 0x116, 0) && pmgr_ps("ATC2_USB_AON", 0x30000d8, 0x123, 0) &&
           pmgr_ps("ATC2_USB", 0x3000140, 0x12d, 0x1230070) && pmgr_ps("ATC2_PHYMXWRAP", 0x3000160, 0x131, 0x1230070) &&
           pmgr_ps("ATC2_COMMON", 0x3e0, 0x70, 0) && pmgr_ps("ATC2_COMMON_DP", 0x30000b0, 0x11e, 0xffffffffu) &&
           has_at("/arm-io/atc-phy2", "clock-gates", 12, usb_common_dp_gate, sizeof(usb_common_dp_gate)) &&
           HAS("/arm-io/usb-drd2", "compatible", "usb-drd,t6040\0usb-drd,t8132") &&
           has_at("/arm-io/usb-drd2", "reg", 0, usb_drd_reg0, sizeof(usb_drd_reg0)) &&
           has_at("/arm-io/usb-drd2", "reg", 48, usb_drd_reg3, sizeof(usb_drd_reg3)) &&
           has_at("/arm-io/usb-drd2", "reg", 64, usb_drd_reg4, sizeof(usb_drd_reg4)) &&
           has("/arm-io/usb-drd2", "interrupts", usb_drd_irq0, 4) &&
           HAS("/arm-io/atc-phy2", "compatible", "atc-phy,t6040") && HASARR("/arm-io/atc-phy2", "reg", atcphy_reg) &&
           HAS("/arm-io/dart-usb2", "compatible", "dart,t8110") && HASARR("/arm-io/dart-usb2", "reg", usb_dart_reg) &&
           HASARR("/arm-io/dart-usb2", "interrupts", usb_dart_irq) &&
           HASARR("/arm-io/nub-spmi-a1", "reg", spmi_a1_reg) && has_at("/arm-io/nub-spmi-a1", "interrupts", 4, spmi_a1_irq1, 4) &&
           HAS("/arm-io/nub-spmi-a1/hpm2", "compatible", "usbc,sn201202x,spmi") &&
           HASARR("/arm-io/nub-spmi-a1/hpm2", "reg", hpm2_reg) && HASARR("/arm-io/nub-spmi-a1/hpm2", "interrupts", hpm2_irq);
}
static int avd_gate(void)
{
    return base_gate() && HASARR("/arm-io/avd0", "reg", avd_reg) &&
           HASARR("/arm-io/pmgr", "reg", pmgr_reg0) &&
           pmgr_has("AVD_SYS", 0x170, 0x22, 0x18000002, 0x21) && pmgr_has("FAB0_SOC", 0x168, 0x21, 0, 0);
}

/* ------------------------------------------------------------ guarded probe */
#ifdef M4_MEDIA_TEST
extern uint32_t m4_media_test_read32(uint64_t address);
#define MEDIA_READ32(a) m4_media_test_read32(a)
#else
#define MEDIA_READ32(a) read32(a)
#endif

enum { REC_VALUE, REC_ABORT, REC_SKIPPED, REC_REFUSED };
enum { SKIP_GATE = 1, SKIP_NOT_ACTIVE, SKIP_PS_UNREADABLE, SKIP_EARLIER_ABORT, SKIP_GUARD_REFUSED,
       SKIP_REMOVED_R36_RISK };
enum { GROUP_PMGR, GROUP_AOP0, GROUP_AOP1, GROUP_AVD, GROUPS };
enum { WIN_PMGR, WIN_AOP0, WIN_AOP1, WIN_AVD };

static struct m4_media_record records[12];
static unsigned record_count, probe_calls;
static uint8_t group_dead[GROUPS], guard_dead;
static uint64_t gate_bits;
static volatile uint64_t current_address;
static volatile uint32_t current_value;

static int read_callback(void)
{
    __asm__ volatile("dsb sy" ::: "memory");
    current_value = MEDIA_READ32(current_address);
    __asm__ volatile("dsb sy" ::: "memory");
    return 0;
}

/* One bounded read of `address`, which must lie in [window, window+size). */
static uint32_t record_read(unsigned id, unsigned group, uint64_t address, uint64_t window,
                            uint64_t size, unsigned skip)
{
    struct m4_media_record *r = &records[record_count++];
    struct j616s_call_result call = {0};
    *r = (struct m4_media_record){.id = id, .state = REC_SKIPPED, .address = address, .value = skip};
    if (!skip) {
        if (address < window || size < 4 || address - window > size - 4 || (address & 3)) r->value = SKIP_GATE;
        else if (guard_dead) r->value = SKIP_GUARD_REFUSED;
        else if (group_dead[group]) r->value = SKIP_EARLIER_ABORT;
        else {
            current_address = address;
            current_value = 0;
            probe_calls++;
            if (j616s_usb_call(read_callback, &call)) {
                r->state = REC_REFUSED; /* guard would not arm: never read again */
                r->value = 0;
                guard_dead = 1;
            } else if (call.status) {
                r->state = REC_ABORT;
                r->value = 0;
                r->esr = call.esr;
                r->far = call.far;
                group_dead[group] = 1;
            } else {
                r->state = REC_VALUE;
                r->value = current_value;
            }
        }
    }
    return r->state == REC_VALUE ? (uint32_t)r->value : 0;
}

/* id, group, window, offset inside the window (no pointers: no relocations). */
struct probe_item { uint8_t id, group, window; uint32_t offset; };
static const struct probe_item items[] = {
    {M4_MEDIA_REC_PMGR_AVD_SYS_PS, GROUP_PMGR, WIN_PMGR, 0x170},
    {M4_MEDIA_REC_PMGR_FAB0_SOC_PS, GROUP_PMGR, WIN_PMGR, 0x168},
    {M4_MEDIA_REC_AOP0_CPU_CONTROL, GROUP_AOP0, WIN_AOP0, 0x44},
    {M4_MEDIA_REC_AOP0_MBOX_A2I_CONTROL, GROUP_AOP0, WIN_AOP0, 0x8110},
    {M4_MEDIA_REC_AOP0_MBOX_I2A_CONTROL, GROUP_AOP0, WIN_AOP0, 0x8114},
    {M4_MEDIA_REC_AOP1_CPU_CONTROL, GROUP_AOP1, WIN_AOP1, 0x44},
    {M4_MEDIA_REC_AVD_CODE_0, GROUP_AVD, WIN_AVD, 0x1080000},
    {M4_MEDIA_REC_AVD_MBOX_1098000_90, GROUP_AVD, WIN_AVD, 0x1098090},
    {M4_MEDIA_REC_AVD_MBOX_10A0000_90, GROUP_AVD, WIN_AVD, 0x10a0090},
    {M4_MEDIA_REC_AVD_MBOX_10A8000_90, GROUP_AVD, WIN_AVD, 0x10a8090},
    {M4_MEDIA_REC_AVD_CTRL_0, GROUP_AVD, WIN_AVD, 0x1100000},
    {M4_MEDIA_REC_AVD_CTRL_4114, GROUP_AVD, WIN_AVD, 0x1104114},
};

/* The gates only read the ADT. R36 evaluated them here with the MMU and caches off, which cost about
 * 0.1 s of uncached ADT walks per boot; the loader now calls this inside its ADT metadata window. */
static uint8_t gates_cached; /* bit 7: evaluated; bits 1:0 as gate_bits */
void m4_media_probe_gates(void)
{
#ifndef M4_MEDIA_NO_PROBE
    if (!gates_cached) gates_cached = 0x80u | (aop_gate() ? 1u : 0u) | (avd_gate() ? 2u : 0u);
#endif
}

void m4_media_probe_run(void)
{
    const uint64_t win[4][2] = {{pmgr_reg0[0] + ARMIO, pmgr_reg0[1]}, {aop_reg[0] + ARMIO, aop_reg[1]},
                                {aop_reg[2] + ARMIO, aop_reg[3]}, {avd_reg[0] + ARMIO, avd_reg[1]}};
    int aop_ok, avd_ok;
    unsigned avd_skip = SKIP_GATE, i;
    uint32_t v;
    if (probe_calls || record_count) return; /* once per boot */
#ifdef M4_MEDIA_NO_PROBE
    return; /* build switch (make M4_MEDIA_NO_PROBE=1): nodes and reservations, no register read */
#endif
    m4_media_probe_gates(); /* no-op when the loader already ran it in a cached window */
    gate_bits = gates_cached & 3u; /* published only when the probe runs, as in R36 */
    aop_ok = gate_bits & 1;
    avd_ok = (gate_bits >> 1) & 1;
    /* PMGR first (read only; items 0 and 1), AOP next, AVD last. */
    for (i = 0; i < sizeof(items) / sizeof(items[0]); i++) {
        const struct probe_item *it = &items[i];
        unsigned skip;
        /* id 6 (second-window CPU_CONTROL) is speculative: never read in R36, record kept for schema stability */
        if (it->group == GROUP_AOP1) skip = SKIP_REMOVED_R36_RISK;
        else if (it->group == GROUP_AOP0) skip = aop_ok ? 0 : SKIP_GATE;
        else if (it->group == GROUP_PMGR) skip = avd_ok ? 0 : SKIP_GATE;
        else skip = avd_skip;
        v = record_read(it->id, it->group, win[it->window][0] + it->offset,
                        win[it->window][0], win[it->window][1], skip);
        if (i == 1) {
            /* Both PS words readable and ACTIVE (actual state, bits 7:4 = 0xf)? */
            if (!avd_ok) avd_skip = SKIP_GATE;
            else if (records[0].state != REC_VALUE || records[1].state != REC_VALUE) avd_skip = SKIP_PS_UNREADABLE;
            else if (((records[0].value >> 4) & 0xf) != 0xf || ((v >> 4) & 0xf) != 0xf) avd_skip = SKIP_NOT_ACTIVE;
            else avd_skip = 0;
        }
    }
}

/* ----------------------------------------------------------------- FDT edit */
static void *T;  /* tree being edited */
static int E;    /* sticky libfdt error: once set, every helper is a no-op */

static void seterr(int r) { if (r < 0 && !E) E = r; }
static int mknode(int parent, const char *name, uint32_t *ph)
{
    int n;
    if (E) return -1;
    n = fdt_add_subnode(T, parent, name);
    seterr(n);
    if (n >= 0 && ph) {
        seterr(fdt_generate_phandle(T, ph));
        if (!E) seterr(fdt_setprop_u32(T, n, "phandle", *ph));
    }
    return n;
}
static void sprop(int n, const char *k, const void *v, int len)
{
    if (!E) seterr(fdt_setprop(T, n, k, v, len));
}
#define STRP(n, k, lit) sprop(n, k, lit, sizeof(lit))
static void u32p(int n, const char *k, uint32_t v) { if (!E) seterr(fdt_setprop_u32(T, n, k, v)); }
static void cellsp(int n, const char *k, unsigned count, const uint32_t *w)
{
    fdt32_t c[12];
    for (unsigned i = 0; i < count; i++) c[i] = cpu_to_fdt32(w[i]);
    sprop(n, k, c, count * 4);
}
static void regp(int n, unsigned count, const uint64_t *w)
{
    fdt64_t c[8];
    for (unsigned i = 0; i < count; i++) c[i] = cpu_to_fdt64(w[i]);
    sprop(n, "reg", c, count * 8);
}
static void disabled(int n) { if (!E) seterr(fdt_setprop_string(T, n, "status", "disabled")); }
static void irq3(int n, uint32_t aic, uint32_t spi)
{
    const uint32_t w[] = {0, spi, 4};
    u32p(n, "interrupt-parent", aic);
    cellsp(n, "interrupts", 3, w);
}

/* AOP firmware region: __TEXT phys (first ADT segment) to the end of the third
 * ADT window: 0x5_10C1_0000 + 0x2E_0000 (aop-firmware/README.md correction 1). */
static int aop_firmware(uint64_t *out)
{
    const uint64_t window = aop_reg[4] + ARMIO, end = window + aop_reg[5];
    uint32_t len;
    int node = adt_path_offset(adt, "/arm-io/aop/iop-aop-nub");
    const uint8_t *r = node < 0 ? NULL : adt_getprop(adt, node, "segment-ranges", &len);
    uint64_t text;
    if (!r || len < 32) return 0;
    memcpy(&text, r, 8);
    if (text < window || text >= end || (text & 0x3fff)) return 0;
    out[0] = text;
    out[1] = end - text;
    return 1;
}

static void build_audio(uint32_t aic)
{
    static const uint32_t aop_irq[] = {0x2c7, 0x2c6, 0x2c9, 0x2c8};
    uint32_t dart_ph, mbox_ph, leap_ph, base_ph, cells[12];
    uint64_t fw[2], r[8];
    int n, aop;
    if (!aop_firmware(fw)) { seterr(-FDT_ERR_BADVALUE); return; }

    n = mknode(0, "iommu@510f00000", &dart_ph);
    STRP(n, "compatible", "apple,t6040-dart");
    r[0] = dart_reg[0] + ARMIO; r[1] = dart_reg[1];
    regp(n, 2, r);
    u32p(n, "#iommu-cells", 1);
    irq3(n, aic, 0x2de);
    disabled(n);

    /* ASC mailbox = ADT window 0 + 0x8000 (m1n1 asc.c; the probe reads back its
     * control words). Interrupts: DCP's pairing of the first four; inferred. */
    n = mknode(0, "mailbox@510608000", &mbox_ph);
    STRP(n, "compatible", "apple,t6040-asc-mailbox");
    r[0] = aop_reg[0] + 0x8000 + ARMIO; r[1] = 0x4000;
    regp(n, 2, r);
    u32p(n, "interrupt-parent", aic);
    for (unsigned i = 0; i < 4; i++) {
        cells[3 * i] = 0;
        cells[3 * i + 1] = aop_irq[i ^ 1];
        cells[3 * i + 2] = 4;
    }
    cellsp(n, "interrupts", 12, cells);
    STRP(n, "interrupt-names", "send-empty\0send-not-empty\0recv-empty\0recv-not-empty");
    u32p(n, "#mbox-cells", 0);
    disabled(n);

    for (unsigned i = 0; i < 2; i++) {
        const uint64_t *reg = i ? base_reg : leap_reg;
        n = mknode(0, i ? "dma-controller@513800000" : "dma-controller@513300000", i ? &base_ph : &leap_ph);
        STRP(n, "compatible", "apple,t6040-admac");
        r[0] = reg[0] + ARMIO; r[1] = reg[1];
        regp(n, 2, r);
        u32p(n, "#dma-cells", 1);
        /* Linux counts both directions (even = TX, odd = RX): the leap ADMAC's 9 per direction in the ADT are 18 (R48 mic request) */
        u32p(n, "dma-channels", i ? 13 : 18);
        irq3(n, aic, i ? 0x2f2 : 0x2f5);
        cells[0] = dart_ph; cells[1] = i ? 8 : 0xa;
        cellsp(n, "iommus", 2, cells);
        disabled(n);
    }

    /* reg[0] = firmware region (the Rust driver patches boot-args at +0x22c);
     * then the ASC wrapper, the second ASC window and the config word. */
    r[0] = fw[0]; r[1] = fw[1];
    r[2] = aop_reg[0] + ARMIO; r[3] = aop_reg[1];
    r[4] = aop_reg[2] + ARMIO; r[5] = aop_reg[3];
    r[6] = aop_reg[6] + ARMIO; r[7] = aop_reg[7];
    aop = mknode(0, "aop@510600000", NULL);
    STRP(aop, "compatible", "apple,t6040-aop");
    regp(aop, 8, r);
    STRP(aop, "reg-names", "firmware\0asc\0asc-aux\0config");
    u32p(aop, "mboxes", mbox_ph);
    STRP(aop, "mbox-names", "mbox");
    cells[0] = dart_ph; cells[1] = 0;
    cellsp(aop, "iommus", 2, cells);
    u32p(aop, "#address-cells", 2);
    u32p(aop, "#size-cells", 2);
    disabled(aop);
    n = mknode(aop, "audio", NULL);
    /* snd-soc-aop binds apple,t6030-aop-audio (3 bytes/sample, HF decimator); the t6040 string stays first */
    STRP(n, "compatible", "apple,t6040-aop-audio\0apple,t6030-aop-audio");
    STRP(n, "apple,chassis-name", "J616");
    STRP(n, "apple,machine-kind", "MacBook Pro");
    cells[0] = leap_ph; cells[1] = 1; /* T6030 uses <&aop_admac 1>: hypothesis O4 */
    cellsp(n, "dmas", 2, cells);
    STRP(n, "dma-names", "dma");
}

static void build_avd(uint32_t aic)
{
    uint32_t ps_ph, dart_ph, cells[6];
    uint64_t r[6];
    int pmgr = fdt_path_offset(T, "/power-management@502800000"), n;

    /* AVD_SYS power state: pmgr block 0 + 0x170. FAB0_SOC (parent) is the SoC
     * fabric, up on any running system, and deliberately not modelled. */
    n = mknode(pmgr, "power-controller@170", &ps_ph);
    STRP(n, "compatible", "apple,t6040-pmgr-pwrstate");
    cells[0] = 0x170; cells[1] = 4;
    cellsp(n, "reg", 2, cells);
    STRP(n, "label", "avd_sys");
    u32p(n, "#power-domain-cells", 0);
    u32p(n, "#reset-cells", 0);
    disabled(n);

    n = mknode(0, "iommu@281020000", &dart_ph);
    STRP(n, "compatible", "apple,t6040-dart");
    r[0] = avd_dart_reg[0] + ARMIO; r[1] = avd_dart_reg[1];
    regp(n, 2, r);
    u32p(n, "#iommu-cells", 1);
    irq3(n, aic, 0x5a9);
    u32p(n, "power-domains", ps_ph);
    disabled(n);

    /* Decoder sub-blocks are INFERRED from other chips (avd FACTS I4-I6). */
    r[0] = avd_reg[0] + ARMIO + 0x1080000; r[1] = 0x14000;
    r[2] = avd_reg[0] + ARMIO + 0x10a8000; r[3] = 0x4000;
    r[4] = avd_reg[0] + ARMIO + 0x1100000; r[5] = 0x10000;
    n = mknode(0, "avd@280000000", NULL);
    STRP(n, "compatible", "apple,t6040-avd");
    regp(n, 6, r);
    STRP(n, "reg-names", "code\0mbox\0ctrl");
    u32p(n, "interrupt-parent", aic);
    cells[0] = 0; cells[1] = 0x5a2; cells[2] = 4; cells[3] = 0; cells[4] = 0x5a3; cells[5] = 4;
    cellsp(n, "interrupts", 6, cells);
    u32p(n, "power-domains", ps_ph);
    u32p(n, "resets", ps_ph);
    cells[0] = dart_ph; cells[1] = 0;
    cellsp(n, "iommus", 2, cells);
    STRP(n, "apple,m4-avd-geometry", "inferred-v1");
    disabled(n);
}

/* ---- AOP DRAM segments: reserved from the ADT at boot, never hard-coded ---- */
#define SEG_NODE 1      /* a no-map reserved-memory node was emitted */
#define SEG_COVERED 2   /* wholly inside an existing reservation: disabled witness */
#define SEG_IN_MEMORY 4 /* overlaps a /memory node of the output tree */
#define SEG_WANTED 8    /* named in the reserve list */
struct segment { char name[8]; uint64_t phys, size, flags, iova; };
static struct segment segments[5];
static unsigned segment_count;

/* ADT `segment-ranges`: {phys, fw_va, remap} u64 + {size, flags} u32 = 32 B;
 * `segment-names` is `;`-separated. */
static void collect_segments(void)
{
    uint32_t nlen, rlen;
    int node = adt_path_offset(adt, "/arm-io/aop/iop-aop-nub");
    const char *names = node < 0 ? NULL : adt_getprop(adt, node, "segment-names", &nlen);
    const uint8_t *ranges = node < 0 ? NULL : adt_getprop(adt, node, "segment-ranges", &rlen);
    segment_count = 0;
    if (!names || !ranges || rlen % 32) return;
    for (uint32_t pos = 0, i = 0; pos < nlen && i < rlen / 32 && segment_count < 5; i++) {
        uint32_t size, l = 0;
        struct segment *g = &segments[segment_count++];
        while (pos + l < nlen && names[pos + l] && names[pos + l] != ';') l++;
        memset(g, 0, sizeof(*g));
        memcpy(g->name, names + pos, l > 8 ? 8 : l);
        memcpy(&g->phys, ranges + 32 * i, 8);
        memcpy(&g->iova, ranges + 32 * i + 16, 8); /* rev 14: the AOP DART address */
        memcpy(&size, ranges + 32 * i + 24, 4);
        g->size = size;
        if (!strncmp(g->name, "__ETEXT", 8) || !strncmp(g->name, "__OS_LOG", 8)) g->flags = SEG_WANTED;
        pos += l + 1;
    }
}

static int overlaps_memory_node(uint64_t lo, uint64_t hi)
{
    int node, depth = 0, len;
    for (node = fdt_next_node(T, -1, &depth); node >= 0; node = fdt_next_node(T, node, &depth)) {
        const char *type = fdt_getprop(T, node, "device_type", &len);
        const fdt64_t *reg;
        if (!type || len != 7 || memcmp(type, "memory", 7)) continue;
        reg = fdt_getprop(T, node, "reg", &len);
        for (int i = 0; reg && i + 16 <= len; i += 16) {
            uint64_t a = fdt64_ld(&reg[i / 8]), s = fdt64_ld(&reg[i / 8 + 1]); /* 4-aligned data */
            if (a < hi && lo < a + s) return 1;
        }
    }
    return 0;
}

/* Is [lo,hi) wholly inside one existing reservation (/memreserve/ or an
 * enabled /reserved-memory child with 2+2 cells)? A partial overlap between
 * different reservations is only a kernel message, so it stays a full node. */
static int covered(uint64_t lo, uint64_t hi)
{
    int count = fdt_num_mem_rsv(T), parent = fdt_path_offset(T, "/reserved-memory"), child, len;
    for (int i = 0; i < count; i++) {
        uint64_t a, s;
        if (!fdt_get_mem_rsv(T, i, &a, &s) && a <= lo && hi <= a + s) return 1;
    }
    if (parent < 0) return 0;
    fdt_for_each_subnode(child, T, parent) {
        const char *status = fdt_getprop(T, child, "status", &len);
        const fdt32_t *reg;
        uint64_t a, s;
        if (status && strcmp(status, "okay")) continue;
        reg = fdt_getprop(T, child, "reg", &len);
        if (!reg || len != 16) continue;
        a = ((uint64_t)fdt32_to_cpu(reg[0]) << 32) | fdt32_to_cpu(reg[1]);
        s = ((uint64_t)fdt32_to_cpu(reg[2]) << 32) | fdt32_to_cpu(reg[3]);
        if (a <= lo && hi <= a + s) return 1;
    }
    return 0;
}

static void reserve_segments(void)
{
    int resv = fdt_path_offset(T, "/reserved-memory");
    if (resv < 0) { /* the direct loader always creates it; make it if a tree lacks it */
        resv = mknode(0, "reserved-memory", NULL);
        u32p(resv, "#address-cells", 2);
        u32p(resv, "#size-cells", 2);
        if (!E) seterr(fdt_setprop_empty(T, resv, "ranges"));
    }
    for (unsigned s = 0; s < segment_count; s++) {
        struct segment *g = &segments[s];
        uint64_t lo, hi, reg[2];
        char name[32];
        int n, cov;
        if (!(g->flags & SEG_WANTED) || !g->size || g->size > 0x1000000 || g->phys + g->size < g->phys)
            continue;
        /* Whole 16 KiB pages around the range, like every other reservation. */
        lo = g->phys & ~0x3fffull;
        hi = (g->phys + g->size + 0x3fff) & ~0x3fffull;
        if (overlaps_memory_node(lo, hi)) g->flags |= SEG_IN_MEMORY;
        if (E) return;
        resv = fdt_path_offset(T, "/reserved-memory");
        cov = covered(lo, hi); /* before the new node exists */
        snprintf(name, sizeof(name), "aop-%s@%lx", g->name[2] == 'E' ? "etext" : "oslog", (unsigned long)lo);
        n = mknode(resv, name, NULL);
        reg[0] = lo; reg[1] = hi - lo;
        regp(n, 2, reg);
        if (!E) seterr(fdt_setprop_empty(T, n, "no-map"));
        if (cov) disabled(n);
        g->flags |= cov ? SEG_COVERED : SEG_NODE;
    }
}

/* R51 lane U: PD controller (hpm2 on nub-spmi-a1) and the USB2 host path of the right-hand port (ATC2). Every node is
 * status = "disabled"; m4-usb-attach enables them at m4-test usb-pd / usb2. Values: usb_gate() above; DT-REQUEST.md of lane U.
 * `select` (irq 13) is the one value that rests on the upstream pattern (irq + 2), not on an ADT property. */
static int ps_child(int parent, const char *name, uint32_t reg, const char *label, uint32_t *ph, uint32_t parent_a, uint32_t parent_b)
{
    uint32_t c[2] = {reg, 4};
    int n = mknode(parent, name, ph);
    STRP(n, "compatible", "apple,t8103-pmgr-pwrstate");
    cellsp(n, "reg", 2, c);
    sprop(n, "label", label, (int)strlen(label) + 1);
    u32p(n, "#power-domain-cells", 0);
    u32p(n, "#reset-cells", 0);
    if (parent_a && !parent_b) u32p(n, "power-domains", parent_a);
    else if (parent_a) { c[0] = parent_a; c[1] = parent_b; cellsp(n, "power-domains", 2, c); }
    disabled(n);
    return n;
}
static void build_usb(uint32_t aic)
{
    uint32_t common_ph, spmi_ph, aon_ph, usbps_ph, phymx_ph, dp_ph, dart0_ph, dart1_ph, phy_ph, spmi_ctl_ph, c[8];
    uint64_t r[8];
    int pm0 = fdt_path_offset(T, "/power-management@502800000"), n, pd, pm1;
    n = ps_child(pm0, "power-controller@3e0", 0x3e0, "atc2_common", &common_ph, 0, 0);
    pm1 = mknode(0, "power-management@508300000", NULL);
    STRP(pm1, "compatible", "apple,t6041-pmgr\0apple,t8103-pmgr\0syscon\0simple-mfd");
    r[0] = pmgr_reg1[0] + ARMIO; r[1] = 0x4000;
    regp(pm1, 2, r);
    u32p(pm1, "#address-cells", 1);
    u32p(pm1, "#size-cells", 1);
    /* lane U's final R51 request: the block is disabled too (R51's loader emitted it enabled; the children need only its node, via
     * syscon_node_to_regmap(node->parent), and the shim creates them by hand) */
    disabled(pm1);
    n = ps_child(pm1, "power-controller@70", 0x70, "nub_spmi_a1", &spmi_ph, 0, 0);
    sprop(n, "apple,always-on", "", 0);
    ps_child(pm1, "power-controller@d8", 0xd8, "atc2_usb_aon", &aon_ph, 0, 0);
    ps_child(pm1, "power-controller@140", 0x140, "atc2_usb", &usbps_ph, common_ph, aon_ph);
    ps_child(pm1, "power-controller@160", 0x160, "atc2_phymxwrap", &phymx_ph, common_ph, aon_ph);
    /* R52: ATC2_COMMON_DP, listed only by atc-phy2's clock-gates. The PHY takes one power-domains phandle, so it is modelled as a
     * genpd child of PHYMXWRAP (USB_AON, COMMON, PHYMXWRAP, DP = the ADT clock-gates order). Not an ADT parent edge. */
    ps_child(pm1, "power-controller@b0", 0xb0, "atc2_common_dp", &dp_ph, phymx_ph, 0);

    for (unsigned i = 0; i < 2; i++) {
        n = mknode(0, i ? "iommu@392f80000" : "iommu@392f00000", i ? &dart1_ph : &dart0_ph);
        STRP(n, "compatible", "apple,t6040-dart\0apple,t8110-dart");
        r[0] = usb_dart_reg[2 * i] + ARMIO; r[1] = 0x4000;
        regp(n, 2, r);
        irq3(n, aic, 1687);
        u32p(n, "power-domains", usbps_ph);
        u32p(n, "#iommu-cells", 1);
        disabled(n);
    }
    n = mknode(0, "phy@392a90000", &phy_ph);
    STRP(n, "compatible", "apple,t6040-atcphy");   /* only this string: no t8122 fallback (it would bind CONFIG_PHY_APPLE_ATC) */
    r[0] = atcphy_reg[0] + ARMIO; r[1] = 0x4000;
    r[2] = atcphy_reg[2] + ARMIO; r[3] = 0x4000;
    r[4] = usb_drd_reg3[0] + ARMIO; r[5] = 0x4000;
    regp(n, 6, r);
    STRP(n, "reg-names", "bank0\0bank1\0pipehandler");
    u32p(n, "#phy-cells", 1);
    u32p(n, "#reset-cells", 0);
    u32p(n, "power-domains", dp_ph);
    disabled(n);

    n = mknode(0, "usb@392280000", NULL);
    STRP(n, "compatible", "apple,t6040-dwc3\0apple,t8103-dwc3");
    r[0] = usb_drd_reg0[0] + ARMIO; r[1] = 0xcd00;
    r[2] = usb_drd_reg0[0] + ARMIO + 0xcd00; r[3] = 0x3200;
    regp(n, 4, r);
    STRP(n, "reg-names", "dwc3-core\0dwc3-apple");
    irq3(n, aic, 1683);
    STRP(n, "dr_mode", "host");
    c[0] = dart0_ph; c[1] = 0; c[2] = dart1_ph; c[3] = 1;
    cellsp(n, "iommus", 4, c);
    u32p(n, "power-domains", usbps_ph);
    u32p(n, "resets", phy_ph);
    c[0] = phy_ph; c[1] = 3;
    cellsp(n, "phys", 2, c);   /* PHY_TYPE_USB2 */
    STRP(n, "phy-names", "usb2-phy");
    disabled(n);

    n = mknode(0, "spmi@509198000", &spmi_ctl_ph);
    STRP(n, "compatible", "apple,t6040-spmi\0apple,spmi");
    r[0] = spmi_a1_reg[0] + ARMIO; r[1] = 0x100;
    regp(n, 2, r);
    u32p(n, "#address-cells", 2);
    u32p(n, "#size-cells", 0);
    sprop(n, "interrupt-controller", "", 0);
    u32p(n, "#interrupt-cells", 2);
    irq3(n, aic, 868);
    u32p(n, "power-domains", spmi_ph);
    disabled(n);
    pd = mknode(n, "usb-pd@c", NULL);
    STRP(pd, "compatible", "apple,sn201202x");
    c[0] = 0xc; c[1] = 0;
    cellsp(pd, "reg", 2, c);
    c[0] = 11; c[1] = 1; c[2] = 13; c[3] = 1; c[4] = 17; c[5] = 1; c[6] = 19; c[7] = 1;   /* IRQ_TYPE_EDGE_RISING */
    cellsp(pd, "interrupts", 8, c);
    STRP(pd, "interrupt-names", "irq\0select\0sleep\0wake");
    disabled(pd);
    n = mknode(pd, "connector", NULL);
    STRP(n, "compatible", "usb-c-connector");
    STRP(n, "label", "USB-C Right");
    STRP(n, "power-role", "dual");
    STRP(n, "data-role", "dual");
}
/* ------------------------------------------------------------- publication */
static void publish_probe(unsigned emitted)
{
    uint64_t w[M4_MEDIA_HEADER_WORDS + 12 * 6 + 5 * 4];
    static const char avd_power[] = "skipped(pmgr-helper-recursive-and-not-restorable;ps-read-only)";
    unsigned n = 0, i;
    int chosen = fdt_path_offset(T, "/chosen");
    if (chosen < 0) { seterr(chosen); return; }
    w[n++] = cpu_to_fdt64(M4_MEDIA_PROBE_MAGIC);
    w[n++] = cpu_to_fdt64(M4_MEDIA_PROBE_VERSION);
    w[n++] = 0;
    w[n++] = cpu_to_fdt64(emitted);
    w[n++] = cpu_to_fdt64(record_count);
    w[n++] = cpu_to_fdt64(segment_count);
    w[n++] = cpu_to_fdt64(gate_bits);
    w[n++] = 0;
    for (i = 0; i < record_count; i++) {
        const struct m4_media_record *r = &records[i];
        w[n++] = cpu_to_fdt64(r->id);
        w[n++] = cpu_to_fdt64(r->state);
        w[n++] = cpu_to_fdt64(r->address);
        w[n++] = cpu_to_fdt64(r->value);
        w[n++] = cpu_to_fdt64(r->esr);
        w[n++] = cpu_to_fdt64(r->far);
    }
    for (i = 0; i < segment_count; i++) {
        memcpy(&w[n], segments[i].name, 8); /* bytes as stored, first char first */
        w[n + 1] = cpu_to_fdt64(segments[i].phys);
        w[n + 2] = cpu_to_fdt64(segments[i].size);
        w[n + 3] = cpu_to_fdt64(segments[i].flags);
        n += 4;
    }
    w[2] = cpu_to_fdt64((uint64_t)n * 8);
    sprop(chosen, M4_MEDIA_PROBE_PROPERTY, w, n * 8);
    sprop(chosen, "m4,media-probe-avd-power-step", avd_power, sizeof(avd_power));
}

/* /chosen/m4,audio-fdt-status and m4,avd-fdt-status = <stage code detail>
 * (names and codes of the earlier lanes): 0 emitted, 2 ADT layout differs,
 * 4 FDT edit failed (detail = libfdt error). */
static void status_props(int32_t ac, int32_t ad, int32_t vc, int32_t vd)
{
    uint32_t w[3] = {1, (uint32_t)ac, (uint32_t)ad};
    int chosen = fdt_path_offset(T, "/chosen");
    if (chosen < 0) { seterr(chosen); return; }
    cellsp(chosen, "m4,audio-fdt-status", 3, w);
    w[1] = (uint32_t)vc; w[2] = (uint32_t)vd;
    cellsp(chosen, "m4,avd-fdt-status", 3, w);
}

/* /chosen/m4,usb-fdt-status = <1 code detail>: 0 emitted, 2 ADT layout differs, 4 FDT edit failed (detail = libfdt error) */
static void usb_status(int32_t code, int32_t detail)
{
    uint32_t w[3] = {1, (uint32_t)code, (uint32_t)detail};
    int chosen = fdt_path_offset(T, "/chosen");
    if (chosen < 0) { seterr(chosen); return; }
    cellsp(chosen, "m4,usb-fdt-status", 3, w);
}

static int commit(void *tree, size_t bytes, void *scratch, size_t capacity)
{
    int ret;
    if (E) return E;
    if ((ret = fdt_pack(scratch)) || (ret = fdt_check_full(scratch, capacity))) return ret;
    if (fdt_totalsize(scratch) > bytes) return -FDT_ERR_NOSPACE;
    m4_ram_copy(tree, scratch, fdt_totalsize(scratch));
    return 0;
}

/* Per-block pre-checks on the scratch copy, so that one block's missing parent
 * or pre-existing node cannot take the other block's nodes down with it. */
static int32_t audio_precheck(void)
{
    uint64_t fw[2];
    if (!aop_firmware(fw)) return -FDT_ERR_BADVALUE;
    return fdt_path_offset(T, "/aop@510600000") >= 0 ? -FDT_ERR_EXISTS : 0;
}
static int32_t usb_precheck(void)
{
    int pm0 = fdt_path_offset(T, "/power-management@502800000");
    if (pm0 < 0) return pm0;
    return fdt_path_offset(T, "/spmi@509198000") >= 0 || fdt_path_offset(T, "/usb@392280000") >= 0 ||
           fdt_path_offset(T, "/power-management@508300000") >= 0 ||
           fdt_subnode_offset(T, pm0, "power-controller@3e0") >= 0 ? -FDT_ERR_EXISTS : 0;
}
static int32_t avd_precheck(void)
{
    int pmgr = fdt_path_offset(T, "/power-management@502800000");
    if (pmgr < 0) return pmgr;
    return fdt_path_offset(T, "/avd@280000000") >= 0 ||
           fdt_subnode_offset(T, pmgr, "power-controller@170") >= 0 ? -FDT_ERR_EXISTS : 0;
}

/* Returns 0 when everything gated in was emitted; otherwise 2 (ADT gate) or
 * 4 (pre-check or edit failure). A failed edit publishes only the status and
 * probe records on an untouched copy; a failed pre-check drops only its block. */
int m4_media_fdt_write(void *tree, size_t bytes, void *scratch, size_t capacity)
{
    int ret, audio_ok, avd_ok, usb_ok, aic, do_audio, do_avd, do_usb;
    int32_t ad = 0, vd = 0, ud = 0;
    if (!tree || !scratch || capacity > INT32_MAX || bytes < sizeof(struct fdt_header) ||
        fdt_check_full(tree, bytes)) return -1;
    audio_ok = aop_gate() && HASARR("/arm-io/dart-aop", "reg", dart_reg) &&
               HASARR("/arm-io/admac-leap-ns", "reg", leap_reg) &&
               HASARR("/arm-io/admac-base-ns", "reg", base_reg);
    avd_ok = avd_gate() && HASARR("/arm-io/dart-avd0", "reg", avd_dart_reg);
    usb_ok = usb_gate();
    collect_segments();
    if ((ret = fdt_open_into(tree, scratch, (int)capacity))) return ret;
    T = scratch;
    E = 0;
    aic = fdt_path_offset(T, "/interrupt-controller@502400000");
    if (audio_ok) ad = aic < 0 ? aic : audio_precheck();
    if (avd_ok) vd = aic < 0 ? aic : avd_precheck();
    if (usb_ok) ud = aic < 0 ? aic : usb_precheck();
    do_audio = audio_ok && !ad;
    do_avd = avd_ok && !vd;
    do_usb = usb_ok && !ud;
    if (do_audio || do_avd || do_usb) {
        uint32_t aic_ph = fdt_get_phandle(T, aic);
        if (!aic_ph) seterr(-FDT_ERR_NOTFOUND);
        if (!E && do_audio) build_audio(aic_ph);
        if (!E && do_avd) build_avd(aic_ph);
        if (!E && do_usb) build_usb(aic_ph);
        if (!E && do_audio) reserve_segments();
        if (!E) status_props(audio_ok ? (ad ? 4 : 0) : 2, ad, avd_ok ? (vd ? 4 : 0) : 2, vd);
        if (!E) usb_status(usb_ok ? (ud ? 4 : 0) : 2, ud);
        if (!E) publish_probe((do_audio ? 1u : 0u) | (do_avd ? 2u : 0u) | (do_usb ? 4u : 0u));
        if (!(ret = commit(tree, bytes, scratch, capacity)))
            return audio_ok && avd_ok ? (ad || vd || ud ? 4 : 0) : 2;   /* usb is optional: it only counts when it was gated in */
        if (do_audio) ad = ret;
        if (do_avd) vd = ret;
        if (do_usb) ud = ret;
        if ((ret = fdt_open_into(tree, scratch, (int)capacity))) return ret;
        T = scratch;
        E = 0;
    }
    /* Refused or failed: reasons and probe record only, on a clean copy. */
    status_props(audio_ok ? 4 : 2, ad, avd_ok ? 4 : 2, vd);
    usb_status(usb_ok ? 4 : 2, ud);
    publish_probe(0);
    if ((ret = commit(tree, bytes, scratch, capacity))) return ret;
    return audio_ok || avd_ok || usb_ok ? 4 : 2;
}

/* ---------------------------------------------------------------- rev 11 */
static void media_delete(void *t, const char *const *paths, unsigned n)
{
    for (unsigned i = 0; i < n; i++) {
        int node = fdt_path_offset(t, paths[i]);
        if (node >= 0) fdt_del_node(t, node);
    }
}
/* A group whose ADT gate fails is not described at all (rev 10 emitted nothing for it). The AOP DRAM segments are
 * adopted into the DTS aop-etext/aop-oslog placeholders as rev 10 created them: page-rounded, no-map, disabled when
 * already inside another reservation. Runs after the loader's own reservations exist. */
int m4_media_fixups(void *tree, size_t capacity)
{
    static const char *const audio[] = {"/aop@510600000", "/dma-controller@513800000", "/dma-controller@513300000",
                                        "/mailbox@510608000", "/iommu@510f00000"};
    static const char *const avd[] = {"/avd@280000000", "/iommu@281020000", "/power-management@502800000/power-controller@170"};
    static const char *const usb[] = {"/spmi@509198000", "/usb@392280000", "/phy@392a90000", "/iommu@392f80000",
                                      "/iommu@392f00000", "/power-management@508300000",
                                      "/power-management@502800000/power-controller@3e0"};
    int audio_ok = aop_gate() && HASARR("/arm-io/dart-aop", "reg", dart_reg) &&
                   HASARR("/arm-io/admac-leap-ns", "reg", leap_reg) && HASARR("/arm-io/admac-base-ns", "reg", base_reg);
    int avd_ok = avd_gate() && HASARR("/arm-io/dart-avd0", "reg", avd_dart_reg);
    int usb_ok = usb_gate();
    if (fdt_open_into(tree, tree, (int)capacity)) return -3;
    T = tree;
    E = 0;
    if (!audio_ok) media_delete(tree, audio, sizeof(audio) / sizeof(*audio));
    if (!avd_ok) media_delete(tree, avd, sizeof(avd) / sizeof(*avd));
    if (!usb_ok) media_delete(tree, usb, sizeof(usb) / sizeof(*usb));
    collect_segments();
    int resv = fdt_path_offset(tree, "/reserved-memory");
    if (resv < 0) return resv;
    int used[2] = {0, 0};
    /* rev 14: iommu-addresses = <&aop iova size>, when the kernel DT gives the AOP node a phandle (never create one) */
    int aop_node = fdt_path_offset(tree, "/aop@510600000");
    uint32_t aop_ph = aop_node >= 0 ? fdt_get_phandle(tree, aop_node) : 0;
    for (unsigned s = 0; audio_ok && s < segment_count; s++) {
        struct segment *g = &segments[s];
        if (!(g->flags & SEG_WANTED) || !g->size || g->size > 0x1000000 || g->phys + g->size < g->phys) continue;
        uint64_t lo = g->phys & ~0x3fffull, hi = (g->phys + g->size + 0x3fff) & ~0x3fffull, reg[2] = {lo, hi - lo};
        unsigned kind = g->name[2] == 'E' ? 0 : 1;
        const char *base = kind ? "aop-oslog" : "aop-etext";
        char name[32];
        int cov = covered(lo, hi), n;
        if (used[kind]) continue;
        resv = fdt_path_offset(tree, "/reserved-memory");
        n = fdt_subnode_offset(tree, resv, base);
        if (n < 0) return n;
        snprintf(name, sizeof(name), "%s@%lx", base, (unsigned long)lo);
        regp(n, 2, reg);
        if (!E) seterr(fdt_setprop_empty(T, n, "no-map"));
        if (cov) disabled(n);
        if (!cov && fdt_getprop(T, n, "status", NULL)) fdt_delprop(T, n, "status");
        if (!E) seterr(fdt_set_name(T, n, name));
        if (!cov && aop_ph && g->iova >= g->phys - lo) {
            uint64_t iova = g->iova - (g->phys - lo), size = hi - lo;
            fdt32_t cells[5] = {cpu_to_fdt32(aop_ph), cpu_to_fdt32(iova >> 32), cpu_to_fdt32(iova),
                                cpu_to_fdt32(size >> 32), cpu_to_fdt32(size)};
            if (!E) seterr(fdt_setprop(T, n, "iommu-addresses", cells, sizeof(cells)));
        }
        used[kind] = 1;
        if (E) return E;
    }
    resv = fdt_path_offset(tree, "/reserved-memory");
    for (unsigned k = 0; k < 2; k++)
        if (!used[k]) {
            int n = fdt_subnode_offset(tree, resv, k ? "aop-oslog" : "aop-etext");
            if (n >= 0) fdt_del_node(tree, n);
        }
    return E ? E : fdt_pack(tree);
}
