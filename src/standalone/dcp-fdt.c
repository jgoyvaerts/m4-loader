/* SPDX-License-Identifier: MIT */
/* J616s internal-panel DCP nodes from the accepted display capture.
 * RAM/FDT work only: no DART, mailbox or DCP access. Every ADT value the
 * nodes depend on is checked against the qualified 26.6.2 layout first; any
 * mismatch leaves the tree as it was apart from /chosen/m4,dcp-fdt-status.
 *
 * Built only when M4_DCP_STAGE is set (the default loader is unchanged).
 * The stage selects what Linux may probe:
 *   1  nodes and reserved mappings only, everything disabled
 *   2  also enable both locked DARTs (checks mapping preservation)
 *   3  also enable mailbox, DCP and display-subsystem (M1 attach) */
#include "display-handoff.h"
#include "fixups.h"
#include <stdio.h>
#include "adt.h"
#include "libfdt.h"
#include "utils.h"
#include <stdint.h>
#include <string.h>

#if !defined(M4_DCP_STAGE) || M4_DCP_STAGE < 1 || M4_DCP_STAGE > 3
#error "build dcp-fdt.c only with M4_DCP_STAGE 1, 2 or 3"
#endif

#define ARMIO UINT64_C(0x200000000)
#define MAX_REGIONS 32u

/* ADT address/size pairs; physical = ADT + ARMIO. */
static const uint64_t dcp_reg[] = {0x202e00000, 0x88000};
static const uint64_t dcp_dart_reg[] = {0x202340000, 0x20000};
static const uint64_t disp0_dart_reg[] = {0x202300000, 0xc000};
static const uint64_t disp0_reg[] = {0x200000000, 0x4000000, 0x202240000, 0x10000,
                                     0x2023a8000, 0x4000, 0x202800000, 0x800000,
                                     0x304800000, 0xbc000};
/* ADT order is not-empty, empty per pair; emitted as empty, not-empty. */
static const uint32_t dcp_irq[] = {0x478, 0x477, 0x47a, 0x479};
/* The boot firmware that loads the 25G83 firmware: macOS 26.6.2's, and the same build as a stub installed from Recovery
 * (25G241) reports it. */
static const char *const boot_firmware[] = {"mBoot-18000.161.10", "mBoot-18000.161.10.701.1"};

struct region { uint64_t pa, size, iova[3]; uint32_t users; };

static int adt_bytes(const char *path, const char *key, const void *want, uint32_t bytes)
{
    uint32_t len;
    int node = adt_path_offset(adt, path);
    const void *p = node < 0 ? NULL : adt_getprop(adt, node, key, &len);
    return p && len >= bytes && !memcmp(p, want, bytes);
}

static int adt_boot_firmware(void)
{
    for (unsigned i = 0; i < sizeof(boot_firmware) / sizeof(*boot_firmware); i++)
        if (adt_bytes("/chosen", "firmware-version", boot_firmware[i], strlen(boot_firmware[i]) + 1)) return 1;
    return 0;
}

static int adt_regs(const char *path, const uint64_t *want, unsigned words)
{
    return adt_bytes(path, "reg", want, words * sizeof(*want));
}

static int adt_layout(void)
{
    const uint64_t range[] = {0, ARMIO};
    const uint32_t one = 1, dart_irq = 0x442, dcp_sid = 0x17, piodma_sid = 0x10;
    return adt_boot_firmware() &&
           adt_bytes("/arm-io", "ranges", range, sizeof(range)) &&
           adt_regs("/arm-io/dcp", dcp_reg, 2) &&
           adt_bytes("/arm-io/dcp", "interrupts", dcp_irq, sizeof(dcp_irq)) &&
           adt_bytes("/arm-io/dcp/iop-dcp-nub", "running", &one, 4) &&
           adt_regs("/arm-io/dart-dcp", dcp_dart_reg, 2) &&
           adt_bytes("/arm-io/dart-dcp", "interrupts", &dart_irq, 4) &&
           adt_bytes("/arm-io/dart-dcp/mapper-dcp", "reg", &dcp_sid, 4) &&
           adt_regs("/arm-io/dart-disp0", disp0_dart_reg, 2) &&
           adt_bytes("/arm-io/dart-disp0", "interrupts", &dart_irq, 4) &&
           adt_bytes("/arm-io/dart-disp0/mapper-disp0-piodma", "reg", &piodma_sid, 4) &&
           adt_regs("/arm-io/disp0", disp0_reg, 10);
}

/* Merge identical spans across the dcp/disp0/piodma streams. A partial
 * overlap between streams is not a layout we have seen; refuse it. */
static int collect(const struct m4_display_handoff *h, struct region *out, unsigned *count)
{
    unsigned n = 0;
    for (unsigned s = 0; s < 3; s++) {
        const struct m4_dt_result *r = &h->result[s];
        for (unsigned j = 0; j < r->spans; j++) {
            const struct m4_dt_span *m = &r->map[j];
            unsigned k;
            for (k = 0; k < n; k++) {
                if (out[k].pa == m->physical && out[k].size == m->size) break;
                if (m->physical < out[k].pa + out[k].size && out[k].pa < m->physical + m->size)
                    return -2;
            }
            if (k == n) {
                if (n == MAX_REGIONS) return -3;
                out[n] = (struct region){m->physical, m->size, {0, 0, 0}, 0};
                n++;
            }
            if (out[k].users & (1u << s)) return -4;
            out[k].users |= 1u << s;
            out[k].iova[s] = m->iova;
        }
    }
    *count = n;
    return n ? 0 : -5;
}

static int reg_prop(void *t, int node, const uint64_t *pairs, unsigned n, uint64_t offset)
{
    fdt64_t v[12];
    if (n > 12) return -FDT_ERR_NOSPACE;
    for (unsigned i = 0; i < n; i++) v[i] = cpu_to_fdt64(pairs[i] + (i & 1 ? 0 : offset));
    return fdt_setprop(t, node, "reg", v, n * sizeof(*v));
}

static int u32s(void *t, int node, const char *name, const uint32_t *w, unsigned n)
{
    fdt32_t v[16];
    if (n > 16) return -FDT_ERR_NOSPACE;
    for (unsigned i = 0; i < n; i++) v[i] = cpu_to_fdt32(w[i]);
    return fdt_setprop(t, node, name, v, n * sizeof(*v));
}

static int strs(void *t, int node, const char *name, const char *list, int bytes)
{
    return fdt_setprop(t, node, name, list, bytes);
}

#define S(t, n, k, lit) strs(t, n, k, lit, sizeof(lit))
#define TRY(x) do { if ((ret = (x)) < 0) return ret; } while (0)

static int add(void *t, int parent, const char *name, uint32_t *phandle)
{
    int node = fdt_add_subnode(t, parent, name), ret;
    if (node < 0 || !phandle) return node;
    TRY(fdt_generate_phandle(t, phandle));
    TRY(fdt_setprop_u32(t, node, "phandle", *phandle));
    return node;
}

static const char *status(int enabled) { return enabled ? "okay" : "disabled"; }

static int build(void *t, const struct region *rg, unsigned count, uint64_t framebuffer)
{
    int ret, node, aic = fdt_path_offset(t, "/interrupt-controller@502400000");
    uint32_t aic_ph = aic < 0 ? 0 : fdt_get_phandle(t, aic);
    uint32_t pmgr_ph, disp_dart_ph, dcp_dart_ph, mbox_ph, dcp_ph, piodma_ph, display_ph;
    uint32_t region_ph[MAX_REGIONS];
    static char region_name[MAX_REGIONS][32];
    if (!aic_ph) return -FDT_ERR_NOTFOUND;
    if (fdt_path_offset(t, "/dcp@402e00000") >= 0) return -FDT_ERR_EXISTS;

    TRY(node = add(t, 0, "power-management@504800000", &pmgr_ph));
    TRY(reg_prop(t, node, &disp0_reg[8], 2, ARMIO));
    TRY(S(t, node, "reg-names", "dcp-fw-pmgr"));
    TRY(fdt_setprop_u32(t, node, "#apple,bw-scratch-cells", 3));

    /* Locked 3-level DARTs: Linux limits them to 36-bit IOVAs, so no
     * apple,dma-range; reserved IOVAs are the captured bare ones. */
    const struct { const char *name; const uint64_t *reg; uint32_t *ph; } darts[] = {
        {"iommu@402300000", disp0_dart_reg, &disp_dart_ph},
        {"iommu@402340000", dcp_dart_reg, &dcp_dart_ph}};
    for (unsigned i = 0; i < 2; i++) {
        const uint32_t irq[] = {0, 0x442, 4};
        TRY(node = add(t, 0, darts[i].name, darts[i].ph));
        TRY(S(t, node, "compatible", "apple,t6040-dart\0apple,t8110-dart"));
        TRY(reg_prop(t, node, darts[i].reg, 2, ARMIO));
        TRY(fdt_setprop_u32(t, node, "#iommu-cells", 1));
        TRY(fdt_setprop_u32(t, node, "interrupt-parent", aic_ph));
        TRY(u32s(t, node, "interrupts", irq, 3));
        TRY(fdt_setprop_string(t, node, "status", status(M4_DCP_STAGE >= 2)));
    }

    const uint32_t mbox_irq[] = {0, dcp_irq[1], 4, 0, dcp_irq[0], 4,
                                 0, dcp_irq[3], 4, 0, dcp_irq[2], 4};
    const uint64_t mbox_reg[] = {dcp_reg[0] + 0x8000, 0x4000};
    TRY(node = add(t, 0, "mailbox@402e08000", &mbox_ph));
    TRY(S(t, node, "compatible", "apple,t6040-asc-mailbox\0apple,asc-mailbox-v4"));
    TRY(reg_prop(t, node, mbox_reg, 2, ARMIO));
    TRY(fdt_setprop_u32(t, node, "interrupt-parent", aic_ph));
    TRY(u32s(t, node, "interrupts", mbox_irq, 12));
    TRY(S(t, node, "interrupt-names", "send-empty\0send-not-empty\0recv-empty\0recv-not-empty"));
    TRY(fdt_setprop_u32(t, node, "#mbox-cells", 0));
    TRY(fdt_setprop_string(t, node, "status", status(M4_DCP_STAGE >= 3)));

    /* coproc is the first 16K of the ASC block; disp-0..3 keep ADT order. */
    const uint64_t dcp_regs[] = {dcp_reg[0], 0x4000, disp0_reg[0], disp0_reg[1],
                                 disp0_reg[2], disp0_reg[3], disp0_reg[4], disp0_reg[5],
                                 disp0_reg[6], disp0_reg[7]};
    TRY(node = add(t, 0, "dcp@402e00000", &dcp_ph));
    TRY(S(t, node, "compatible", "apple,t6040-dcp\0apple,dcp"));
    TRY(fdt_setprop_u32(t, node, "mboxes", mbox_ph));
    TRY(S(t, node, "mbox-names", "mbox"));
    const uint32_t dcp_iommu[] = {dcp_dart_ph, 0x17};
    TRY(u32s(t, node, "iommus", dcp_iommu, 2));
    TRY(S(t, node, "reg-names", "coproc\0disp-0\0disp-1\0disp-2\0disp-3"));
    TRY(reg_prop(t, node, dcp_regs, 10, ARMIO));
    /* Scratch offset 0x20000 is t8132's; unverified on T6040 (O2). */
    const uint32_t scratch[] = {pmgr_ph, 0, 4, 0x20000};
    TRY(u32s(t, node, "apple,bw-scratch", scratch, 4));
    /* R38: M4_DCP_SURFACES = 2 (default) grants a second IOMFB surface and layer so appledrm can expose a cursor
     * plane (appledrm cursor_plane=1; without that parameter the second surface is an unused overlay plane).
     * t8132 uses four surfaces in order <2 0 1 3>; those are not tried here. -DM4_DCP_SURFACES=1 restores the
     * single primary surface of R37 exactly. */
#ifndef M4_DCP_SURFACES
#define M4_DCP_SURFACES 2
#endif
#if M4_DCP_SURFACES != 1 && M4_DCP_SURFACES != 2
#error "M4_DCP_SURFACES must be 1 or 2"
#endif
#if M4_DCP_SURFACES == 2
    const uint32_t surfaces[] = {1, 1, 0, 0}, order[] = {0, 1};
#else
    const uint32_t surfaces[] = {1, 0, 0, 0}, order[] = {0};
#endif
    TRY(u32s(t, node, "apple,iomfb-surfaces", surfaces, 4));
    TRY(u32s(t, node, "apple,iomfb-surface-order", order, M4_DCP_SURFACES));
    TRY(fdt_setprop_u32(t, node, "apple,iomfb-primary-surface", 0));
    TRY(fdt_setprop_u32(t, node, "apple,iomfb-max-layers", M4_DCP_SURFACES));
    const uint32_t version[] = {26, 6, 2};
    TRY(u32s(t, node, "apple,firmware-version", version, 3));
    TRY(u32s(t, node, "apple,firmware-compat", version, 3));
    TRY(fdt_setprop_u32(t, node, "#address-cells", 2));
    TRY(fdt_setprop_u32(t, node, "#size-cells", 2));
    TRY(fdt_setprop_string(t, node, "status", status(M4_DCP_STAGE >= 3)));

    int dcp = node;
    TRY(node = add(t, dcp, "piodma", &piodma_ph));
    const uint32_t piodma_iommu[] = {disp_dart_ph, 0x10};
    TRY(u32s(t, node, "iommus", piodma_iommu, 2));

    /* Disabled panel: eDP connector, no max-brightness, so the driver
     * refuses backlight registration and never drives the mini-LED (O6). */
    dcp = fdt_path_offset(t, "/dcp@402e00000");
    TRY(node = add(t, dcp, "panel", NULL));
    TRY(S(t, node, "compatible", "apple,panel-mini-led\0apple,panel"));
    TRY(fdt_setprop_u32(t, node, "width-mm", 345));
    TRY(fdt_setprop_u32(t, node, "height-mm", 223));
    TRY(fdt_setprop_string(t, node, "status", "disabled"));

    TRY(node = add(t, 0, "display-subsystem", &display_ph));
    TRY(S(t, node, "compatible", "apple,display-subsystem"));
    const uint32_t display_iommu[] = {disp_dart_ph, 0};
    TRY(u32s(t, node, "iommus", display_iommu, 2));
    TRY(fdt_setprop_empty(t, node, "dma-noncoherent"));
    TRY(fdt_setprop_string(t, node, "status", status(M4_DCP_STAGE >= 3)));

    /* Captured regions are outside every Linux memory node; no-map only
     * describes them so iommu-addresses can keep the live translations. */
    const uint32_t owner[3] = {dcp_ph, display_ph, piodma_ph};
    int resv = fdt_path_offset(t, "/reserved-memory");
    TRY(resv);
    for (unsigned i = 0; i < count; i++) {
        char name[40];
        uint32_t cells[15]; unsigned n = 0;
        /* appledrm finds the boot framebuffer (to evict simpledrm) by the m1n1
         * convention: memory-region-names "framebuffer" -> compatible "framebuffer". */
        int fb = framebuffer >= rg[i].pa && framebuffer - rg[i].pa < rg[i].size;
        snprintf(region_name[i], sizeof(region_name[i]), fb ? "framebuffer" : "dcp-map@%lx",
                 (unsigned long)rg[i].pa);
        snprintf(name, sizeof(name), "%s@%lx", fb ? "framebuffer" : "dcp-map",
                 (unsigned long)rg[i].pa);
        resv = fdt_path_offset(t, "/reserved-memory");
        TRY(node = add(t, resv, name, &region_ph[i]));
        if (fb)
            TRY(S(t, node, "compatible", "framebuffer"));
        else
            TRY(S(t, node, "compatible", "apple,asc-mem"));
        const uint64_t reg[] = {rg[i].pa, rg[i].size};
        TRY(reg_prop(t, node, reg, 2, 0));
        TRY(fdt_setprop_empty(t, node, "no-map"));
        for (unsigned s = 0; s < 3; s++)
            if (rg[i].users & (1u << s)) {
                cells[n++] = owner[s];
                cells[n++] = (uint32_t)(rg[i].iova[s] >> 32); cells[n++] = (uint32_t)rg[i].iova[s];
                cells[n++] = (uint32_t)(rg[i].size >> 32); cells[n++] = (uint32_t)rg[i].size;
            }
        TRY(u32s(t, node, "iommu-addresses", cells, n));
    }
    const char *paths[3] = {"/dcp@402e00000", "/display-subsystem", "/dcp@402e00000/piodma"};
    for (unsigned s = 0; s < 3; s++)
        for (unsigned i = 0; i < count; i++)
            if (rg[i].users & (1u << s)) {
                TRY(node = fdt_path_offset(t, paths[s]));
                TRY(fdt_appendprop_u32(t, node, "memory-region", region_ph[i]));
                TRY(fdt_appendprop_string(t, node, "memory-region-names", region_name[i]));
            }

    TRY(node = fdt_path_offset(t, "/aliases"));
    TRY(fdt_setprop_string(t, node, "dcp", "/dcp@402e00000"));
    TRY(fdt_setprop_string(t, node, "disp0", "/display-subsystem"));
    TRY(fdt_setprop_string(t, node, "disp0_piodma", "/dcp@402e00000/piodma"));
    return 0;
}
#undef TRY
#undef S

/* /chosen/m4,dcp-fdt-status = <stage code detail>: code 0 emitted,
 * 1 capture not accepted, 2 ADT layout differs, 3 unexpected spans
 * (detail = collect() reason), 4 FDT edit failed (detail = libfdt error). */
static int finish(void *tree, size_t bytes, void *scratch, size_t capacity,
                  int32_t code, int32_t detail)
{
    const fdt32_t v[] = {cpu_to_fdt32(M4_DCP_STAGE), cpu_to_fdt32((uint32_t)code),
                         cpu_to_fdt32((uint32_t)detail)};
    int ret, chosen = fdt_path_offset(scratch, "/chosen");
    if (chosen < 0) return chosen;
    if ((ret = fdt_setprop(scratch, chosen, "m4,dcp-fdt-status", v, sizeof(v)))) return ret;
    /* detail stays a libfdt error: a tree past the caller's reserve is
     * no space (-3), not -4/-5 (= -FDT_ERR_BADOFFSET/-FDT_ERR_BADPATH). */
    if ((ret = fdt_pack(scratch)) || (ret = fdt_check_full(scratch, capacity))) return ret;
    if (fdt_totalsize(scratch) > bytes) return -FDT_ERR_NOSPACE;
    memcpy(tree, scratch, fdt_totalsize(scratch));
    return 0;
}

int m4_dcp_fdt_write(void *tree, size_t bytes, void *scratch, size_t capacity,
                     const struct m4_display_handoff *h)
{
    static struct region regions[MAX_REGIONS];
    unsigned count = 0;
    int32_t code, detail = 0;
    int ret;
    if (!tree || !scratch || !h || capacity > INT32_MAX || bytes < sizeof(struct fdt_header) ||
        fdt_check_full(tree, bytes)) return -1;
    if (!m4_display_handoff_accepted(h)) code = 1;
    else if (!adt_layout()) code = 2;
    else if ((detail = collect(h, regions, &count))) code = 3;
    else {
        if ((ret = fdt_open_into(tree, scratch, (int)capacity))) return ret;
        detail = build(scratch, regions, count, h->framebuffer);
        if (!detail && !(detail = finish(tree, bytes, scratch, capacity, 0, 0))) return 0;
        code = 4;
    }
    /* Refused or failed: publish only the reason on an untouched copy. */
    if ((ret = fdt_open_into(tree, scratch, (int)capacity))) return ret;
    if ((ret = finish(tree, bytes, scratch, capacity, code, detail))) return ret;
    return code;
}

/* ---------------------------------------------------------------- rev 11 */
#define POOL 16u
static int phandle_of(void *t, int node, uint32_t *ph)
{
    int ret;
    uint32_t v = fdt_get_phandle(t, node);
    if (!v) {
        if ((ret = fdt_generate_phandle(t, &v)) < 0 || (ret = fdt_setprop_u32(t, node, "phandle", v)) < 0) return ret;
    }
    *ph = v;
    return 0;
}
static void delete_pool(void *t)
{
    int resv = fdt_path_offset(t, "/reserved-memory");
    for (unsigned i = 0; i < POOL && resv >= 0; i++) {
        char name[24];
        snprintf(name, sizeof(name), "dcp-map-pool%u", i);
        int n = fdt_subnode_offset(t, resv, name);
        if (n >= 0) fdt_del_node(t, n);
    }
}
/* The DTS carries the DCP/DART/mailbox/display nodes disabled and POOL region placeholders. An accepted capture
 * adopts the placeholders (same names, reg and iommu-addresses rev 10 emitted), fills the three memory-region lists
 * and enables the five stage-3 nodes. Otherwise the pool is deleted and every node stays disabled. */

static int adt_u64(int node, const char *key, uint64_t *v)
{
    uint32_t len;
    const void *p = adt_getprop(adt, node, key, &len);
    if (!p || len != sizeof(*v)) return -1;
    memcpy(v, p, sizeof(*v));
    return 0;
}

/* The ADT source of each GPU reserved region, by its memory-region-names entry: an sgx "<key>-base"/"-size" pair, or
 * (key NULL) the firmware's TEXT segment, gfx-asc segment-ranges[0]. */
static const struct { const char *name, *key; } gpu_regions[] = {
    {"ttbs", "gpu-region"}, {"pagetables", "gfx-shared-region"}, {"handoff", "gfx-handoff"},
    {"l2-pagetables", "gfx-shared-l2-region"}, {"fw-data", "gfx-data"}, {"fw-text", NULL},
};

static int gpu_region_source(int sgx, int asc, const char *key, uint64_t *base, uint64_t *size)
{
    if (!key) {
        /* segment-ranges entries: phys, iova, remap (u64 each), size, flags (u32 each) */
        uint32_t len, sz;
        const uint8_t *seg = adt_getprop(adt, asc, "segment-ranges", &len);
        const char *names = adt_getprop(adt, asc, "segment-names", &len);
        if (!seg || !names || strncmp(names, "__TEXT;", 7)) return -1;
        memcpy(base, seg, 8);
        memcpy(&sz, seg + 24, 4);
        *size = sz;
    } else {
        char k[48];
        snprintf(k, sizeof(k), "%s-base", key);
        if (adt_u64(sgx, k, base)) return -1;
        snprintf(k, sizeof(k), "%s-size", key);
        if (adt_u64(sgx, k, size)) return -1;
    }
    return *base && *size && !(*base & 0x3fff) && !(*size & 0x3fff) ? 0 : -1;
}

/* Every region the GPU node names gets its reg from the ADT and is enabled; a name without an ADT source fails. */
static int gpu_fill_regions(void *t, int sgx, int asc)
{
    int len, gpu = fdt_path_offset(t, "gpu");
    const fdt32_t *ph = gpu < 0 ? NULL : fdt_getprop(t, gpu, "memory-region", &len);
    const char *names = gpu < 0 ? NULL : fdt_getprop(t, gpu, "memory-region-names", NULL);
    if (!ph || !names || len <= 0 || len % 4) return -1;
    for (int i = 0; i < len / 4; i++) {
        const char *name = fdt_stringlist_get(t, fdt_path_offset(t, "gpu"), "memory-region-names", i, NULL);
        unsigned r = 0;
        while (r < sizeof(gpu_regions) / sizeof(*gpu_regions) && (!name || strcmp(name, gpu_regions[r].name))) r++;
        uint64_t reg[2];
        if (r == sizeof(gpu_regions) / sizeof(*gpu_regions) ||
            gpu_region_source(sgx, asc, gpu_regions[r].key, &reg[0], &reg[1]))
            return -1;
        /* the GPU node and its phandle list can move with every edit: resolve both again */
        ph = fdt_getprop(t, fdt_path_offset(t, "gpu"), "memory-region", NULL);
        int node = fdt_node_offset_by_phandle(t, fdt32_to_cpu(ph[i])), ret;
        if (node < 0) return -1;
        if ((ret = reg_prop(t, node, reg, 2, 0)) < 0) return ret;
        fdt_delprop(t, node, "status");
        char unit[48];
        const char *old = fdt_get_name(t, node, NULL), *at = old ? strchr(old, '@') : NULL;
        snprintf(unit, sizeof(unit), "%.*s@%lx", (int)(at ? at - old : (old ? (long)strlen(old) : 0)), old ? old : "",
                 (unsigned long)reg[0]);
        if ((ret = fdt_set_name(t, node, unit)) < 0) return ret;
    }
    return 0;
}

/* ADT perf-states: perf-state-table-count tables of perf-state-count {Hz, mV} rows, table after table. Each OPP of
 * the operating-points-v2 table, in order, gets the row's frequency and one microvolt cell per table. */
static int gpu_fill_opps(void *t, int sgx)
{
    uint32_t count, tables, len;
    const uint32_t *rows = adt_getprop(adt, sgx, "perf-states", &len);
    if (ADT_GETPROP(adt, sgx, "perf-state-count", &count) < 0 ||
        ADT_GETPROP(adt, sgx, "perf-state-table-count", &tables) < 0 || !rows || !count || count > 32 ||
        !tables || tables > 2 || len != count * tables * 8)
        return -1;
    int gpu = fdt_path_offset(t, "gpu"), plen;
    const fdt32_t *ph = gpu < 0 ? NULL : fdt_getprop(t, gpu, "operating-points-v2", &plen);
    int opps = ph && plen == 4 ? fdt_node_offset_by_phandle(t, fdt32_to_cpu(*ph)) : -1, opp;
    if (opps < 0) return -1;
    uint32_t i = 0;
    fdt_for_each_subnode(opp, t, opps) {
        uint32_t uv[2];
        if (i >= count) return -1;
        for (uint32_t k = 0; k < tables; k++) {
            if (rows[(k * count + i) * 2] != rows[i * 2]) return -1;
            uv[k] = rows[(k * count + i) * 2 + 1] * 1000;
        }
        int ret;
        if ((ret = fdt_setprop_u64(t, opp, "opp-hz", rows[i * 2])) < 0 ||
            (ret = u32s(t, opp, "opp-microvolt", uv, tables)) < 0)
            return ret;
        i++;
    }
    return i == count ? 0 : -1;
}

/* ADT cs-/afr-perf-states: dies and count (u64), then count rows per die of {uV, Hz} (u64); the J616s has one more
 * u64 after them, which is not an operating point. One cell per die. */
static int gpu_fill_aux_opps(void *t, int sgx, const char *key, const char *prop)
{
    uint32_t len;
    const uint64_t *ps = adt_getprop(adt, sgx, key, &len);
    if (!ps || len < 16) return -1;
    uint64_t dies = ps[0], count = ps[1];
    if (!dies || dies > 2 || !count || count > 32 || len < 16 + dies * count * 16) return -1;
    int gpu = fdt_path_offset(t, "gpu"), plen;
    const fdt32_t *ph = gpu < 0 ? NULL : fdt_getprop(t, gpu, prop, &plen);
    int opps = ph && plen == 4 ? fdt_node_offset_by_phandle(t, fdt32_to_cpu(*ph)) : -1, opp;
    if (opps < 0) return -1;
    uint32_t i = 0;
    fdt_for_each_subnode(opp, t, opps) {
        uint32_t uv[2];
        if (i >= count) return -1;
        for (uint64_t d = 0; d < dies; d++) uv[d] = (uint32_t)ps[2 + (d * count + i) * 2];
        int ret;
        if ((ret = fdt_setprop_u64(t, opp, "opp-hz", ps[2 + i * 2 + 1])) < 0 ||
            (ret = u32s(t, opp, "opp-microvolt", uv, (unsigned)dies)) < 0)
            return ret;
        i++;
    }
    return i == count ? 0 : -1;
}

static uint32_t single_phandle(void *t, int node, const char *prop)
{
    int len;
    const fdt32_t *p = node < 0 ? NULL : fdt_getprop(t, node, prop, &len);
    return p && len == 4 ? fdt32_to_cpu(*p) : 0;
}

static int set_always_on(void *t, int node, int on)
{
    if (node < 0) return node;
    if (on) return fdt_setprop_empty(t, node, "apple,always-on");
    int ret = fdt_delprop(t, node, "apple,always-on");
    return ret == -FDT_ERR_NOTFOUND ? 0 : ret;
}

/* The GPU's companions follow its status (t6040-j616s.dts): its mailbox is enabled, and its power state, that state's
 * parent and the PMP report entry on it are always on, so that the PMP sees the GPU on exactly while it runs. With no
 * GPU driver an always-on PMP report crashes the PMP firmware. -1 when the tree does not describe them. */
static int gpu_companions(void *t, int on)
{
    int gpu = fdt_path_offset(t, "gpu"), ret, node;
    uint32_t mbox = single_phandle(t, gpu, "mboxes"), ps = single_phandle(t, gpu, "power-domains");
    uint32_t parent = single_phandle(t, fdt_node_offset_by_phandle(t, ps), "power-domains");
    unsigned reports = 0;
    if (!mbox || !ps || !parent || fdt_node_offset_by_phandle(t, mbox) < 0) return -1;
    if ((ret = fdt_setprop_string(t, fdt_node_offset_by_phandle(t, mbox), "status", on ? "okay" : "disabled")) < 0 ||
        (ret = set_always_on(t, fdt_node_offset_by_phandle(t, ps), on)) < 0 ||
        (ret = set_always_on(t, fdt_node_offset_by_phandle(t, parent), on)) < 0)
        return ret;
    /* an edit inside a node moves only what follows it, so the search goes on from the edited node */
    for (node = fdt_node_offset_by_compatible(t, -1, "apple,t6000-pmp-v2-report-entry"); node >= 0;
         node = fdt_node_offset_by_compatible(t, node, "apple,t6000-pmp-v2-report-entry"))
        if (single_phandle(t, node, "power-domains") == ps) {
            if ((ret = set_always_on(t, node, on)) < 0) return ret;
            reports++;
        }
    return reports == 1 ? 0 : -1;
}

/* The J616s GPU node carries no per-unit data of its own: the boot loader fills it from this unit's ADT, as m1n1's
 * dt_set_gpu() does on other SoCs (the reserved regions, the operating points with their voltages, the CS and AFR
 * operating points, the firmware version), and enables it and its companions last. Anything missing leaves the GPU and
 * its companions disabled, with the reason in /chosen m4,gpu. 0 filled or no GPU node, 1 left disabled, <0 a tree error. */
int m4_gpu_fixup(void *t, size_t capacity)
{
    const uint32_t version[] = {26, 6, 2};
    const char *why = NULL;
    int ret, known;
    if (fdt_path_offset(t, "gpu") < 0) return 0;
    /* The driver proves the GPU firmware build from the loaded TEXT (fw-text); the version only names it. */
    known = adt_boot_firmware();
    int sgx = adt_path_offset(adt, "/arm-io/sgx"), asc = adt_path_offset(adt, "/arm-io/gfx-asc");
    /* The DCP and media fix-ups leave the tree packed. */
    if ((ret = fdt_open_into(t, t, (int)capacity)) < 0) return ret;
    if (!known) why = "boot-firmware";
    else if (sgx < 0 || asc < 0) why = "adt";
    else if ((ret = gpu_fill_regions(t, sgx, asc))) why = "regions";
    else if ((ret = gpu_fill_opps(t, sgx))) why = "opps";
    else if ((ret = gpu_fill_aux_opps(t, sgx, "cs-perf-states", "apple,cs-opp"))) why = "cs-opps";
    else if ((ret = gpu_fill_aux_opps(t, sgx, "afr-perf-states", "apple,afr-opp"))) why = "afr-opps";
    else if ((ret = u32s(t, fdt_path_offset(t, "gpu"), "apple,firmware-version", version, 3)) < 0) return ret;
    if (ret < -1) return ret; /* libfdt error (-1 is "no ADT source") */
    if (!why && (ret = gpu_companions(t, 1)) == -1) why = "companions";
    if (ret < -1) return ret;
    if (why && (ret = gpu_companions(t, 0)) < -1) return ret; /* -1: nothing to turn off */
    if ((ret = fdt_setprop_string(t, fdt_path_offset(t, "gpu"), "status", why ? "disabled" : "okay")) < 0) return ret;
    /* resolved after the status edit, which can move it */
    int chosen = fdt_path_offset(t, "/chosen");
    if (chosen >= 0 && (ret = fdt_setprop_string(t, chosen, "m4,gpu", why ? why : "filled")) < 0) return ret;
    return fdt_pack(t) ? -1 : !!why;
}

/* ---------------------------------------------------------------- external DCPs
 * dcpext0/dcpext1 (HDMI, USB-C DisplayPort) run iBoot's firmware, loaded before Linux like the internal DCP's. Their
 * firmware memory is per unit and per macOS: board.dtb describes dcpextN with a memory-region list of three disabled
 * placeholders (text, data, heap) and the loader fills them from /arm-io/dcpextN/iop-dcpextN-nub segment-ranges.
 * Nothing else is touched: dcpextN, its DART and mailbox stay disabled (they are enabled after boot, in order), and no
 * handoff marker is published, because the loader neither reads nor locks the dcpext DARTs. A tree without
 * placeholders (the memory written into it statically) is left alone. A unit whose ADT is not the known layout loses
 * its placeholders and memory-region, so its node stays valid and without firmware memory.
 * /chosen m4,dcpext = <unit0 unit1>: 0 filled, 1 no placeholders, 2 memory already static, 3 unknown boot firmware,
 * 4 ADT layout differs, 5 a region lies in Linux RAM (or the /memory cells cannot be read). */
#define DCPEXT_UNITS 2u
#define DCPEXT_REGIONS 3u
enum { DCPEXT_FILLED, DCPEXT_NONE, DCPEXT_STATIC, DCPEXT_FIRMWARE, DCPEXT_LAYOUT, DCPEXT_RAM };

/* ADT segments: __TEXT, __DATA, __OS_LOG (per boot, left out as for the internal DCP) and the heap. */
static const char dcpext_segment_names[] = "__TEXT;__DATA;__OS_LOG;unknown";
static const unsigned dcpext_segment[DCPEXT_REGIONS] = {0, 1, 3};
static const char *const dcpext_region[DCPEXT_REGIONS] = {"text", "data", "heap"};

struct dcpext_seg { uint64_t phys, iova, size; };

/* segment-ranges entries: phys, iova, remap (u64 each), size, flags (u32 each). The device address of a segment is its
 * remap address minus 1 << 40; sizes are rounded up to 16 KiB. */
static int dcpext_adt(unsigned unit, struct dcpext_seg *out)
{
    char path[48];
    uint32_t len, running;
    snprintf(path, sizeof(path), "/arm-io/dcpext%u/iop-dcpext%u-nub", unit, unit);
    int nub = adt_path_offset(adt, path);
    const uint8_t *seg = nub < 0 ? NULL : adt_getprop(adt, nub, "segment-ranges", &len);
    uint32_t nlen, rlen;
    const char *names = nub < 0 ? NULL : adt_getprop(adt, nub, "segment-names", &nlen);
    const uint32_t *run = nub < 0 ? NULL : adt_getprop(adt, nub, "running", &rlen);
    if (!seg || len != 4 * 32 || !names || nlen != sizeof(dcpext_segment_names) ||
        memcmp(names, dcpext_segment_names, nlen) || !run || rlen != 4)
        return -1;
    memcpy(&running, run, 4);
    if (running != 1) return -1;
    for (unsigned i = 0; i < DCPEXT_REGIONS; i++) {
        uint64_t phys, remap;
        uint32_t size;
        const uint8_t *e = seg + dcpext_segment[i] * 32;
        memcpy(&phys, e, 8);
        memcpy(&remap, e + 16, 8);
        memcpy(&size, e + 24, 4);
        if (!phys || (phys & 0x3fff) || phys >= (UINT64_C(1) << 48) || !size || size > 0x10000000u || (remap & 0x3fff) ||
            (remap & ~UINT64_C(0xffffffff)) != UINT64_C(0x10000000000))
            return -1;
        out[i] = (struct dcpext_seg){phys, remap & 0xffffffff, (size + 0x3fffu) & ~0x3fffu};
    }
    for (unsigned i = 0; i < DCPEXT_REGIONS; i++)
        for (unsigned j = i + 1; j < DCPEXT_REGIONS; j++)
            if (out[i].phys < out[j].phys + out[j].size && out[j].phys < out[i].phys + out[i].size) return -1;
    return 0;
}

/* Does [phys, phys + size) meet a /memory range? A root without 2/2 cells cannot be checked and counts as RAM. */
static int dcpext_in_ram(void *t, uint64_t phys, uint64_t size)
{
    int node;
    if (fdt_address_cells(t, 0) != 2 || fdt_size_cells(t, 0) != 2) return 1;
    fdt_for_each_subnode(node, t, 0) {
        const char *type = fdt_getprop(t, node, "device_type", NULL);
        int len;
        const fdt32_t *r = type && !strcmp(type, "memory") ? fdt_getprop(t, node, "reg", &len) : NULL;
        for (int i = 0; r && i + 16 <= len; i += 16, r += 4) {
            uint64_t b = ((uint64_t)fdt32_to_cpu(r[0]) << 32) | fdt32_to_cpu(r[1]);
            uint64_t s = ((uint64_t)fdt32_to_cpu(r[2]) << 32) | fdt32_to_cpu(r[3]);
            if (phys < b + s && b < phys + size) return 1;
        }
    }
    return 0;
}

/* The unit (0 or 1) whose ADT register base is `phys`, or -1. */
static int dcpext_unit_of(uint64_t phys)
{
    for (unsigned unit = 0; unit < DCPEXT_UNITS; unit++) {
        char path[24];
        uint32_t len;
        snprintf(path, sizeof(path), "/arm-io/dcpext%u", unit);
        int dev = adt_path_offset(adt, path);
        const uint64_t *r = dev < 0 ? NULL : adt_getprop(adt, dev, "reg", &len);
        if (r && len >= 16 && r[0] + ARMIO == phys) return (int)unit;
    }
    return -1;
}

/* The placeholders of a device node: its memory-region phandles, three distinct disabled children of /reserved-memory.
 * Anything else is memory the tree already describes and is left alone. */
static int dcpext_placeholders(void *t, int dev, uint32_t ph[DCPEXT_REGIONS])
{
    int len, resv = fdt_path_offset(t, "/reserved-memory");
    const fdt32_t *list = fdt_getprop(t, dev, "memory-region", &len);
    if (!list) return DCPEXT_NONE;
    if (len != DCPEXT_REGIONS * 4 || resv < 0) return DCPEXT_STATIC;
    for (unsigned i = 0; i < DCPEXT_REGIONS; i++) {
        ph[i] = fdt32_to_cpu(list[i]);
        int node = fdt_node_offset_by_phandle(t, ph[i]);
        const char *st = node < 0 ? NULL : fdt_getprop(t, node, "status", NULL);
        if (!st || strcmp(st, "disabled") || fdt_parent_offset(t, node) != resv) return DCPEXT_STATIC;
        for (unsigned j = 0; j < i; j++) if (ph[j] == ph[i]) return DCPEXT_STATIC;
    }
    return DCPEXT_FILLED;
}

/* Refused: the placeholders go and the node keeps no firmware memory. */
static int dcpext_drop(void *t, uint32_t dev_ph, const uint32_t ph[DCPEXT_REGIONS])
{
    int ret, node;
    for (unsigned i = 0; i < DCPEXT_REGIONS; i++)
        if ((node = fdt_node_offset_by_phandle(t, ph[i])) >= 0 && (ret = fdt_del_node(t, node)) < 0) return ret;
    if ((node = fdt_node_offset_by_phandle(t, dev_ph)) < 0) return node;
    if ((ret = fdt_delprop(t, node, "memory-region")) < 0) return ret;
    ret = fdt_delprop(t, node, "memory-region-names");
    return ret == -FDT_ERR_NOTFOUND ? 0 : ret;
}

#define NODE(v) do { if ((node = fdt_node_offset_by_phandle(t, (v))) < 0) return node; } while (0)

/* One device node by phandle: sets *state and returns 0, or a libfdt error. */
static int dcpext_unit(void *t, uint32_t dev_ph, unsigned unit, int known, int *state)
{
    static const uint32_t version[] = {26, 6, 2};
    struct dcpext_seg seg[DCPEXT_REGIONS];
    uint32_t ph[DCPEXT_REGIONS];
    int ret, node;
    NODE(dev_ph);
    if ((*state = dcpext_placeholders(t, node, ph)) != DCPEXT_FILLED) return 0;
    if (!known) *state = DCPEXT_FIRMWARE;
    else if (dcpext_adt(unit, seg)) *state = DCPEXT_LAYOUT;
    for (unsigned i = 0; *state == DCPEXT_FILLED && i < DCPEXT_REGIONS; i++)
        if (dcpext_in_ram(t, seg[i].phys, seg[i].size)) *state = DCPEXT_RAM;
    if (*state != DCPEXT_FILLED) return dcpext_drop(t, dev_ph, ph);
    for (unsigned i = 0; i < DCPEXT_REGIONS; i++) {
        char unitname[48];
        uint32_t cells[5] = {dev_ph, (uint32_t)(seg[i].iova >> 32), (uint32_t)seg[i].iova,
                             (uint32_t)(seg[i].size >> 32), (uint32_t)seg[i].size};
        const uint64_t rg[] = {seg[i].phys, seg[i].size};
        NODE(ph[i]);
        if ((ret = reg_prop(t, node, rg, 2, 0)) < 0 || (ret = u32s(t, node, "iommu-addresses", cells, 5)) < 0)
            return ret;
        fdt_delprop(t, node, "status");
        snprintf(unitname, sizeof(unitname), "dcpext%u-%s@%lx", unit, dcpext_region[i], (unsigned long)seg[i].phys);
        if ((ret = fdt_set_name(t, node, unitname)) < 0) return ret;
    }
    NODE(dev_ph);
    if ((ret = fdt_delprop(t, node, "memory-region-names")) < 0 && ret != -FDT_ERR_NOTFOUND) return ret;
    for (unsigned i = 0; i < DCPEXT_REGIONS; i++) {
        char name[32];
        snprintf(name, sizeof(name), "dcp-map@%lx", (unsigned long)seg[i].phys);
        NODE(dev_ph);
        if ((ret = fdt_appendprop_string(t, node, "memory-region-names", name)) < 0) return ret;
    }
    NODE(dev_ph);
    if ((ret = u32s(t, node, "apple,firmware-version", version, 3)) < 0) return ret;
    NODE(dev_ph);
    return u32s(t, node, "apple,firmware-compat", version, 3);
}
#undef NODE

/* Fills the dcpext placeholders of board.dtb from the ADT. 0 done (nothing to do, filled or memory already static),
 * 1 a unit was left without firmware memory (reason in /chosen m4,dcpext), <0 a tree error. */
int m4_dcpext_fixup(void *t, size_t capacity)
{
    uint32_t phandle[DCPEXT_UNITS], record[DCPEXT_UNITS] = {DCPEXT_NONE, DCPEXT_NONE};
    int unit_of[DCPEXT_UNITS] = {-1, -1}, found = 0, seen = 0, ret, node, left = 0, known;
    known = adt_boot_firmware();
    if ((ret = fdt_open_into(t, t, (int)capacity)) < 0) return ret;
    /* the device nodes are found by their ADT register base, so no alias is needed; phandles first, edits after */
    for (node = fdt_node_offset_by_compatible(t, -1, "apple,t6041-dcpext"); node >= 0;
         node = fdt_node_offset_by_compatible(t, node, "apple,t6041-dcpext")) {
        int len, unit;
        const fdt32_t *r = fdt_getprop(t, node, "reg", &len);
        if (!r || len < 8) continue;
        unit = dcpext_unit_of(((uint64_t)fdt32_to_cpu(r[0]) << 32) | fdt32_to_cpu(r[1]));
        if (unit < 0 || (seen & (1 << unit))) continue;
        seen |= 1 << unit;
        unit_of[found] = unit;
        if ((ret = phandle_of(t, node, &phandle[found])) < 0) return ret;
        found++;
    }
    for (int i = 0; i < found; i++) {
        int state;
        if ((ret = dcpext_unit(t, phandle[i], (unsigned)unit_of[i], known, &state)) < 0) return ret;
        record[unit_of[i]] = (uint32_t)state;
        left |= state > DCPEXT_STATIC;
    }
    if (found) {
        fdt32_t v[DCPEXT_UNITS];
        int chosen = fdt_path_offset(t, "/chosen");
        for (unsigned i = 0; i < DCPEXT_UNITS; i++) v[i] = cpu_to_fdt32(record[i]);
        if (chosen >= 0 && (ret = fdt_setprop(t, chosen, "m4,dcpext", v, sizeof(v))) < 0) return ret;
    }
    return fdt_pack(t) ? -1 : left;
}

int m4_dcp_fixup(void *t, size_t capacity, const struct m4_display_handoff *h)
{
    static struct region regions[MAX_REGIONS];
    static char region_name[POOL][32];
    unsigned count = 0;
    int ret, node;
    if ((ret = fdt_open_into(t, t, (int)capacity)) < 0) return ret;
    int ok = h && m4_display_handoff_accepted(h) && adt_layout() && !collect(h, regions, &count) && count <= POOL;
    if (!ok) { delete_pool(t); return fdt_pack(t) ? -1 : 1; }
    uint32_t owner[3], region_ph[POOL];
    static const char *const paths[3] = {"/dcp@402e00000", "/display-subsystem", "/dcp@402e00000/piodma"};
    for (unsigned s = 0; s < 3; s++) {
        if ((node = fdt_path_offset(t, paths[s])) < 0) return node;
        if ((ret = phandle_of(t, node, &owner[s])) < 0) return ret;
        fdt_delprop(t, node, "memory-region"); fdt_delprop(t, node, "memory-region-names");
    }
    for (unsigned i = 0; i < count; i++) {
        char name[40], pool[24];
        uint32_t cells[15]; unsigned n = 0;
        int fb = h->framebuffer >= regions[i].pa && h->framebuffer - regions[i].pa < regions[i].size;
        snprintf(region_name[i], sizeof(region_name[i]), fb ? "framebuffer" : "dcp-map@%lx", (unsigned long)regions[i].pa);
        snprintf(name, sizeof(name), "%s@%lx", fb ? "framebuffer" : "dcp-map", (unsigned long)regions[i].pa);
        snprintf(pool, sizeof(pool), "dcp-map-pool%u", i);
        int resv = fdt_path_offset(t, "/reserved-memory");
        node = resv < 0 ? resv : fdt_subnode_offset(t, resv, pool);
        if (node < 0) return node;
        if ((ret = phandle_of(t, node, &region_ph[i])) < 0) return ret;
        if ((ret = fdt_setprop_string(t, node, "compatible", fb ? "framebuffer" : "apple,asc-mem")) < 0) return ret;
        const uint64_t reg[] = {regions[i].pa, regions[i].size};
        if ((ret = reg_prop(t, node, reg, 2, 0)) < 0 || (ret = fdt_setprop_empty(t, node, "no-map")) < 0) return ret;
        fdt_delprop(t, node, "status");
        for (unsigned s = 0; s < 3; s++)
            if (regions[i].users & (1u << s)) {
                cells[n++] = owner[s];
                cells[n++] = (uint32_t)(regions[i].iova[s] >> 32); cells[n++] = (uint32_t)regions[i].iova[s];
                cells[n++] = (uint32_t)(regions[i].size >> 32); cells[n++] = (uint32_t)regions[i].size;
            }
        if ((ret = u32s(t, node, "iommu-addresses", cells, n)) < 0) return ret;
        if ((ret = fdt_set_name(t, node, name)) < 0) return ret;
    }
    {
        int resv = fdt_path_offset(t, "/reserved-memory");
        for (unsigned i = count; i < POOL && resv >= 0; i++) {
            char pool[24];
            snprintf(pool, sizeof(pool), "dcp-map-pool%u", i);
            int n = fdt_subnode_offset(t, resv, pool);
            if (n >= 0 && (ret = fdt_del_node(t, n)) < 0) return ret;
        }
    }
    for (unsigned s = 0; s < 3; s++)
        for (unsigned i = 0; i < count; i++)
            if (regions[i].users & (1u << s)) {
                if ((node = fdt_path_offset(t, paths[s])) < 0) return node;
                if ((ret = fdt_appendprop_u32(t, node, "memory-region", region_ph[i])) < 0 ||
                    (ret = fdt_appendprop_string(t, node, "memory-region-names", region_name[i])) < 0) return ret;
            }
    static const char *const enable[] = {"/mailbox@402e08000", "/iommu@402340000", "/iommu@402300000",
                                         "/dcp@402e00000", "/display-subsystem"};
    for (unsigned i = 0; i < sizeof(enable) / sizeof(*enable); i++) {
        if ((node = fdt_path_offset(t, enable[i])) < 0) return node;
        if ((ret = fdt_setprop_string(t, node, "status", "okay")) < 0) return ret;
    }
    return fdt_pack(t);
}
