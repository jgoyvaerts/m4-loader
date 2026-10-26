/* rev 12 gate: the board.dtb half of unified-harness.c only (no rev 10 reference), built once per loader tree.
 * Host DT harness: runs the real rev 10 DT emitters (fdt.c, direct-fdt.c, peripheral*, bluetooth*, dcp-fdt.c,
 * media-probe.c, cpu-idle-regs.c, direct-smp.c planner) on this Mac's real ADT with synthetic runtime inputs, and
 * writes the final DT (what the loader copies to arena+0x10000 minus pure diagnostics records).
 *   dt-harness ADT.bin TEMPLATE.dtb OUT.dtb
 * Linked at 0x10005280000 so every address the emitters compare lies where the loader's does (loader, arena, ADT). */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/mman.h>
#include "adt.h"
#include "libfdt.h"
#include "xnuboot.h"
#include "j616s_usb.h"
#include "types.h"
#include "fdt.h"
#include "direct-smp.h"
#include "direct-capture.h"
#include "direct-bluetooth.h"
#include "dart-capture.h"
#include "display-handoff.h"
#include "boot-profile.h"
#include "cpu-idle-regs.h"
#include "media-probe.h"
#include "loader-cache.h"
#include "loader-speed.h"
#include "direct-peripheral-adt.h"
#include "fixups.h"
#include "bt-boot-admission.h"

extern void adt_double_init(void *, uint32_t);
extern uint8_t m4_template[], m4_template_end[];
extern int m4_metadata(struct m4_memory *, struct m4_handoff *, u64, u64);
extern int m4_dcp_fdt_write(void *, size_t, void *, size_t, const struct m4_display_handoff *);
extern const char _end[], __executable_start[], etext[];

/* ---- state the loader keeps in BSS ---- */
struct boot_args cur_boot_args;
struct m4_direct_capture_page m4_direct_capture __attribute__((aligned(M4_SMP_PAGE)));
__asm__(".global handoff\n.set handoff,m4_direct_capture+256\n");
#define handoff m4_direct_capture.handoff
struct m4_cache_state m4_cache;
struct m4_loader_speed m4_speed;
uint64_t m4_boot_profile_ticks[M4_PROFILE_COUNT];
int64_t m4_boot_profile_display_rc, m4_boot_profile_media_rc;
const struct midr_part_features *cpu_features;
static uint8_t pool[M4_ARENA_BYTES + 0x400000] __attribute__((aligned(16384)));
static uint8_t tree[M4_FDT_CAPACITY] __attribute__((aligned(16384)));
static uint8_t peripheral_tree[M4_FDT_CAPACITY] __attribute__((aligned(16384)));
static struct m4_memory memory;
/* code symbols the planner takes the address of */
__asm__(".bss\n.balign 16384\n"
        ".globl m4_direct_smp_failed_loop\nm4_direct_smp_failed_loop:\n.skip 64\n.balign 16384\n.globl _vectors_start\n_vectors_start:\n.skip 2048\n.balign 16384\n"
        ".globl m4_direct_smp_reset_start\nm4_direct_smp_reset_start:\n.skip 2048\n"
        ".globl m4_direct_smp_reset_end\nm4_direct_smp_reset_end:\n.balign 16384\n"
        ".globl m4_direct_smp_release_start\nm4_direct_smp_release_start:\n.skip 1024\n"
        ".globl m4_direct_smp_release_end\nm4_direct_smp_release_end:\n.skip 16384\n.text\n");
void m4_direct_smp_failed_park(void) { for (;;); }
void m4_direct_smp_linux_wait(struct m4_direct_smp_state *s) { (void)s; for (;;); }
void init_cpu(void) {}
void udelay(u32 d) { (void)d; }
int j616s_screen_printf(const char *f, ...) { (void)f; return 0; }
void j616s_screen_flush(void) {}
int j616s_exec_call(int (*fn)(void), struct j616s_call_result *r)
{   /* the cpu-idle-regs probe executes mrs of EL2 registers: report each as a faulted read, deterministically */
    memset(r, 0, sizeof(*r)); r->status = 1; r->esr = 0x2000000; return 0; (void)fn;
}
uint32_t m4_media_test_read32(uint64_t a)
{
    if (a == 0x502800170ull) return 0xff;
    if (a == 0x502800168ull) return 0xff;
    return (uint32_t)(a * 2654435761u) ^ 0x5a5a0000u;
}
uint64_t m4_profile_test_counter(void) { return 0; }
int j616s_usb_call(int (*fn)(void), struct j616s_call_result *r) { memset(r, 0, sizeof(*r)); r->value = (uint64_t)fn(); return 0; }
int m4_display_handoff_accepted(const struct m4_display_handoff *h) { return !!h && !getenv("M4_DCP_REFUSE"); }
__asm__(".section .rodata\n.balign 16\n.globl m4_template\nm4_template:\n.incbin \"" TEMPLATE_PATH "\"\n.globl m4_template_end\nm4_template_end:\n.text\n");

static void *slurp(const char *p, size_t *n)
{
    FILE *f = fopen(p, "rb"); if (!f) { perror(p); exit(2); }
    fseek(f, 0, SEEK_END); *n = ftell(f); rewind(f);
    void *b = malloc(*n + 16); if (fread(b, 1, *n, f) != *n) exit(2); fclose(f); return b;
}
static uint64_t prop64(const char *path, const char *name, int idx)
{
    int n = adt_path_offset(adt, path); uint32_t l;
    const uint64_t *p = n < 0 ? NULL : adt_getprop(adt, n, name, &l);
    if (!p) { fprintf(stderr, "ADT %s %s missing\n", path, name); exit(3); }
    return p[idx];
}
/* the BT config read stub: bus numbers 0 and command 0 on the root port, as native R9 sees it before Linux enumerates (deferred) */
static uint64_t bt_last;
static int bt_read32(void *o, uint64_t a, uint32_t *v)
{
    (void)o; bt_last = a;
    if (getenv("M4_BT_REFUSE") && a == 0x1cb0000000ull) { *v = 0x12345678; return 0; }
    if (a == 0x1cb0000000ull) *v = 0x100c106b;
    else if (a == 0x1cb000000cull) *v = 0x00010000;
    else *v = 0;
    return 0;
}
static int adt_get(void *ctx, const char *path, const char *prop, const void **p, uint32_t *n)
{   /* codec_getprop shape is checked by compile; implemented against the real ADT */
    (void)ctx; int node = adt_path_offset(adt, path); if (node < 0) return -1;
    u32 l; const void *v = adt_getprop(adt, node, prop, &l); if (!v) return -1;
    *p = v; *n = l; return 0;
}

int main(int argc, char **argv)
{
    if (argc < 4) return 2;
    static char ref_args[1100] = "root=UUID=0 rw";
    size_t adt_n, tpl_n; (void)tpl_n;
    void *file = slurp(argv[1], &adt_n);
    /* /chosen/memory-map: the live ADT carries it, the ioreg dump does not. DeviceTree + BootArgs only. */
    const uint64_t adt_addr = 0x1000ae00000ull;
    void *m = mmap((void *)adt_addr, (adt_n + 0xffff) & ~0xffffull, PROT_READ | PROT_WRITE,
                   MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
    if (m != (void *)adt_addr) { perror("mmap adt"); return 3; }
    memcpy(m, file, adt_n);
    adt_double_init(m, (uint32_t)adt_n);
    cur_boot_args.revision = 3; cur_boot_args.version = 2;
    cur_boot_args.devtree_size = (u32)adt_n;
    struct m4_dt_range_ { uint64_t b, s; };
    uint64_t usable_b = prop64("/chosen/carveout-memory-map", "region-id-24", 0),
             usable_s = prop64("/chosen/carveout-memory-map", "region-id-24", 1);
    uint64_t loader_base = (uint64_t)__executable_start & ~16383ull, loader_end = ((uint64_t)_end + 16383) & ~16383ull;
    cur_boot_args.phys_base = usable_b; cur_boot_args.mem_size = usable_s;
    cur_boot_args.top_of_kernel_data = loader_end + 0x400000; /* everything below the first free page */
    uint64_t vb = prop64("/vram", "reg", 0);
    cur_boot_args.video.base = vb; cur_boot_args.video.display = 1;
    cur_boot_args.video.width = 3456; cur_boot_args.video.height = 2234;
    cur_boot_args.video.stride = 3456 * 4; cur_boot_args.video.depth = 30;
    /* memory-map: DeviceTree, BootArgs, ramdisk-free */
    int chosen = adt_path_offset(adt, "/chosen");
    (void)chosen;
    uint64_t lower = loader_base, upper = (uint64_t)etext;
    memset(&memory, 0, sizeof(memory));
    int rc = m4_metadata(&memory, &handoff, lower, upper);
    fprintf(stderr, "metadata rc=%d free=%u framebuffer=0x%lx\n", rc, memory.free_count, (unsigned long)handoff.framebuffer);
    if (rc) return 10;
    m4_media_probe_gates();
    uint64_t a = ((uint64_t)pool + 16383) & ~16383ull;
    uint64_t arena = (((uint64_t)pool + 0x50000 + 0x1fffff) & ~0x1fffffull) - 0x50000;
    handoff.arena_base = arena; handoff.arena_size = M4_ARENA_BYTES;
    struct m4_range span = {handoff.arena_base, handoff.arena_size};
    struct m4_range loader = {loader_base, loader_end - loader_base};
    struct m4_range protected = {lower & ~16383ull, ((upper + 4096 + 16383) & ~16383ull) - (lower & ~16383ull)};
    (void)a;
    handoff.kernel = (arena + 0x20000 + 0x1fffff) & ~0x1fffffull;
    handoff.kernel_size = 0x1c90000;           /* kernel window of the embedded/SSD Image: synthetic */
    handoff.initrd = arena + M4_ARENA_INITRD_OFFSET;
    handoff.initrd_size = 0x600000;
    handoff.affinity = 0x10100; handoff.epoch = 1760000000;
    static const struct m4_direct_peer_plan plans[13] = {
        {5, 0x101, 0x10101}, {6, 0x102, 0x10102}, {7, 0x103, 0x10103}, {8, 0x104, 0x10104},
        {10, 0x200, 0x10200}, {11, 0x201, 0x10201}, {12, 0x202, 0x10202}, {13, 0x203, 0x10203}, {14, 0x204, 0x10204},
        {0, 0, 0}, {1, 1, 1}, {2, 2, 2}, {3, 3, 3}};
    rc = m4_direct_smp_prepare_owned(&memory, loader, span, protected, plans, 13);
    fprintf(stderr, "smp prepare_owned rc=%d\n", rc);
    if (rc) return 11;
    unsigned count; uint64_t release;
    const struct m4_range *regions = m4_direct_smp_regions(&count, &release);
    unsigned peer_count; const struct m4_direct_peer_record *peers = m4_direct_smp_peer_records(&peer_count);
    fprintf(stderr, "regions=%p count=%u peers=%p peer_count=%u\n", (void *)regions, count, (void *)peers, peer_count);
    if (!regions || !peers) return 12;
    m4_direct_capture.descriptor = (struct m4_direct_capture_descriptor){
        .magic = M4_DIRECT_CAPTURE_MAGIC, .version = M4_DIRECT_CAPTURE_VERSION, .bytes = sizeof(m4_direct_capture.descriptor),
        .loader_base = loader.base, .loader_end = loader.base + loader.size,
        .arena_base = span.base, .arena_size = span.size,
        .kernel = handoff.kernel, .kernel_memory = handoff.kernel_size,
        .initrd = handoff.initrd, .initrd_bytes = handoff.initrd_size,
        .linux_dt = arena + 0x10000, .handoff_addr = (uint64_t)&handoff,
        .adt = (uint64_t)adt, .adt_bytes = cur_boot_args.devtree_size,
        .bootargs_addr = (uint64_t)&cur_boot_args, .release_addr = release,
        .release_page_base = regions[2].base, .release_page_bytes = regions[2].size,
        .primary_mpidr = 0x10100, .secondary_mpidr = 0x10101,
        .framebuffer = handoff.framebuffer, .framebuffer_bytes = handoff.framebuffer_size,
        .peer_count = peer_count, .peer_table = (uint64_t)peers, .peer_entry_bytes = sizeof(*peers)};
    memset(tree, 0, sizeof(tree)); memset(peripheral_tree, 0, sizeof(peripheral_tree));
    {
    size_t board_n; void *board = slurp(argv[2], &board_n);
    if (fdt_open_into(board, tree, sizeof(tree))) { fprintf(stderr, "board.dtb does not open\n"); return 13; }
    const char *cmdline = getenv("M4_CMDLINE");
    rc = m4_fixup_base(tree, sizeof(tree), &memory, &handoff, cmdline ? cmdline : ref_args);
    fprintf(stderr, "[rev11] fixup_base rc=%d\n", rc); if (rc) return 13;
    rc = m4_fixup_direct(tree, sizeof(tree));
    fprintf(stderr, "fixup_direct rc=%d\n", rc); if (rc) return 14;
    rc = m4_prepare_direct_peripherals(adt, cur_boot_args.devtree_size, tree, sizeof(tree));
    fprintf(stderr, "peripherals rc=%d\n", rc); if (rc) return 15;
    {
        struct m4_bt_boot_report report;
        int ar = m4_bt_boot_admit_current(tree, fdt_totalsize(tree), bt_read32, NULL, &report);
        int st = ar == M4_BT_BOOT_DEFERRED ? M4_BT_STATE_DEFERRED : !ar ? M4_BT_STATE_ADMITTED : M4_BT_STATE_REFUSED;
        rc = m4_fixup_wireless(tree, sizeof(tree), st);
        fprintf(stderr, "wireless admit=%d state=%d rc=%d\n", ar, st, rc);
    }
    /* the record was read once above (once per boot); the same state is published again */
    fdt_open_into(tree, tree, sizeof(tree));
    (void)m4_cpu_idle_regs_publish(tree, sizeof(tree));
    static struct m4_display_handoff h;
    h.framebuffer = handoff.framebuffer; h.framebuffer_bytes = handoff.framebuffer_size;
    struct { uint64_t pa, size; unsigned users; } rg[] = {
        {0x100002f4000ull, 0x6d0000, 1}, {0x10bd10e4000ull, 0x2c4000, 1}, {0x10bd146c000ull, 0x4000, 1},
        {handoff.framebuffer, 0, 6}, {0x10bd8738000ull, 0xc0000, 1}, {0x10bd87f8000ull, 0x360000, 3},
        {0x10bd8b58000ull, 0x1000000, 2}};
    unsigned n[3] = {0, 0, 0};
    uint64_t next[3] = {0x100000000ull, 0x200000000ull, 0x300000000ull};
    for (unsigned i = 0; i < sizeof(rg) / sizeof(*rg); i++) {
        uint64_t size = rg[i].size ? rg[i].size : handoff.framebuffer_size;
        for (unsigned s = 0; s < 3; s++) if (rg[i].users & (1u << s)) {
            h.result[s].map[n[s]++] = (struct m4_dt_span){next[s], rg[i].pa, size, 0x000fff0000000001ull};
            next[s] += (size + 0xffffff) & ~0xffffffull;
        }
    }
    for (unsigned s = 0; s < 3; s++) h.result[s].spans = n[s];
    rc = m4_dcp_fixup(tree, sizeof(tree), &h);
    fprintf(stderr, "dcp_fixup rc=%d\n", rc);
    rc = m4_media_fixups(tree, sizeof(tree));
    fprintf(stderr, "media_fixups rc=%d\n", rc);
    rc = m4_gpu_fixup(tree, sizeof(tree));
    fprintf(stderr, "gpu_fixup rc=%d\n", rc);
    rc = m4_dcpext_fixup(tree, sizeof(tree));
    fprintf(stderr, "dcpext_fixup rc=%d\n", rc);
    rc = fdt_pack(tree); (void)rc;
    FILE *o2 = fopen(argv[3], "wb"); fwrite(tree, 1, fdt_totalsize(tree), o2); fclose(o2);
    fprintf(stderr, "final dtb bytes=%u\n", fdt_totalsize(tree));
    }
    return 0;
}
