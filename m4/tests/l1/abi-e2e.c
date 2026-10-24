/* SPDX-License-Identifier: MIT */
/* Boot ABI v1 end-to-end test of the Level 1 blob: the real blob (level1-blob.c with its HAL replaced by the model),
 * the real ADT of this machine, the real libfdt on the embedded rescue board.dtb (the one built from dts/), and one
 * real mke2fs image per scenario behind the scripted ANS/NVMe model. Checks the decision (entry or rescue and why),
 * the bytes in the final slots, the published /chosen m4,loader-* / m4,boot-* properties and the hand-off state.
 *   abi-e2e <adt.bin> <board.dtb> <fixture-dir> */
#define _GNU_SOURCE
#include "l1-model.h"
#include "level1-host.h"
#include <fcntl.h>
#include <unistd.h>
#include <setjmp.h>
#include <sys/mman.h>
#include "libfdt.h"

extern void l1_attempt(const struct l1_host *, struct l1_attempt_args *);
extern int l1_adt_check(const struct l1_host *, const struct l1_ans_cfg *);
extern int l1_publish(const struct l1_host *, void *, void *, size_t);

uint64_t l1_test_cnt(void) { return t_us * 1000; }
const struct l1_hal l1_test_hal = {f_r32, f_w32, f_r64, f_w64, f_udelay, f_now, f_clean, f_inval, f_syslog, f_trace, 0};

/* ---- ADT reader (the loader's Rust one, minimal port incl. reg translation) ---- */
static uint8_t *ADT;
static size_t ADTN;
static uint32_t rd32(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }

struct prop { const uint8_t *val; uint32_t size; };
static size_t node_end(size_t off)
{
    uint32_t np = rd32(ADT + off), nc = rd32(ADT + off + 4);
    size_t p = off + 8;
    for (uint32_t i = 0; i < np; i++) { uint32_t sz = rd32(ADT + p + 32) & 0x7fffffff; p += 36 + ((sz + 3) & ~3u); }
    for (uint32_t i = 0; i < nc; i++) p = node_end(p);
    return p;
}
static int node_prop(size_t off, const char *name, struct prop *out)
{
    uint32_t np = rd32(ADT + off);
    size_t p = off + 8;
    for (uint32_t i = 0; i < np; i++) {
        uint32_t sz = rd32(ADT + p + 32) & 0x7fffffff;
        if (!strncmp((const char *)ADT + p, name, 32)) { out->val = ADT + p + 36; out->size = sz; return 0; }
        p += 36 + ((sz + 3) & ~3u);
    }
    return -1;
}
static int name_eq(size_t node, const char *comp, size_t n)
{
    struct prop pr;
    if (node_prop(node, "name", &pr)) return 0;
    size_t len = strnlen((const char *)pr.val, pr.size);
    if (len == n && !memcmp(pr.val, comp, n)) return 1;
    return 0;
}
static int h_path_trace(const void *adt, const char *path, int *offsets)
{
    (void)adt;
    size_t cur = 0;
    int depth = 0;
    const char *p = path;
    while (*p == '/') p++;
    while (*p) {
        const char *e = p;
        while (*e && *e != '/') e++;
        uint32_t np = rd32(ADT + cur), nc = rd32(ADT + cur + 4);
        size_t q = cur + 8;
        for (uint32_t i = 0; i < np; i++) { uint32_t sz = rd32(ADT + q + 32) & 0x7fffffff; q += 36 + ((sz + 3) & ~3u); }
        int found = 0;
        for (uint32_t i = 0; i < nc; i++) {
            if (name_eq(q, p, (size_t)(e - p))) { cur = q; found = 1; break; }
            q = node_end(q);
        }
        if (!found) return -1;
        if (depth < 7) offsets[depth] = (int)cur;
        depth++;
        p = e;
        while (*p == '/') p++;
    }
    if (depth < 8) offsets[depth] = 0;
    return (int)cur;
}
static const void *h_getprop(const void *adt, int node, const char *name, uint32_t *len)
{
    (void)adt;
    struct prop pr;
    if (node_prop((size_t)node, name, &pr)) return NULL;
    if (len) *len = pr.size;
    return pr.val;
}
static uint64_t cells(const uint8_t *p, unsigned n)
{
    uint64_t v = 0;
    for (unsigned i = 0; i < n; i++) v |= (uint64_t)rd32(p + 4 * i) << (32 * i);
    return v;
}
static uint64_t cells_be_chunks(const uint8_t *p, unsigned n) /* get_cells_u8: reverse chunks of 4 */
{
    uint64_t v = 0;
    for (int i = (int)n - 1; i >= 0; i--) v = (v << 32) | rd32(p + 4 * i);
    return v;
}
static int h_get_reg(const void *adt, int *path, const char *prop, int idx, uint64_t *addr, uint64_t *size)
{
    (void)adt;
    int nodes[9], n = 0;
    while (n < 8 && path[n]) { nodes[n] = path[n]; n++; }
    if (!n) return -1;
    int cursor = n - 1;
    size_t node = (size_t)nodes[cursor];
    size_t parent = cursor > 0 ? (size_t)nodes[cursor - 1] : 0;
    struct prop pr, ac, sc;
    if (node_prop(parent, "#address-cells", &ac) || node_prop(parent, "#size-cells", &sc)) return -2;
    unsigned addr_cells = rd32(ac.val), size_cells = rd32(sc.val);
    if (addr_cells < 1 || addr_cells > 2 || size_cells > 2) return -3;
    if (node_prop(node, prop, &pr) || !pr.size) return -4;
    if (pr.size < (uint32_t)(idx + 1) * (addr_cells + size_cells) * 4) return -5;
    const uint8_t *a = pr.val + (size_t)idx * (addr_cells + size_cells) * 4;
    uint64_t ad = cells(a, addr_cells), sz = cells(a + 4 * addr_cells, size_cells);
    while (parent != 0) {
        cursor--;
        node = parent;
        parent = cursor > 0 ? (size_t)nodes[cursor - 1] : 0;
        struct prop rg;
        if (node_prop(node, "ranges", &rg)) break;
        struct prop pc;
        if (node_prop(parent, "#address-cells", &pc)) return -6;
        unsigned paddr = rd32(pc.val);
        unsigned entry = 4 * (paddr + addr_cells + size_cells);
        for (uint32_t o = 0; o + entry <= rg.size; o += entry) {
            const uint8_t *r = rg.val + o;
            uint64_t ca = cells_be_chunks(r, addr_cells), pa = cells_be_chunks(r + 4 * addr_cells, paddr),
                     cs = cells_be_chunks(r + 4 * (addr_cells + paddr), size_cells);
            if (ad >= ca && ad + sz <= ca + cs) { ad = ad - ca + pa; break; }
        }
        if (node_prop(parent, "#size-cells", &sc)) return -7;
        size_cells = rd32(sc.val);
        addr_cells = paddr;
    }
    *addr = ad;
    if (size) *size = sz;
    return 0;
}

/* ---- host services ------------------------------------------------------ */
static size_t bump;
static void *h_memalign(size_t al, size_t sz)
{
    bump = (bump + al - 1) & ~(al - 1);
    if (bump + sz > UNI) return NULL;
    void *p = U + bump;
    bump += sz;
    return p;
}
static unsigned lines;
static char last_line[128], all_lines[40][128];
static void h_line(const char *s)
{
    snprintf(last_line, sizeof(last_line), "%s", s);
    if (lines < 40) snprintf(all_lines[lines], 128, "%s", s);
    lines++;
}
static uint64_t h_counter(void) { return t_us * 1000; }
static int abort_phase = -1, phase_n, guard_calls, status2_from = -1, status2_count;
static jmp_buf jbs[16];
static int jdepth;
static char phase_log[64];
static void access_abort(void)       /* the register model calls this at the chosen access: like a fault there */
{
    if (jdepth) longjmp(jbs[jdepth - 1], 1);
}
/* the real guard: runs fn; a fault (here: abort_phase at its start, or fault_at_access inside) returns status 1 */
static int h_guarded(int (*fn)(void *), void *arg, struct l1_exec_result *res)
{
    memset(res, 0, sizeof(*res));
    res->vector = 0xff;
    int this = phase_n++;
    guard_calls++;
    if (status2_from >= 0 && this >= status2_from && this < status2_from + status2_count) {
        res->status = 2; res->esr = 0xbf000000; res->elr = 0x4321; res->vector = 7;    /* pending SError taken at entry: fn not run */
        return 0;
    }
    if (this == abort_phase) {
        res->status = 1; res->esr = 0x96000010; res->elr = 0x1234; res->vector = 4;   /* the guard caught a data abort */
        return 0;                                                                    /* fn never ran */
    }
    if (jdepth >= 15) return -1;
    if (setjmp(jbs[jdepth++]) == 0) {
        fn(arg);
        jdepth--;
        return 0;
    }
    jdepth--;
    res->status = 1; res->esr = 0xbf000000; res->elr = 0x5678; res->vector = 7;       /* SError taken inside */
    return 0;
}

/* ---- disk: GPT + real ext4 image ---------------------------------------- */
static int fsfd = -1;
static uint8_t gptbuf[8 * 4096];
static void put32(uint8_t *p, uint32_t v) { for (int i = 0; i < 4; i++) p[i] = (uint8_t)(v >> (8 * i)); }
static void put64(uint8_t *p, uint64_t v) { for (int i = 0; i < 8; i++) p[i] = (uint8_t)(v >> (8 * i)); }
static uint32_t crc32b(const uint8_t *p, size_t n)
{
    uint32_t c = 0xffffffffu;
    while (n--) { c ^= *p++; for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xedb88320u & (0u - (c & 1))); }
    return ~c;
}
static void build_gpt(void)
{
    memset(gptbuf, 0, sizeof(gptbuf));
    uint8_t *e = gptbuf + 2 * 4096;
    static const uint8_t p4guid[16] = L1_P4_GUID_BYTES;   /* private per-install input (level1-install.h) */
    static const uint8_t basic[16] = {0xa2, 0xa0, 0xd0, 0xeb, 0xe5, 0xb9, 0x33, 0x44, 0x87, 0xc0, 0x68, 0xb6, 0xb7, 0x26, 0x99, 0xc7};
    /* generic layout around the pinned partition (the other entries only make the GPT look like a real one) */
    struct { uint64_t f, l; int p4; } part[5] = {{6, 128005, 0}, {128006, L1_P4_FIRST / 2, 0}, {L1_P4_FIRST / 2 + 1, L1_P4_FIRST - 72, 0},
                                                 {L1_P4_FIRST, L1_P4_END - 1, 1}, {L1_P4_END + 47, L1_P4_END + 100000, 0}};
    for (int i = 0; i < 5; i++) {
        uint8_t *x = e + i * 128;
        memcpy(x, basic, 16);
        if (part[i].p4) memcpy(x + 16, p4guid, 16); else x[16] = (uint8_t)(0x40 + i);
        put64(x + 32, part[i].f); put64(x + 40, part[i].l);
    }
#if L1_EXTRA_N
    /* the install's extra Linux partitions, with their own PARTUUIDs and ranges (published only when found like this) */
    static const uint64_t xr[L1_EXTRA_N][2] = L1_EXTRA_RANGES;
    static const uint8_t xg[L1_EXTRA_N][16] = L1_EXTRA_GUIDS;
    for (int i = 0; i < L1_EXTRA_N; i++) {
        uint8_t *x = e + (5 + i) * 128;
        memcpy(x, basic, 16); memcpy(x + 16, xg[i], 16);
        put64(x + 32, xr[i][0]); put64(x + 40, xr[i][1] - 1);
    }
#endif
    uint8_t *h = gptbuf + 4096;
    memcpy(h, "EFI PART", 8);
    put32(h + 8, 0x00010000); put32(h + 12, 92); put64(h + 24, 1); put64(h + 32, 244276264);
    put64(h + 40, 6); put64(h + 48, 244276259); put64(h + 72, 2); put32(h + 80, 128); put32(h + 84, 128);
    put32(h + 88, crc32b(e, 128 * 128));
    put32(h + 16, crc32b(h, 92));
}
static const uint8_t *file_block(uint64_t lba)
{
    static uint8_t b[4096];
    if (lba < 8) return gptbuf + lba * 4096;
    memset(b, 0, sizeof(b));
    ssize_t n = pread(fsfd, b, 4096, (off_t)((lba - L1_P4_FIRST) * 4096));
    (void)n;
    return b;
}

/* ---- scenario plumbing --------------------------------------------------- */

/* ---- scenario plumbing --------------------------------------------------- */
static int failures, total;
static uint8_t dtbuf[262144] __attribute__((aligned(16)));
#define DTBSTAGE (U + 0x2e00000)   /* inside the DMA universe, between the kernel and initrd slots */
#define dtbstage DTBSTAGE
#define DTBWIN 65536u
static uint8_t *BOARD;
static size_t BOARDN;
#define KWINDOW 29949952u       /* loader.c: initrd - kernel - 0x10000 with initrd at arena+0x1E00000, kernel at arena+0x50000 */
struct run {
    struct l1_attempt_args a;
    int publish_rc;
    char source[24], entry[80], reason[64], detail[96], version[32];
    uint32_t abi;
    int has_abi, has_live, live_n, has_syslog, wr_n;
    uint8_t live[32], wr[64];
};
static int get_str(const void *dt, int c, const char *k, char *out, size_t cap)
{
    int n; const char *v = fdt_getprop(dt, c, k, &n);
    if (!v) { out[0] = 0; return -1; }
    snprintf(out, cap, "%s", v);
    return 0;
}
static int refused_mode, refused_reason;   /* the shim's whole-attempt guard refused: only publish runs, h->refused set */
static void do_run(struct run *R, int capable, int cache_live, int abort_at)
{
    memset(R, 0, sizeof(*R));
    model_reset();
    bump = 0x100000;
    lines = 0; last_line[0] = 0; phase_n = 0; guard_calls = 0; jdepth = 0; abort_phase = abort_at;
    refused_mode = refused_reason;
    abort_hook = access_abort;
    uint8_t *kslot = U + 0x1000000, *islot = U + 0x3000000;
    memset(kslot, 0x5a, KWINDOW); memset(islot, 0x5a, 16u << 20);
    memset(dtbstage, 0xaa, DTBWIN);
    struct l1_host h = {0};
    h.abi = L1_HOST_ABI; h.size = sizeof(h);
    h.memalign = h_memalign; h.line = h_line; h.counter = h_counter; h.guarded = h_guarded;
    h.adt = ADT;
    h.adt_path_offset_trace = h_path_trace; h.adt_getprop = h_getprop;
    h.adt_get_reg = h_get_reg;
    h.vsnprintf = vsnprintf; h.adt_check = l1_adt_check;
    h.fdt_path_offset = fdt_path_offset;
    h.fdt_node_offset_by_phandle = fdt_node_offset_by_phandle; h.fdt_parent_offset = fdt_parent_offset;
    h.fdt_getprop = (const void *(*)(const void *, int, const char *, int *))fdt_getprop;
    h.fdt_setprop_inplace = fdt_setprop_inplace; h.fdt_setprop = fdt_setprop;
    h.fdt_open_into = fdt_open_into; h.fdt_pack = fdt_pack; h.fdt_check_full = fdt_check_full;
    h.kernel = 0xdeadbeef000ull; h.kernel_bytes = 14102536; h.initrd = 0xdeadc000000ull; h.initrd_bytes = 14680064;
    h.cache_window_live = (uint32_t)cache_live; h.cntfrq_ok = 1;
    h.loader_version = "rev11-test"; h.level1_capable = (uint32_t)capable;
    R->a.kernel = (uint64_t)(uintptr_t)kslot; R->a.kernel_window = KWINDOW;
    R->a.initrd = (uint64_t)(uintptr_t)islot; R->a.initrd_window = 16u << 20;
    R->a.tree = BOARD; R->a.tree_capacity = BOARDN;
    R->a.dtb = dtbstage; R->a.dtb_window = DTBWIN;
    h.refused = (uint32_t)refused_mode;
    if (!refused_mode) l1_attempt(&h, &R->a);
    /* publish into the final DT like the late stage: the embedded rescue tree stands in for the fixed-up one */
    if (fdt_open_into(BOARD, dtbuf, sizeof(dtbuf))) { printf("board.dtb open failed\n"); exit(2); }
    R->publish_rc = l1_publish(&h, R->a.state, dtbuf, sizeof(dtbuf));
    int c = fdt_path_offset(dtbuf, "/chosen"), n;
    get_str(dtbuf, c, "m4,boot-source", R->source, sizeof(R->source));
    get_str(dtbuf, c, "m4,boot-entry", R->entry, sizeof(R->entry));
    get_str(dtbuf, c, "m4,boot-reason", R->reason, sizeof(R->reason));
    get_str(dtbuf, c, "m4,boot-detail", R->detail, sizeof(R->detail));
    get_str(dtbuf, c, "m4,loader-version", R->version, sizeof(R->version));
    const uint8_t *p = fdt_getprop(dtbuf, c, "m4,loader-abi", &n);
    if (p && n == 4) { uint32_t v; memcpy(&v, p, 4); R->abi = __builtin_bswap32(v); R->has_abi = 1; }
    p = fdt_getprop(dtbuf, c, "m4,ans-live", &n);
    if (p && n > 0 && n <= (int)sizeof(R->live)) { R->has_live = 1; R->live_n = n; memcpy(R->live, p, (size_t)n); }
    p = fdt_getprop(dtbuf, c, "m4,ans-syslog", &n);
    if (p && n == 20) R->has_syslog = 1;
    p = fdt_getprop(dtbuf, c, "m4,write-lba-range", &n);
    if (p && n > 0 && n <= (int)sizeof(R->wr)) { R->wr_n = n; memcpy(R->wr, p, (size_t)n); }
}
static uint64_t wr_at(const struct run *R, int i) { uint64_t v; memcpy(&v, R->wr + 8 * i, 8); return __builtin_bswap64(v); }
static void check(const char *name, int ok, const char *extra)
{
    total++;
    if (!ok) failures++;
    printf("%s %s%s%s\n", ok ? "PASS" : "FAIL", name, extra && extra[0] ? "  " : "", extra ? extra : "");
}
static int handoff_clean(char *why, size_t cap) { return ans_state_ok(why, cap); }   /* l1-model.h */
static int ans_untouched(void)
{
    return wake_start && M.inits == 0 && M.pm_writes == 0 && M.cpu_ctrl == 0x10 && M.iop_state == 0x20;
}
static int bytes_equal_file(const uint8_t *mem, size_t n, const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) return 0;
    uint8_t *b = malloc(n + 1);
    size_t r = fread(b, 1, n + 1, f);
    fclose(f);
    int ok = r == n && !memcmp(b, mem, n);
    free(b);
    return ok;
}
static const char *CMD = "root=UUID=00000000-0000-4000-8000-00000000e2e0 rw quiet loglevel=3 m4.test=1";
static char FX[512];
static void open_fs(const char *name)
{
    char p[700];
    if (fsfd >= 0) close(fsfd);
    snprintf(p, sizeof(p), "%s/%s.ext4", FX, name);
    fsfd = open(p, O_RDONLY);
    if (fsfd < 0) { perror(p); exit(2); }
}

/* one scenario: which reason, which source; rescue boots must leave ANS handed back cleanly (or untouched) */
static void rescue_case(const char *name, const char *fs, const char *reason, const char *detail_prefix)
{
    struct run R; char extra[400], why[96];
    open_fs(fs);
    do_run(&R, 1, 1, -1);
    snprintf(extra, sizeof(extra), "source=%s entry=%s reason=%s detail='%s' ssd=%u", R.source, R.entry, R.reason, R.detail, R.a.ssd);
    int ok = !R.a.ssd && !strcmp(R.source, "rescue") && !strcmp(R.entry, "rescue") && !strcmp(R.reason, reason) &&
             (!detail_prefix || !strncmp(R.detail, detail_prefix, strlen(detail_prefix))) &&
             R.publish_rc == 0 && R.has_abi && R.abi == 1 && !strcmp(R.version, "rev11-test") && handoff_clean(why, sizeof(why)) &&
             /* every B case is decided after the GPT check, so the rescue set gets the verified root range */
             R.wr_n >= 16 && R.wr_n % 16 == 0 && wr_at(&R, 0) == L1_P4_FIRST && wr_at(&R, 1) == L1_P4_END;
    check(name, ok, extra);
}

int main(int argc, char **argv)
{
    if (argc < 4) { fprintf(stderr, "usage: abi-e2e adt.bin board.dtb fixture-dir\n"); return 2; }
    FILE *f = fopen(argv[1], "rb"); fseek(f, 0, SEEK_END); ADTN = (size_t)ftell(f); fseek(f, 0, SEEK_SET);
    ADT = malloc(ADTN); if (fread(ADT, 1, ADTN, f) != ADTN) return 2; fclose(f);
    f = fopen(argv[2], "rb"); fseek(f, 0, SEEK_END); BOARDN = (size_t)ftell(f); fseek(f, 0, SEEK_SET);
    BOARD = malloc(BOARDN); if (fread(BOARD, 1, BOARDN, f) != BOARDN) return 2; fclose(f);
    snprintf(FX, sizeof(FX), "%s", argv[3]);
    uni_size = 80u << 20;
    U = mmap((void *)0x40000000000ull, UNI, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
    D = mmap((void *)0x50000000000ull, UNI, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, -1, 0);
    if (U == MAP_FAILED || D == MAP_FAILED) { perror("mmap"); return 2; }
    build_gpt();
    disk_fn = file_block;
    allow_gpt_reads = 1;
    char path[600], extra[600], why[96];
    struct run R;
    fault_at_access = -1;

    /* A: the default entry boots: bytes in the final slots, strings out, ANS left live and handed back */
    open_fs("ok");
    do_run(&R, 1, 1, -1);
    snprintf(extra, sizeof(extra), "source=%s entry=%s reason=%s detail='%s' ssd=%u image=%llu memory=%llu initrd=%llu dtb=%llu kver=%s",
             R.source, R.entry, R.reason, R.detail, R.a.ssd, (unsigned long long)R.a.image_bytes,
             (unsigned long long)R.a.image_memory, (unsigned long long)R.a.initrd_bytes, (unsigned long long)R.a.dtb_bytes, R.a.kver);
    check("A1 default entry boots: source=entry, entry=e1, reason=default, loader-abi=1, version",
          R.a.ssd && !strcmp(R.source, "entry") && !strcmp(R.entry, "e1") && !strcmp(R.reason, "default") && !R.detail[0] &&
          R.has_abi && R.abi == 1 && !strcmp(R.version, "rev11-test") && R.publish_rc == 0, extra);
    snprintf(path, sizeof(path), "%s/fx/Image", FX);
    int img_ok = bytes_equal_file((uint8_t *)(uintptr_t)R.a.kernel, (size_t)R.a.image_bytes, path);
    snprintf(path, sizeof(path), "%s/fx/initramfs.img", FX);
    int ird_ok = bytes_equal_file((uint8_t *)(uintptr_t)R.a.initrd, (size_t)R.a.initrd_bytes, path);
    snprintf(path, sizeof(path), "%s/fx/board.dtb", FX);
    int dtb_ok = bytes_equal_file(dtbstage, (size_t)R.a.dtb_bytes, path);
    check("A2 Image, initramfs.img and board.dtb in their slots equal the files; image memory from the header",
          img_ok && ird_ok && dtb_ok && R.a.image_memory == 2u * 1024 * 1024 && R.a.initrd_bytes == 512 * 1024 + 7 &&
          R.a.image_bytes == 1024 * 1024 + 123, "");
    check("A3 cmdline and kver from entry.conf", !strcmp(R.a.cmdline, CMD) && !strcmp(R.a.kver, "6.17.0-m4"), R.a.cmdline);
    check("A4 hand-off clean, /chosen m4,ans-live published, no m4,level1-*/m4,carrier-*/m4,payload-* left",
          handoff_clean(why, sizeof(why)) && R.has_live && R.live_n > 0, why);
    {
        int c = fdt_path_offset(dtbuf, "/chosen"), bad = 0;
        static const char *gone[] = {"m4,carrier-id", "m4,carrier-variant", "m4,level1-status", "m4,level1-source", "m4,level1-log",
                                     "m4,level1-ms", "m4,level1-entry", "m4,payload-image", "m4,payload-initrd", "m4,level1-capable"};
        for (unsigned i = 0; i < sizeof(gone) / sizeof(*gone); i++) if (fdt_getprop(dtbuf, c, gone[i], NULL)) bad = 1;
        check("A5 retired properties are absent", !bad, "");
    }
    {   /* the root first, then every extra partition the GPT holds with its exact range and PARTUUID */
        int ok = R.wr_n == 16 * (1 + L1_EXTRA_N) && wr_at(&R, 0) == L1_P4_FIRST && wr_at(&R, 1) == L1_P4_END;
#if L1_EXTRA_N
        static const uint64_t xr[L1_EXTRA_N][2] = L1_EXTRA_RANGES;
        for (int i = 0; i < L1_EXTRA_N; i++) ok &= wr_at(&R, 2 + 2 * i) == xr[i][0] && wr_at(&R, 3 + 2 * i) == xr[i][1];
#endif
        snprintf(extra, sizeof(extra), "%d bytes, %d extra", R.wr_n, L1_EXTRA_N);
        check("A7 /chosen m4,write-lba-range: the GPT-checked root and extra partitions", ok, extra);
    }
    {   /* sizes that are not a multiple of 4 KiB / 16 KiB: nothing is written past the true length (the kernel gets the true initrd-end) */
        const uint8_t *i_end = (const uint8_t *)(uintptr_t)R.a.initrd + R.a.initrd_bytes;
        const uint8_t *k_end = (const uint8_t *)(uintptr_t)R.a.kernel + R.a.image_bytes;
        int clean = 1;
        for (int i = 0; i < 4096; i++) if (i_end[i] != 0x5a || k_end[i] != 0x5a) clean = 0;
        check("A6 an initramfs/Image of odd size (524,295 / 1,048,699 B): the byte after the true length is untouched", clean && R.a.initrd_bytes % 16384 != 0 && R.a.image_bytes % 4096 != 0, "");
    }
    long accesses = access_n;
    (void)accesses;

    /* B: the rescue set, each with its /chosen m4,boot-reason */
    rescue_case("B1 boot-pending (with content): previous-boot-incomplete", "pending", "previous-boot-incomplete", NULL);
    rescue_case("B2 boot-pending (empty file): previous-boot-incomplete", "pending-empty", "previous-boot-incomplete", NULL);
    rescue_case("B3 ext4 state not clean: ext4-dirty", "dirty-state", "ext4-dirty", NULL);
    rescue_case("B4 ext4 needs_recovery: ext4-dirty", "dirty-recover", "ext4-dirty", NULL);
    rescue_case("B5 no loader.conf: no-loader-conf", "noconf", "no-loader-conf", NULL);
    rescue_case("B6 loader.conf abi=2: bad-abi", "loader-abi2", "bad-abi", NULL);
    rescue_case("B7 loader.conf without abi: bad-conf", "loader-noabi", "bad-conf", NULL);
    rescue_case("B8 default= with a path: bad-conf", "loader-baddefault", "bad-conf", NULL);
    rescue_case("B9 duplicate default=: bad-conf", "loader-dupkey", "bad-conf", NULL);
    rescue_case("B10 default= names no entry: no-entry", "no-entry", "no-entry", NULL);
    rescue_case("B11 entry.conf abi=2: bad-abi", "entry-abi2", "bad-abi", NULL);
    rescue_case("B12 entry.conf without cmdline=: bad-conf", "entry-nocmdline", "bad-conf", NULL);
    rescue_case("B13 cmdline= of 1025 bytes: bad-conf", "entry-longcmdline", "bad-conf", NULL);
    rescue_case("B14 entry.conf without Image_crc32: bad-conf", "entry-nocrc", "bad-conf", NULL);
    rescue_case("B15 entry.conf without board.dtb_size: bad-conf", "entry-nosize", "bad-conf", NULL);
    rescue_case("B16 Image missing: missing-file", "missing-image", "missing-file", NULL);
    rescue_case("B17 Image size differs: size-mismatch", "size-image", "size-mismatch", NULL);
    rescue_case("B18 board.dtb size differs: size-mismatch", "size-dtb", "size-mismatch", NULL);
    rescue_case("B19 initramfs.img CRC differs: digest-mismatch", "crc-initrd", "digest-mismatch", NULL);
    rescue_case("B20 Image CRC differs: digest-mismatch", "crc-image", "digest-mismatch", NULL);
    rescue_case("B21 board.dtb CRC differs: digest-mismatch", "crc-dtb", "digest-mismatch", NULL);
    rescue_case("B22 Image magic wrong (CRC matches): bad-image", "bad-image-magic", "bad-image", NULL);
    rescue_case("B23 Image text_offset != 0: bad-image", "image-textoff", "bad-image", NULL);
    rescue_case("B24 Image memory over the window: too-large", "image-window", "too-large", NULL);
    rescue_case("B25 board.dtb is not a DTB (CRC matches): bad-dtb", "bad-dtb", "bad-dtb", NULL);
    rescue_case("B26 board.dtb truncated (size and CRC match the file): bad-dtb", "truncated-dtb", "bad-dtb", NULL);

    /* G: a GPT that does not match the pinned root (PARTUUID, or its first LBA) is refused before the
     * file system is read, and that rescue boot must publish no write ranges (Linux then has no verified range at all).
     * (l1_gpt_find_p4 accepts the exact range with the pinned PARTUUID or a Linux/basic-data type.) */
    {
        uint8_t *e = gptbuf + 2 * 4096 + 3 * 128, *h = gptbuf + 4096;
        open_fs("ok");
        e[16] ^= 0xff; e[0] ^= 0xff; put32(h + 88, crc32b(gptbuf + 2 * 4096, 128 * 128)); put32(h + 16, 0); put32(h + 16, crc32b(h, 92));
        do_run(&R, 1, 1, -1);
        snprintf(extra, sizeof(extra), "source=%s reason=%s detail='%s' wr=%d", R.source, R.reason, R.detail, R.wr_n);
        check("G1 root PARTUUID and type differ: rescue, ssd-unreadable, no write ranges",
              !R.a.ssd && !strcmp(R.source, "rescue") && !strcmp(R.reason, "ssd-unreadable") && !R.wr_n && R.publish_rc == 0, extra);
        build_gpt();
        put64(e + 32, L1_P4_FIRST + 1); put32(h + 88, crc32b(gptbuf + 2 * 4096, 128 * 128)); put32(h + 16, 0); put32(h + 16, crc32b(h, 92));
        do_run(&R, 1, 1, -1);
        snprintf(extra, sizeof(extra), "source=%s reason=%s detail='%s' wr=%d", R.source, R.reason, R.detail, R.wr_n);
        check("G2 root first LBA differs: rescue, ssd-unreadable, no write ranges",
              !R.a.ssd && !strcmp(R.source, "rescue") && !strcmp(R.reason, "ssd-unreadable") && !R.wr_n && R.publish_rc == 0, extra);
        build_gpt();
    }

    /* C: limits */
    open_fs("entry-maxcmdline");
    do_run(&R, 1, 1, -1);
    check("C1 cmdline= of exactly 1024 bytes is accepted", R.a.ssd && strlen(R.a.cmdline) == 1024 && !strcmp(R.reason, "default"), R.reason);

    /* D: non-capable (embedded) variant and refusals never read the disk */
    open_fs("ok");
    do_run(&R, 0, 1, -1);
    snprintf(extra, sizeof(extra), "source=%s reason=%s tn=%d", R.source, R.reason, (int)tn);
    check("D1 variant without Level 1: rescue, reason loader-disabled, ANS untouched",
          !R.a.ssd && !strcmp(R.source, "rescue") && !strcmp(R.reason, "loader-disabled") && ans_untouched() && R.has_abi, extra);
    refused_reason = 7;
    do_run(&R, 1, 1, -1);
    refused_reason = 0;
    open_fs("ok");
    check("D2 shim guard refused the attempt: rescue, detail guard-refused-7",
          !strcmp(R.source, "rescue") && !strcmp(R.reason, "ssd-unavailable") && !strcmp(R.detail, "guard-refused-7"), R.detail);
    do_run(&R, 1, 0, -1);
    check("D3 no cache window: rescue, ssd-unavailable, ANS untouched",
          !R.a.ssd && !strcmp(R.source, "rescue") && !strcmp(R.reason, "ssd-unavailable") && ans_untouched(), R.detail);

    /* E: an abort at every guard class: the rescue set boots, ANS ends handed back or untouched, never half-way */
    open_fs("ok");
    do_run(&R, 1, 1, -1);
    int phases = guard_calls;
    int bad_runs = 0, rescue_runs = 0;
    for (int ph = 0; ph < phases; ph++) {
        do_run(&R, 1, 1, ph);
        if (R.a.ssd) { bad_runs++; continue; }
        rescue_runs++;
        if (strcmp(R.source, "rescue") || !handoff_clean(why, sizeof(why))) { bad_runs++; printf("  phase %d: source=%s %s\n", ph, R.source, why); }
    }
    snprintf(extra, sizeof(extra), "%d guard phases, %d rescue, %d not clean", phases, rescue_runs, bad_runs);
    check("E1 an abort in each guarded phase: rescue set, hand-off clean", rescue_runs > 0 && bad_runs == 0, extra);

    printf("%s: %d/%d passed\n", failures ? "FAIL" : "ALL PASS", total - failures, total);
    return failures ? 1 : 0;
}
