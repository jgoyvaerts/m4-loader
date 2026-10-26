/* Host test of m4_dcpext_fixup() (dcp-fdt.c): the real function on an ADT and a board.dtb fixture.
 *   dcpext-test ADT.bin IN.dtb OUT.dtb EXPECT
 * EXPECT: any (fuzzed ADT: the tree must match the published record) | none | units:A,B (the /chosen m4,dcpext record; 0 filled) | tree-error.
 * units 0 checks every value against the static t6040-j616s-extdisp-mem.dtsi this unit has run with (macOS 25G83:
 * identical in all retained captures and in the 25G241 ADT), so the loader and the static file cannot drift apart. */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "adt.h"
#include "libfdt.h"
#include "fixups.h"

extern void adt_double_init(void *, uint32_t);

static const struct { uint64_t phys, iova, size; } want[2][3] = {
    {{0x100002f4000ull, 0x090d4000, 0x6d0000}, {0x10bd0e20000ull, 0x097a4000, 0x2c4000}, {0x10bcfe20000ull, 0x09a68000, 0x1000000}},
    {{0x100002f4000ull, 0x090d4000, 0x6d0000}, {0x10bcfb5c000ull, 0x097a4000, 0x2c4000}, {0x10bceb5c000ull, 0x09a68000, 0x1000000}}};
static const char *const part[3] = {"text", "data", "heap"};
static int bad;
#define CHECK(c, ...) do { if (!(c)) { fprintf(stderr, "FAIL: " __VA_ARGS__); fputc('\n', stderr); bad = 1; } } while (0)

static void *slurp(const char *p, size_t extra, size_t *n)
{
    FILE *f = fopen(p, "rb"); if (!f) { perror(p); exit(2); }
    fseek(f, 0, SEEK_END); *n = (size_t)ftell(f); rewind(f);
    void *b = calloc(1, *n + extra); if (fread(b, 1, *n, f) != *n) exit(2); fclose(f); return b;
}
static uint64_t cell64(const fdt32_t *c) { return ((uint64_t)fdt32_to_cpu(c[0]) << 32) | fdt32_to_cpu(c[1]); }

/* exact: compare with the static dtsi values; otherwise (fuzzed ADT) only that the node agrees with itself */
/* the dcpext node of a unit: by compatible and register base (as the loader finds it) */
static int dcpext_node(void *t, unsigned unit)
{
    const uint64_t base[2] = {0x3a2e00000ull, 0x602e00000ull};
    for (int n = fdt_node_offset_by_compatible(t, -1, "apple,t6041-dcpext"); n >= 0;
         n = fdt_node_offset_by_compatible(t, n, "apple,t6041-dcpext")) {
        int len;
        const fdt32_t *r = fdt_getprop(t, n, "reg", &len);
        if (r && len >= 8 && cell64(r) == base[unit]) return n;
    }
    return -1;
}

static void check_filled(void *t, unsigned unit, const char *label, int exact)
{
    int dev = dcpext_node(t, unit), len;
    CHECK(dev >= 0, "%s missing", label);
    if (dev < 0) return;
    const char *st = fdt_getprop(t, dev, "status", NULL);
    CHECK(st && !strcmp(st, "disabled"), "%s must stay disabled (is %s)", label, st ? st : "unset");
    const fdt32_t *list = fdt_getprop(t, dev, "memory-region", &len);
    CHECK(list && len == 12, "%s memory-region has %d bytes", label, list ? len : -1);
    const uint32_t *ver = fdt_getprop(t, dev, "apple,firmware-version", &len), *cmp = fdt_getprop(t, dev, "apple,firmware-compat", &len);
    CHECK(ver && cmp && !memcmp(ver, cmp, 12) && fdt32_to_cpu(ver[0]) == 26 && fdt32_to_cpu(ver[1]) == 6 && fdt32_to_cpu(ver[2]) == 2,
          "%s firmware version/compat", label);
    for (unsigned i = 0; list && len == 12 && i < 3; i++) {
        int rn = fdt_node_offset_by_phandle(t, fdt32_to_cpu(list[i]));
        CHECK(rn >= 0, "%s region %u phandle", label, i);
        if (rn < 0) continue;
        char name[64], rname[40];
        const fdt32_t *reg = fdt_getprop(t, rn, "reg", &len);
        CHECK(reg && len == 16, "%s %s reg", label, part[i]);
        uint64_t phys = reg && len == 16 ? cell64(reg) : 0, size = reg && len == 16 ? cell64(reg + 2) : 0;
        if (exact) CHECK(phys == want[unit][i].phys && size == want[unit][i].size, "%s %s reg value", label, part[i]);
        snprintf(name, sizeof(name), "dcpext%u-%s@%lx", unit, part[i], (unsigned long)phys);
        CHECK(!strcmp(fdt_get_name(t, rn, NULL), name), "%s region %u name %s, want %s", label, i, fdt_get_name(t, rn, NULL), name);
        const fdt32_t *io = fdt_getprop(t, rn, "iommu-addresses", &len);
        CHECK(io && len == 20 && fdt32_to_cpu(io[0]) == fdt_get_phandle(t, dev) && cell64(io + 3) == size &&
              (!exact || cell64(io + 1) == want[unit][i].iova), "%s %s iommu-addresses", label, part[i]);
        CHECK(io && len == 20 && !(phys & 0x3fff) && !(size & 0x3fff) && size, "%s %s alignment", label, part[i]);
        CHECK(!fdt_getprop(t, rn, "status", NULL), "%s %s still has a status", label, part[i]);
        CHECK(fdt_getprop(t, rn, "no-map", NULL) && !strcmp(fdt_getprop(t, rn, "compatible", NULL), "apple,asc-mem"), "%s %s no-map/compatible", label, part[i]);
        snprintf(rname, sizeof(rname), "dcp-map@%lx", (unsigned long)phys);
        const char *got = fdt_stringlist_get(t, dev, "memory-region-names", (int)i, NULL);
        CHECK(got && !strcmp(got, rname), "%s memory-region-names[%u] = %s, want %s", label, i, got ? got : "(none)", rname);
    }
    CHECK(fdt_stringlist_count(t, dev, "memory-region-names") == 3, "%s memory-region-names count", label);
}

static void check_refused(void *t, unsigned unit)
{
    char label[16];
    snprintf(label, sizeof(label), "dcpext%u", unit);
    int dev = dcpext_node(t, unit), len;
    CHECK(dev >= 0, "%s missing", label);
    if (dev < 0) return;
    CHECK(!fdt_getprop(t, dev, "memory-region", &len) && !fdt_getprop(t, dev, "memory-region-names", &len) &&
          !fdt_getprop(t, dev, "apple,firmware-version", &len), "%s kept memory properties", label);
}

int main(int argc, char **argv)
{
    if (argc != 5) return 2;
    size_t adt_n, dtb_n;
    void *file = slurp(argv[1], 0, &adt_n);
    /* the ADT is read in place by the double; keep it 16-byte aligned */
    void *adt_mem = aligned_alloc(16, (adt_n + 15) & ~15ull);
    memcpy(adt_mem, file, adt_n);
    adt_double_init(adt_mem, (uint32_t)adt_n);
    const size_t cap = 1 << 20;
    void *board = slurp(argv[2], 0, &dtb_n);
    void *t = calloc(1, cap);
    if (fdt_open_into(board, t, cap)) { fprintf(stderr, "dtb does not open\n"); return 3; }
    const char *expect = argv[4];
    int rc = m4_dcpext_fixup(t, cap);
    fprintf(stderr, "m4_dcpext_fixup rc=%d\n", rc);
    int chosen = fdt_path_offset(t, "/chosen"), len;
    const fdt32_t *rec = chosen < 0 ? NULL : fdt_getprop(t, chosen, "m4,dcpext", &len);
    if (rec && len == 8) fprintf(stderr, "m4,dcpext = <%u %u>\n", fdt32_to_cpu(rec[0]), fdt32_to_cpu(rec[1]));
    if (!strcmp(expect, "tree-error")) CHECK(rc < 0, "expected a tree error, rc=%d", rc);
    else CHECK(rc >= 0, "unexpected tree error %d", rc);
    CHECK(!fdt_check_full(t, cap) || rc < 0, "result tree is not valid");
    if (!strcmp(expect, "any") && rec && len == 8) {
        /* fuzzed ADT: whatever the loader decided, the tree must be consistent with the record it published */
        for (unsigned u = 0; u < 2; u++) {
            char label[16];
            snprintf(label, sizeof(label), "dcpext%u", u);
            unsigned st = fdt32_to_cpu(rec[u]);
            if (st == 0) check_filled(t, u, label, 0);
            else if (st > 2) check_refused(t, u);
        }
    } else if (!strcmp(expect, "none")) {
        CHECK(rc == 0 && !rec, "no dcpext node: no record");
    } else if (!strncmp(expect, "units:", 6)) {
        /* units:A,B = the record, per unit: 0 filled, 1 no placeholders, 2 memory already static, 3 firmware, 4 ADT, 5 RAM */
        unsigned st[2];
        CHECK(sscanf(expect + 6, "%u,%u", &st[0], &st[1]) == 2, "bad EXPECT %s", expect);
        CHECK(rec && len == 8 && fdt32_to_cpu(rec[0]) == st[0] && fdt32_to_cpu(rec[1]) == st[1], "record is not <%u %u>", st[0], st[1]);
        CHECK(rc == (st[0] > 2 || st[1] > 2), "rc %d", rc);
        for (unsigned u = 0; u < 2; u++) {
            char label[16];
            snprintf(label, sizeof(label), "dcpext%u", u);
            if (st[u] == 0) check_filled(t, u, label, 1);
            else if (st[u] > 2) check_refused(t, u);
        }
        int resv = fdt_path_offset(t, "/reserved-memory"), n, left = 0, expect_left = 0;
        fdt_for_each_subnode(n, t, resv) if (strstr(fdt_get_name(t, n, NULL), "dcpext")) left++;
        for (unsigned u = 0; u < 2; u++) expect_left += st[u] == 0 ? 3 : st[u] > 2 ? 0 : 3;
        CHECK(left == expect_left, "%d dcpext reserved nodes, want %d", left, expect_left);
    }
    FILE *o = fopen(argv[3], "wb"); fwrite(t, 1, fdt_totalsize(t), o); fclose(o);
    fprintf(stderr, bad ? "FAIL %s\n" : "PASS %s\n", expect);
    return bad;
}
