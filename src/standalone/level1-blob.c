/* SPDX-License-Identifier: MIT */
/* Level 1 blob, rev 11 (Boot ABI v1): DT-derived addresses of the embedded rescue board.dtb (cross-checked against the
 * ADT), read-only probe, HAL, guarded attempt (boot-pending, loader.conf, entry.conf, Image + initramfs.img +
 * board.dtb with size and CRC-32), DT publication of m4,ans-live/ans-syslog and the m4,boot-* /chosen properties.
 *
 * Runs inside the loader's cache window (RAM Normal write-back, MMIO Device) under the
 * level1-guard.S guard, which contains synchronous aborts of every class AND SErrors. The whole
 * attempt runs under one guard (the shim's) and again under one (ph_whole); every hardware step
 * has its own phase guard, and the probe reads every register under a guard of its own, so a
 * fault anywhere ends in status + a printed `L1 abort ...` line + the hand-back in its own guard
 * + the final `L1 embedded <status>` line. The caller boots the embedded payload unless args->ssd.
 *
 * No writable static data, no dynamic relocations: all state lives in one
 * allocation from the loader pool (struct l1_rec). See level1-host.h. */
#include <stdarg.h>
#include "level1.h"
#include "level1-ans.h"
#include "level1-host.h"

#define PROBE_DEV_ITEMS L1P_DEV_COUNT
#define ATTEMPT_BUDGET_US 6000000u /* reported as over-budget in the log when exceeded */
#define READ_DEADLINE_US 4000000u  /* reads (and with them the SHA-256 pass) must have started by then */

#ifdef L1_TEST
/* host test build: fake counter and HAL supplied by tests/l1-e2e.c */
extern uint64_t l1_test_cnt(void);
extern const struct l1_hal l1_test_hal;
#define CNT() l1_test_cnt()
#else
#define CNT() ({ uint64_t _v; __asm__ volatile("isb; mrs %0, cntpct_el0" : "=r"(_v) : : "memory"); _v; })
#endif

#ifndef M4_LEVEL1
#define M4_LEVEL1 0
#endif

struct l1_rec {
    const struct l1_host *h;
    struct l1_attempt_args *a;
    struct l1_ans ans;
    struct l1_result res;
    struct l1_load load;
    struct l1_dev dev;
    struct l1_hal hal;
    struct l1_ans_cfg cfg;
    uint8_t *dma, *meta;
    uint64_t t0_ns, attempt_t0_ns, verify_t0;   /* t0_ns: loader start (log base); attempt_t0_ns: deadlines */
    unsigned sys_lines, log_n;
    char log[L1_LOG_MAX + 16];
    char why[24];
    char probe[240];                            /* what the probe read, compact hex (also m4,level1-probe) */
    char abort_txt[120];                        /* the first abort, one line (also m4,level1-abort) */
    unsigned aborts;
    int ran, ssd, handback_done, dev_probed, live_done;
    enum l1_status st;
    uint32_t ms[6], verify_ms;
    uint64_t win[4];                            /* kernel, kernel window, initrd, initrd window (m4,level1-windows) */
};

/* ------------------------------------------------------------ small helpers */

static size_t slen(const char *s) { size_t n = 0; while (s[n]) n++; return n; }
static int sdiff(const char *a, const char *b) { while (*a && *a == *b) { a++; b++; } return (unsigned char)*a - (unsigned char)*b; }
static void scpy(char *d, size_t cap, const char *s)
{
    size_t i = 0;
    while (s[i] && i + 1 < cap) { d[i] = s[i]; i++; }
    d[i] = 0;
}

/* Formatting is the loader's vsnprintf (host table): the blob's own formatter cost 850 bytes of its 32 KiB slot.
 * Returns the length the text would have had (C99), so appends are clamped with fit(). */
static int fmt_h(const struct l1_host *h, char *out, size_t cap, const char *f, ...)
{
    va_list ap;
    va_start(ap, f);
    int n = h->vsnprintf(out, cap, f, ap);
    va_end(ap);
    return n;
}
#define fmt(...) fmt_h(fh, __VA_ARGS__)

static uint32_t elapsed_ms(const struct l1_rec *r) { return (uint32_t)((CNT() - r->t0_ns) / 1000000); }


static void say(struct l1_rec *r, const char *s) { if (r->h->line) r->h->line(s); }

/* ------------------------------------------------------------ log */

static void log_add(struct l1_rec *r, const char *s, uint64_t v, int with_val)
{
    const struct l1_host *fh = r->h;
    char t[96];
    int n = with_val ? fmt(t, sizeof(t), "%s=%lu@%u;", s, v, elapsed_ms(r))
                     : fmt(t, sizeof(t), "%s@%u;", s, elapsed_ms(r));
    if (n <= 0 || r->log_n + (unsigned)n >= L1_LOG_MAX) return;
    for (int i = 0; i < n; i++) r->log[r->log_n + i] = t[i];
    r->log_n += (unsigned)n;
    r->log[r->log_n] = 0;
}

static void on_syslog(void *ctx, const char *line)
{
    struct l1_rec *r = ctx;
    const struct l1_host *fh = r->h;
    if (r->sys_lines >= 20) return;
    r->sys_lines++;
    char clean[64];
    unsigned k = 0;
    for (; line[k] && k < 56; k++) clean[k] = (line[k] == ';' || line[k] == '=') ? ',' : line[k];
    clean[k] = 0;
    char t[80];
    int n = fmt(t, sizeof(t), "sl:%s@%u;", clean, elapsed_ms(r));
    if (n > 0 && r->log_n + (unsigned)n < L1_LOG_MAX) {
        for (int i = 0; i < n; i++) r->log[r->log_n + i] = t[i];
        r->log_n += (unsigned)n;
        r->log[r->log_n] = 0;
    }
}

static void on_trace(void *ctx, const char *step, uint64_t v)
{
    log_add(ctx, step, v, 1);
}

static void on_step(void *ctx, const char *name, uint64_t v)
{
    struct l1_rec *r = ctx;
    if (!sdiff(name, "hashed")) {
        if (r->verify_t0) r->verify_ms += (uint32_t)((CNT() - r->verify_t0) / 1000000);
        return;
    }
    log_add(r, name, v, 1);
}

static void pre_verify(void *ctx, void *p, uint64_t n)
{
    struct l1_rec *r = ctx;
    (void)p; (void)n;
    r->verify_t0 = CNT();
}

#ifndef L1_TEST
/* ------------------------------------------------------------ HAL (state-free)
 * Every MMIO access is followed by dsb sy; isb: with the guard's SError unmasked, an asynchronous
 * error from this access is taken here, inside the phase guard that issued it, not at some later
 * access (and never after the guard has been left). */

static void mmio_sync(void) { __asm__ volatile("dsb sy; isb" : : : "memory"); }
static uint32_t h_r32(void *ctx, uint64_t a) { (void)ctx; uint32_t v = *(volatile uint32_t *)a; mmio_sync(); return v; }
static void h_w32(void *ctx, uint64_t a, uint32_t v) { (void)ctx; *(volatile uint32_t *)a = v; mmio_sync(); }
static uint64_t h_r64(void *ctx, uint64_t a) { (void)ctx; uint64_t v = *(volatile uint64_t *)a; mmio_sync(); return v; }
static void h_w64(void *ctx, uint64_t a, uint64_t v) { (void)ctx; *(volatile uint64_t *)a = v; mmio_sync(); }
static void h_udelay(uint32_t us)
{
    uint64_t end = CNT() + (uint64_t)us * 1000;
    while (CNT() < end) {}
}
static uint64_t h_now(void) { return CNT() / 1000; } /* CNTFRQ is checked to be 1 GHz */
static uint64_t line_bytes(void)
{
    uint64_t ctr;
    __asm__ volatile("mrs %0, ctr_el0" : "=r"(ctr));
    return 4ull << ((ctr >> 16) & 15);
}
static void h_clean(const void *p, size_t n)
{
    uint64_t line = line_bytes(), a = (uint64_t)p & ~(line - 1), end = (uint64_t)p + n;
    for (; a < end; a += line) __asm__ volatile("dc cvac, %0" : : "r"(a) : "memory");
    __asm__ volatile("dsb sy" : : : "memory");
}
static void h_inval(const void *p, size_t n)
{
    uint64_t line = line_bytes(), a = (uint64_t)p & ~(line - 1), end = (uint64_t)p + n;
    __asm__ volatile("dsb sy" : : : "memory");
    for (; a < end; a += line) __asm__ volatile("dc ivac, %0" : : "r"(a) : "memory");
    __asm__ volatile("dsb sy; isb" : : : "memory");
}

#endif


/* ------------------------------------------------------------ Linux DT: where the addresses come from
 * /nvme@44dcc0000: reg + reg-names (nvme, ans, nvmmu), mboxes -> mailbox node reg, apple,sart -> sart node
 * reg, power-domains (+ power-domain-names ans/apcie0/apcie1) -> the pmgr power-controller@OFF nodes
 * (address = parent pmgr node reg + OFF) and, recursively, their own power-domains parents. */

static void be32(uint8_t *p, uint32_t v) { p[0] = (uint8_t)(v >> 24); p[1] = (uint8_t)(v >> 16); p[2] = (uint8_t)(v >> 8); p[3] = (uint8_t)v; }
static uint32_t be32p(const uint8_t *p) { return (uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | p[3]; }
static uint64_t be64p(const uint8_t *p) { return (uint64_t)be32p(p) << 32 | be32p(p + 4); }

static int dt_reg(const struct l1_host *h, const void *t, int node, unsigned n, unsigned idx, uint64_t *b, uint64_t *s)
{
    int len = 0;
    const uint8_t *r = node < 0 ? 0 : h->fdt_getprop(t, node, "reg", &len);
    if (!r || len < (int)(16 * (idx + 1)) || len != (int)(16 * n)) return -1; /* #address-cells = #size-cells = 2 */
    *b = be64p(r + 16 * idx); *s = be64p(r + 16 * idx + 8);
    return 0;
}

static int dt_phandle_node(const struct l1_host *h, const void *t, int node, const char *prop, unsigned idx, unsigned count)
{
    int len = 0;
    const uint8_t *p = h->fdt_getprop(t, node, prop, &len);
    if (!p || len != (int)(4 * count)) return -1;
    return h->fdt_node_offset_by_phandle(t, be32p(p + 4 * idx));
}

/* a pmgr power-controller@OFF node: reg = <OFF 4> under power-management@BASE (#address-cells 1) */
static int dt_pm_addr(const struct l1_host *h, const void *t, int node, uint64_t *addr)
{
    int len = 0;
    const uint8_t *r = h->fdt_getprop(t, node, "reg", &len);
    int parent = h->fdt_parent_offset(t, node);
    uint64_t base, size;
    if (!r || len != 8 || be32p(r + 4) != 4 || parent < 0 || dt_reg(h, t, parent, 1, 0, &base, &size)) return -1;
    uint32_t off = be32p(r);
    if ((uint64_t)off + 4 > size) return -1;
    *addr = base + off;
    return 0;
}

static int dt_pm_add(const struct l1_host *h, const void *t, struct l1_ans_cfg *c, int node, int own, unsigned depth)
{
    int len = 0;
    if (depth > 4 || node < 0) return -1;
    const uint8_t *pd = h->fdt_getprop(t, node, "power-domains", &len);
    if (pd) {
        if (len & 3 || len > 16) return -1;
        for (int i = 0; i < len; i += 4) {
            int pn = h->fdt_node_offset_by_phandle(t, be32p(pd + i));
            if (dt_pm_add(h, t, c, pn, 0, depth + 1)) return -1;
        }
    }
    uint64_t a;
    if (dt_pm_addr(h, t, node, &a)) return -1;
    for (unsigned i = 0; i < c->pm_count; i++)
        if (c->pm_reg[i] == a) { if (own) c->pm_own[i] = 1; return 0; }
    if (c->pm_count >= L1_PM_MAX) return -1;
    c->pm_reg[c->pm_count] = a;
    c->pm_own[c->pm_count++] = (uint8_t)own;
    return 0;
}

static int str_idx(const char *list, int len, const char *want)
{
    int idx = 0;
    for (int i = 0; i < len;) {
        const char *e = list + i;
        int k = 0;
        while (e[k] && want[k] && e[k] == want[k]) k++;
        if (!e[k] && !want[k]) return idx;
        while (i < len && list[i]) i++;
        i++; idx++;
    }
    return -1;
}

/* 0 = ok; else status ans-power:dt<N>: 1 no /nvme@44dcc0000, 2 reg-names, 3 reg, 4 mboxes, 5 apple,sart, 6 power-domain-names,
 * 7 power-domains chain, 8 resets */
static int discover_dt(const struct l1_host *h, const void *t, struct l1_ans_cfg *c)
{
    for (unsigned i = 0; i < sizeof(*c); i++) ((uint8_t *)c)[i] = 0;
    int n = h->fdt_path_offset(t, "/nvme@44dcc0000"), len = 0;
    if (n < 0) return 1;
    const char *names = h->fdt_getprop(t, n, "reg-names", &len);
    if (!names) return 2;
    int i_nvme = str_idx(names, len, "nvme"), i_ans = str_idx(names, len, "ans"), i_mmu = str_idx(names, len, "nvmmu");
    if (i_nvme < 0 || i_ans < 0 || i_mmu < 0) return 2;
    if (dt_reg(h, t, n, 3, (unsigned)i_nvme, &c->nvme, &c->nvme_size) || dt_reg(h, t, n, 3, (unsigned)i_ans, &c->asc, &c->asc_size) ||
        dt_reg(h, t, n, 3, (unsigned)i_mmu, &c->nvmmu, &c->nvmmu_size)) return 3;
    int mb = dt_phandle_node(h, t, n, "mboxes", 0, 1);
    if (dt_reg(h, t, mb, 1, 0, &c->mbox, &c->mbox_size)) return 4;
    int st = dt_phandle_node(h, t, n, "apple,sart", 0, 1);
    if (dt_reg(h, t, st, 1, 0, &c->sart, &c->sart_size)) return 5;
    const char *pnames = h->fdt_getprop(t, n, "power-domain-names", &len);
    int i_pans = pnames ? str_idx(pnames, len, "ans") : -1;
    if (i_pans < 0 || str_idx(pnames, len, "apcie0") < 0 || str_idx(pnames, len, "apcie1") < 0) return 6;
    for (unsigned k = 0; k < 3; k++) {
        int pn = dt_phandle_node(h, t, n, "power-domains", k, 3);
        if (dt_pm_add(h, t, c, pn, 1, 0)) return 7;
    }
    int ans = dt_phandle_node(h, t, n, "power-domains", (unsigned)i_pans, 3);
    int rs = dt_phandle_node(h, t, n, "resets", 0, 1);
    uint64_t a;
    if (ans < 0 || ans != rs || dt_pm_addr(h, t, ans, &a)) return 8;
    c->pm_ans = a;
    return 0;
}


/* ------------------------------------------------------------ phases */

static int ph_power(void *arg) { struct l1_rec *r = arg; r->st = l1_ans_power(&r->ans); return 0; }
static int ph_cpu(void *arg) { struct l1_rec *r = arg; r->st = l1_ans_check_cpu(&r->ans); return 0; }
static int ph_start(void *arg) { struct l1_rec *r = arg; r->st = l1_ans_start(&r->ans); return 0; }
static int ph_nvme(void *arg) { struct l1_rec *r = arg; r->st = l1_ans_nvme(&r->ans); return 0; }

static int ph_policy(void *arg)
{
    struct l1_rec *r = arg;
    r->ans.deadline_us = r->attempt_t0_ns / 1000 + READ_DEADLINE_US;
    l1_run(&r->load, &r->res);
    return 0;
}

static int ph_leave(void *arg)
{
    struct l1_rec *r = arg;
    enum l1_status s = l1_ans_leave(&r->ans);
    if (s != L1_OK && r->st == L1_OK) { r->st = s; scpy(r->why, sizeof(r->why), "handback"); }
    return 0;
}

static int ph_stop(void *arg)
{
    struct l1_rec *r = arg;
    l1_ans_stop(&r->ans);
    return 0;
}

static int ph_emergency(void *arg)
{
    struct l1_rec *r = arg;
    l1_ans_emergency(&r->ans);
    return 0;
}

struct probe_arg { struct l1_ans *a; unsigned item; uint32_t v; uint64_t addr; int rc; };
static int ph_probe_one(void *arg)
{
    struct probe_arg *p = arg;
    p->rc = l1_ans_probe(p->a, p->item, &p->v, &p->addr);
    return 0;
}

/* One abort, printed live and kept: the first one goes to m4,level1-abort. */
static void abort_line(struct l1_rec *r, const char *name, const struct l1_exec_result *x, int refused)
{
    const struct l1_host *fh = r->h;
    char b[132];
    int n = fmt(b, sizeof(b), "L1 abort %s s=%lu v=%lu esr=%lx elr=%lx far=%lx at=%lx%s#%u", name, x->status, x->vector, x->esr,
                x->elr, x->far, r->ans.last_addr, r->ans.last_wr ? "w" : "r", r->ans.nacc);
    (void)n; (void)refused;
    say(r, b);
    if (!r->aborts++) scpy(r->abort_txt, sizeof(r->abort_txt), b + 9);
}

/* 0 = ran to completion; else the guard status (the line is printed here): 1 aborted inside fn, 2 an SError that was
 * pending at entry was taken and fn did NOT run (callers that must run retry once), 3 refused */
static int guarded(struct l1_rec *r, int (*fn)(void *), const char *name)
{
    struct l1_exec_result res = {0, 0, 0, 0, 0, 0xff, 0, 0};
    int rc = r->h->guarded(fn, r, &res);
    if (rc || res.status) {
        if (rc && !res.status) res.status = 3;
        abort_line(r, name, &res, rc);
        return (int)res.status;
    }
    return 0;
}

static void set_status(struct l1_rec *r, enum l1_status s, const char *why)
{
    r->st = s;
    if (why) scpy(r->why, sizeof(r->why), why);
}

static const char *gpt_why(int d)
{
    switch (d) { case 1: return "read"; case 2: return "sig"; case 3: return "header"; case 4: return "crc";
                 case 5: return "entries"; case 6: return "no-p4"; case 7: return "range"; default: return "?"; }
}
static const char *hdr_why(int d)
{
    switch (d) { case 1: return "short"; case 2: return "magic"; case 3: return "textoff"; case 4: return "flags";
                 case 5: return "size"; case 6: return "window"; default: return "?"; }
}

/* ---- probe -------------------------------------------------------------- */

static int pm_on(uint32_t v) { return ((v >> 4) & 15) == 0xf || ((v & 15) == 0xf && (v & (1u << 28))); }

/* Reads items [from, to) one per guard. 0 = all read, 1 = a read faulted (abort already printed). */
static int probe_range(struct l1_rec *r, unsigned from, unsigned to, uint32_t *out)
{
    for (unsigned i = from; i < to; i++) {
        struct probe_arg p = {&r->ans, i, 0, 0, 0};
        struct l1_exec_result res = {0, 0, 0, 0, 0, 0xff, 0, 0};
        int rc = r->h->guarded(ph_probe_one, &p, &res);
        if (rc || res.status || p.rc) {
            if (!res.status) { res.status = rc ? 3 : 4; }       /* 4: refused by the allow-list */
            abort_line(r, "probe", &res, rc);
            return 1;
        }
        out[i] = p.v;
    }
    return 0;
}

/* fmt() returns the length the text would have had; keep an append offset inside the buffer */
static size_t fit(size_t o, size_t cap) { return o < cap ? o : cap - 1; }

static void probe_pm_line(struct l1_rec *r, const uint32_t *v, unsigned n)
{
    const struct l1_host *fh = r->h;
    size_t o = (size_t)fmt(r->probe, sizeof(r->probe), "pm=");
    for (unsigned i = 0; i < n; i++) o = fit(o + (size_t)fmt(r->probe + o, sizeof(r->probe) - o, "%s%x", i ? "," : "", v[i]), sizeof(r->probe));
}

static void probe_dev_line(struct l1_rec *r, const uint32_t *v, unsigned pm)
{
    const struct l1_host *fh = r->h;
    const uint32_t *d = v + pm;
    size_t from = fit(slen(r->probe) + 1, sizeof(r->probe));
    uint32_t mask = 0;
    for (unsigned i = 0; i < 16; i++) if (d[L1P_SART0 + i] & 0xff) mask |= 1u << i;
    size_t o = from - 1;
    o = fit(o + (size_t)fmt(r->probe + o, sizeof(r->probe) - o, ";cpu=%x sart=%x", d[L1P_CPU], mask), sizeof(r->probe));
    for (unsigned i = 0; i < 16; i++)
        if (d[L1P_SART0 + i] & 0xff)
            o = fit(o + (size_t)fmt(r->probe + o, sizeof(r->probe) - o, ":%u=%x", i, d[L1P_SART0 + i]), sizeof(r->probe));
}

/* The device group: CPU_CONTROL and the SART config, nothing else (see l1_ans_probe). */
static int dev_probe(struct l1_rec *r, uint32_t *pv, unsigned pm)
{
    if (probe_range(r, pm, pm + PROBE_DEV_ITEMS, pv)) return 1;
    probe_dev_line(r, pv, pm);
    r->dev_probed = 1;
    return 0;
}

/* ANS was left live: /chosen m4,ans-live (and m4,ans-syslog) are written by l1_publish into the final DT. Linux's
 * nvme-apple adopts that session instead of replaying INIT into it. */
static void publish_live(struct l1_rec *r)
{
    if (!r->ans.st.left || !r->ans.live_n || r->live_done) return;
    r->live_done = 1;
    log_add(r, "live", r->ans.live_n, 1);
}

/* ---- handback ----------------------------------------------------------- */

static void handback(struct l1_rec *r)
{
    struct l1_ans_stats *s = &r->ans.st;
    if (r->handback_done) return;
    if (s->cpu_started || s->nvme_enabled || s->rtkit_up || s->ioq_up || s->reset_touched || s->sart_added != s->sart_removed + s->sart_kept) {
        /* guard status 2: a pending SError was consumed and the phase did not run: run it once more */
        int aborted = guarded(r, ph_leave, "handback");
        if (aborted == 2) aborted = guarded(r, ph_leave, "handback");
        if (aborted || s->cpu_started || s->nvme_enabled || s->reset_touched || s->sart_added != s->sart_removed + s->sart_kept) {
            if (guarded(r, ph_emergency, "emergency") == 2) guarded(r, ph_emergency, "emergency");
            /* the emergency is one guard: a block that keeps faulting must not keep CPU_CONTROL.RUN set */
            if ((s->cpu_started || s->reset_touched) && guarded(r, ph_stop, "stop") == 2) guarded(r, ph_stop, "stop");
            if (r->st == L1_OK || aborted) set_status(r, L1_ABORT, "handoff");
        }
        r->ms[4] = s->ms_shutdown;
    }
    r->handback_done = 1;
}

/* ---- the attempt -------------------------------------------------------- */

/* a bring-up step failed: the driver's reason, or the allow-list refusal, becomes the status text */
static void step_failed(struct l1_rec *r, const char *step)
{
    const struct l1_host *fh = r->h;
    if (r->ans.dead && r->ans.bad_addr) {
        char w[24];
        fmt(w, sizeof(w), "addr-check");
        set_status(r, L1_ANS_POWER, w);
        log_add(r, "refused", r->ans.bad_addr, 1);
    } else if (r->ans.why) {
        scpy(r->why, sizeof(r->why), r->ans.why);
    } else {
        scpy(r->why, sizeof(r->why), step);
    }
    log_add(r, step, r->st, 1);
}

/* the whole attempt after the gates; everything that can touch hardware is in a phase guard */
static int ph_whole(void *arg)
{
    struct l1_rec *r = arg;
    const struct l1_host *h = r->h;
    const struct l1_host *fh = h;
    struct l1_attempt_args *a = r->a;
    if (!h->level1_capable) { set_status(r, L1_DISABLED, "embedded-variant"); return 0; }
    if (!h->cntfrq_ok) { set_status(r, L1_DISABLED, "cntfrq"); return 0; }
    if (!h->cache_window_live) { set_status(r, L1_DISABLED, "no-cache"); return 0; }
    /* addresses: the Linux DT, then refuse anything this Mac's ADT does not declare */
    /* addresses: the Linux DT, then refuse anything this Mac's ADT does not declare (host: level1-addr.c) */
    int dt_bad = discover_dt(h, a->tree, &r->cfg);
    if (dt_bad) {
        char w[24];
        fmt(w, sizeof(w), "dt%u", (unsigned)dt_bad);
        set_status(r, L1_ANS_POWER, w);
        log_add(r, "dt-check", (uint64_t)dt_bad, 1);
        return 0;
    }
    int adt_bad = h->adt_check(h, &r->cfg);
    if (adt_bad) {
        char w[24];
        fmt(w, sizeof(w), "addr-check:adt%u", (unsigned)adt_bad);
        set_status(r, L1_ANS_POWER, w);
        log_add(r, "addr-check", (uint64_t)adt_bad, 1);
        return 0;
    }
    /* one allocation: rings/TCBs + 1 MiB of firmware buffers; fall back to 256 KiB of them */
    size_t dma_bytes = L1_ANS_MEM;
    r->dma = h->memalign(16384, dma_bytes);
    if (!r->dma) { dma_bytes = L1_ANS_MEM_MIN; r->dma = h->memalign(16384, dma_bytes); }
    r->meta = h->memalign(16384, 4 * 4096);
    if (!r->dma || !r->meta) { set_status(r, L1_ANS_POWER, "no-memory"); return 0; }
#ifdef L1_TEST
    r->hal = l1_test_hal;
#else
    r->hal.r32 = h_r32; r->hal.w32 = h_w32; r->hal.r64 = h_r64; r->hal.w64 = h_w64;
    r->hal.udelay = h_udelay; r->hal.now_us = h_now; r->hal.clean = h_clean; r->hal.inval = h_inval;
#endif
    r->hal.syslog = on_syslog; r->hal.trace = on_trace; r->hal.ctx = r;
    l1_ans_init(&r->ans, &r->hal, &r->cfg, r->dma, dma_bytes);
    r->ans.deadline_us = r->attempt_t0_ns / 1000 + READ_DEADLINE_US;
    r->dev.read = l1_ans_read; r->dev.ctx = &r->ans;
    r->load.dev = &r->dev; r->load.meta = r->meta;
    r->load.image_dst = (uint8_t *)a->kernel; r->load.image_window = a->kernel_window;
    r->load.initrd_dst = (uint8_t *)a->initrd; r->load.initrd_window = a->initrd_window;
    r->load.dtb_dst = (uint8_t *)a->dtb; r->load.dtb_window = a->dtb_window;
    r->load.image_floor = 64;
    r->load.step = on_step; r->load.pre_verify = pre_verify; r->load.hook_ctx = r;
    log_add(r, "start", 0, 0);

    /* 1. read-only probe: power registers first (always readable), device registers only when every
     *    domain of the nvme node is on (a gated block must not be read) */
    uint32_t pv[L1_PM_MAX + L1P_DEV_COUNT];
    unsigned pm = r->cfg.pm_count;
    for (unsigned i = 0; i < sizeof(pv) / sizeof(*pv); i++) pv[i] = 0;
    if (probe_range(r, 0, pm, pv)) { set_status(r, L1_ABORT, "probe"); return 0; }
    probe_pm_line(r, pv, pm);
    int own_off = 0;
    uint32_t ans_word = 0;
    for (unsigned i = 0; i < pm; i++) {
        if (r->cfg.pm_own[i] && !pm_on(pv[i])) own_off = 1;
        if (r->cfg.pm_reg[i] == r->cfg.pm_ans) ans_word = pv[i];
    }
    /* an ANS that is on but held in RESET / DEV_DISABLE is not read either (l1_ans_power stops on it) */
    if (!own_off && !(ans_word & 0x80000400u)) {
        if (dev_probe(r, pv, pm)) { set_status(r, L1_ABORT, "probe"); return 0; }
    } else {
        size_t o = fit(slen(r->probe), sizeof(r->probe));
        fmt(r->probe + o, sizeof(r->probe) - o, ";dev=skipped");
    }

    /* 2. bring-up, one printed + guarded step at a time */
    if (guarded(r, ph_power, "power")) { set_status(r, L1_ABORT, "power"); return 0; }
    if (r->st != L1_OK) { step_failed(r, "power"); return 0; }
    if (!r->dev_probed && dev_probe(r, pv, pm)) { set_status(r, L1_ABORT, "probe"); return 0; }
    if (guarded(r, ph_cpu, "cpu")) { set_status(r, L1_ABORT, "cpu"); return 0; }
    if (r->st != L1_OK) { step_failed(r, "cpu"); return 0; }
    if (guarded(r, ph_start, "start")) set_status(r, L1_ABORT, "start");
    else if (r->st != L1_OK) step_failed(r, "start");
    if (r->st == L1_OK) {
        if (guarded(r, ph_nvme, "nvme")) set_status(r, L1_ABORT, "nvme");
        else if (r->st != L1_OK) step_failed(r, "nvme");
    }
    if (r->st == L1_OK) {
        if (guarded(r, ph_policy, "policy")) {
            set_status(r, L1_ABORT, "policy");
        } else if (r->res.status != L1_OK) {
            set_status(r, r->res.status, 0);
            char w[24];
            switch (r->res.status) {
            case L1_GPT: scpy(w, sizeof(w), gpt_why(r->res.detail)); break;
            case L1_EXT4: case L1_NOT_FOUND: case L1_NO_CONF: case L1_NVME_READ:
                scpy(w, sizeof(w), l1_ext4_detail_name(r->res.detail)); break;
            case L1_IMAGE_HEADER: scpy(w, sizeof(w), hdr_why(r->res.detail)); break;
            case L1_BAD_CONF: fmt(w, sizeof(w), "e%u", (unsigned)(r->res.detail < 0 ? -r->res.detail : r->res.detail)); break;
            case L1_SIZE: case L1_DIGEST: case L1_TOO_LARGE: if (r->res.file[0]) scpy(w, sizeof(w), r->res.file); else w[0] = 0; break;
            default: if (r->res.detail) fmt(w, sizeof(w), "d%u", (unsigned)r->res.detail); else w[0] = 0; break;
            }
            scpy(r->why, sizeof(r->why), w);
        }
    }
    r->ms[0] = r->ans.st.ms_ans_boot;
    r->ms[1] = r->ans.st.ms_nvme_ready;
    r->ms[2] = r->ans.st.ms_read;

    /* 3. hand ANS back whatever happened: left running, or the emergency stop if that did not take */
    handback(r);

    /* 4. accept only after a clean hand-back and with no abort recorded anywhere (a retried status-2 SError is a
     * fault of earlier code: the payload it may have touched is not trusted); board.dtb must be a whole tree */
    if (r->st == L1_OK && r->res.status == L1_OK && r->aborts) set_status(r, L1_ABORT, "pending");
    if (r->st == L1_OK && r->res.status == L1_OK &&
        (r->res.dtb_bytes > a->dtb_window || h->fdt_check_full(a->dtb, (size_t)r->res.dtb_bytes) ||
         be32p((const uint8_t *)a->dtb + 4) != r->res.dtb_bytes)) {
        r->res.status = L1_BAD_DTB;
        set_status(r, L1_BAD_DTB, "board.dtb");
    }
    if (r->st == L1_OK && r->res.status == L1_OK) {
        r->ssd = 1;
        a->ssd = 1;
        a->image_bytes = r->res.image_bytes;
        a->image_memory = r->res.image_memory;
        a->initrd_bytes = r->res.initrd_bytes;
        a->dtb_bytes = r->res.dtb_bytes;
        scpy(a->entry, sizeof(a->entry), r->res.entry);
        scpy(a->kver, sizeof(a->kver), r->res.kver);
        scpy(a->cmdline, sizeof(a->cmdline), r->res.cmdline);
    }
    return 0;
}

/* Failure only: what the probe read and the tail of the step log (the full log is m4,level1-log) */
static void failure_details(struct l1_rec *r)
{
    const struct l1_host *fh = r->h;
    char b[268];
    if (r->probe[0]) { fmt(b, sizeof(b), "L1 probe %s", r->probe); say(r, b); }
    unsigned from = r->log_n > 288 ? r->log_n - 288 : 0;
    for (unsigned o = from; o < r->log_n; o += 96) {
        unsigned n = r->log_n - o < 96 ? r->log_n - o : 96;
        fmt(b, sizeof(b), "L1 log %.*s", (int)n, r->log + o);
        say(r, b);
    }
}

static void final_line(struct l1_rec *r)
{
    const struct l1_host *fh = r->h;
    char b[120];
    r->ms[3] = r->verify_ms;
    r->ms[5] = (uint32_t)((CNT() - r->attempt_t0_ns) / 1000000); /* the SSD attempt itself */
    if (!r->ssd) failure_details(r);
    if (r->ssd) fmt(b, sizeof(b), "L1 entry %s @%u", r->res.entry, r->ms[5]);
    else fmt(b, sizeof(b), "L1 rescue %s%s%s @%u", l1_status_name(r->st), r->why[0] ? ":" : "", r->why, r->ms[5]);
    say(r, b);
    log_add(r, r->ssd ? "entry" : "rescue", 0, 0);
    if ((uint64_t)r->ms[5] * 1000ull > ATTEMPT_BUDGET_US && !r->ssd) log_add(r, "over-budget", r->ms[5], 1);
}

void l1_attempt(const struct l1_host *h, struct l1_attempt_args *a)
{
    a->ssd = 0; a->image_bytes = a->image_memory = a->initrd_bytes = a->dtb_bytes = 0; a->state = 0;
    uint64_t t0 = CNT();
    struct l1_rec *r = h->memalign(16384, (sizeof(*r) + 16383) & ~(size_t)16383);
    if (!r) { if (h->line) h->line("L1 embedded no-memory"); return; } /* publish() reports "no-memory" */
    for (size_t i = 0; i < sizeof(*r); i++) ((uint8_t *)r)[i] = 0;
    a->state = r;
    r->h = h;
    r->a = a;
    r->win[0] = a->kernel; r->win[1] = a->kernel_window; r->win[2] = a->initrd; r->win[3] = a->initrd_window;
    r->attempt_t0_ns = t0;
    r->t0_ns = h->loader_t0_ns && h->loader_t0_ns <= t0 ? h->loader_t0_ns : t0;
    r->ran = 1;
    if (guarded(r, ph_whole, "attempt")) {
        /* a fault in the glue between the phase guards: nothing is trusted, the rescue set boots */
        a->ssd = 0; r->ssd = 0;
        set_status(r, L1_ABORT, "attempt");
    }
    handback(r);   /* no-op when ph_whole already did it */
    publish_live(r);
    if (a->ssd && !r->ssd) a->ssd = 0;
    final_line(r);
}

/* The shim's whole-attempt guard fired (or refused): the record may be missing or half way. */
void l1_recover(const struct l1_host *h, struct l1_attempt_args *a)
{
    struct l1_rec *r = a->state;
    const struct l1_host *fh = h;
    char b[132];
    a->ssd = 0;
    fmt(b, sizeof(b), "L1 abort entry s=%lu v=%lu esr=%lx elr=%lx far=%lx", a->guard.status, a->guard.vector, a->guard.esr,
        a->guard.elr, a->guard.far);
    if (h->line) h->line(b);
    if (!r) {
        /* the fault came before the record existed: make a small one so publish() can say what happened */
        r = h->memalign(16384, (sizeof(*r) + 16383) & ~(size_t)16383);
        if (r) {
            for (size_t i = 0; i < sizeof(*r); i++) ((uint8_t *)r)[i] = 0;
            r->h = h; r->a = a; r->ran = 1;
            r->win[0] = a->kernel; r->win[1] = a->kernel_window; r->win[2] = a->initrd; r->win[3] = a->initrd_window;
            r->attempt_t0_ns = r->t0_ns = CNT();
            a->state = r;
        }
    }
    if (!r) { if (h->line) h->line("L1 embedded abort:entry"); return; }
    r->ssd = 0;
    if (!r->aborts++) scpy(r->abort_txt, sizeof(r->abort_txt), b + 9);
    set_status(r, L1_ABORT, "entry");
    handback(r);
    publish_live(r);
    final_line(r);
}

/* Text for the boot-stop screen: the final status, the first abort, what the probe read. */
size_t l1_describe(const struct l1_host *h, void *state, char *buf, size_t cap)
{
    struct l1_rec *r = state;
    const struct l1_host *fh = h;
    (void)h;
    if (!cap) return 0;
    if (!r) return (size_t)fmt(buf, cap, "L1: no record");
    size_t o = (size_t)fmt(buf, cap, "L1 %s:%s aborts=%u first=%s\nL1 probe %s", r->ssd ? "ssd" : l1_status_name(r->st),
                           r->why, r->aborts, r->abort_txt, r->probe);
    return o < cap ? o : cap - 1;
}

/* ------------------------------------------------------------ DT publication */

static int set_str(const struct l1_host *h, void *dt, int n, const char *k, const char *v)
{
    return h->fdt_setprop(dt, n, k, v, (int)slen(v) + 1);
}

static const char *reason_of(const struct l1_rec *r)
{
    if (!r) return "ssd-unavailable";
    if (r->ssd) return "default";
    if (!r->ran) return "loader-disabled";
    if (r->st == L1_DISABLED) return r->why[0] == 'e' ? "loader-disabled" : "ssd-unavailable"; /* "embedded-variant" vs cntfrq/no-cache */
    return l1_reason(r->st, r->res.detail);
}

/* the policy step itself returned this status, and every one of these is decided after l1_gpt_find_p4() accepted the root */
static int gpt_verified_rescue(const struct l1_rec *r)
{
    if (r->st != r->res.status) return 0;
    switch (r->res.status) {
    case L1_PENDING: case L1_EXT4: case L1_NO_CONF: case L1_BAD_CONF: case L1_BAD_ABI: case L1_NO_ENTRY: case L1_NOT_FOUND:
    case L1_SIZE: case L1_DIGEST: case L1_TOO_LARGE: case L1_IMAGE_HEADER: case L1_BAD_DTB: case L1_SHA_IMAGE: case L1_SHA_INITRD:
    case L1_NVME_READ:
        return 1;
    default:
        return 0;
    }
}

int l1_publish(const struct l1_host *h, void *state, void *dt, size_t capacity)
{
    struct l1_rec *r = state;
    const struct l1_host *fh = h;
    char detail[72];
    if (!dt || h->fdt_check_full(dt, capacity)) return -1;
    if (!r && h->refused) fmt(detail, sizeof(detail), "guard-refused-%u", h->refused);
    else if (!r) fmt(detail, sizeof(detail), "%s", h->level1_capable ? "no-memory" : "disabled");
    else if (r->ssd) detail[0] = 0;
    else fmt(detail, sizeof(detail), "%s%s%s", l1_status_name(r->st), r->why[0] ? ":" : "", r->why);
    detail[63] = 0;
    if (h->fdt_open_into(dt, dt, (int)capacity)) return -2;
    int c = h->fdt_path_offset(dt, "/chosen");
    if (c < 0) return -3;
    uint8_t abi[4];
    be32(abi, 1);
    int rc = 0;
    rc |= h->fdt_setprop(dt, c, "m4,loader-abi", abi, 4);
    rc |= set_str(h, dt, c, "m4,loader-version", h->loader_version ? h->loader_version : "unknown");
    rc |= set_str(h, dt, c, "m4,boot-source", r && r->ssd ? "entry" : "rescue");
    rc |= set_str(h, dt, c, "m4,boot-entry", r && r->ssd ? r->res.entry : "rescue");
    rc |= set_str(h, dt, c, "m4,boot-reason", reason_of(r));
    if (detail[0]) rc |= set_str(h, dt, c, "m4,boot-detail", detail);
    /* The partitions this boot checked against the GPT (exact range and PARTUUID): Linux's NVMe write guard takes its
     * ranges from here (<first_hi first_lo end_hi end_lo> per partition, 4 KiB units, end exclusive), the root first. */
    /* A rescue boot whose cause was found after the GPT check (boot-pending, a dirty ext4 left by a power cut, a bad
     * loader.conf or entry) gets the same verified ranges: the rescue system must be able to replay the journal, remove
     * boot-pending or `m4-boot select` another entry. Without them Linux falls back to the board DT's ranges, which only
     * describe the disk of the unit the DT was written on. A GPT refusal or an ANS/NVMe failure before it publishes nothing. */
    if (r && (r->ssd || gpt_verified_rescue(r))) {
        uint8_t wr[16 * (1 + L1_EXTRA_N)];
        unsigned n = 0;
        be32(wr, (uint32_t)(L1_P4_FIRST >> 32)); be32(wr + 4, (uint32_t)L1_P4_FIRST);
        be32(wr + 8, (uint32_t)(L1_P4_END >> 32)); be32(wr + 12, (uint32_t)L1_P4_END);
        n = 16;
#if L1_EXTRA_N
        static const uint64_t extra[L1_EXTRA_N][2] = L1_EXTRA_RANGES;
        for (unsigned x = 0; x < L1_EXTRA_N; x++) {
            if (!(r->res.extra_ok & (1u << x))) continue;
            be32(wr + n, (uint32_t)(extra[x][0] >> 32)); be32(wr + n + 4, (uint32_t)extra[x][0]);
            be32(wr + n + 8, (uint32_t)(extra[x][1] >> 32)); be32(wr + n + 12, (uint32_t)extra[x][1]);
            n += 16;
        }
#endif
        rc |= h->fdt_setprop(dt, c, "m4,write-lba-range", wr, (int)n);
    }
    if (r && r->ans.st.left && r->ans.live_n) {
        rc |= h->fdt_setprop(dt, c, "m4,ans-live", r->ans.live_eps, r->ans.live_n);
        /* the syslog buffer the firmware keeps writing to: <iova_hi iova_lo size n_entries msg_size>, only when it asked for
         * a buffer and sent syslog init; Linux maps it (apple_rtkit_reuse_syslog_buffer) instead of dropping 46 messages a boot */
        const struct l1_ans *a = &r->ans;
        if (a->sys_buf && a->sys_size && a->sys_entries && a->sys_msg_size) {
            uint8_t sl[20];
            be32(sl, (uint32_t)(a->sys_buf >> 32)); be32(sl + 4, (uint32_t)a->sys_buf);
            be32(sl + 8, (uint32_t)a->sys_size); be32(sl + 12, a->sys_entries); be32(sl + 16, a->sys_msg_size);
            rc |= h->fdt_setprop(dt, c, "m4,ans-syslog", sl, sizeof(sl));
        }
    }
    int pk = h->fdt_pack(dt); /* always leave the tree packed, even when a property did not fit */
    if (rc || pk || h->fdt_check_full(dt, capacity)) return -4;
    return 0;
}
