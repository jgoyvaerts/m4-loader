/* SPDX-License-Identifier: MIT */
/* Sequential direct-loader read-only capture. No DART init/control API. */
#include "display-handoff.h"
#include "fixups.h"
#include "j616s_usb.h"
#include "adt.h"
#include "utils.h"
#include <string.h>
#include "boot-profile.h"
#include "media-probe.h"
#ifndef M4_DISPLAY_TEST
#include "loader-cache.h"
#include "loader-speed.h"
#endif
#if !defined(M4_DCP_STAGE) && defined(M4_BRINGUP_EXPORT)
#include "bringup-handoff.h"
#include "usb-c-metadata.h"

static const void *m4_bringup_adt_get(void *ctx, const char *path, const char *key, unsigned *length)
{
    int node = adt_path_offset(ctx, path);
    return node < 0 ? NULL : adt_getprop(ctx, node, key, length);
}
#endif


static struct {
    struct m4_display_handoff handoff;
    struct m4_dart_capture metadata;
    struct m4_dt_workspace workspace;
    struct m4_dt_range owners[M4_MAX_RESERVED];
    uint64_t start,last;
    unsigned attempted,owner_count;
} capture __attribute__((aligned(16384)));
static unsigned char publish_tree[65536] __attribute__((aligned(16384)));
extern int m4_media_handoff_write(void *,size_t,void *,size_t);
_Static_assert(M4_DP_MAX_OWNERS==M4_MAX_RESERVED,"shared physical owner capacity");

#ifdef M4_DISPLAY_TEST
extern uint64_t m4_display_test_register(const char *);
#define REGISTER(r) m4_display_test_register(#r)
#define SCTLR_ADMITTED(v) (!((v)&0x1005))
#else
#define REGISTER mrs
/* Uncached as before, or inside the loader's own cached window. */
#define SCTLR_ADMITTED(v) m4_cache_sctlr_admitted(v)
#endif

uint64_t m4_dart_counter(void) { return REGISTER(CNTPCT_EL0); }
uint64_t m4_dt_counter(void) { return m4_dart_counter(); }
uint32_t m4_dart_read32(uint64_t a) { return read32(a); }
uint32_t m4_dt_read32(uint64_t a) { return read32(a); }
uint64_t m4_dt_read64(uint64_t a) { return read64(a); }
static int progress(void)
{
    uint64_t now=m4_dart_counter();
    /* Ownership/ADT validation is bounded RAM work, not a device read. The
     * collector starts this shared register/table budget immediately before
     * its first read; keep capture.start for truthful total elapsed time. */
    if(now<capture.last || now<capture.metadata.begin ||
       now-capture.metadata.begin>UINT64_C(2000000000))return -1;
    capture.last=now; return 0; /* An in-flight MMIO load has no timeout. */
}
int m4_dart_before(unsigned i,const struct m4_dart_read *r)
{ (void)i;(void)r;return progress(); }
void m4_dart_after(unsigned i,const struct m4_dart_read *r)
{ (void)i;(void)r; }
int m4_dt_progress(unsigned pass,uint64_t address)
{ (void)pass;(void)address;return progress(); }

int m4_display_handoff_capture(const struct m4_memory *memory,struct m4_range loader,
                               struct m4_range arena,uint64_t framebuffer,uint64_t bytes)
{
    if(capture.attempted++)return -20;
    struct m4_display_handoff *h=&capture.handoff;
    capture.start=capture.last=m4_dart_counter();
    m4_boot_profile_ticks[M4_PROFILE_DISPLAY_BEGIN]=capture.start;
    h->status=-211;
    if(!memory || !m4_contains(loader,(struct m4_range){(uintptr_t)memory,sizeof(*memory)}))return h->status;
    h->status=-212;
    if(REGISTER(CurrentEL)!=8 || !SCTLR_ADMITTED(REGISTER(SCTLR_EL2)) || (REGISTER(HCR_EL2)&1) ||
       (REGISTER(MPIDR_EL1)&UINT64_C(0xff00ffffff))!=0x10100 ||
       REGISTER(CNTFRQ_EL0)!=UINT64_C(1000000000))return h->status;
    h->status=-213;
    if(!m4_contains(loader,(struct m4_range){(uintptr_t)&capture,sizeof(capture)}) ||
       !m4_contains(loader,(struct m4_range){(uintptr_t)publish_tree,sizeof(publish_tree)}) ||
       m4_overlaps(arena,(struct m4_range){(uintptr_t)&capture,sizeof(capture)}) ||
       m4_overlaps(arena,(struct m4_range){(uintptr_t)publish_tree,sizeof(publish_tree)}))return h->status;
    h->status=-214;
    if(memory->reserved_count>M4_MAX_RESERVED || !memory->reserved_count)return h->status;
    h->status=-215;
    if(m4_direct_desktop_ownership(memory,loader,arena))return h->status;
    m4_boot_profile_mark(M4_PROFILE_DISPLAY_OWNERSHIP_READY);
    h->dram=(struct m4_dt_range){memory->dram.base,memory->dram.size};
    h->framebuffer=framebuffer; h->framebuffer_bytes=bytes; h->frequency=REGISTER(CNTFRQ_EL0);
    capture.owner_count=memory->reserved_count;
    for(unsigned i=0;i<capture.owner_count;i++)
        capture.owners[i]=(struct m4_dt_range){memory->reserved[i].base,memory->reserved[i].size};
    h->status=-22;
    if(m4_dart_prepare(&capture.metadata,adt,adt_get_size()) ||
       m4_dart_collect(&capture.metadata))return -22;
    m4_boot_profile_mark(M4_PROFILE_DISPLAY_METADATA_READY);
    h->metadata_completed=capture.metadata.completed; h->metadata_phase=capture.metadata.phase;
    h->status=-23;
    if(m4_dt_prepare(&capture.metadata,h->input,h->segment))return -23;
    m4_boot_profile_mark(M4_PROFILE_DISPLAY_TABLE_ADT_READY);
    for(unsigned i=0;i<3;i++) {
        struct m4_dt_input *d=&h->input[i];
        const struct m4_dt_range excluded[]={
            {loader.base,loader.size},{arena.base,arena.size},
            {(uintptr_t)adt,adt_get_size()},{framebuffer,bytes}};
        d->excluded_count=4; memcpy(d->excluded,excluded,sizeof(excluded));
        h->active=i+1; h->status=-24;
        if(m4_dt_collect(d,h->segment,&capture.workspace,&h->result[i]))return -24;
        h->status=-25;
        if(m4_display_handoff_plan(d,&h->result[i],&capture.workspace,capture.owners,
                                   capture.owner_count,(struct m4_dt_range){framebuffer,bytes},&h->plan[i]))return -25;
        h->completed=i+1;
        m4_boot_profile_mark((enum m4_boot_profile_stage)(M4_PROFILE_DISPLAY_STREAM0_FINISHED+i));
    }
    h->status=0; return 0;
}
int m4_display_handoff_publish(void *tree,size_t capacity,void *scratch,size_t scratch_capacity,
                               int guard,const struct j616s_call_result *r)
{
    if(!r)return -30;
    /* The loader passes its 256 KiB DT-window scratch; NULL keeps the 64 KiB one. */
    if(!scratch) { scratch=publish_tree; scratch_capacity=sizeof(publish_tree); }
    struct m4_display_handoff *h=&capture.handoff;
    h->metadata_completed=capture.metadata.completed; h->metadata_phase=capture.metadata.phase;
    h->capture_error=capture.metadata.error;
    if(h->active>=1 && h->active<=3) {
        const struct m4_dt_result *t=&h->result[h->active-1];
        h->capture_phase=t->phase;h->capture_error=t->error;h->active_address=t->active_address;
    }
    uint64_t now=m4_dart_counter();
    h->elapsed=now>=capture.start?now-capture.start:UINT64_MAX;
    if(guard || r->status) { h->status=-901;h->esr=r->esr;h->far=r->far; }
    /* The guarded callback returns C int in w0, zero-extended into its u64
     * result. Restore signed int semantics before serializing the refusal. */
    else if(r->value)h->status=(int32_t)(uint32_t)r->value;
    (void)scratch; (void)scratch_capacity;
    /* rev 11: no m4-display-handoff / m4-media-handoff records. An accepted capture adopts the DCP placeholders of
     * board.dtb, a refused one deletes them; the media groups and AOP segments follow. */
    int ret=m4_dcp_fixup(tree,capacity,h);
    m4_boot_profile_display_rc=ret;
    m4_boot_profile_mark(M4_PROFILE_DISPLAY_WIRE_FINISHED);
    int media=m4_media_fixups(tree,capacity);
    m4_boot_profile_media_rc=media;
    m4_boot_profile_mark(M4_PROFILE_DISPLAY_MEDIA_FINISHED);
    int gpu=m4_gpu_fixup(tree,capacity);
    int ext=m4_dcpext_fixup(tree,capacity);
    if(ret>0)ret=0; /* DCP left disabled: not an error of the publication */
    if(!ret)ret=media;
    if(!ret&&ext<0)ret=ext; /* a dcpext unit left without firmware memory (1) is not an error either */
    if(!ret&&gpu<0)ret=gpu; /* another boot firmware (1) only leaves the GPU without a version */
    return ret;
}
