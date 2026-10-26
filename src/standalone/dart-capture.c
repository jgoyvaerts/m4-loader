/* Current-boot, read-only T6040 DART metadata. No mapping or write API. */
#include "dart-capture.h"
#include "adt.h"
#include <string.h>
#include "ram.h"
/* Bounded structural counterpart of the retained tools/m4_adt.py reader.
 * Keep the existing parser for lookups only AFTER the full walk. The Rust
 * FFI uses global adt and does not itself bound every property traversal. */
int m4_dart_adt_bounds(const void *raw,uint32_t bytes){
 if(!raw||((uintptr_t)raw&3)||bytes<8||bytes>8*1024*1024||(uintptr_t)raw>UINT64_MAX-bytes)return -1;
 /* One primary validates ADT; this scratch retains no admission across calls. */
 static uint32_t names[4096] __attribute__((aligned(8)));
 const unsigned char *data=raw;uint32_t left[33]={1},depth=0,pos=0,nodes=0;
 for(;;){
  if(!left[depth]){if(!depth)break;depth--;continue;}
  left[depth]--;
  if(++nodes>10000||bytes-pos<8)return -2;
  uint32_t props,children;memcpy(&props,data+pos,4);memcpy(&children,data+pos+4,4);pos+=8;
  if(!props||props>2048||children>2048)return -3;
  unsigned slots=1;while(slots<2*props)slots<<=1;
  m4_ram_zero(names,slots*sizeof(*names));
  unsigned has_name=0;
  for(unsigned i=0;i<props;i++){
   if(bytes-pos<36)return -4;
   const unsigned char *key=data+pos;unsigned end=0;uint32_t hash=2166136261U;
   while(end<32&&key[end]){
    if(key[end]<32||key[end]>126)return -5;
    hash=(hash^key[end])*16777619U;end++;
   }
   if(!end||end==32)return -5;
   uint32_t length;memcpy(&length,key+32,4);
   /* No masked flag bits: this exact Rust parser consumes the raw size.
    * Its supported size is <1MiB; the retained native tree has no flags. */
   if(length>=1024*1024)return -6;
   uint32_t padded=(length+3)&~3U;
   if(padded>bytes-pos-36)return -6;
   unsigned slot=hash&(slots-1);
   /* Hash only selects candidates; collisions still compare complete names. */
   while(names[slot]){
    if(!strcmp((const char *)data+names[slot],(const char *)key))return -7;
    slot=(slot+1)&(slots-1);
   }
   names[slot]=pos;
   if(!strcmp((const char *)key,"name")){
    const unsigned char *name=key+36;unsigned n=0;
    while(n<length&&name[n]){if(name[n]<32||name[n]>126)return -8;n++;}
    if(!n||n==length)return -8;has_name=1;
   }
   pos+=36+padded;
  }
  if(!has_name)return -8;
  if(children){if(depth==32)return -9;left[++depth]=children;}
 }
 for(;pos<bytes;pos++)if(data[pos])return -10;
 return 0;
}
struct adt_view {const void *data;uint32_t bytes;};
static const void *property(const struct adt_view *v,int node,const char *key,uint32_t *bytes){
 const void*p=adt_getprop(v->data,node,key,bytes);uintptr_t start=(uintptr_t)v->data,address=(uintptr_t)p;
 if(!p||address<start||*bytes>v->bytes||address-start>v->bytes-*bytes){*bytes=0;return 0;}
 return p;
}
static const void *exact(const struct adt_view *tree,int node,const char *key,uint32_t bytes){
 uint32_t n=0;const void *p=property(tree,node,key,&n);return p&&n==bytes?p:0;
}
static int word(const struct adt_view *tree,int node,const char *key,uint32_t *out){
 const void*p=exact(tree,node,key,4);if(!p)return -1;memcpy(out,p,4);return 0;
}
static int equal(const struct adt_view *tree,int node,const char *key,const void *value,uint32_t bytes){
 const void*p=exact(tree,node,key,bytes);return p&&!memcmp(p,value,bytes);
}
static int cells(const struct adt_view *tree,int node){uint32_t a=0,s=0;
 return !word(tree,node,"#address-cells",&a)&&!word(tree,node,"#size-cells",&s)&&a==2&&s==2;
}
static int resource(const struct adt_view *tree,int arm,const char *path,const uint64_t *reg,
                    uint32_t bytes,uint64_t expected,uint64_t *base,uint64_t *span){
 int trace[8]={0},node=adt_path_offset_trace(tree->data,path,trace);
 if(node<0||!equal(tree,node,"compatible","dart,t8110",11)||!equal(tree,node,"reg",reg,bytes))return -1;
 uint32_t n=0;const unsigned char *ranges=property(tree,arm,"ranges",&n);
 if(!ranges||!n||n>4096||n%24||!reg[1]||reg[0]>UINT64_MAX-reg[1])return -1;
 unsigned matches=0;uint64_t translated=0;
 for(unsigned i=0;i<n;i+=24){uint64_t r[3];memcpy(r,ranges+i,24);
  if(!r[2]||r[0]>UINT64_MAX-r[2]||r[1]>UINT64_MAX-r[2])return -1;
  if(reg[0]>=r[0]&&reg[0]+reg[1]<=r[0]+r[2]){translated=r[1]+reg[0]-r[0];matches++;}
 }
 if(matches!=1||translated!=expected||adt_get_reg(tree->data,trace,"reg",0,base,span)||
    *base!=translated||*span!=reg[1])return -1;
 return 0;
}
static int mapper(const struct adt_view *tree,const char *path,uint32_t sid,uint32_t *handle){
 int node=adt_path_offset(tree->data,path);uint32_t reg=UINT32_MAX;
 return node<0||!equal(tree,node,"compatible","iommu-mapper",13)||word(tree,node,"reg",&reg)||reg!=sid||
        word(tree,node,"AAPL,phandle",handle)||!*handle ? -1:0;
}
static int refuse(struct m4_dart_capture *c,int error){c->error=error;c->phase=DART_REFUSED;return error;}
int m4_dart_prepare(struct m4_dart_capture *c,const void *raw,uint32_t bytes){
 if(!c)return -1;
 if(c->phase)return -2; /* Never clear/reuse an interrupted or completed capture. */
 c->phase=DART_VALIDATING;
 if(!raw||raw!=adt||bytes!=adt_get_size()||!bytes||bytes>8*1024*1024||(uintptr_t)raw>UINT64_MAX-bytes)return refuse(c,-3);
 if(m4_dart_adt_bounds(raw,bytes)||adt_check_header(raw))return refuse(c,-14);
 const struct adt_view view={raw,bytes};const struct adt_view *tree=&view;
 c->adt_address=(uintptr_t)raw;c->adt_bytes=bytes;
 int chosen=adt_path_offset(tree->data,"/chosen"),arm=adt_path_offset(tree->data,"/arm-io");uint32_t chip=0;
 if(chosen<0||arm<0||!equal(tree,0,"target-type","J616s",6)||!equal(tree,0,"model","Mac16,7",8)||
    word(tree,chosen,"chip-id",&chip)||chip!=0x6040||!cells(tree,0)||!cells(tree,arm)||
    !equal(tree,arm,"compatible","arm-io,t6041",13))return refuse(c,-4);
 /* The observed OS-firmware field is padded to 256 bytes; every byte matters. */
 static const unsigned char os_firmware[256]="mBoot-18000.161.10";
 static const unsigned char os_firmware_recovery[256]="mBoot-18000.161.10.701.1"; /* 26.6.2 installed from Recovery (25G241) */
 if((!equal(tree,chosen,"firmware-version",os_firmware,sizeof(os_firmware))&&
     !equal(tree,chosen,"firmware-version",os_firmware_recovery,sizeof(os_firmware_recovery)))||
    !equal(tree,chosen,"system-firmware-version","mBoot-20457.1.29",17))return refuse(c,-5);
 static const uint64_t dcp_reg[]={UINT64_C(0x202340000),0x20000,UINT64_C(0x300578000),0x4000};
 static const uint64_t disp_reg[]={UINT64_C(0x202300000),0xc000,UINT64_C(0x202314000),0x4000,UINT64_C(0x300578000),0x4000};
 if(resource(tree,arm,"/arm-io/dart-dcp",dcp_reg,sizeof(dcp_reg),UINT64_C(0x402340000),&c->base[0],&c->span[0])||
    resource(tree,arm,"/arm-io/dart-disp0",disp_reg,sizeof(disp_reg),UINT64_C(0x402300000),&c->base[1],&c->span[1]))return refuse(c,-6);
 uint32_t h[3]={0};
 if(mapper(tree,"/arm-io/dart-dcp/mapper-dcp",23,&h[0])||
    mapper(tree,"/arm-io/dart-disp0/mapper-disp0",0,&h[1])||
    mapper(tree,"/arm-io/dart-disp0/mapper-disp0-piodma",16,&h[2])||
    h[0]==h[1]||h[0]==h[2]||h[1]==h[2])return refuse(c,-7);
 int dcp=adt_path_offset(tree->data,"/arm-io/dcp"),disp=adt_path_offset(tree->data,"/arm-io/disp0");
 if(dcp<0||disp<0||!equal(tree,dcp,"compatible","iop,ascwrap-v6",15)||
    !equal(tree,disp,"compatible","disp0,t604x",12)||
    !equal(tree,dcp,"iommu-parent",h,4)||!equal(tree,disp,"iommu-parent",h+1,8))return refuse(c,-8);
 for(unsigned i=0;i<3;i++)c->mapper[i]=h[i];
 /* Pinned apple-dart.c SHA 48afabd1...13645: five RO parameters/protect,
  * then T8110 TCR and TTBR for current ADT SID23, SID0 and SID16. */
 static const uint16_t offset[M4_DART_COUNT]={0,4,8,12,0x200,0x105c,0x145c,0,4,8,12,0x200,0x1000,0x1400,0x1040,0x1440};
 for(unsigned i=0;i<M4_DART_COUNT;i++){unsigned node=i>=7;
  if(offset[i]>c->span[node]-4||c->base[node]>UINT64_MAX-offset[i])return refuse(c,-9);
  c->read[i].address=c->base[node]+offset[i];
 }
 c->phase=DART_READY;return 0;
}
int m4_dart_collect(struct m4_dart_capture *c){
 if(!c||c->phase!=DART_READY)return -10;
 c->phase=DART_READING;c->begin=m4_dart_counter();
 for(unsigned i=0;i<M4_DART_COUNT;i++){
  /* T8110 reports its SID count in PARAMS4. Refuse a missing/out-of-range
   * stream bank before accessing TCR/TTBR, retaining the raw parameter words. */
  if(i==5||i==12){unsigned p=i==5?3:10,highest=i==5?23:16;
   unsigned count=c->read[p].value&0x1ff;
   if(count<=highest||count>256)return refuse(c,-11);
  }
  c->active=i;
  struct m4_dart_read *r=&c->read[i];r->before=m4_dart_counter();
  if(r->before<c->begin||(i&&r->before<c->read[i-1].after))return refuse(c,-12);
  if(m4_dart_before(i,r))return refuse(c,-13);
  /* Exactly one volatile 32-bit read. Synchronous abort abandons this whole
   * callback through the existing guard; never replay the faulting load. */
  r->value=m4_dart_read32(r->address);
  r->after=m4_dart_counter();r->done=1;c->completed=i+1;
  m4_dart_after(i,r);
  if(r->after<r->before)return refuse(c,-12);
 }
 c->end=m4_dart_counter();if(c->end<c->read[M4_DART_COUNT-1].after)return refuse(c,-12);
 c->phase=DART_COMPLETE;return 0;
}
int m4_dart_photo_hold(uint64_t hz){
 if(hz<1000000||hz>1000000000)return -1;
 uint64_t start=m4_dart_counter(),last=start,quantum=hz/100;
 for(unsigned slice=0;slice<3001;slice++){
  if(last-start>=hz*30)return 0;
  unsigned spin;
  for(spin=0;spin<100000000;spin++){
   uint64_t now=m4_dart_counter();if(now<last)return -2;
   if(now-last>=quantum){last=now;break;}
  }
  if(spin==100000000)return -3;
 }
 return -4;
}
