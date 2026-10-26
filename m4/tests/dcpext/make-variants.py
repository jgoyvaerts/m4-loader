#!/usr/bin/env python3 -I
"""usage: make-variants.py IN.adt OUTDIR -- writes ADT variants of one real ADT for the dcpext cases (same sizes, in place):
fw-unknown (/chosen firmware-version), not-running (dcpext1 nub running = 0), names (segment-names), remap (remap of
dcpext0 heap not in the 1<<40 window), unaligned (dcpext1 data phys not 16 KiB aligned), overlap (dcpext0 data inside text), phys-top (dcpext0 heap phys in the last 16 KiB: phys + size wraps)."""
import struct, sys
src, out = sys.argv[1], sys.argv[2]
orig = open(src, 'rb').read()
def find(b, want):
    def walk(off, parent):
        np, nc = struct.unpack_from('<II', b, off); o = off + 8; pr = {}
        for _ in range(np):
            k = bytes(b[o:o+32]).split(b'\0')[0].decode(); sz = struct.unpack_from('<I', b, o+32)[0] & 0x7fffffff
            pr[k] = (o + 36, sz); o += 36 + ((sz + 3) & ~3)
        nm = bytes(b[pr['name'][0]:pr['name'][0]+32]).split(b'\0')[0].decode()
        path = nm if parent is None else (parent.rstrip('/') + '/' + nm if parent != '/' else '/' + nm)
        if path.endswith(want[0]) and want[1] in pr: res.append(pr[want[1]])
        for _ in range(nc): o = walk(o, path)
        return o
    res = []; walk(0, None); return res
def variant(name, node, prop, fn):
    b = bytearray(orig)
    hits = find(b, (node, prop)); assert len(hits) == 1, (name, node, prop, len(hits))
    off, sz = hits[0]; fn(b, off, sz)
    open(f'{out}/{name}.adt', 'wb').write(b)
def setb(i, v):
    def f(b, off, sz): b[off + i] = v
    return f
def set64(i, v):
    def f(b, off, sz): struct.pack_into('<Q', b, off + i, v)
    return f
variant('fw-unknown', 'chosen', 'firmware-version', setb(8, ord('X')))
variant('not-running', 'iop-dcpext1-nub', 'running', setb(0, 0))
variant('names', 'iop-dcpext0-nub', 'segment-names', setb(2, ord('x')))
variant('remap', 'iop-dcpext0-nub', 'segment-ranges', set64(3 * 32 + 16, 0x20000000000 + 0x9a68000))
variant('unaligned', 'iop-dcpext1-nub', 'segment-ranges', lambda b, off, sz: struct.pack_into('<Q', b, off + 32, struct.unpack_from('<Q', b, off + 32)[0] + 0x1000))
variant('overlap', 'iop-dcpext0-nub', 'segment-ranges', lambda b, off, sz: struct.pack_into('<Q', b, off + 32, struct.unpack_from('<Q', b, off)[0] + 0x1000))
variant('phys-top', 'iop-dcpext0-nub', 'segment-ranges', set64(3 * 32, 0xffffffffffffc000))
