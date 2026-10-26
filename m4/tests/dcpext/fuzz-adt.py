#!/usr/bin/env python3 -I
"""usage: fuzz-adt.py IN.adt OUTDIR COUNT -- COUNT ADTs, each with 1-4 random bytes of the dcpext properties changed
(the dcpextN node's reg and the nub's segment-names, segment-ranges, running), fixed seed."""
import random, struct, sys
src, out, count = sys.argv[1], sys.argv[2], int(sys.argv[3])
orig = open(src, 'rb').read()
spans = []
def walk(off, parent):
    np, nc = struct.unpack_from('<II', orig, off); o = off + 8; pr = {}
    for _ in range(np):
        k = orig[o:o+32].split(b'\0')[0].decode(); sz = struct.unpack_from('<I', orig, o+32)[0] & 0x7fffffff
        pr[k] = (o + 36, sz); o += 36 + ((sz + 3) & ~3)
    nm = orig[pr['name'][0]:pr['name'][0]+32].split(b'\0')[0].decode()
    if nm.startswith('iop-dcpext'):
        for k in ('segment-names', 'segment-ranges', 'running'): spans.append(pr[k])
    elif nm in ('dcpext0', 'dcpext1'): spans.append(pr['reg'])
    elif nm == 'chosen': spans.append(pr['firmware-version'])
    for _ in range(nc): o = walk(o, nm)
    return o
walk(0, None)
rnd = random.Random(0x4d34)
for i in range(count):
    b = bytearray(orig)
    for _ in range(rnd.randint(1, 4)):
        off, sz = rnd.choice(spans)
        b[off + rnd.randrange(max(sz, 1))] = rnd.randrange(256)
    open(f'{out}/fuzz-{i:04d}.adt', 'wb').write(b)
