#!/usr/bin/env python3
"""Recompute the USB contract id of src/standalone/usb-contract-id.h from an ADT and compare.

  python3 -I m4/tools/usb-contract-id.py ADT.bin src/standalone/usb-contract-id.h

The id (`j616s-usb2-right-v1:<sha256>`) is the sha256 of the JSON of 28 selected ADT properties (model, compatible, chip-id,
board-id, firmware strings, /arm-io ranges, pmgr ps-groups, usb-drd2, atc-phy2, dart-usb2, mapper-usb2) plus the sha256 of
pmgr `reg` and `devices`, as the retained generator make-contract.py (2026-10-05, round 4 USB) computed it. All of that is
per-model and per-firmware, none per-unit. After a macOS/iBoot update, a different id here means the USB gates of
usb-adt-gates.h no longer match that Mac's ADT: the loader then writes `m4,usb2-right-contract = refused-current-adt`
(USB stays disabled; the boot continues). macOS 26.6.2 installed from Recovery (25G241) reports /chosen firmware-version
mBoot-18000.161.10.701.1, which the loader accepts as 25G83's mBoot-18000.161.10: the id is computed with the latter.
Exit 0 when the id matches.
"""
import hashlib, json, os, re, sys

src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'gates-unit.py')).read().replace('\nmain()\n', '\n')
ns = {}
exec(compile(src, 'gates-unit.py', 'exec'), ns)
sha = lambda b: hashlib.sha256(b).hexdigest()
adt = ns['parse'](open(sys.argv[1], 'rb').read())
want = re.search(r'usb2-right-v1:([0-9a-f]+)', open(sys.argv[2]).read()).group(1)
selected = {'/': ['compatible', 'target-type', 'model'],
            '/chosen': ['chip-id', 'board-id', 'firmware-version', 'system-firmware-version'],
            '/arm-io': ['ranges'], '/arm-io/pmgr': ['ps-groups'],
            '/arm-io/usb-drd2': ['compatible', 'reg', 'clock-gates', 'atc-phy-parent', 'iommu-parent', 'port-number'],
            '/arm-io/atc-phy2': ['compatible', 'reg', 'clock-gates', 'port-number'],
            '/arm-io/dart-usb2': ['compatible', 'reg', 'clock-gates'], '/arm-io/dart-usb2/mapper-usb2': ['reg']}
RECOVERY_OS_FIRMWARE = b'mBoot-18000.161.10.701.1'.ljust(256, b'\0')
if adt['/chosen']['firmware-version'] == RECOVERY_OS_FIRMWARE:
    adt['/chosen']['firmware-version'] = b'mBoot-18000.161.10'.ljust(256, b'\0')
records = [dict(path=p, key=k, hex=adt[p][k].hex()) for p, keys in selected.items() for k in keys]
pmgr = {key: sha(adt['/arm-io/pmgr'][key]) for key in ['reg', 'devices']}
got = sha(json.dumps(dict(properties=records, existing_pmgr_gates=pmgr), sort_keys=True).encode())
print('contract id %s' % ('MATCHES' if got == want else 'DIFFERS (%s... vs %s...)' % (got[:8], want[:8])))
sys.exit(0 if got == want else 1)
