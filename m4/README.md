# m4/: everything of ours that is not m1n1 layout

The repo keeps the m1n1 layout at the root (`src/`, `sysinc/`, `rust/`, `Makefile`, the linker scripts, `data/`, `font/`).
Our loader code is in `src/standalone/` and the `j616s_*` files of `src/`. This directory holds the rest:

| Path | What |
|---|---|
| `LAYOUT.md`, `BOOT-ABI.md` | the loader/kernel layout contract and the Boot ABI v1 (as of N1; the Linux side keeps the current BOOT-ABI.md) |
| `build.sh`, `env.sh`, `tools/build-rev11.py`, `tools/loader-layout.py` | the build (see the header of `build.sh`) |
| `check.sh`, `objhash.sh`, `objcmp.sh`, `tests/`, `reference/` | the tests and the rev 13 / rev 14 reference hashes |
| `tests/dcpext/` | `m4_dcpext_fixup()` (dcpext0/1 firmware memory from the ADT into the `board.dtb` placeholders): `run-dcpext-test.sh TREE ADT [ADT-701] WORK`, `SAN=1 FUZZ=N` for ASan/UBSan and fuzzed ADTs; `check.sh dcpext` runs it |
| `docker/Dockerfile` | the container recipe |
| `tools/privatize-tree.py`, `gates-unit.py`, `level1-install.py`, `audit-tree.py` | per-unit and per-install values: split out, regenerate, check |
| `tools/verify-repo.sh`, `tools/add-rev.sh` | prove a tag byte for byte; add a rev as a commit and re-run the checks |
| `payloads/README`, `inputs/rescue-cmdline.txt.template` | how `level1.bin`, `pcie.bin`, `librust.a` and the rescue inputs come to be |

## Not in git, needed to build (put them next to the sources, they are in .gitignore)

* `src/standalone/gates-unit.h`: four ADT records of one Mac (Wi-Fi calibration, module instance, antenna SKU, keyboard
  language). `gates.h` makes the loader refuse to boot unless the live ADT equals every record, so this file ties the
  image to that Mac. Generate with `m4/tools/gates-unit.py make ADT.bin ...`.
* `src/standalone/level1-install.h`: the Linux partition Level 1 mounts (4 KiB-unit range and PARTUUID). Generate with
  `m4/tools/level1-install.py` from the disk's GPT.
* The root file-system UUID, as `ROOT_UUID` for `rescue-cmdline.txt.template`.

## Adding a rev

`m4/tools/add-rev.sh REPO TREE revNN "DATE" msg.txt` (see its header). It replaces the tree, moves per-unit text out, audits, commits as
Jonathan Goyvaerts, tags, and with `EXPECTED_SHA256`/`VERSION`/`INPUTS`/`PRIVATE`/`WORK` set rebuilds the tag from a fresh clone.
Do not use `git commit -a` on a tree that still has gates-unit.h: it is ignored by .gitignore, keep it that way.
