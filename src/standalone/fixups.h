/* SPDX-License-Identifier: MIT */
#ifndef M4_STANDALONE_FIXUPS_H
#define M4_STANDALONE_FIXUPS_H
#include <stddef.h>
#include <stdint.h>
#include "fdt.h"
#include "direct-smp.h"

/* Rev 11: board.dtb (built from the kernel tree: /boot/m4/entries/<entry>/board.dtb or the embedded rescue copy)
 * carries every node. These functions fill in what only the loader knows. They create no node: a DTS placeholder is
 * renamed/filled (fdt_set_name) or deleted. Each returns 0 on success and a negative value naming the refusal. */

/* memory ranges, /chosen bootargs and initrd, boot framebuffer node */
int m4_fixup_base(void *dt, size_t capacity, const struct m4_memory *memory, const struct m4_handoff *handoff,
                  const char *cmdline);
/* loader-owned reservations, cpu-release-addr, /chosen descriptors, rng-seed, keyboard country */
int m4_fixup_direct(void *dt, size_t capacity);
/* headphone codec ADT contract, right-hand USB-C contract id */
int m4_fixup_peripherals(void *dt, size_t capacity);
/* Wi-Fi and Bluetooth per-unit data from the ADT; Bluetooth admission result */
enum { M4_BT_STATE_REFUSED = 0, M4_BT_STATE_DEFERRED = 1, M4_BT_STATE_ADMITTED = 2 };
int m4_fixup_wireless(void *dt, size_t capacity, int bt_state);

struct m4_display_handoff;
/* DCP/display nodes from an accepted display capture (NULL or not accepted: the nodes stay disabled and the region
 * pool is deleted). Returns 0 enabled, 1 left disabled, <0 DT error. */
int m4_dcp_fixup(void *dt, size_t capacity, const struct m4_display_handoff *h);
/* usb/avd/audio groups are deleted when their ADT gate fails; AOP DRAM segments are reserved from the ADT */
int m4_media_fixups(void *dt, size_t capacity);
int m4_gpu_fixup(void *dt, size_t capacity);
/* dcpext0/1 firmware memory placeholders from the ADT (dcp-fdt.c); 0 done, 1 a unit left without it, <0 DT error */
int m4_dcpext_fixup(void *dt, size_t capacity);
#endif
