/* SPDX-License-Identifier: GPL-2.0-only */
/* Call after direct provider validation. Admission is data-only; the private
 * modules still own their resource/stop checks and are never auto-loaded. */
#ifndef M4_DIRECT_PERIPHERAL_FDT_H
#define M4_DIRECT_PERIPHERAL_FDT_H
#include "direct-capture.h"
#include "codec-contract.h"
#include "usb-adt-gates.h"
#include "usb-contract-id.h"

/* macOS 26.6.2 installed from Recovery (25G241) reports the gated OS firmware with a .701.1 suffix: the same contract. */
static const unsigned char m4_usb_os_firmware_recovery[256] = "mBoot-18000.161.10.701.1";

static int m4_usb_gate_matches(const struct m4_usb_adt_gate *g, const void *p, unsigned length)
{
    if (p && length == g->size && !memcmp(p, g->bytes, length)) return 1;
    return p && !strcmp(g->path, "/chosen") && !strcmp(g->key, "firmware-version") &&
           length == sizeof(m4_usb_os_firmware_recovery) && !memcmp(p, m4_usb_os_firmware_recovery, length);
}

static int m4_direct_peripheral_fdt(void *dt, size_t capacity,
                                   codec_getprop get, void *ctx)
{
    int chosen, codec, bytes, rc, usb = 1;
    const char *contract;
    /* The actual direct loader supplies 48KiB, with bounded 64KiB and 128KiB (rev 12) successors.
     * Only these reviewed buffer capacities may reach the writer. */
    if (!dt || !get || (capacity != 49152 && capacity != 65536 && capacity != 131072) ||
        fdt_check_full(dt, capacity) ||
        fdt_node_check_compatible(dt, 0, "apple,j616s") ||
        fdt_node_check_compatible(dt, 0, "apple,t6040")) return -1;
    chosen = fdt_path_offset(dt, "/chosen");
    codec = fdt_path_offset(dt, "/headphone-codec-probe");
    contract = fdt_getprop(dt, chosen, "m4,direct-desktop-contract", &bytes);
    if (chosen < 0 || codec < 0 || !contract ||
        bytes != sizeof(M4_DIRECT_DESKTOP_CONTRACT) ||
        memcmp(contract, M4_DIRECT_DESKTOP_CONTRACT, bytes) ||
        fdt_node_check_compatible(dt, codec, "lab,j616s-headphone-control-v1") ||
        fdt_open_into(dt, dt, capacity)) return -1;
    /* Never inherit optional admission from an older template or boot. */
    codec = fdt_path_offset(dt, "/headphone-codec-probe");
    if (fdt_setprop_string(dt, codec, "status", "disabled")) return -1;
    rc = fdt_delprop(dt, codec, "lab,current-adt-validated");
    if (rc && rc != -FDT_ERR_NOTFOUND) return -1;
    rc = codec_contract(get, ctx);
    if (fdt_setprop_u32(dt, codec, "lab,admission-result", rc)) return -1;
    if (!rc && (fdt_setprop_u32(dt, codec, "lab,current-adt-validated", 1) ||
                fdt_setprop_string(dt, codec, "status", "okay"))) return -1;
    for (unsigned i = 0; i < sizeof(m4_usb_adt_gates) / sizeof(m4_usb_adt_gates[0]); i++) {
        const struct m4_usb_adt_gate *g = &m4_usb_adt_gates[i];
        unsigned length = 0;
        const void *p = get(ctx, g->path, g->key, &length);
        if (!m4_usb_gate_matches(g, p, length)) {
            usb = 0;
            break;
        }
    }
    chosen = fdt_path_offset(dt, "/chosen");
    if (fdt_setprop_string(dt, chosen, "m4,usb2-right-contract",
                          usb ? M4_USB_CONTRACT_ID : "refused-current-adt") ||
        fdt_pack(dt) || fdt_check_full(dt, capacity)) return -1;
    return 0;
}
#endif
