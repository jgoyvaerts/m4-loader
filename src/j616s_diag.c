/* SPDX-License-Identifier: GPL-2.0-only */
#include "../build/build_cfg.h"

#ifdef J616S_MEMORY_DIAGNOSTIC
#include "adt.h"
#include "j616s_diag.h"
#include "string.h"
#include "uart.h"
#include "xnuboot.h"

#ifdef J616S_SCREEN_DIAGNOSTIC
#include "j616s_screen.h"
extern void j616s_screen_show_error(void);
#include "j616s_probe.h"
#include "j616s_usb.h"
extern void j616s_standalone_report(u64,u64);
#include "j616s_firmware_buffer.h"
/* Reuse the metadata checks without touching or polling UART registers. */
#define uart_puts j616s_screen_puts
#define uart_printf j616s_screen_printf
#define uart_flush j616s_screen_flush
#endif

extern char _base[], _end[];

static void diag_stop(void) __attribute__((noreturn));
static void diag_stop(void)
{
    j616s_screen_show_error();
    uart_puts("J616S-DIAG: STOP; no secondary startup or kernel handoff");
    uart_puts("J616S-DIAG: watchdog unchanged; this diagnostic may reset");
    uart_flush();
    for (;;)
        __asm__ volatile("nop");
}

/* Check the length before copying: do not use the legacy ADT copy wrapper. */
static bool diag_get(int node, const char *name, void *out, u32 expected)
{
    u32 len = 0;
    const void *value = node < 0 ? NULL : adt_getprop(adt, node, name, &len);
    if (!value || len != expected) {
        uart_printf("J616S-DIAG: missing/malformed %s (length %u, expected %u)\r\n", name, len,
                    expected);
        return false;
    }
    memcpy(out, value, expected);
    return true;
}

#ifdef J616S_SCREEN_DIAGNOSTIC
/* Observe rejected properties without treating unknown bytes as a C string.
 * The full declared extent must be inside this invocation's ADT. Even then,
 * read and display at most 128 bytes, in rows shorter than the minimum console.
 * This changes only refusal reporting, never what qualifies as valid metadata. */
static void diag_rejected_string(const char *name, const void *value, u32 len)
{
    uart_printf("J616S-PROP: %s present=%u length=%u\r\n", name, value != NULL, len);
    if (!value || !len) {
        uart_puts("J616S-PROP: no bytes to read");
        return;
    }
    if (!j616s_range_contains((u64)adt, cur_boot_args.devtree_size, (u64)value, len)) {
        uart_puts("J616S-PROP: extent outside ADT; bytes not read");
        return;
    }
    u32 count = len < 128 ? len : 128;
    uart_printf("J616S-PROP: showing %u/%u bytes; hex |ASCII|; dot=nonprintable\r\n",
                count, len);
    const u8 *bytes = value;
    static const char digits[] = "0123456789abcdef";
    for (u32 offset = 0; offset < count; offset += 16) {
        char hex[49], ascii[17];
        u32 row = count - offset;
        if (row > 16)
            row = 16;
        for (u32 i = 0; i < row; i++) {
            u8 c = bytes[offset + i];
            hex[3 * i] = digits[c >> 4];
            hex[3 * i + 1] = digits[c & 15];
            hex[3 * i + 2] = ' ';
            ascii[i] = c >= 32 && c <= 126 ? c : '.';
        }
        hex[3 * row] = 0;
        ascii[row] = 0;
        uart_printf("%04x: %s|%s|\r\n", offset, hex, ascii);
    }
    uart_flush();
}
#endif

static const char *diag_string(int node, const char *name)
{
    u32 len = 0;
    const char *value = node < 0 ? NULL : adt_getprop(adt, node, name, &len);
#ifdef J616S_SCREEN_DIAGNOSTIC
    /* Only this observed property gains the fixed-buffer representation.
     * Do not apply it to target-type, system-firmware-version or compatible. */
    if (value && len == 256 && !strcmp(name, "firmware-version") &&
        j616s_range_contains((u64)adt, cur_boot_args.devtree_size, (u64)value, len)) {
        struct j616s_fw_buffer_result format = j616s_check_firmware_buffer(value, len);
        if (format.status == J616S_FW_BUFFER_VALID) {
            uart_printf("J616S-PROP: firmware-version length=256 text=%u; all padding checked\r\n",
                        format.offset);
            return value;
        }
        if (format.status == J616S_FW_BUFFER_NONZERO_PADDING)
            uart_printf("J616S-PROP: nonzero padding at offset=%u byte=0x%02x\r\n",
                        format.offset, (unsigned)(u8)value[format.offset]);
        else if (format.status == J616S_FW_BUFFER_NO_NUL)
            uart_puts("J616S-PROP: no terminator within first 128 bytes");
    }
#endif
    if (!value || !len || len > 128 || value[len - 1] || memchr(value, 0, len - 1)) {
        uart_printf("J616S-DIAG: missing/malformed string %s\r\n", name);
#ifdef J616S_SCREEN_DIAGNOSTIC
        diag_rejected_string(name, value, len);
#endif
        return NULL;
    }
    return value;
}


void j616s_diag_run(void)
{
    uart_puts("J616S-DIAG: checking metadata; NOT a general memory-safety approval");
    if (cur_boot_args.revision != 3 || adt_check_header(adt) < 0) {
        uart_puts("J616S-DIAG: unsupported boot-args revision or ADT");
        diag_stop();
    }

    int chosen = adt_path_offset(adt, "/chosen");
    int mcc = adt_path_offset(adt, "/arm-io/mcc");
    const char *target = diag_string(0, "target-type");
    u32 chip;
    if (!target || strcmp(target, "J616s") || !diag_get(chosen, "chip-id", &chip, sizeof(chip)) ||
        chip != 0x6040) {
        uart_puts("J616S-DIAG: refusing a non-J616s/T6040 target");
        diag_stop();
    }
    uart_printf("J616S-DIAG: target=%s chip=0x%x bootargs revision=%u version=%u\r\n", target, chip,
                cur_boot_args.revision, cur_boot_args.version);

    const char *os_fw = diag_string(chosen, "firmware-version");
    const char *sys_fw = diag_string(chosen, "system-firmware-version");
    const char *controller = diag_string(mcc, "compatible");
    if (!os_fw || !sys_fw || !controller)
        diag_stop();
    uart_printf("J616S-DIAG: os-fw=%s system-fw=%s mcc=%s\r\n", os_fw, sys_fw, controller);
    /* V4 observed this exact mixed pair on J616s. Accept it only for the
     * existing screen metadata capture: no controller/protection semantics
     * are inferred from a matching version, and no normal boot path changes. */
#ifdef J616S_SCREEN_DIAGNOSTIC
    const char *expected_system_fw = "mBoot-20457.1.29";
#else
    const char *expected_system_fw = "mBoot-18000.161.10";
#endif
    /* macOS 26.6.2 installed from Recovery (25G241) reports the same OS firmware
     * with a .701.1 suffix. */
    if ((strcmp(os_fw, "mBoot-18000.161.10") && strcmp(os_fw, "mBoot-18000.161.10.701.1")) ||
        strcmp(sys_fw, expected_system_fw) || strcmp(controller, "mcc,t6041")) {
        uart_puts("J616S-DIAG: outside the reviewed firmware/controller baseline");
        diag_stop();
    }
#ifdef J616S_SCREEN_DIAGNOSTIC
    uart_puts("J616S-DIAG: observed firmware pair matched");
#endif

    u64 base, size;
    u64 usable = cur_boot_args.phys_base, usable_size = cur_boot_args.mem_size;
    u64 loader = (u64)_base, loader_size = (u64)_end - loader;
    if (!diag_get(chosen, "dram-base", &base, sizeof(base)) ||
        !diag_get(chosen, "dram-size", &size, sizeof(size)))
        diag_stop();
    uart_printf("J616S-DIAG: DRAM start=0x%lx size=0x%lx\r\n", base, size);
    uart_printf("J616S-DIAG: usable start=0x%lx size=0x%lx actual=0x%lx\r\n", usable, usable_size,
                cur_boot_args.rv3.mem_size_actual);
    uart_printf("J616S-DIAG: loader start=0x%lx size=0x%lx top-of-kdata=0x%lx\r\n", loader,
                loader_size, cur_boot_args.top_of_kernel_data);
    uart_printf("J616S-DIAG: ADT start=0x%lx size=0x%x framebuffer=0x%lx\r\n", (u64)adt,
                cur_boot_args.devtree_size, cur_boot_args.video.base);
    if (!j616s_range_aligned(base, size) || !j616s_range_aligned(usable, usable_size) ||
        !j616s_range_contains(base, size, usable, usable_size) ||
        !j616s_range_contains(usable, usable_size, loader, loader_size) ||
        !j616s_range_contains(base, size, (u64)adt, cur_boot_args.devtree_size)) {
        uart_puts("J616S-DIAG: INVALID basic memory intervals");
        diag_stop();
    }

    /* Selected evidence, not an assertion that these are all protected regions. */
    static const char *const names[] = {
        "region-id-1", "region-id-2", "region-id-4", "region-id-14", "region-id-24",
    };
    int carveouts = adt_path_offset(adt, "/chosen/carveout-memory-map");
    for (unsigned i = 0; i < sizeof(names) / sizeof(names[0]); i++) {
        u64 range[2];
        if (!diag_get(carveouts, names[i], range, sizeof(range)))
            diag_stop();
#ifndef J616S_SCREEN_DIAGNOSTIC
        uart_printf("J616S-DIAG: %s start=0x%lx size=0x%lx\r\n", names[i], range[0], range[1]);
#endif
        if (!j616s_range_aligned(range[0], range[1]) ||
            !j616s_range_contains(base, size, range[0], range[1])) {
            uart_puts("J616S-DIAG: INVALID selected range; do not infer protection semantics");
            diag_stop();
        }
#ifdef J616S_SCREEN_DIAGNOSTIC
        uart_printf("%s start=0x%lx size=0x%lx overlaps usable=%u loader=%u ADT=%u\r\n",
                    names[i], range[0], range[1],
#else
        uart_printf("J616S-DIAG: overlaps usable=%u loader=%u ADT=%u\r\n",
#endif
                    j616s_range_overlaps(range[0], range[1], usable, usable_size),
                    j616s_range_overlaps(range[0], range[1], loader, loader_size),
                    j616s_range_overlaps(range[0], range[1], (u64)adt, cur_boot_args.devtree_size));
#ifdef J616S_SCREEN_DIAGNOSTIC
        if (i < 4 && j616s_range_overlaps(range[0], range[1], loader, loader_size)) {
            uart_puts("USB REFUSED: selected firmware region overlaps loader");
            diag_stop();
        }
#endif
    }
    uart_puts("J616S-DIAG: metadata captured; overlaps are observations, not pass/fail");
#ifdef J616S_SCREEN_DIAGNOSTIC
    j616s_probe_report();
    u64 lower, upper;
    if (j616s_probe_bounds(&lower, &upper))
        j616s_standalone_report(lower, upper);
    else
        uart_puts("USB REFUSED: both raw register reads must succeed");
#endif
#ifdef J616S_SCREEN_DIAGNOSTIC
    uart_puts("J616S-DIAG: protection semantics and final FDT still unverified");
#else
    uart_puts("J616S-DIAG: CTRR NOT READ; semantics and final FDT still unverified");
#endif
    diag_stop();
}
#endif
