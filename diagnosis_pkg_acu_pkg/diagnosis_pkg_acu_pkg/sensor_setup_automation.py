#!/usr/bin/env python3
import os
import re
import sys
import subprocess
import pyudev

RULES_FILE = '/etc/udev/rules.d/99-autodev.rules'

def load_mappings():
    """Parse existing rules for serial->name mappings."""
    mapping = {}
    if not os.path.exists(RULES_FILE):
        return mapping
    rule_re = re.compile(
        r'ATTRS\{serial\}=="(?P<serial>[^"]+)"\s*,\s*SYMLINK\+="(?P<name>[^"]+)"'
    )
    for line in open(RULES_FILE):
        m = rule_re.search(line)
        if m:
            mapping[m.group('serial')] = m.group('name')
    return mapping

def append_rule(vendor, product, serial, name):
    """Add a new rule and reload udev."""
    rule = (
        f'SUBSYSTEM=="tty", '
        f'ATTRS{{idVendor}}=="{vendor}", ATTRS{{idProduct}}=="{product}", '
        f'ATTRS{{serial}}=="{serial}", '
        f'SYMLINK+="{name}", MODE="0666", GROUP="plugdev"\n'
    )
    with open(RULES_FILE, 'a') as f:
        f.write(rule)
    subprocess.run(['udevadm', 'control', '--reload-rules'], check=True)
    subprocess.run(['udevadm', 'trigger'],        check=True)

def main():
    if os.geteuid() != 0:
        print("⚠️  Must run as root: sudo python3 udev_auto_mapper.py")
        sys.exit(1)

    mapping = load_mappings()
    context = pyudev.Context()
    monitor = pyudev.Monitor.from_netlink(context)
    monitor.filter_by('tty')

    print("🔌 Watching for new serial devices. Ctrl-C to quit.")
    for device in iter(monitor.poll, None):
        if device.action != 'add':
            continue

        vid    = device.get('ID_VENDOR_ID')
        pid    = device.get('ID_MODEL_ID')
        serial = device.get('ID_SERIAL_SHORT')
        model  = device.get('ID_MODEL', '').strip()

        # skip if we already know it
        if serial in mapping:
            print(f"[✓] {model or vid+pid} ({serial}) → /dev/{mapping[serial]}")
            continue

        # ask the user for a name
        default = (model or f"{vid}_{pid}").lower().replace(' ', '_')
        prompt  = (
            f"\nNew device detected:\n"
            f"  Vendor:Product = {vid}:{pid}\n"
            f"  Serial         = {serial}\n"
            f"  Model          = {model or '(unknown)'}\n"
            f"Enter symlink name [default: {default}]: "
        )
        name = input(prompt).strip() or default

        append_rule(vid, pid, serial, name)
        mapping[serial] = name
        print(f"[SUCCESS] /dev/{name} → {device.device_node}\n")

        # if you want to stop once *all* your expected devices are seen,
        # you could check here and sys.exit(0). Otherwise it will run forever.

if __name__ == '__main__':
    main()
