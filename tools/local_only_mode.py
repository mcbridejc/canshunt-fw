 #!/usr/bin/env python3
""" Script to control local-only mode on a CANShunt device

Usage:
    `python local_only_mode.py`: Enable local only mode
    `python local_only_mode.py --disable`: Disable local only (bridge mode)

This script should work even while the device is being used as a socketcan device.

Requires `click` and `PyUSB` packages.
"""

import click
import usb.core

VID = 0x1209
PID = 0x5F4D

REQUEST = 0xC0
MAGIC = 0x4353
VERSION = 0x01

def set_status(dev, local_only: bool):
    value = int(local_only)

    # Set the control bit
    dev.ctrl_transfer(
        0x40,           # vendor | device | OUT
        REQUEST,
        value,              # local-only mode
        MAGIC,
        [VERSION],
        timeout=1000,
    )

    # read it back and verify
    status = dev.ctrl_transfer(
        0xC0,           # vendor | device | IN
        REQUEST,
        0,
        MAGIC,
        2,
        timeout=1000,
    )

    if bytes(status) != bytes([VERSION, value]):
        raise SystemExit(f"Unexpected CANShunt status: {bytes(status).hex()}")


@click.command()
@click.option(
    "--disable",
    is_flag=True,
    help="Disable local-only mode.",
)
def main(disable: bool):
    """Configure CANShunt local-only mode."""
    print("Searching for device")
    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        raise SystemExit("CANShunt not found")

    print("Found CANShunt Device")
    local_only = not disable
    set_status(dev, local_only)
    state = "enabled" if local_only else "disabled"
    print(f"CANShunt local-only mode is {state}")


if __name__ == "__main__":
    main()
