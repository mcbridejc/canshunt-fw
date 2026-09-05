# canshunt-fw

Embedded software for the CANShunt board. 

## Programming via DFU

### Generating .bin file

You will need cargo binutils: `cargo install cargo-binutils`

Generate the binary file:
`cargo objcopy --release -- -O binary canshunt.bin`

### Programming .bin file

If the device already has a working application in flash, dfu-util will automatically trigger the device to detach and reattach as a bootloader. Otherwise, the bootloader can be activated manually by holding down the mode button while powering the device on.

To program with dfu-util:
```
dfu-util -d 1209:5f4d,0483:df11 -a 0 -D canshunt.bin -s 0x8000000:leave
```

The first VID/PID is for the device, the second is for the ST DFU bootloader.

## Dev Notes

Currently depends on a [stm32-hal2 fork](https://github.com/mcbridejc/stm32-hal), for some L5 specific ADC definitions.

## CAN Bridge Mode

The USB CAN interface can operate in two modes: 
  - bridge: The USB-CAN and physical CAN bus are bridged
  - local only: The physical CAN bus is disabled and the USB-CAN communicates only with the device
  
At power-on, the device is in bridge mode. It can be switched between modes via a USB command. The
`tools/local_only_mode.py` script can be used to control the mode. 

When in bridge mode, congestion or error conditions on the bus can throttle message throughput, and
cause messages to be lost, and because the USB-CAN only mirrors images on the physical CAN bus,
these messages are lost. Therefor, for reliable USB communication in these situations, it may be
necessary to enable local only mode. 

### USB Protocol Definition

Overview
--------

CANShunt implements the standard gs_usb interface for CAN frames. In addition, it implements a
small product-specific control protocol on USB endpoint zero. The extension does not add an
interface or endpoint and does not alter the gs_usb descriptors or frame format.

USB identity
------------

Vendor ID:  0x1209
Product ID: 0x5F4D

Control request constants
-------------------------

bRequest:         0xC0
wIndex magic:     0x4353
Protocol version: 0x01

All multibyte USB setup fields use the standard USB little-endian representation.

Set routing mode
----------------

Direction:       Host to device
bmRequestType:   0x40 (vendor, device recipient, OUT)
bRequest:        0xC0
wValue:          0 = bridge mode
                 1 = local-only mode
wIndex:          0x4353
wLength:         1
Data:            0x01 (protocol version)

The device stalls the request if the version, mode, magic value, request type, or payload length
zis invalid. A successful status stage means the requested routing mode has been applied.

Get routing mode
----------------

Direction:       Device to host
bmRequestType:   0xC0 (vendor, device recipient, IN)
bRequest:        0xC0
wValue:          ignored
wIndex:          0x4353
wLength:         2

Two response bytes are returned:

Byte 0:          Protocol version (currently 0x01)
Byte 1:          0 = bridge mode
                 1 = local-only mode
