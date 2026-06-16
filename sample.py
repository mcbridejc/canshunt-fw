import can
import click
import numpy as np
import struct


@click.command()
@click.option("--raw", is_flag=True, help="Sample raw ADC values.")
def main(raw):

    if raw: 
        base_address = 0x300
    else: 
        base_address = 0x200

    filters = [
        {"can_id": base_address, "can_mask": 0x7FF, "extended": False},
        {"can_id": base_address + 1, "can_mask": 0x7FF, "extended": False},
        {"can_id": base_address + 2, "can_mask": 0x7FF, "extended": False},
        {"can_id": base_address + 3, "can_mask": 0x7FF, "extended": False},
    ]
    bus = can.Bus(interface="socketcan", channel="can0", can_filters=filters)

    msgs = [[], [], [], []]

    N_SAMPLES = 20

    while True:
        msg = bus.recv()
        msgs[msg.arbitration_id - base_address].append(msg.data)

        if all([len(f) > N_SAMPLES for f in msgs]):
            break
        
    def average(msgs):
        sums = np.zeros(4)
        for m in msgs:
            values = struct.unpack("<hhhh", m)
            sums += values  
        return sums / len(msgs)

    averages = np.array([average(m) for m in msgs]).flatten()
    
    if raw:
        for i in range(8):
            ina = averages[i]
            zxc = averages[i + 8]
            print(f"CH{i}: INA={ina:.1f} V={zxc:.1f}")
    else:
        for i in range(8):

            current = averages[i]
            voltage = averages[i + 8]
            print(F"CH{i}: I={current/1000.0:.3f}A, V={voltage/1000.0:.3f}V")
                  
    bus.shutdown()


if __name__ == "__main__":
    main()
