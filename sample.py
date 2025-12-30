import can
import numpy as np
import struct


def main():
    filters = [
        {"can_id": 0x200, "can_mask": 0x7FF, "extended": False},
        {"can_id": 0x201, "can_mask": 0x7FF, "extended": False},
        {"can_id": 0x202, "can_mask": 0x7FF, "extended": False},
        {"can_id": 0x203, "can_mask": 0x7FF, "extended": False},
    ]
    bus = can.Bus(interface="socketcan", channel="can0", can_filters=filters)

    msgs = [[], [], [], []]

    N_SAMPLES = 20

    while True:
        msg = bus.recv()
        msgs[msg.arbitration_id - 0x200].append(msg.data)

        if all([len(f) > N_SAMPLES for f in msgs]):
            break
        
    def average(msgs):
        sums = np.zeros(4)
        for m in msgs:
            values = struct.unpack("<hhhh", m)
            sums += values  
        return sums / len(msgs)

    averages = np.array([average(m) for m in msgs]).flatten()
    
    for i in range(8):
        ina = averages[i]
        zxc = averages[i+8]
        print(f"CH{i}: INA={ina:.1f} ZXC={zxc:.1f}")
    
    bus.shutdown()


if __name__ == "__main__":
    main()