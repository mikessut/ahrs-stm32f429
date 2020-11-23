
import can
import argparse
import struct

my_node_id = 113  # Have no idea what this should be...

PARAMS = {
    'BARO': {'id': 0x190, 'struct_func': lambda x: struct.pack('H', int(float(x)*1000))}
}

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('param', type=lambda x: x.upper())
    parser.add_argument('val')
    args = parser.parse_args()

    bus = can.interface.Bus('vcan0', bustype='socketcan')

    data = bytearray([12]) + \
           bytearray(struct.pack('H', PARAMS[args.param]['id'])) + \
           bytearray(PARAMS[args.param]['struct_func'](args.val))

    msg = can.Message(arbitration_id=my_node_id + 1792, 
                      data=data, is_extended_id=False)
    print(msg)

    print(bus.send(msg))