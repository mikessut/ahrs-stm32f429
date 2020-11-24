
import can
import argparse
import struct

my_node_id = 113  # Have no idea what this should be...

PARAMS = {
    'BARO': {'id': 0x190, 'struct_func': lambda x: struct.pack('H', int(float(x)*1000))},
    'SAT':  {'id': 0x407, 'struct_func': lambda x: struct.pack('h', int(float(x)*100))},
}

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('param', type=lambda x: x.upper())
    parser.add_argument('val')
    parser.add_argument('-s', '--set', action='store_true', default=False, help='Send parameter set command (otherwise use normal msg)')
    args = parser.parse_args()

    bus = can.interface.Bus('vcan0', bustype='socketcan')

    if args.set:
        # Send a Parameter set command
        data = bytearray([12]) + \
            bytearray(struct.pack('H', PARAMS[args.param]['id'])) + \
            bytearray(PARAMS[args.param]['struct_func'](args.val))

        msg = can.Message(arbitration_id=my_node_id + 1792, 
                        data=data, is_extended_id=False)
    else:
        # normal message
        data = bytearray([my_node_id, 0, 0]) + bytearray(PARAMS[args.param]['struct_func'](args.val))
        msg = can.Message(arbitration_id=PARAMS[args.param]['id'], 
                           data=data, is_extended_id=False)

    print(msg)

    print(bus.send(msg))