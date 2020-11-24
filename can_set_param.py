"""

Examples:

Send temperature:
python3 can_set_param.py SAT 25.1

Cfg message to set hard iron x:
python3 can_set_param.py -c hard_ironx 30

Cfg qry of hard iron x:
python3 can_set_param.py -q hard_ironx 

Set parameter command of altimeter setting:
python3 can_set_param.py -s baro 29.92

"""
import can
import argparse
import struct

my_node_id = 0x71  # Have no idea what this should be...

PARAMS = {
    'BARO': {'id': 0x190, 'struct_pack': lambda x: struct.pack('H', int(float(x)*1000))},
    'SAT':  {'id': 0x407, 'struct_pack': lambda x: struct.pack('h', int(float(x)*100))},
    'HARD_IRONX': {'struct_pack': lambda x: struct.pack('f', float(x)), 'byte2': 0, 'struct_unpack': lambda x: struct.unpack('f', x)},
    'HARD_IRONY': {'struct_pack': lambda x: struct.pack('f', float(x)), 'byte2': 1, 'struct_unpack': lambda x: struct.unpack('f', x)},
    'HARD_IRONZ': {'struct_pack': lambda x: struct.pack('f', float(x)), 'byte2': 2, 'struct_unpack': lambda x: struct.unpack('f', x)},
}

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('param', type=lambda x: x.upper())
    parser.add_argument('val', nargs='?')
    parser.add_argument('-s', '--set', action='store_true', default=False, help='Send parameter set command (otherwise use normal msg)')
    parser.add_argument('-c', '--cfg', action='store_true', default=False, help='Send configure set specific command (9)')
    parser.add_argument('-q', '--qry', action='store_true', default=False, help='Send configure set specific command (10)')
    parser.add_argument('--dest', default=0x12, help='Destination node id')
    args = parser.parse_args()

    bus = can.interface.Bus('vcan0', bustype='socketcan')

    if args.set:
        # Send a Parameter set command
        data = bytearray([12]) + \
               bytearray(struct.pack('H', PARAMS[args.param]['id'])) + \
               bytearray(PARAMS[args.param]['struct_pack'](args.val))

        msg = can.Message(arbitration_id=my_node_id + 1792, 
                        data=data, is_extended_id=False)
    elif args.cfg:
        data = bytearray([9, args.dest, PARAMS[args.param]['byte2']]) + \
               bytearray(PARAMS[args.param]['struct_pack'](args.val))

        msg = can.Message(arbitration_id=my_node_id + 1792, 
                        data=data, is_extended_id=False)
    elif args.qry:
        data = bytearray([10, args.dest, PARAMS[args.param]['byte2']])

        msg = can.Message(arbitration_id=my_node_id + 1792, 
                        data=data, is_extended_id=False)
    else:
        # normal message
        data = bytearray([my_node_id, 0, 0]) + bytearray(PARAMS[args.param]['struct_pack'](args.val))
        msg = can.Message(arbitration_id=PARAMS[args.param]['id'], 
                           data=data, is_extended_id=False)

    print(msg)

    bus.send(msg)
    if args.qry:
        print("Waiting for response to query...")
        while True:
            msg = bus.recv()
            if (msg.arbitration_id == (args.dest + 1792)) and (msg.data[1] == my_node_id):
                print(msg)
                print(PARAMS[args.param]['struct_unpack'](msg.data[3:]))
                break