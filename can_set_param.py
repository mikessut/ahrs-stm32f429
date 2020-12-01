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
import sys
import yaml
import time

THIS_NODE_ID = 0x71  # Have no idea what this should be...
CANFIX_NODE_MSGS_OFFSET = 0x6E0

PARAMS = {
    'BARO': {'id': 0x190, 'struct_pack': lambda x: struct.pack('H', int(float(x)*1000))},
    'SAT':  {'id': 0x407, 'struct_pack': lambda x: struct.pack('h', int(float(x)*100))},
    'HARD_IRONX': {'struct_pack': lambda x: struct.pack('f', float(x)), 'byte2': 0, 'struct_unpack': lambda x: struct.unpack('f', x)},
    'HARD_IRONY': {'struct_pack': lambda x: struct.pack('f', float(x)), 'byte2': 1, 'struct_unpack': lambda x: struct.unpack('f', x)},
    'HARD_IRONZ': {'struct_pack': lambda x: struct.pack('f', float(x)), 'byte2': 2, 'struct_unpack': lambda x: struct.unpack('f', x)},
    'WBX': {'struct_pack': lambda x: struct.pack('f', float(x)), 'byte2': 3, 'struct_unpack': lambda x: struct.unpack('f', x)},
    'WBY': {'struct_pack': lambda x: struct.pack('f', float(x)), 'byte2': 4, 'struct_unpack': lambda x: struct.unpack('f', x)},
    'WBZ': {'struct_pack': lambda x: struct.pack('f', float(x)), 'byte2': 5, 'struct_unpack': lambda x: struct.unpack('f', x)},
    'ABX': {'struct_pack': lambda x: struct.pack('f', float(x)), 'byte2': 6, 'struct_unpack': lambda x: struct.unpack('f', x)},
    'ABY': {'struct_pack': lambda x: struct.pack('f', float(x)), 'byte2': 7, 'struct_unpack': lambda x: struct.unpack('f', x)},
    'ABZ': {'struct_pack': lambda x: struct.pack('f', float(x)), 'byte2': 8, 'struct_unpack': lambda x: struct.unpack('f', x)},
    'Q0': {'struct_pack': lambda x: struct.pack('f', float(x)),  'byte2': 9, 'struct_unpack': lambda x: struct.unpack('f', x)},
    'Q1': {'struct_pack': lambda x: struct.pack('f', float(x)),  'byte2': 10, 'struct_unpack': lambda x: struct.unpack('f', x)},
    'Q2': {'struct_pack': lambda x: struct.pack('f', float(x)),  'byte2': 11, 'struct_unpack': lambda x: struct.unpack('f', x)},
    'Q3': {'struct_pack': lambda x: struct.pack('f', float(x)),  'byte2': 12, 'struct_unpack': lambda x: struct.unpack('f', x)},
    'STATUS': {'struct_pack': lambda x: struct.pack('B', int(x, 0)),  'byte2': 13, 'struct_unpack': lambda x: struct.unpack('B', x)},
    'DPRESS': {'struct_pack': lambda x: struct.pack('f', float(x)),  'byte2': 14, 'struct_unpack': lambda x: struct.unpack('f', x)},
    
}


def set_cfg_param(dest, key_idx, value, struct_pack_func):
    data = bytearray([9, dest, key_idx]) + \
           bytearray(struct_pack_func(value))

    msg = can.Message(arbitration_id=THIS_NODE_ID + CANFIX_NODE_MSGS_OFFSET, 
                      data=data, is_extended_id=False)
    return msg


def qry_cfg_param(dest, key_idx):
    data = bytearray([10, dest, key_idx])

    msg = can.Message(arbitration_id=THIS_NODE_ID + CANFIX_NODE_MSGS_OFFSET, 
                      data=data, is_extended_id=False)
    return msg


def rcv_msg(bus, msg_id, param_name, timeout=5.0):
    t0 = time.time()
    while (time.time() - t0) < timeout:
        msg = bus.recv()
        if (msg.arbitration_id == (args.dest + CANFIX_NODE_MSGS_OFFSET)) and (msg.data[1] == THIS_NODE_ID):
            #print(msg)
            #print(PARAMS[args.param]['struct_unpack'](msg.data[3:]))
            #return PARAMS[param_name]['struct_unpack'](msg.data[3:])[0]
            return msg.data
    return None


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--config-file', help="YAML file of config parameters.  All parameters can be set or queried using -s or -c", default=None)
    parser.add_argument('-l', '--list', help='List parameter names', action='store_true', default=False)
    parser.add_argument('param', nargs='?', type=lambda x: x.upper())
    parser.add_argument('val', nargs='?')
    parser.add_argument('-s', '--set', action='store_true', default=False, help='Send parameter set command (otherwise use normal msg)')
    parser.add_argument('-c', '--cfg', action='store_true', default=False, help='Send configure set specific command (9)')
    parser.add_argument('-q', '--qry', action='store_true', default=False, help='Send configure set specific command (10)')
    parser.add_argument('--dest', default=0x12, help='Destination node id')
    
    args = parser.parse_args()

    if args.list:
        for p in PARAMS.keys():
            print(p)
        sys.exit(0)
    bus = can.interface.Bus('can0', bustype='socketcan')

    if args.config_file is not None:
        if args.cfg:
            y = yaml.safe_load(open(args.config_file, 'r'))
            for k, v in y.items():
                print(k, v)
                msg = set_cfg_param(args.dest, PARAMS[k]['byte2'], v, PARAMS[k]['struct_pack'])
                bus.send(msg)
                data = rcv_msg(bus, args.dest + CANFIX_NODE_MSGS_OFFSET, k)
                if data[2] != 0:
                    #Error
                    print("Cfg set ack not received")
                    sys.exit(-1)
        elif args.qry:
            y = yaml.safe_load(open(args.config_file, 'r'))
            for k in y.keys():
                msg = qry_cfg_param(args.dest, PARAMS[k]['byte2'])
                bus.send(msg)
                data = rcv_msg(bus, args.dest + CANFIX_NODE_MSGS_OFFSET, k)
                val = PARAMS[k]['struct_unpack'](data[3:])[0]
                print(f"{k}: {val}")
        else:
            print("Config file must include -c or -q argument")
            raise argparse.ArgumentError
    elif args.set:
        # Send a Parameter set command
        data = bytearray([12]) + \
               bytearray(struct.pack('H', PARAMS[args.param]['id'])) + \
               bytearray(PARAMS[args.param]['struct_pack'](args.val))

        msg = can.Message(arbitration_id=THIS_NODE_ID + CANFIX_NODE_MSGS_OFFSET, 
                        data=data, is_extended_id=False)
        bus.send(msg)
    elif args.cfg:
        msg = set_cfg_param(args.dest, PARAMS[args.param]['byte2'], args.val, PARAMS[args.param]['struct_pack'])
        bus.send(msg)
    elif args.qry:
        msg = qry_cfg_param(args.dest, PARAMS[args.param]['byte2'])
        bus.send(msg)
        val = PARAMS[args.param]['struct_unpack'](rcv_msg(bus, args.dest + CANFIX_NODE_MSGS_OFFSET, args.param)[3:])[0]
        print(f"{args.param}: {val}")
    else:
        # normal message
        data = bytearray([THIS_NODE_ID, 0, 0]) + bytearray(PARAMS[args.param]['struct_pack'](args.val))
        msg = can.Message(arbitration_id=PARAMS[args.param]['id'], 
                           data=data, is_extended_id=False)
        bus.send(msg)

    # print("Sending msg:", msg)
# 
    # bus.send(msg)
    # if args.qry:
    #     print("Waiting for response to query...")
    #     while True:
    #         msg = bus.recv()
    #         if (msg.arbitration_id == (args.dest + CANFIX_NODE_MSGS_OFFSET)) and (msg.data[1] == THIS_NODE_ID):
    #             print(msg)
    #             print(PARAMS[args.param]['struct_unpack'](msg.data[3:]))
    #             break