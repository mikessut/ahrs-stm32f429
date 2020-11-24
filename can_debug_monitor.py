
import can
import canfix
import curses
import struct

MSGS = {
    0x600: {'name': 'CAN_KF_WX', 'disp': (1, 15    )},      
    0x601: {'name': 'CAN_KF_WY', 'disp': (1, 15+9  )},      
    0x602: {'name': 'CAN_KF_WZ', 'disp': (1, 15+2*9)},      
    0x603: {'name': 'CAN_KF_WBX', 'disp': (0, 0)},     
    0x604: {'name': 'CAN_KF_WBY', 'disp': (0, 0)},     
    0x605: {'name': 'CAN_KF_WBZ', 'disp': (0, 0)},     
    0x606: {'name': 'CAN_KF_AX', 'disp': (5, 15    )},      
    0x607: {'name': 'CAN_KF_AY', 'disp': (5, 15+9  )},      
    0x608: {'name': 'CAN_KF_AZ', 'disp': (5, 15+2*9)},      
    0x609: {'name': 'CAN_KF_ABX', 'disp': (0, 0)},     
    0x60A: {'name': 'CAN_KF_ABY', 'disp': (0, 0)},     
    0x60B: {'name': 'CAN_KF_ABZ', 'disp': (0, 0)},     
    0x60C: {'name': 'CAN_WX', 'disp': (0, 15    )},         
    0x60D: {'name': 'CAN_WY', 'disp': (0, 15+9  )},         
    0x60E: {'name': 'CAN_WZ', 'disp': (0, 15+2*9)},         
    0x60F: {'name': 'CAN_AX', 'disp': (4, 15    )},         
    0x610: {'name': 'CAN_AY', 'disp': (4, 15+9  )},         
    0x611: {'name': 'CAN_AZ', 'disp': (4, 15+2*9)},         
    0x612: {'name': 'CAN_MAGX', 'disp': (8, 15    )},       
    0x613: {'name': 'CAN_MAGY', 'disp': (8, 15+9  )},       
    0x614: {'name': 'CAN_MAGZ', 'disp': (8, 15+2*9)},  
    0x615: {'name': 'CAN_PRESSA', 'disp': (10, 15)},     
    0x616: {'name': 'CAN_PRESSD', 'disp': (11, 15)},     
    0x617: {'name': 'CAN_DT', 'disp': (15, 15)},  
    0x180: {'name': 'CANFIX_PITCH', 'disp': (17, 15+9), 'unpack_func': lambda x: struct.unpack('h', x[3:])[0]/100},
    0x181: {'name': 'CANFIX_ROLL', 'disp':  (17, 15), 'unpack_func': lambda x: struct.unpack('h', x[3:])[0]/100},
    0x185: {'name': 'CANFIX_HEAD', 'disp':  (17, 15+9*2), 'unpack_func': lambda x: struct.unpack('H', x[3:])[0]/10},
    0x183: {'name': 'CANFIX_IAS', 'disp': (13, 15), 'unpack_func': lambda x: struct.unpack('H', x[3:])[0]/10},
    0x18D: {'name': 'CANFIX_TAS', 'disp': (13, 15+9), 'unpack_func': lambda x: struct.unpack('H', x[3:])[0]/10},
    0x184: {'name': 'CANFIX_ALT', 'disp': (13, 15+9*2), 'unpack_func': lambda x: struct.unpack('i', x[3:])[0]},
    0x190: {'name': 'CANFIX_ALT_SET', 'disp': (19, 15), 'unpack_func': lambda x: struct.unpack('H', x[3:])[0] / 1000},
    0x407: {'name': 'CANFIX_SAT', 'disp': (14, 15), 'unpack_func': lambda x: struct.unpack('h', x[3:])[0] / 100},
}

if __name__ == '__main__':

    bus = can.interface.Bus('vcan0', bustype='socketcan')

    screen = curses.initscr()
    screen.nodelay(True)

    # setup screen
    screen.addstr(0, 0, 'Raw gyros')
    screen.addstr(1, 0, 'KF gyros')

    screen.addstr(4, 0, 'Raw accels')
    screen.addstr(5, 0, 'KF accels')

    screen.addstr(8, 0, 'Raw mag')

    screen.addstr(10, 0, 'Abs P')
    screen.addstr(11, 0, 'Diff P')

    screen.addstr(13, 0, 'IAS, TAS, Alt')
    screen.addstr(14, 0, 'OAT (SAT)')

    screen.addstr(15, 0, 'dt')

    screen.addstr(17, 0, "R, P, H")

    run_bool = True

    while run_bool:

        msg = bus.recv()

        if msg.arbitration_id in MSGS.keys():
            v = MSGS[msg.arbitration_id]
            if 'unpack_func' in v:
                try: 
                    screen.addstr(*v['disp'], f"{v['unpack_func'](msg.data):9.3f}")
                except struct.error:
                    print(f"Trying to unpack {v}. {msg.data}")
            else:
                screen.addstr(*v['disp'], f"{struct.unpack('f', msg.data)[0]:9.3f}")
                

        screen.refresh()
        c = screen.getch()
        if c != -1:
            curses.endwin()
            run_bool = False