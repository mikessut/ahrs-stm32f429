
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
    0x617: {'name': 'CAN_DT', 'disp': (13, 15)},     
}

if __name__ == '__main__':

    bus = can.interface.Bus('can0', bustype='socketcan')

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

    screen.addstr(13, 0, 'dt')

    run_bool = True

    while run_bool:

        msg = bus.recv()

        if msg.arbitration_id in MSGS.keys():
            v = MSGS[msg.arbitration_id]
            screen.addstr(*v['disp'], f"{struct.unpack('f', msg.data)[0]:9.3f}")

        screen.refresh()
        c = screen.getch()
        if c != -1:
            curses.endwin()
            run_bool = False