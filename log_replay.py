from can_debug_monitor import MSGS
#import fixgw.netfix as netfix
import can
import os
import numpy as np
import quaternion
import struct
import time
import sys
import argparse
import threading
import kalman_cpp

KTS2MS = 1/1.94384


GENERATING_NODE_ID = 0x12

class DataReader:

    PARAMS = [x['name'] for x in MSGS.values()]

    def __init__(self, date_str):
        self._date_str = date_str

        self._fids = {}
        for p in self.PARAMS:
            fn = f'logs/{self._date_str}_{p}.log'
            if os.path.exists(fn):
                self._fids[p] = open(f'logs/{self._date_str}_{p}.log', 'r')

    def read_one(self):
        for p in self.PARAMS:
            if p not in self._fids:
                continue
            line = self._fids[p].readline()
            cols = line.split()
            try:
                setattr(self, p + '_t', float(cols[0]))
                setattr(self, p, float(cols[1]))
            except IndexError:
                pass

    def a(self):
        return np.array([self.CAN_AX, self.CAN_AY, self.CAN_AZ])

    def w(self):
        return np.array([self.CAN_WX, self.CAN_WY, self.CAN_WZ])

    def m(self):
        return np.array([self.CAN_MAGX, self.CAN_MAGY, self.CAN_MAGZ])

    def CAN_xmit(self, bus):
        for msg_id, v in MSGS.items():
            if hasattr(self, v['name']):
                if 'pack_func' in v:
                    data = bytearray([GENERATING_NODE_ID, 0, 0]) + \
                           v['pack_func'](getattr(self, v['name']))
                else:
                    data = struct.pack('f', getattr(self, v['name']))
                msg = can.Message(arbitration_id=msg_id, data=data, is_extended_id=False)
                bus.send(msg)


def replay_func(run_bool: threading.Event, pause_bool, data, bus, ctr, dt, run_kf):
    t = data.CAN_AX_t

    if run_kf:
        kf = kalman_cpp.KalmanCpp()

    while run_bool.is_set():
        while pause_bool.is_set():
            time.sleep(.1)
        #print(ctr)
    
        # q = quaternion.from_rotation_vector(np.array([0,0,1])*data.CANFIX_HEAD*np.pi/180) * \
        #     quaternion.from_rotation_vector(np.array([1,0,0])*data.CANFIX_PITCH*np.pi/180) * \
        #     quaternion.from_rotation_vector(np.array([0,1,1])*data.CANFIX_ROLL*np.pi/180) 

        qtmp = quaternion.from_rotation_vector(np.array([1, 0, 0]) * data.CANFIX_ROLL*np.pi/180) * \
               quaternion.from_rotation_vector(np.array([0, 1, 0]) * data.CANFIX_PITCH*np.pi/180)
        
        sensor_heading = quaternion.as_float_array(qtmp * np.quaternion(0, data.CAN_MAGX-18, data.CAN_MAGY, data.CAN_MAGZ) * qtmp.inverse())[1:]
        sensor_heading[2] = 0;
        sensor_heading /= np.linalg.norm(sensor_heading)
        sensor_heading[1] *= -1;
        data.CAN_HEAD_VEC_X = sensor_heading[0]
        data.CAN_HEAD_VEC_Y = sensor_heading[1]

        if run_kf:
            kf.predict(dt, data.CANFIX_TAS * KTS2MS)
            kf.update_accel(data.a())
            kf.update_gyro(data.w())
            mag = data.m() + np.array([-18, 0, 0.0], dtype=float)
            kf.update_mag(mag)
            es = kf.eulers()*180/np.pi  #roll, pitch, heading
            data.CANFIX_ROLL = es[0]
            data.CANFIX_PITCH = es[1]
            data.CANFIX_HEAD = es[2] + 360 if es[2] < 0 else es[2]
            for n, xyz in zip(range(3), 'XYZ'):
                setattr(data, f'CAN_KF_A{xyz}', kf.a[n])
                setattr(data, f'CAN_KF_W{xyz}', kf.w[n])
                setattr(data, f'CAN_MAG{xyz}', mag[n])
            
        data.CAN_xmit(bus)

        data.read_one()
        #dt = data.CAN_AX_t - t
        #t = data.CAN_AX_t
        #sys.stdout.write(f"{dt:.3f}        \r")
        sys.stdout.write(f"{ctr:10d}        \r")
        time.sleep(dt)
        ctr += 1


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--datestr')
    parser.add_argument('-o', '--offset', help="Number of steps to offset", type=int, default=1)
    parser.add_argument('-dt', default=.027, type=float)
    parser.add_argument('-kf', default=False, action='store_true', help="Run local Kalman filter")
    args = parser.parse_args()
    
    data = DataReader(args.datestr)
    bus = can.interface.Bus('vcan0', bustype='socketcan')

    ctr = 0
    for _ in range(args.offset):
        data.read_one()
        ctr += 1

    print("Offset complete.")
    
    run_bool = threading.Event()
    run_bool.set()
    pause_bool = threading.Event()
    pause_bool.clear()

    thread = threading.Thread(target=replay_func, args=(run_bool, pause_bool, data, bus, ctr, args.dt, args.kf))
    thread.start()

    while True:
        tmp = input()
        if tmp == 'p':
            pause_bool.set()
            print("Pausing")
        elif tmp == 'r':
            pause_bool.clear()
            print("resuming")
        elif tmp == 'q':
            run_bool.clear()
            pause_bool.clear()
            print("quiting")
            break
        time.sleep(.1)