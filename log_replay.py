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

# mag comp
mag_bias = np.array([13.090315006860967, 14.178095542426329, -1.072964248047726])
mag_sfs = np.array([0.9746667233620181, 1.0603891667040906, 0.9930517656245322])
mag_abc = np.array([-0.08097574636641483, -0.008138292996472557, 0.0035385802786986856])

class DataReader:

    PARAMS = [
        'CAN_WX',
        'CAN_WY',
        'CAN_WZ',
        'CAN_AX',
        'CAN_AY',
        'CAN_AZ',
        'CAN_MAGX',
        'CAN_MAGY',
        'CAN_MAGZ',
        'CANFIX_TAS',
        ]

    def __init__(self, fn):        

        self._fid = open(fn)
        
    def read_msg(self):
        line = self._fid.readline().strip()
        t = float(line[1:18])
        msg_id = int(line[25:28], 16)
        if msg_id in MSGS:
          data = bytes.fromhex(line[29:])
          val = MSGS[msg_id].get('unpack_func', lambda x:struct.unpack('f', x)[0])(data)
          name = MSGS[msg_id]['name']
          setattr(self, name, val)
          setattr(self, name + '_t', t)
          return name

    def read_one(self):
        params_read = {k: False for k in self.PARAMS}
        ctr = 0
        while not all(params_read.values()):
            p = self.read_msg()
            if p in params_read:
                params_read[p] = True

    def a(self):
        return np.array([self.CAN_AX, self.CAN_AY, self.CAN_AZ])

    def w(self):
        if (max(self.CAN_WX_t, self.CAN_WY_t, self.CAN_WZ_t) - min(self.CAN_WX_t, self.CAN_WY_t, self.CAN_WZ_t)) > .01:
            raise ValueError(f"dt off; max t: {max(self.CAN_WX_t, self.CAN_WY_t, self.CAN_WZ_t)} min: {min(self.CAN_WX_t, self.CAN_WY_t, self.CAN_WZ_t)} delta: {max(self.CAN_WX_t, self.CAN_WY_t, self.CAN_WZ_t) - min(self.CAN_WX_t, self.CAN_WY_t, self.CAN_WZ_t)}")
        return np.array([self.CAN_WX, self.CAN_WY, self.CAN_WZ])

    def m(self):
        return np.array([self.CAN_MAGX, self.CAN_MAGY, self.CAN_MAGZ])

    def m_compensation(self, bias, sfs, a, b, c):
        sensor = self.m()
        return np.array([[sfs[0], a, b],
                         [a, sfs[1], c],
                         [b, c, sfs[2]]]).dot(np.vstack(sensor + bias)).flatten()

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


def replay_func(run_bool: threading.Event, pause_bool, data, bus, dt, run_kf, dt_sim, 
                init_heading_deg=None, mag_update: bool=True):
    if run_kf:
        print("Running local KF")
        #import pdb; pdb.set_trace()
        kf = kalman_cpp.KalmanCpp()
        if init_heading_deg is not None:
            kf.set_heading(init_heading_deg)
        #for _ in range(400):
        #    kf.predict(dt, data.CANFIX_TAS * KTS2MS)
        #    kf.update_accel(data.a())
        #    kf.update_gyro(data.w() * 2)  #  + np.array([0, -.5, 0])*np.pi/180)
        #    mag = data.m_compensation(mag_bias, mag_sfs, *mag_abc)
        #    kf.update_mag(mag)
    else:
        print("Replaying recorded KF.")

    t = 0
    while run_bool.is_set():
        while pause_bool.is_set():
            time.sleep(.1)
        #print(ctr)
    
        # q = quaternion.from_rotation_vector(np.array([0,0,1])*data.CANFIX_HEAD*np.pi/180) * \
        #     quaternion.from_rotation_vector(np.array([1,0,0])*data.CANFIX_PITCH*np.pi/180) * \
        #     quaternion.from_rotation_vector(np.array([0,1,1])*data.CANFIX_ROLL*np.pi/180) 

        #data.CAN_HEAD_VEC_X = sensor_heading[0]
        #data.CAN_HEAD_VEC_Y = sensor_heading[1]

        if run_kf:
            #import pdb; pdb.set_trace()
            kf.predict(dt, data.CANFIX_TAS * KTS2MS)
            kf.update_accel(data.a())
            #data.CAN_WY = 0.
            #data.CAN_AX = 0.
            kf.update_gyro(data.w())
            if mag_update:
                mag = data.m_compensation(mag_bias, mag_sfs, *mag_abc)
                kf.update_mag(mag)
            es = kf.eulers()*180/np.pi  #roll, pitch, heading
            data.CANFIX_ROLL = es[0]
            data.CANFIX_PITCH = es[1]
            data.CANFIX_HEAD = es[2] + 360 if es[2] < 0 else es[2]
            data.CANFIX_TURNRATE = kf.turn_rate()*180/np.pi
            for n, xyz in zip(range(3), 'XYZ'):
                setattr(data, f'CAN_KF_A{xyz}', kf.a[n])
                setattr(data, f'CAN_KF_W{xyz}', kf.w[n])
                setattr(data, f'CAN_KF_WB{xyz}', kf.wb[n])
                setattr(data, f'CAN_KF_AB{xyz}', kf.ab[n])
                if mag_update:
                    setattr(data, f'CAN_MAG{xyz}', mag[n])
            
        data.CAN_xmit(bus)

        data.read_one()
        #dt = data.CAN_AX_t - t
        #t = data.CAN_AX_t
        #sys.stdout.write(f"{dt:.3f}        \r")
        t += dt
        sys.stdout.write(f"{t:9.1f}        \r")
        time.sleep(dt_sim)
        #ctr += 1


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('-fn', help="candump log created with candump -l")
    parser.add_argument('-o', '--offset', help="Time offset", type=float, default=0)
    parser.add_argument('-dt', default=.029, type=float)
    parser.add_argument('-kf', default=False, action='store_true', help="Run local Kalman filter")
    parser.add_argument('-dts', default=None, type=float, help='Simulation dt')
    parser.add_argument('--no-mag', default=False, action='store_true')
    parser.add_argument('--init-heading', type=float, default=None)
    args = parser.parse_args()
    
    data = DataReader(args.fn)
    bus = can.interface.Bus('vcan0', bustype='socketcan')

    data.read_one()
    t0 = data.CAN_AX_t
    while (data.CAN_AX_t - t0) < args.offset:
        data.read_one()

    print("Offset complete.")
    
    run_bool = threading.Event()
    run_bool.set()
    pause_bool = threading.Event()
    pause_bool.clear()

    # def replay_func(run_bool: threading.Event, pause_bool, data, bus, dt, run_kf, dt_sim, 
    #            init_heading_deg=None, mag_update: bool=True):
    thread = threading.Thread(target=replay_func, args=(run_bool, pause_bool, data, bus, args.dt, args.kf,
                                                        args.dt if args.dts is None else args.dts,
                                                        args.init_heading, not args.no_mag))
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