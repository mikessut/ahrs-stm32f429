"""
awk replay
awk '{if (substr($1, 2, 17) > 1607190440) { print $0}}' candump-2020-12-05_102623.log | canplayer vcan0=can0
"""
import numpy as np
from can_debug_monitor import MSGS
from can_set_param import PARAMS, CANFIX_NODE_MSGS_OFFSET
import struct


class DataHolder:

  def __init__(self, log_keys: list, t0=None):
    self.data = {}
    self.log_keys = log_keys
    self.t0 = t0

  def append(self, key, t, value):
    if key not in self.log_keys:
      return
    if key not in self.data.keys():
      self.data[key] = [[t], [value]]
      if self.t0 is None:
        self.t0 = t
    else:
      self.data[key][0].append(t)
      self.data[key][1].append(value)

  def __getattr__(self, key):
    if key in self.data.keys():
      return np.array(self.data[key][0]) - self.t0, np.array(self.data[key][1])

  def __getitem__(self, key):
    if key in self.data.keys():
      return np.array(self.data[key][0]) - self.t0, np.array(self.data[key][1])

  def get(self, keys: list):
    """
    Align them in time
    """
    t = getattr(self, keys[0])[0]
    return t, tuple(np.interp(t, *getattr(self, k)) for k in keys)

  def __repr__(self):
    return f"<DataHolder Contains data for {self.log_keys}>"


def normal_msgs(fn, msg_list, t0=None):
  #alldata = DataHolder(['CAN_MAGX', 'CAN_MAGY', 'CAN_MAGZ'], t0 = 1607189183.264231)
  alldata = DataHolder(msg_list, t0=t0)
  # fn = 'logs/candump-2020-12-05_102623.log'
  with open(fn, 'r') as fid:
      line = fid.readline().strip()
      while len(line) > 0:
        cols = line.split()
        t = float(line[1:18])
        msg_id = int(cols[2][:3], 16)
        #print(line, line[25:28], msg_id)
        if msg_id in MSGS.keys():
          data = bytes.fromhex(cols[2][4:])
          val = MSGS[msg_id].get('unpack_func', lambda x:struct.unpack('f', x)[0])(data)
          alldata.append(MSGS[msg_id]['name'], t, val)
        line = fid.readline().strip()
  return alldata


def cfg_msgs(fn, msg_list, t0=None):
  # cfg messages
  alldata = DataHolder(msg_list, t0=t0)

  with open(fn, 'r') as fid:
    line = fid.readline().strip()
    while len(line) > 0:
      t = float(line[1:18])
      msg_id = int(line[25:28], 16)
      if msg_id > CANFIX_NODE_MSGS_OFFSET:
        data = bytes.fromhex(line[29:])
        if len(data) == 7:
          idx = struct.unpack('B', data[2:3])[0]
          val = struct.unpack('f', data[3:])[0]
          name = next(k for k, v in PARAMS.items() if ('byte2' in v) and v['byte2'] == idx)
          #print(t-t0, name, val)
          alldata.append(name, t, val)
        
      line = fid.readline().strip()
  return alldata
