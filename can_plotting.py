import numpy as np
from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph
from PyQt5.QtCore import pyqtSignal, pyqtSlot
import pyqtgraph as pg
import queue
import threading
import time
import itertools
import can
from can_debug_monitor import MSGS
import struct



class Plotter(pyqtgraph.GraphicsLayoutWidget):

    data_acquired = pyqtSignal(int, float)

    def __init__(self, names):
        super().__init__()
        ### START QtApp #####
        #self.app = QtGui.QApplication([])            # you MUST do this once (initialize things)
        ####################

        win = pg.GraphicsWindow(title="Kalman Filter Scope") # creates a window
        self.win = win
        self.nplots = max([x[1] for x in names]) + 1
        self.ncurves = len(names)
        self.curves2plots = [x[1] for x in names]
        self.sfs = [x[2] for x in names]
        names.extend(['']*(self.ncurves-len(names)))
        self.names = [x[0] for x in names]

        #import pdb; pdb.set_trace()

        self.Xm = []
        self.ptrs = []
        self.ctrs = []
        self.curve = []
        windowWidth = 1000                       # width of the window displaying the curve
        self.windowWidth = windowWidth
        colors = itertools.cycle([{'color': x} for x in 'rgbcmyw'])
        plots = []
        for nn in range(self.nplots):
            plots.append(win.addPlot(row=nn, col=0))  #  Creates PlotItem
            plots[-1].addLegend()

        for nc in range(self.ncurves):            
            self.curve.append(plots[self.curves2plots[nc]].plot(pen=next(colors), name=self.names[nc]))
            self.Xm.append(np.linspace(0, 0, windowWidth*2))
            self.ptrs.append(0)
            self.ctrs.append(-windowWidth)

        self.data_acquired.connect(self.update)
        # self.start_thread_queue()
        #self.app.exec_()

    @pyqtSlot(int, float)
    def update(self, n, value):
        #for n in range(self.ncurves):
        #self.Xm[n][:-1] = self.Xm[n][1:]                      # shift data in the temporal mean 1 sample left
        
        self.Xm[n][self.ptrs[n]+self.windowWidth] = value * self.sfs[n]                 # vector containing the instantaneous values     
        self.ptrs[n] += 1                              # update x position for displaying the curve
        self.ctrs[n] += 1
        if self.ptrs[n] == self.windowWidth:
          self.Xm[n][:self.windowWidth] = self.Xm[n][self.windowWidth:]
          self.ptrs[n] = 0
        self.curve[n].setData(self.Xm[n][self.ptrs[n]:self.ptrs[n]+self.windowWidth])                     # set the curve with this data
        self.curve[n].setPos(self.ctrs[n], 0)                   # set x position in the graph to 0

    def update_thread(self, run_bool):

      bus = can.interface.Bus('vcan0', bustype='socketcan')
      while run_bool.is_set():
        msg = bus.recv(.1)
        if msg is None:
          continue
        if msg.arbitration_id in MSGS.keys():
          msg_name = MSGS[msg.arbitration_id]['name']
          if msg_name in self.names:
            val = MSGS[msg.arbitration_id].get('unpack_func', lambda x: struct.unpack('f', x)[0])(msg.data)
            #p.update([val])
            #print(val)
            p.data_acquired.emit(self.names.index(msg_name), val)


if __name__ == '__main__':
  keys = [f'CAN_W{x}' for x in 'XYZ'] + [f'CAN_A{x}' for x in 'XYZ']
  #keys[-1] += 'x'
  keys = [#('CAN_WX', 0, 180/np.pi),
          #('CAN_KF_WX', 0, 180/np.pi),
          ('CAN_WY', 0, 180/np.pi),
          ('CAN_KF_WY', 0, 180/np.pi),
          #('CAN_WZ', 0, 180/np.pi),
          #('CAN_KF_WZ', 0, 180/np.pi),
          ('CAN_KF_WBX', 1, 180/np.pi),
          ('CAN_KF_WBY', 1, 180/np.pi),
          ('CAN_KF_WBZ', 1, 180/np.pi),
          ('CAN_KF_ABX', 2, 1/9.81),
          ('CAN_KF_ABY', 2, 1/9.81),
          ('CAN_KF_ABZ', 2, 1/9.81),
          #('CAN_AX', 1, 1/9.81),
          #('CAN_KF_AX', 1, 1/9.81),
          #('CAN_AY', 1, 1/9.81),
          #('CAN_KF_AY', 1, 1/9.81),
          #('CANFIX_TURNRATE', 1, 1.0)]
  ]
  p = Plotter(keys)

  run_bool = threading.Event()
  run_bool.set()
  thread = threading.Thread(target=p.update_thread, args=(run_bool, ))
  thread.start()

  QtGui.QApplication.instance().exec_()
  run_bool.clear()
