#!/usr/bin/env python3
"""Striso data visualisation tool

Created: 01 Feb 2015
Author: Piers Titus van der Torren <pierstitus@striso.org>
"""

import time

from pyqtgraph.Qt import QtGui, QtCore
import pyqtgraph as pg
import numpy as np
from numpy import r_, c_, s_

import dcompose
import striso_util

class Visualizer(object):
    def __init__(self, update_interval=40, graph_length=1000, graph_mode='max'):
        self.striso = None
        self.calib = None
        self.update_interval = update_interval
        self.graph_mode = graph_mode
        self.graph_length = graph_length
        self.data_pres = np.zeros(self.graph_length)
        self.data_velo = np.zeros(self.graph_length)
        self.ptr = 2
        self.count = 0

    def start(self, calibrate=False):
        self.win = win = pg.GraphicsLayoutWidget()
        win.setWindowTitle('Striso visualizer')
        win.showMaximized()
        pg.setConfigOptions(antialias=True)

        self.striso = striso = striso_util.Striso()
        striso.start(calibrate=calibrate)
        if striso.calib is not None:
            self.calib = striso.calib.strip().split('\n')[-1]
        print(striso.version)
        print(striso.calib)

        self.fact = 1000

        self.cmap = pg.ColorMap([0,1],[[0,0,0,255],[255,255,255,255]])

        self.but_pos = but_pos = np.array([b.pos for b in striso.buttons]) * [1,-1]
        self.plot_striso = win.addViewBox()
        self.plot_striso.setAspectLocked(True)
        self.plotitem_striso = pg.ScatterPlotItem(pos=but_pos, size=14, pxMode=False)
        self.plot_striso.addItem(self.plotitem_striso)
        self.plotitem_strisopos = pg.ScatterPlotItem(pos=but_pos, size=9, pxMode=True, symbol='+')
        self.plot_striso.addItem(self.plotitem_strisopos)
        self.plotitem_info = pg.TextItem('Striso')
        self.plot_striso.addItem(self.plotitem_info)
        # self.plotitem_info.textItem.setPos((2,2))

        p3 = win.addPlot()
        # Use automatic downsampling and clipping to reduce the drawing load
        # p3.setDownsampling(mode='peak')
        # p3.setClipToView(True)
        # p4.setClipToView(True)
        p3.setRange(xRange=[-self.graph_length, 0], yRange=[0,1200])
        # p3.setLimits(xMax=0)
        self.curve_pres = p3.plot(pen=[255,255,0])
        self.curve_pres.setPos(-self.graph_length, 0)

        ## create a new ViewBox, link the right axis to its coordinate system
        p3a = pg.ViewBox()
        p3.showAxis('right')
        p3.scene().addItem(p3a)
        p3.getAxis('right').linkToView(p3a)
        p3a.setXLink(p3)
        p3.getAxis('right').setLabel('velo', color='#00ffff')
        p3a.setRange(yRange=[-1,1])

        ## Handle view resizing
        def updateViews():
            ## view has resized; update auxiliary views to match
            p3a.setGeometry(p3.vb.sceneBoundingRect())

            ## need to re-update linked axes since this was called
            ## incorrectly while views had different shapes.
            ## (probably this should be handled in ViewBox.resizeEvent)
            p3a.linkedViewChanged(p3.vb, p3a.XAxis)

        updateViews()
        p3.vb.sigResized.connect(updateViews)

        self.curve_velo = pg.PlotCurveItem(pen=[0,255,255])
        p3a.addItem(self.curve_velo)
        self.curve_velo.setPos(-self.graph_length, 0)

        self.striso_active = True
        self.striso_curbut = np.argmax(striso.button_state['p'])
        self.striso_state = striso.button_state[self.striso_curbut]

        self.striso.update = self.updatepres
        self.updated = True

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(self.update_interval)

        self.count_time = time.time()

    def stop(self):
        self.striso.stop()

    def updatepres(self, idx):
        p = self.striso.button_state['p'].copy()
        v = self.striso.button_state['v'].copy()
        striso_active = np.any(p)
        if self.striso_active or striso_active:
            if striso_active and not (self.graph_mode == 'first' and self.striso_active):
                self.striso_curbut = np.argmax(p)

            if idx == self.striso_curbut:
                self.data_pres[self.ptr] = p[self.striso_curbut]
                self.data_velo[self.ptr] = v[self.striso_curbut]
                self.ptr += 1
                self.count += 1
                if self.ptr >= len(self.data_pres):
                    self.ptr = 0

            self.updated = True
            self.striso_active = striso_active

    def update(self):
        if self.updated:
            self.striso_state = self.striso.button_state[self.striso_curbut].copy()
            self.curve_pres.setData(np.roll(self.data_pres, -self.ptr)*self.fact)
            self.curve_velo.setData(np.roll(self.data_velo, -self.ptr))
            self.updated = False

            p = self.striso.button_state['p']#.copy()
            # self.plotitem_striso.setBrush(self.cmap.mapToQColor(striso.button_state['p']))
            self.plotitem_striso.setData(pos=self.but_pos, size=14,
                brush=self.cmap.mapToQColor(0.2*(p>0)+0.8*p))
            xy = c_[self.striso.button_state['x'], self.striso.button_state['y']]
            self.plotitem_strisopos.setData(pos=self.but_pos + 7*xy)

            if time.time() - self.count_time > 0.1:
                mps = self.count/(time.time() - self.count_time)
                self.plotitem_info.setText(f'{mps:.0f} mps')
                self.count_time = time.time()
                self.count = 0

## Start Qt event loop unless running in interactive mode.
if __name__ == '__main__':
    import sys
    import argparse
    parser = argparse.ArgumentParser(
        description='Visualize Striso input')
    parser.add_argument(
        '--update-interval', '-i', default=40, type=int,
        help='screen update interval in ms')
    parser.add_argument(
        '--graph-length', '-l', default=1000, type=int,
        help='graph length')
    parser.add_argument(
        '--graph-mode', '-g', default='max',
        help='graph mode [max|first]')
    parser.add_argument(
        '--calibrate', '-c', action='store_true',
        help='enable calibration mode')
    args = parser.parse_args()

    try:
        visualizer = Visualizer(update_interval=args.update_interval,
                                graph_length=args.graph_length,
                                graph_mode=args.graph_mode)
        visualizer.start(calibrate=args.calibrate)

        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            QtGui.QApplication.instance().exec_()
            visualizer.stop()

    finally:
        visualizer.stop()
