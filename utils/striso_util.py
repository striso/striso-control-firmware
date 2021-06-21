"""striso_util wrapper for low level communication with the Striso.

Created: 6 Nov 2019
Author: Piers Titus van der Torren <pierstitus@striso.org>
"""

import numpy as np
import sh

import dcompose

striso_util = sh.Command('striso_util')

def int2float(msg):
    # divide msg by 8191, maximum of 14bit signed int
    return [b/0x1fff for b in msg]

def calculate(msg):
    signals = int2float(msg)
    if len(msg) == 4:
        pres = signals[0]
        vpres = signals[1]
        but_x = signals[2]
        but_y = signals[3]
        return [0,0,0,0,0,0, pres, vpres, but_x, but_y]
    elif len(msg) == 6:
        CENTERTEND = 0.02
        pres = (signals[0] + signals[1] + signals[2])/3
        vpres = (signals[3] + signals[4] + signals[5])/3
        m = max(signals[:3])
        if (m > 0.0):
            fact = 1.0/(m + CENTERTEND/m - CENTERTEND)
            but_x = (signals[2] - signals[0]) * fact
            but_y = (0.5 * (signals[0] + signals[2]) - signals[1]) * fact
        else:
            but_x = 0.0
            but_y = 0.0
        return signals + [pres, vpres, but_x, but_y]

class Striso(object):
    def __init__(self, update=None):
        self.layout, self.buttons = dcompose.get_default()
        # but_id = list(range(51)) + [51, 53, 55, 56, 58, 60, 62, 63, 65, 67]
        # but_id = np.r_[44:61,27:44,10:27,0,2,4,5,7,9,11,12,14,16]
        self.button_id = np.r_[51,53,55,56,58,60,62,63,65,67,34:51,17:34,0:17]
        self.button_idx = {n:idx for idx, n in enumerate(self.button_id)}
        self.button_state = np.zeros(len(self.buttons),
            dtype=[('s0','float'),('s1','float'),('s2','float'),
                   ('v0','float'),('v1','float'),('v2','float'),
                   ('p','float'),('v','float'),('x','float'),('y','float')])
        self.motion_state = np.zeros(1,
            dtype=[('ax','float'),('ay','float'),('az','float'),
                   ('rx','float'),('ry','float'),('rz','float'),
                   ('abs','float')])
        self.striso = None
        self.version = None
        self.calib = None
        self.update = update

    def start(self, calibrate=False):
        if self.striso is None:
            # make sure data stream is stopped before getting version
            striso_util('-s')
            self.version = striso_util('-v')
            if calibrate:
                self.calib = striso_util('-C')
            # start striso data stream
            self.striso = striso_util('-T', _out=self.process_output, _bg=True)

    def stop(self):
        if self.striso is not None:
            self.striso.terminate()
            # stop data stream
            striso_util('-s')
            self.striso = None

    def process_output(self, line):
        msg = line.split(',')
        nvalues = len(msg) - 2
        if nvalues < 0:
            return
        msg = [int(n) for n in msg]
        src = msg[0]
        id = msg[1]
        if src == 0 and id in self.button_idx and nvalues in [6, 4]: # dis, old or new format
            idx = self.button_idx[id]
            self.button_state[idx] = tuple(calculate(msg[2:]))
            if self.update is not None:
                self.update(idx)
