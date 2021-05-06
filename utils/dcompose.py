"""DCompose layout calculation for the Striso.

Created: 2010
Author: Piers Titus van der Torren <pierstitus@striso.org>
"""

from __future__ import division
import numpy as np
import math

def latin(coord, note_names=None, accidentals=None):
    if note_names == None:
        note_names = ['F', 'C', 'G', 'D', 'A', 'E', 'B']
    if accidentals == None:
        accidentals = ['bb', 'b', '', '#', 'x']
    acc = int(round(coord[1]/7.))
    return note_names[coord[1] - acc*7 + 3] + accidentals[acc+2]

class Button(object):
	def __init__(self, coord, name, octave, pitch, pos):
		self.coord = coord
		self.name = name
		self.octave = octave
		self.pitch = pitch
		self.pos = pos

	def fullname(self):
		return '{}{}'.format(self.name, self.octave)

	def __repr__(self):
		return 'Button(coord="{0.coord}", name="{0.name}", octave={0.octave}, pitch={0.pitch}, pos={0.pos})'.format(self)

class DCompose(object):

	fgen = [1200,700]
	xgen = []
	ygen = []
	board_width = 176
	board_height = 150
	offset_x = 0
	offset_y = 0
	octave_start = 0.8
	min_pitch = -900
	max_pitch = 3300
	min_note = -8
	max_note = 8

	def __init__(self, xgen1=9.):
		self.set_dcompose(xgen1)

	def set_dcompose(self, x):
		x = float(x)
		self.xgen = [0., x]
		octave = 4.5356 * x
		self.ygen = [octave,octave*7/12]

	def flip(self):
		self.ygen = [-n for n in self.ygen]

	def dist(self, coord):
		pos = self.pos(coord)
		return math.sqrt(pos[0]**2 + pos[1]**2)

	def pos(self, coord):
		return (coord[0]*self.xgen[0]+coord[1]*self.xgen[1],
			    coord[0]*self.ygen[0]+coord[1]*self.ygen[1])

	def pitch(self, coord):
		return self.fgen[0] * coord[0] + self.fgen[1] * coord[1]

	def get_notes(self):
		fact = self.fgen[1]/self.fgen[0]
		notes1 = [(-int(math.floor(f*fact+self.octave_start)),f) for f in range(self.min_note, self.max_note+1)]
		p = [self.pitch(n) for n in notes1]
		notes = []
		for o in range(int(math.floor((self.min_pitch - min(p))/1200)), int(math.ceil((self.max_pitch - max(p))/1200))+1):
			notes.extend([(n[0]+o, n[1], o) for n in notes1])
		notes = [n for n in notes if self.min_pitch <= self.pitch(n) <= self.max_pitch]
		return notes

	def get_buttons(self, latin=latin):
		notes = self.get_notes()
		buttons = [Button(n[:2], latin(n), n[2], self.pitch(n), self.pos(n)) for n in notes]
		xs = [b.pos[0] for b in buttons]
		x_mean = (min(xs) + max(xs))/2.
		x_shift = self.board_width / 2. - round(x_mean) + self.offset_x
		ys = [b.pos[1] for b in buttons]
		y_mean = (min(ys) + max(ys))/2.
		y_shift = self.board_height / 2. - round(y_mean) + self.offset_y
		for b in buttons:
			b.pos = [b.pos[0] + x_shift, b.pos[1] + y_shift]
		return buttons


# bas buttons
#xgen = [0.,7.5]
#octave = 19
#ygen = [octave,octave*6/12]

def get_default():
	layout = DCompose(8.5)
	layout.flip()
	layout.octave_start = 0.2
	layout.board_width = 160.
	layout.board_height = 160.
	layout.offset_x = 20.
	layout.offset_y = 20.
	layout.min_pitch = -900+1200
	layout.max_pitch = 3300+1200
	buttons = layout.get_buttons()
	return layout, buttons

def test():
	dc = DCompose()
	sec = [-1,2]
	fif = [0,1]
	four = [1,-1]
	for a in [sec, four, fif]:
	  print(dc.dist(a))

	holes = [(-68,142),(92,142),(92,8),(-68,8),(-68,80),(0,8),(-19.5,72),(16,142),(43.5,68.5),(92,70)]

if __name__ == '__main__':
	test()
