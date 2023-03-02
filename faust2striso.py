#!/usr/bin/env python3

import re
import subprocess
import os

def touch(fname, times=None):
    with open(fname, 'a'):
        os.utime(fname, times)

def interface2embedded_interface(interface_cpp):
    v_re = re.compile(r'ui_interface->open\w*\("v(_bas|)(\d+)"\);')
    add_re = re.compile(r'ui_interface->add\w*\(("\w*", &\w*),')
    ch_lines = []
    cur_voice = ''
    cur_voice_type = ''
    max_voice = -1
    for line in interface_cpp:
        line = line.strip()
        if line.startswith('ui_interface->open'):
            v = v_re.findall(line)
            if v:
                cur_voice_type = v[0][0]
                cur_voice = v[0][1]
                if int(cur_voice) > max_voice:
                    max_voice = int(cur_voice)
        elif line.startswith('ui_interface->add'):
            par = add_re.findall(line)[0].split(',')
            name = par[0].strip('"')
            loc = par[1].strip()
            if name in ('acc_abs','acc_x','acc_y','acc_z','rot_x','rot_y','rot_z','pedal'):
                ch_lines.append('synth_interface.{} = {};'.format(name, loc))
            elif cur_voice and name in ('note','pres','vpres','but_x','but_y'):
                ch_lines.append('synth_interface{}.{}[{}] = {};'.format(
                    cur_voice_type, name, cur_voice, loc))

    return ch_lines, max_voice

def faust_postprocess(source):

    # disable cmath, gives compile error since ubuntu 15.10
    source = source.replace('#include <cmath>', '//#include <cmath>')

    # remove clone method as it uses the unsupported new operator
    source = source.replace('\tvirtual mydsp* clone() {\n\t\treturn new mydsp();\n\t}', '')

    # replace interface function with simpler version without strings
    start = source.find('virtual void buildUserInterface(UI* ui_interface) {')
    stop = source.find('}',start)
    interface_embedded, max_voice = interface2embedded_interface(source[start:stop].splitlines())

    new_source = ''.join((
        source[:start],
        'virtual void buildUserInterfaceEmbedded() {\n\t\t',
        '\n\t\t'.join(interface_embedded),
        '\n\t',
        source[stop:]))

    return new_source, max_voice

def faust_replace_fixed_sliders(faust_in):
    replace_slider = re.compile(r'hslider\(".*",([^,]*),[^)]*\)')
    return replace_slider.sub(r'\1',faust_in)

def main():
    faust_source = 'synth.dsp'
    faust_template = 'faust_synth_template.cpp'

    tmp_file = faust_source + '.tmp'

    # preprocess: replace hsliders with fixed values, useful for quick testing
    # of settings on the pc but compiling the as fixed on embedded platform.
    # vsliders stay variable.
    with open(tmp_file,'w') as f:
        f.write(faust_replace_fixed_sliders(open(faust_source).read()))

    # run faust compiler
    faust_cpp = subprocess.check_output(['faust','-a',faust_template,tmp_file,'-lang','ocpp']).decode()

    # do some postprocessing for efficient embedded use
    faust_cpp, max_voice = faust_postprocess(faust_cpp)
    with open('synth.cpp','w') as f:
        f.write(faust_cpp)

    # change VOICECOUNT in 'synth.h' if different from voicecount in synth.dsp
    synth_h = open('synth.h').readlines()
    vc_line = [n for n,line in enumerate(synth_h) if line.startswith('#define VOICECOUNT')][0]
    vc = int(synth_h[vc_line].split()[-1])
    if vc != max_voice + 1:
        synth_h[vc_line] = '#define VOICECOUNT {}\n'.format(max_voice+1)
        with open('synth.h', 'w') as f:
            f.writelines(synth_h)
        # make sure synth_contol gets compiled again
        touch('synth_control.cpp')

if __name__ == '__main__':
    main()
