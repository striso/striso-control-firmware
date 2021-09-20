#!/usr/bin/env python3

def convert_to_carray(file_content, var_name):
    outp = "const unsigned long %s_len = %d;\n" % (var_name, len(file_content))
    outp += "const unsigned char %s[] __attribute__((aligned(16))) = {" % var_name
    for i in range(len(file_content)):
        if i % 16 == 0:
            outp += "\n"
        outp += "0x%02x, " % file_content[i]
    outp += "\n};\n"
    return outp

with open("config_editor/config.htm", mode='rb') as f:
    inpbuf = f.read()

p1, remainder = inpbuf.split(b"{FWVERSION}")
p2, p3 = remainder.split(b"{CONFIGDATA}")
outpbuf = convert_to_carray(p1, "confightm_p1")
outpbuf += convert_to_carray(p2, "confightm_p2")
outpbuf += convert_to_carray(p3, "confightm_p3")

with open("config_editor/config_editor.h", mode='w') as f:
    f.write(outpbuf)
