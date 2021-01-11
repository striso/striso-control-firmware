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

pre, post = inpbuf.split(b"{CONFIGDATA}")
outpbuf = convert_to_carray(pre, "confightm_pre")
outpbuf += convert_to_carray(post, "confightm_post")

with open("config_editor.h", mode='w') as f:
    f.write(outpbuf)