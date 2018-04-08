#!/usr/bin/python3
import os
from os.path import join
import pkgutil
import encodings
import re


def all_encodings():
    modnames = set(
        [modname for importer, modname, ispkg in pkgutil.walk_packages(
            path=[os.path.dirname(encodings.__file__)], prefix='')])
    aliases = set(encodings.aliases.aliases.values())
    return modnames.union(aliases)


all_enc = all_encodings()


def find_encoding(filename):
    for enc in all_enc:
        try:
            with open(filename, encoding=enc) as f:
                # print the encoding and the first 500 characters
                tmp = f.read(500)
                return enc
                # print(enc, f.read(500))
        except Exception:
            pass


def safe_walk(root, target):
    rst = []
    for f in os.listdir(root):
        if f[0] == '.':
            continue
        path = join(root, f)
        if os.path.isdir(path):
            rst += safe_walk(path, target)
        # elif ".hpp" in f or ".cpp" in f:
        elif any([t in f for t in target]):
            rst.append(path)
    return rst


# def replace_code_file():
    # code_files = safe_walk(".")
    # for path in code_files:
        # new = []
        # enc = find_encoding(path)
        # replace = False
        # with open(path, 'r', encoding=enc) as f:
            # old = f.readlines()
            # for l in old:
                # if "#include" in l and "Config.hpp" in l:
                    # new.append('#include "config/Config.hpp"\n')
                    # replace = True
                    # # print("enc:%s, line:%s" % (enc, l))
                # else:
                    # new.append(l)
        # if replace:
            # with open(path, 'w', encoding=enc) as f:
            # # with open("/tmp/replace_tmp.txt", 'w', encoding=enc) as f:
                # f.writelines(new)


def rp_cmake():
    code_files = safe_walk(".", ["CMakeLists.txt"])
    for path in code_files:
        new = []
        enc = find_encoding(path)
        replace = False
        with open(path, 'r', encoding=enc) as f:
            old = f.readlines()
            for l in old:
                if "target_link_libraries" in l:
                    new_l = re.sub("config", "haoLib::config", l)
                    new.append(new_l)
                    # new.append('#include "config/Config.hpp"\n')
                    replace = True
                    # print("enc:%s, line:%s" % (enc, l))
                else:
                    new.append(l)
        if replace:
            with open(path, 'w', encoding=enc) as f:
                f.writelines(new)
