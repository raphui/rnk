#!/usr/bin/env python

from pyfdt.pyfdt import *
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Device tree parser")
    parser.add_argument('filename', help="input dtb filename")
    parser.add_argument('out', help="output dtb filename")
    args = parser.parse_args()

    with open(args.filename, "rb") as file:
        dtb = FdtBlobParse(file)

    fdt = dtb.to_fdt()

    disabled_path = list()

    for (path, node) in fdt.resolve_path('/').walk():
        if "status" in path and "disabled" in node[0]:
            disabled_path.append(path[:-(len("status") + 1)])


    for path in disabled_path:
        node = fdt.resolve_path(path)
        node.get_parent_node().remove(node.get_name())

    with open(args.out, "wb") as file:
        file.write(fdt.to_dtb())
