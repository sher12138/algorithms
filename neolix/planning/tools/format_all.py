#!/usr/bin/python
# -*- coding:utf-8 -*-

import os, sys

g_dir = './'

def main():
  for root, dirs, filenames in os.walk(g_dir):
    for filename in filenames:
      (prefix, suffix) = os.path.splitext(filename)
      if ".c" == suffix or ".cu" == suffix or ".cc" == suffix or ".h" == suffix or ".hpp" == suffix or ".cpp" == suffix or ".proto" == suffix:
        cmd = "/usr/bin/clang-format -style=Google -i " + root+"/"+filename
        print(root+"/"+filename)
        os.system(cmd)

if __name__ == '__main__':
  if (len(sys.argv) >= 2):
    g_dir = sys.argv[1]
  sys.exit(main())
