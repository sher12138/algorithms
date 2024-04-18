#!/usr/bin/python
# -*- coding:utf-8 -*-

import os, sys, subprocess

def main():
  while True:
    cmd = "ls -a"
    result = subprocess.getstatusoutput(cmd)
    if ".git" in result[1]:
      break
    else:
      cmd = "cd .."
      subprocess.call(cmd, shell=True)
  ignore_folder_list = []
  if os.path.exists("./tools/clang_format_ignore"):
    with open("./tools/clang_format_ignore", "r") as fp:
      for line in fp.readlines():
        ignore_folder_list.append(line.strip("\n"))
  cmd = "git show --name-only"
  result = subprocess.getstatusoutput(cmd)
  file_list = result[1].split("\n")
  final_list = file_list[:]
  for path in file_list:
    for ignore_folder in ignore_folder_list:
      if path.startswith(ignore_folder):
        final_list.remove(path)
        break
  for path in final_list:
    file_extensions = [".cpp", ".c", ".cu", ".cc", ".h", ".hpp", ".proto"]
    if any(path.endswith(ext) for ext in file_extensions):
      print("format", path)
      cmd = "/usr/bin/clang-format -style=Google -i " + path
      subprocess.call(cmd, shell=True)

  cmd = "git diff"
  result = subprocess.getstatusoutput(cmd)
  if result[1] != "":
    print("Current branch is not clean!")
    sys.exit(1)
  cmd = "git fetch"
  subprocess.call(cmd, shell=True)
  cmd = "git log origin/dev | head -n 1 | awk '{print $2}'"
  commit_id = subprocess.getstatusoutput(cmd)
  cmd = "git log | grep " + commit_id[1]
  result = subprocess.getstatusoutput(cmd)
  if commit_id[1] not in result[1]:
    print("Curent branch is not synced with latest origin/dev!")
    sys.exit(1)

  cmd = "git status | grep modified"
  result = subprocess.getstatusoutput(cmd)
  print(result[1])
  if "modified" in result[1]:
    print("The code has been formatted with clang-format, please add and push again!")
    sys.exit(1)

if __name__ == "__main__":
  main()