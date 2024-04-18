#!/usr/bin/python3
from cyber_record.record import Record
import sys, os

filter_channels = [
    "/localization/100hz/localization_pose", 
    "/perception/traffic_light", 
    "/hmi/stop_start",
    "/openapi/planning_interface", 
    "/planning/proxy/DuDriveChassis",
    "/pnc/control",
    "/pnc/global_state",
    "/pnc/prediction",
    "/router/routing",
    "/perception/free_space",
    "/perception/obstacles",
    "/pnc/planning",
    "/pnc/decision",
    "/localization_dr/100hz/localization_dr"
]


class RecordHandler:

  def __init__(self, record):
    self.messages = record.read_messages(filter_channels)
    self.t = 0
    self.topic = None
    self.msg = None

  def ReadMessage(self):
    try:
      for topic, msg, t in self.messages:
        yield (topic, msg, t)
    except StopIteration:
      self.t = sys.float_info.max
      self.topic = None
      self.msg = None
      return (self.topic, self.msg, self.t)

  def RollMsg(self):
    try:
      self.topic, self.msg, self.t = next(self.ReadMessage())
    except StopIteration:
      self.t = sys.float_info.max

  def GetLatestMessage(self):
    if self.t == 0:
      self.RollMsg()
    return self.topic, self.msg, self.t


def DumpMessages():
  min_idx = -2
  while min_idx != -1:
    min_t = sys.float_info.max
    min_idx = -1
    for record_idx in range(len(record_list)):
      topic, msg, t = record_list[record_idx].GetLatestMessage()
      if t != sys.float_info.max and t < min_t:
        min_t = t
        min_idx = record_idx
    if min_idx != -1 and min_idx != -2:
      topic, msg, t = record_list[min_idx].GetLatestMessage()
      record_list[min_idx].RollMsg()
      out_record.write(topic, msg, t)


def InfoErrorRecordFiles(error_list):
  if len(error_list) > 0:
    print('Damaged files list below:')
    print(error_list)


if __name__ == "__main__":
  out_record = Record('out_record.data', 'w')
  file_list = []
  for arg_idx in range(1, len(sys.argv)):
    abs_path = os.path.abspath(sys.argv[arg_idx]) + '/'
    file_list = file_list + [
        abs_path + file for file in os.listdir(sys.argv[arg_idx])
    ]
  inited_list = []
  error_list = []
  record_list = []
  for each_file in file_list:
    try:
      record = Record(each_file)
      record_list.append(RecordHandler(record))
    except:
      error_list.append(each_file)
  DumpMessages()
  out_record.close()
  InfoErrorRecordFiles(error_list)