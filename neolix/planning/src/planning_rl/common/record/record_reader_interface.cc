#include <iostream>
#include <string>
#include "cyber.h"
#include "message/raw_message.h"
#include "perception_obstacle.pb.h"
#include "record.pb.h"
#include "record/record_message.h"
#include "record/record_reader.h"
#include "record/record_writer.h"

using neodrive::cyber::message::RawMessage;
using ::neodrive::cyber::record::RecordMessage;
using ::neodrive::cyber::record::RecordReader;
using ::neodrive::cyber::record::RecordWriter;
using neodrive::global::perception::PerceptionObstacles;
using PerceptionObstaclesShrPtr = std::shared_ptr<PerceptionObstacles>;
const char CHANNEL_NAME_1[] = "/perception/obstacles";

void test_read(const std::string &readfile) {
  RecordReader reader(readfile);
  RecordMessage message;
  uint64_t msg_count = reader.GetMessageNumber(CHANNEL_NAME_1);
  std::cout << "MSGTYPE: " << reader.GetMessageType(CHANNEL_NAME_1)
            << std::endl;
  std::cout << "NUMS: " << msg_count << std::endl;
  //   std::cout << "MSGDESC: " <<
  //   reader.GetProtoDesc(CHANNEL_NAME_1)<<std::endl;

  //   read all message
  uint64_t i = 0;
  uint64_t valid = 0;
  while (true) {
    if (reader.ReadMessage(&message)) {
      std::cout << "msg[" << i << "]-> "
                << "channel name: "
                << message.channel_name
                // << "; content: " << message.content
                << "; msg time: " << message.time << std::endl;
      valid++;
      if (message.channel_name == CHANNEL_NAME_1) {
        PerceptionObstacles perceptionobstacles;
        perceptionobstacles.ParseFromString(message.content);
        std::cout << perceptionobstacles.perception_obstacle_size()
                  << std::endl;
      }

    } else {
      std::cout << "read msg[" << i << "] failed" << std::endl;
      break;
    }
    i = i + 1;
  }
  std::cout << "static msg=================" << std::endl;
  std::cout << "MSG validmsg:totalcount: " << valid << ":" << msg_count
            << std::endl;
}