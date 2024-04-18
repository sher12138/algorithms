#include <vector>

#include "common/math/math_utils.h"
#include "common/math/smooth.h"
#include "cyber.h"
#include "gtest/gtest.h"
#include "neolix_log.h"
#include "perception_obstacle.pb.h"
#include "time/time.h"

namespace neodrive {
namespace planning_rl {

// using neodrive::global::localization::LocalizationEstimate;
using neodrive::global::perception::PerceptionObstacles;

TEST(ProtoTest, test_init) {}
// std::fstream output("a.1", std::ios::out | std::ios::app | std::ios::binary);
// if (!container_msg_->perception_obstacles->SerializeToOstream(&output)) {
//     std::cout << "Failed to write address book." << std::endl;
//     return -1;
// }
// PerceptionObstacles outp;
// std::fstream input("a.1", std::ios::in | std::ios::binary);
// if (!input) {
//     std::cout << "a.1" << ": File not found.  Creating a new file." <<
//     std::endl;
// } else if (!outp.ParseFromIstream(&input)) {
//     std::cout << "Failed to parse address book." << std::endl;
//     return -1;
// }
// std::cout<<outp.header().sequence_num()<<std::endl;

}  // namespace planning_rl
}  // namespace neodrive