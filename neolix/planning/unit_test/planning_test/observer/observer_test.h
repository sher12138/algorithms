#include "src/planning/observer/observer.h"

namespace neodrive {
namespace planning {

class ObserverTest {
 public:
  ObserverTest() = default;
  void TestObserveAebMsg();
  void TestGetNewObservedMessages();
  void TestGetNewObservedMessages2();

 private:
  Observer* observer_{Observer::Instance()};
};

}  // namespace planning
}  // namespace neodrive
