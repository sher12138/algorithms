#include "aeb_switch_proxy.h"

#include "common/data_center/data_center.h"

namespace neodrive {
namespace planning {
void AebSwitchProxy::SetForbidAeb(bool is_forbid) {
  frobid_aeb_.set_value(is_forbid);
}
}  // namespace planning
}  // namespace neodrive
