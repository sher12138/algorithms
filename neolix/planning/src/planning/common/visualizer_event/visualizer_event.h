/// A singleton to send the debug event
#pragma once

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

#include "cyber.h"
#include "cyber/common/macros.h"
#include "cyber/common/memory_pool.h"
#include "visualizer_event.pb.h"

namespace neodrive {
namespace vis {

/// @class Send the event to channel
class EventSender {
  DECLARE_SINGLETON(EventSender);

 public:
  /// Init the sender, this sender will not work until it is inited
  /// @param node Cyber node
  /// @param channel Cyber channel
  /// @param prefix Add a prefix to event id
  void Init(std::shared_ptr<neodrive::cyber::Node> node,
            const std::string& channel, const std::string& prefix = "");

  /// Get the event with id
  /// @param id The id of event,
  /// @return the event word
  visualizer::Event* GetEvent(const std::string& id);

  /// Send the event to channel
  void Send();

 private:
  std::unordered_map<std::string, visualizer::Event> id_events_{};
  std::shared_ptr<cyber::Writer<visualizer::Events>> writer_{};
  std::string prefix_{};
  neodrive::cyber::common::MemoryPool<visualizer::Events> event_msg_pool_;
};

}  // namespace vis
}  // namespace neodrive
