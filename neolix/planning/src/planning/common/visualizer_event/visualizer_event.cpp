#include "common/visualizer_event/visualizer_event.h"

#include "cyber/time/time.h"

namespace neodrive {
namespace vis {

EventSender::EventSender() {}

void EventSender::Init(std::shared_ptr<neodrive::cyber::Node> node,
                       const std::string& channel, const std::string& prefix) {
  writer_ = node->CreateWriter<visualizer::Events>(channel);
  CHECK(writer_ != nullptr);
  prefix_ = prefix;
}

visualizer::Event* EventSender::GetEvent(const std::string& id) {
  return writer_ ? &id_events_[id] : &id_events_["dump"];
}

void EventSender::Send() {
  if (!writer_) return;

  auto msg = event_msg_pool_.GetSharedPtr();
  CHECK_NOTNULL(msg);
  msg->mutable_header()->set_timestamp_sec(cyber::Time::Now().ToSecond());
  static unsigned int seq{0};
  msg->mutable_header()->set_sequence_num(++seq);
  msg->clear_event();
  msg->mutable_event()->Reserve(id_events_.size());
  for (auto& [id, event] : id_events_) {
    event.set_id(id);
    std::swap(*msg->mutable_event()->Add(), event);
  }
  id_events_.clear();

  writer_->Write(msg);
}

}  // namespace vis
}  // namespace neodrive
