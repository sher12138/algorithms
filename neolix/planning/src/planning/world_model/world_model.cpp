#include "world_model.h"

#include "common/environment.h"
#include "common_config/config/common_config.h"
#include "reader_manager/reader_manager.h"
#include "world_model/config/world_model_config.h"
#include "world_model/estimation/estimation.h"
#include "world_model/observer/observer.h"

namespace neodrive {
namespace world_model {

using neodrive::cyber::Node;

WorldModel::WorldModel() {}

WorldModel::~WorldModel() {}

bool WorldModel::Init(std::shared_ptr<neodrive::cyber::Node> &node) {
  if (initialized_) {
    return true;
  }
  LOG_INFO("WorldModel init started.");
  node_ = node;
  if (!common::ReaderManager::Instance()->Init(node_)) {
    LOG_ERROR("ReaderManager init failed!");
    return false;
  }
  neodrive::common::config::CommonConfig::Instance();
  config::WorldModelConfig::Instance();
  if (!coordinate_tranform_->Init(node_)) {
    LOG_ERROR("CoordinateTransform Init failed");
    return false;
  }
  if (!Observer::Instance()->Init(node_)) {
    LOG_ERROR("Observer Init failed");
    return false;
  }
  if (!Estimation::Instance()->Init()) {
    LOG_ERROR("Estimation Init failed");
    return false;
  }

  LOG_INFO("WorldModel init finished.");
  initialized_ = true;
  return true;
}

std::string WorldModel::Name() const { return std::string("WorldModel"); }

void WorldModel::RunOnce() {
  LOG_DEBUG("WorldModel Process started.");
  neodrive::common::config::CommonConfig::Instance()->ReLoadConfigFromJson();
  config::WorldModelConfig::Instance()->ReLoadConfigFromJson();
  Observer::Instance()->Observe();
  Estimation::Instance()->Estimate();
  LOG_DEBUG("WorldModel Process ended.");
}

}  // namespace world_model
}  // namespace neodrive
