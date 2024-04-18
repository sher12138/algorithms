#include "cloud_navigation.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "neolix_log.h"

bool RequestHandler::InitCyberverse(std::string map_folder) {
  using namespace neodrive::cyberverse;
  std::string map = "/home/caros/adu_data/" + map_folder + "/map/base_map.bin";
  uint32_t offset = 0;
  // if (hdmap_ != nullptr) delete hdmap_;
  hdmap_ = new HDMapCloud();
  hdmap_->InitSharedMemoryMap(map, buffer_, offset);
  LOG_INFO("memory write size:{}", offset);
  uint32_t read_size = hdmap_->LoadMapInfosFromMemory(buffer_);
  LOG_INFO("memory read size:{}", read_size);
  return true;
}

RequestHandler::RequestHandler(std::string &version, std::string &url) {
  auto folder_name = version.substr(0, version.size() - 7);
  map_folder_ = folder_name;
  system(("mkdir /home/caros/adu_data/" + folder_name).c_str());
  auto fp = fopen(
      ("/home/caros/adu_data/" + folder_name + "/" + version).c_str(), "wb");
  auto curl = curl_easy_init();
  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, fp);
  curl_easy_setopt(curl, CURLOPT_XFERINFOFUNCTION, 0);
  curl_easy_setopt(curl, CURLOPT_XFERINFODATA, 0);
  curl_easy_setopt(curl, CURLOPT_NOPROGRESS, 0);
  curl_easy_perform(curl);
  curl_easy_cleanup(curl);
  fclose(fp);
  system(("tar -xvzf /home/caros/adu_data/" + folder_name + "/" + version +
          " -C /home/caros/adu_data/" + folder_name)
             .c_str());
  std::string basemap_path("/home/caros/adu_data/" + folder_name +
                           "/map/base_map.bin");
  std::string topomap_path("/home/caros/adu_data/" + folder_name +
                           "/topo_map.bin");
  if (Init()) {
    request_ = std::make_shared<neodrive::global::routing::RoutingRequest>();
    is_inited_ = true;
  } else {
    is_inited_ = false;
  }
}

bool RequestHandler::Init() {
  auto drive_strategy = neodrive::common::config::GetDriveStrategy();
  auto drive_stratedy_type_ =
      neodrive::common::config::GetDriveStrategyType(drive_strategy);
  if (drive_stratedy_type_ ==
      neodrive::common::config::DriveStrategyType::ERROR) {
    LOG_ERROR("GetDriveStrategyType::ERROR");
    return false;
  }
  uint32_t total_size = 1000 * 1024 * 1024 * 1;
  buffer_ = new char[total_size];
  InitCyberverse(map_folder_);
  if (hdmap_ == nullptr || hdmap_->GetRoadTopo() == nullptr ||
      hdmap_->GetLaneTopo() == nullptr) {
    LOG_ERROR("null hdmap or topo map: {}, {}, {}", hdmap_ == nullptr);
    return false;
  }
  using namespace neodrive::planning;
  using namespace neodrive::cyberverse;
  ctx_.hdmap = static_cast<HDMap *>(hdmap_);
  ctx_.road_topo = static_cast<RoadTopo *>(hdmap_->GetRoadTopo());
  ctx_.lane_topo = static_cast<LaneTopo *>(hdmap_->GetLaneTopo());
  ctx_.is_nonmotorway_map = IsNonmotorwayMap();

  if (!ctx_.is_nonmotorway_map) {
    auto &navigation_config =
        config::NavigationConfig::Instance()->navigation_config();
    NavigationStrategy type =
        static_cast<NavigationStrategy>(navigation_config.navigation_strategy);
    if (type == NavigationStrategy::ASTAR) {
      navigation_ptr_ = std::make_shared<AStarNavigationProcess>(&ctx_);
    } else if (type == NavigationStrategy::BREADTH_FIRST) {
      navigation_ptr_ = std::make_shared<BreadthFirstNavigationProcess>(&ctx_);
    } else {
      return false;
    }
  } else {
    navigation_ptr_ = std::make_shared<NonMotorwayNavigationProcess>(&ctx_);
  }

  if (!navigation_ptr_->Init()) return false;
  ctx_.is_cloud_navigation = true;
  return true;
}

bool RequestHandler::Process(
    std::shared_ptr<neodrive::global::routing::RoutingRequest> &routing_request,
    std::shared_ptr<neodrive::global::routing::RoutingResult> &routing_response,
    nlohmann::json &response) {
  if (!is_inited_) return false;
  std::lock_guard<std::mutex> lck(mtx_);
  // InitCyberverse(map_folder_);

  if (routing_request->request_type() ==
      neodrive::global::routing::RoutingRequest_RequestType_NAVIGATER) {
    navigation_ptr_->ProcessRoadSeqRequest(routing_request);
    if (ctx_.road_seqs.empty()) return false;
    for (auto &road_seq : ctx_.road_seqs) {
      routing_response->Clear();
      ctx_.Reset();
      ctx_.road_seq = road_seq;
      LOG_INFO("request type:{}", (int)routing_request->request_type());
      navigation_ptr_->Process(routing_request, routing_response);
      bool ret = navigation_ptr_->GenerateKeyWayPoints(routing_request,
                                                       routing_response);
      if (ret) GetRoutingPath(*routing_response, response);
    }
  } else {
    LOG_INFO("request type:{}", (int)routing_request->request_type());
    for (auto &each_pt_seq : routing_request->point_seq()) {
      auto each_request =
          std::make_shared<neodrive::global::routing::RoutingRequest>();
      for (int i = 0; i < each_pt_seq.waypoint_size(); ++i) {
        each_request->add_waypoint()->CopyFrom(each_pt_seq.waypoint(i));
      }
      each_request->set_request_type(
          neodrive::global::routing::RoutingRequest_RequestType_PATH_POINT);

      routing_response->Clear();
      navigation_ptr_->Process(each_request, routing_response);
      bool ret =
          navigation_ptr_->GenerateKeyWayPoints(each_request, routing_response);
      if (ret) GetRoutingPath(*routing_response, response);
    }
  }
  return true;
}

void RequestHandler::GetRoutingPath(
    const neodrive::global::routing::RoutingResult &routing_response,
    nlohmann::json &response) {
  if (!routing_response.has_routing_path() ||
      routing_response.routing_path().path_points_size() == 0 ||
      routing_response.key_points().path_points_size() == 0) {
    return;
  }

  for (const auto &road : routing_response.route()) {
    for (const auto &passage_region : road.road_info().passage_region()) {
      for (const auto &segment : passage_region.segment()) {
        if (segment.start_s() >= segment.end_s()) {
          continue;
        }
        auto path_points = nlohmann::json::array();
        std::vector<neodrive::global::common::PointENU> tmp_points;
        hdmap_->GetPointsBetweenS(std::hash<std::string>()(segment.id()),
                                  segment.start_s(), segment.end_s(),
                                  tmp_points);
        for (auto &pt : tmp_points) {
          auto point = nlohmann::json::array();
          point.push_back(pt.x());
          point.push_back(pt.y());
          point.push_back(pt.z());
          path_points.push_back(point);
        }
        nlohmann::json lane_object;
        lane_object["lane_type"] =
            IsMotorwayLane(std::hash<std::string>()(segment.id()));
        lane_object["path_points"] = path_points;
        response["data"]["routePath"].push_back(lane_object);
      }
    }
  }

  auto keyPoints = nlohmann::json::array();
  for (auto &each_key_point : routing_response.key_points().path_points()) {
    LOG_INFO("size:{}, x,y,z:{},{},{}",
             routing_response.key_points().path_points_size(),
             each_key_point.x(), each_key_point.y(), each_key_point.z());
    keyPoints.push_back(
        {each_key_point.x(), each_key_point.y(), each_key_point.z()});
  }
  response["data"]["keyPoints"].push_back(keyPoints);
}

int RequestHandler::IsMotorwayLane(uint64_t id) {
  auto lane = hdmap_->GetLaneById(id);
  if (lane == nullptr) return 0;
  auto multi_type = lane->LaneMultipleType();
  for (int i = 0; i < multi_type.size(); ++i) {
    // if lane is nonmotorway, return 2
    if (multi_type[i] ==
        static_cast<uint32_t>(neodrive::global::hdmap::Lane::BIKING))
      return 2;
  }
  // lane is motorway
  return 1;
}

bool RequestHandler::IsNonmotorwayMap() {
  std::string lowercaseInput = map_folder_;
  std::transform(lowercaseInput.begin(), lowercaseInput.end(),
                 lowercaseInput.begin(), ::tolower);

  // 检查是否包含特定的城市名
  return (lowercaseInput.find("beijing") != std::string::npos) ||
         (lowercaseInput.find("shanghai") != std::string::npos) ||
         (lowercaseInput.find("yancheng") != std::string::npos);
}

bool RequestHandler::GetPointFloor(double x, double y, double h_threshold,
                                   int &floor_num,
                                   std::vector<double> &floor_heights) {
  std::vector<neodrive::cyberverse::LaneInfoConstPtr> lanes;
  hdmap_->GetLanes({x, y}, 3.0, &lanes);
  if (lanes.empty()) {
    LOG_INFO("pt({},{}) get no lane", x, y);
    return false;
  }
  std::vector<double> heights{};
  for (const auto lane : lanes) {
    neodrive::common::math::Vec2d tmp_pt;
    double s = 0.0, l = 0.0, lane_z = -10;
    lane->GetProjection({x, y}, &s, &l);
    lane->GetSmoothPoint(s, tmp_pt, &lane_z);
    heights.push_back(lane_z);
    LOG_INFO("pt({},{}) get lane z:{}", x, y, lane_z);
  }
  std::sort(heights.begin(), heights.end());
  floor_num = 1;
  double height_sum = 0.0, average_h = 0.0;
  int height_num = 0;
  for (int i = 0; i < heights.size(); ++i) {
    if (i >= 1 && heights[i] - heights[i - 1] > h_threshold) {
      ++floor_num;
      height_sum = 0.0;
      height_num = 0;
      floor_heights.push_back(average_h);
    }
    height_sum += heights[i];
    ++height_num;
    average_h = height_sum / height_num;
    if (i == heights.size() - 1) floor_heights.push_back(average_h);
  }
  return true;
}

static const char WS_URL[] = "/websocket";
struct ws_connection {
  struct mg_connection *conn;
  int update;
  int closing;
};
#define CONNECTIONS 16
static struct ws_connection ws_conn[CONNECTIONS];
static size_t write_data(void *ptr, size_t size, size_t nmemb, void *stream) {
  size_t written = fwrite(ptr, size, nmemb, (FILE *)stream);
  return written;
}

int websocket_connect_handler(const mg_connection *conn, void *userdata) {
  int i;
  LOG_INFO("connect handler\n");
  for (i = 0; i < CONNECTIONS; ++i) {
    if (ws_conn[i].conn == NULL) {
      LOG_INFO("...prep for server {}", i);
      ws_conn[i].conn = (struct mg_connection *)conn;
      ws_conn[i].closing = 0;
      ws_conn[i].update = 0;
      break;
    }
  }
  if (i >= CONNECTIONS) {
    LOG_INFO("Refused connection: Max connections exceeded\n");
    return 1;
  }

  return 0;
}

void websocket_ready_handler(struct mg_connection *conn, void *userdata) {
  int i;
  LOG_INFO("ready handler");
  std::cout << "ready handler" << std::endl;
  for (i = 0; i < CONNECTIONS; ++i) {
    if (ws_conn[i].conn == conn) {
      LOG_INFO("...start server {}", i);
      std::cout << "...start server " << i << std::endl;
      mg_start_thread(ws_server_thread, (void *)(long)i);
      break;
    }
  }
}

void websocket_close_handler(const struct mg_connection *conn, void *userdata) {
  int i;
  // fprintf(stderr, "close handler\n");   /* called for every close, not just
  // websockets */
  for (i = 0; i < CONNECTIONS; ++i) {
    if (ws_conn[i].conn == conn) {
      LOG_INFO("...close server {}", i);
      std::cout << "...close server " << i << std::endl;
      ws_conn[i].closing = 1;
    }
  }
}

int websocket_data_handler(struct mg_connection *conn, int flags, char *data,
                           size_t data_len, void *userdata) {
  int i;
  int wsd;
  for (i = 0; i < CONNECTIONS; ++i) {
    if (ws_conn[i].conn == conn) {
      wsd = i;
      break;
    }
  }
  if (i >= CONNECTIONS) {
    LOG_INFO("Received websocket data from unknown connection");
    std::cout << "Received websocket data from unknown connection" << std::endl;
    return 1;
  }
  if (flags & 0x80) {
    flags &= 0x7f;
    switch (flags) {
      case MG_WEBSOCKET_OPCODE_CONTINUATION:
        LOG_INFO("CONTINUATION...");
        std::cout << "CONTINUATION..." << std::endl;
        break;
      case MG_WEBSOCKET_OPCODE_TEXT: {
        std::string text_data(data, data_len);
        LOG_INFO("TEXT: {}", text_data);
        std::cout << "TEXT: " << text_data << std::endl;
        if (data_len > 20) {
          nlohmann::json param_dispatch = nlohmann::json::parse(text_data);
          auto ret = request_handler(param_dispatch);
          LOG_INFO(ret);
          mg_websocket_write(conn, MG_WEBSOCKET_OPCODE_TEXT, ret.c_str(),
                             ret.size());
        }
        break;
      }
      case MG_WEBSOCKET_OPCODE_BINARY:
        LOG_INFO("BINARY...");
        std::cout << "BINARY..." << std::endl;
        break;
      case MG_WEBSOCKET_OPCODE_CONNECTION_CLOSE:
        LOG_INFO("CLOSE...");
        std::cout << "CLOSE..." << std::endl;
        /* If client initiated close, respond with close message in
         * acknowlegement */
        if (!ws_conn[wsd].closing) {
          mg_websocket_write(conn, MG_WEBSOCKET_OPCODE_CONNECTION_CLOSE, data,
                             data_len);
          ws_conn[wsd].closing = 1; /* we should not send addional messages when
                                       close requested/acknowledged */
        }
        return 0; /* time to close the connection */
        break;
      case MG_WEBSOCKET_OPCODE_PING:
        /* client sent PING, respond with PONG */
        mg_websocket_write(conn, MG_WEBSOCKET_OPCODE_PONG, data, data_len);
        break;
      case MG_WEBSOCKET_OPCODE_PONG:
        /* received PONG to our PING, no action */
        break;
      default:
        LOG_INFO("Unknown flags: {}", flags);
        std::cout << "Unknown flags: " << flags << std::endl;
        break;
    }
  }
  return 1; /* keep connection open */
}

void *ws_server_thread(void *parm) {
  int wsd = (long)parm;
  struct mg_connection *conn = ws_conn[wsd].conn;
  uint64_t timer = 0;
  LOG_INFO("ws_server_thread {}", wsd);
  std::cout << "ws_server_thread " << wsd << std::endl;
  while (!ws_conn[wsd].closing) {
    usleep(10000); /* 0.01 second */
    timer++;
    if (ws_conn[wsd].update) {
      // mg_websocket_write(conn, MG_WEBSOCKET_OPCODE_TEXT, tstr, strlen(tstr));
    }
    if (timer > 500 && !ws_conn[wsd].closing) {
      timer = 0;
      mg_websocket_write(conn, MG_WEBSOCKET_OPCODE_PING, NULL, 0);
    }
  }

  LOG_INFO("ws_server_thread {} exiting", wsd);
  ws_conn[wsd].conn = NULL;
  ws_conn[wsd].update = 0;
  ws_conn[wsd].closing = 2;

  return NULL;
}

std::string request_handler(nlohmann::json &json_obj) {
  nlohmann::json response;
  std::string url = json_obj["address"];
  std::string version = json_obj["version"];
  response["address"] = url;
  response["version"] = version;
  response["type"] = json_obj["type"];
  response["timestamp"] = json_obj["timestamp"];
  response["data"]["stamp"] = json_obj["data"]["stamp"];
  response["data"]["uuid"] = json_obj["data"]["uuid"];
  response["data"]["requestType"] = json_obj["data"]["requestType"];
  response["data"]["routePath"] = nlohmann::json::array();
  response["data"]["keyPoints"] = nlohmann::json::array();
  auto handler = HandlerPool::Instacne().GetHandler(version, url);
  auto request = std::make_shared<neodrive::global::routing::RoutingRequest>();
  auto result = std::make_shared<neodrive::global::routing::RoutingResult>();
  auto context = handler->GetContext();
  context->request_type = json_obj["data"]["requestType"];
  if (json_obj["data"]["requestType"] == 0) {
    LOG_INFO("get multi-waypoints type request");
    for (auto pointSeq : json_obj["data"]["pointSeqs"]) {
      auto each_pt_seq = request->add_point_seq();
      auto start = pointSeq["start"];
      auto end = pointSeq["end"];

      auto start_pt = each_pt_seq->add_waypoint();
      start_pt->mutable_pose()->set_x(start["x"]);
      start_pt->mutable_pose()->set_y(start["y"]);
      start_pt->mutable_pose()->set_z(-1000);
      start_pt->set_type(to_string(start["type"]));
      for (auto &each_waypoint : pointSeq["waypoints"]) {
        auto waypoint_pt = each_pt_seq->add_waypoint();
        waypoint_pt->mutable_pose()->set_x(each_waypoint["x"]);
        waypoint_pt->mutable_pose()->set_y(each_waypoint["y"]);
        waypoint_pt->mutable_pose()->set_z(-1000);
        waypoint_pt->set_type(to_string(each_waypoint["type"]));
      }
      auto end_pt = each_pt_seq->add_waypoint();
      end_pt->mutable_pose()->set_x(end["x"]);
      end_pt->mutable_pose()->set_y(end["y"]);
      end_pt->mutable_pose()->set_z(-1000);
      end_pt->set_type(to_string(end["type"]));
    }
    request->set_request_type(
        neodrive::global::routing::RoutingRequest_RequestType_PATH_POINT);
  } else if (json_obj["data"]["requestType"] == 1) {
    LOG_INFO("get road-id seq type request");
    for (auto road_seq : json_obj["data"]["roadSeqs"]) {
      auto add_road_seq = request->add_road_seq();
      for (auto road_id : road_seq) {
        add_road_seq->add_road_id(road_id);
      }
    }
    context->motorway_pts_vec.clear();
    for (auto motorway_seg : json_obj["data"]["motorwayPoints"]) {
      std::vector<neodrive::common::math::Vec2d> motorway_pts{};
      for (auto motorway_sub_seg : motorway_seg) {
        for (auto motorway_pt : motorway_sub_seg)
          motorway_pts.push_back({motorway_pt[0], motorway_pt[1]});
      }
      context->motorway_pts_vec.push_back(motorway_pts);
    }
    request->set_request_type(
        neodrive::global::routing::RoutingRequest_RequestType_NAVIGATER);
  } else {
    LOG_INFO("get floor height");
    int floor_num = 0;
    std::vector<double> floor_heights{};
    handler->GetPointFloor(
        json_obj["data"]["floorPoint"][0], json_obj["data"]["floorPoint"][1],
        json_obj["data"]["hThreshold"], floor_num, floor_heights);
    response["data"]["floorNum"] = floor_num;
    response["data"]["floorHeights"] = nlohmann::json::array();
    for (auto h : floor_heights) response["data"]["floorHeights"].push_back(h);
    return response.dump();
  }
  // std::cout << request->DebugString() << std::endl;
  bool ret = handler->Process(request, result, response);
  // std::cout << result->DebugString() << std::endl;
  if (result->error_code().error_id() ==
          neodrive::global::routing::RoutingResult_ErrorCode_ErrorID_SUCCESS &&
      ret) {
    response["data"]["status"] = "success";
  } else {
    response["data"]["status"] = "target_error";
  }
  // std::cout << response.dump(2) << std::endl;
  return response.dump();
}

int main(int argc, char **argv) {
  INIT_NEOLOG_NAME("cloud_navigation");
  neodrive::cyber::Init(argv[0]);
  curl_global_init(CURL_GLOBAL_ALL);
  struct mg_context *ctx;
  struct mg_callbacks callbacks = {0};
  const char *options[] = {"listening_ports", "1108", "document_root",
                           "/home/caros/adu_data", NULL};
  LOG_INFO("11");
  mg_init_library(0);
  LOG_INFO("11");
  ctx = mg_start(&callbacks, NULL, options);
  LOG_INFO("11");
  mg_set_websocket_handler(ctx, WS_URL, websocket_connect_handler,
                           websocket_ready_handler, websocket_data_handler,
                           websocket_close_handler, nullptr);
  neodrive::cyber::WaitForShutdown();
  mg_exit_library();
  LOG_INFO("finish");
  std::cout << "finish" << std::endl;
  return 0;
}
