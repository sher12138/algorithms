#include "ud_model_predict.h"

namespace neodrive {
namespace planning_rl {

std::vector<std::vector<double>> ud_inference(const FeaturePreparationV2 &fp) {
    utils::TimeLogger time_log_{"ModelPredictTrt"};
    auto vision_before = fp.target_before();
    auto agents_window_availability = fp.agents_window_availability();
    // std::vector<std::vector<std::vector<double>>> ego_input, agent_input, refline_input;
    // int i_limit = 5;
    // int j_limit = 165;
    // int k_limit = 18;
    int refline_points_num = 100;
    int agents_num = 64;
    int history_num = 4;
    int batch_size = 1;
    double ego_input[5][1][18];
    double agent_input[5][64][18];
    double refline_input[5][100][18];
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 1; j++) {
            for (int k = 0; k < 18; k++) {
                ego_input[i][j][k] = vision_before[i][j][k];
            }
        }
        for (int j = 1; j < 65; j++) {
            for (int k = 0; k < 18; k++) {
                agent_input[i][j - 1][k] = vision_before[i][j][k];
            }
        }
        for (int j = 65; j < 165; j++) {
            for (int k = 0; k < 18; k++) {
                refline_input[i][j - 65][k] = vision_before[i][j][k];
            }
        }
    }
    float ego_trajectory_polyline[1][1][5][18];
    float other_agents_polyline[1][64][5][18];
    float refline[1][100][5][18];
    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 1; j++) {
            for (int k = 0; k < 18; k++) {
                ego_trajectory_polyline[0][j][i][k] = ego_input[i][j][k];
            }
        }
        for (int j = 1; j < 65; j++) {
            for (int k = 0; k < 18; k++) {
                other_agents_polyline[0][j][i][k] = agent_input[i][j][k];
            }
        }
        for (int j = 65; j < 165; j++) {
            for (int k = 0; k < 18; k++) {
                refline[0][j][i][k] = refline_input[i][j][k];
            }
        }
    }
    float other_agents_types[1][64];
    float polyline_types[1];
    bool ego_polyline_availability[1][1][5];
    for (int i = 0; i < 64; i++) {
        other_agents_types[0][i] = agent_input[4][i][7];
    }
    polyline_types[0] = 1;
    for (int i = 0; i < 5; i++) {
        ego_polyline_availability[0][0][i] = true;
    }
    int pad_bottom_size = 0;
    if (agents_num > agents_window_availability.size()) {
        pad_bottom_size = agents_num - agents_window_availability.size();
    }
    std::vector<int> pad_list {0, 0, 0, pad_bottom_size};
    auto agents_window_availability1 = ConstrantPad2d(agents_window_availability, pad_list, 0.0);
    bool other_agents_polyline_availability[1][64][5];
    bool refline_availabilities[1][100][5];
    for (int i = 0; i < 64; i++) {
        for (int j = 0; j < 5; j++) {
            if (agents_window_availability1[i][j] > 0) {
                other_agents_polyline_availability[0][i][j] = true;
            } else {
                other_agents_polyline_availability[0][i][j] = false;
            }
        }
    }
    for (int i = 0; i < 100; i++) {
        for (int j = 0; j < 5; j++) {
            refline_availabilities[0][i][j] = true;
        }
    }

    auto *ud_model_evaluator =
        neodrive::urbandrivernet::UrbanDriver_Evaluator::Instance();

    auto output = ud_model_evaluator->Evaluate(
      ego_trajectory_polyline, other_agents_polyline, refline, other_agents_types, polyline_types, ego_polyline_availability,
      other_agents_polyline_availability, refline_availabilities);
    time_log_.RegisterTimeAndPrint("urban driver predicts done");
    LOG_INFO("model_ouput:");
    for (int i = 0; i < 80; i++) {
        LOG_INFO("{}  {}  {}  {}  {}  {}  {}", i, output[0][i][0], output[0][i][1], output[0][i][2], output[0][i][3], output[0][i][4], output[0][i][5]);
    }
    return output[0];
}

}  // namespace planning_rl
}  // namespace neodrive