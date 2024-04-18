#include "src/planning/math/ilqr_solver/solver.h"

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <memory>
#include <utility>

#include "gtest/gtest.h"
#include "src/planning/math/ilqr_solver/cost_function.h"
#include "src/planning/math/ilqr_solver/dynamic_model.h"
#include "src/planning/math/ilqr_solver/problem.h"
#include "src/planning/math/ilqr_solver/utils.h"

namespace neodrive {
namespace planning {
namespace ilqr {

namespace {
constexpr bool kLogOutput = false;
}  // namespace

TEST(SolverTest, FirstOrderSystem) {
  const int num_steps = 20;
  const double step_size = 0.1;
  auto x_weight = Eigen::Matrix<double, 1, 1>::Ones();
  auto u_weight = Eigen::Matrix<double, 1, 1>::Constant(0.1);
  auto x_target = Eigen::Matrix<double, 1, 1>::Constant(10);
  auto x0 = Eigen::Matrix<double, 1, 1>::Zero();
  {
    Problem<1, 1> problem(x0);
    Problem<1, 1>::VectorOfVecU init_solution;
    for (int i = 0; i < num_steps; ++i) {
      std::unique_ptr<CostFunction<1, 1>> cost =
          std::make_unique<QuadCost<1, 1>>(x_weight, u_weight, x_target);
      std::unique_ptr<DynamicModel<1, 1>> model =
          std::make_unique<FirstOrderModel>(step_size);
      problem.AddStep(std::move(cost), std::move(model));
      init_solution.emplace_back(0.0);
    }
    auto zero_u_weight = Eigen::Matrix<double, 1, 1>::Zero();
    auto terminal_x_weight = Eigen::Matrix<double, 1, 1>::Constant(10);
    std::unique_ptr<CostFunction<1, 1>> terminal_cost =
        std::make_unique<QuadCost<1, 1>>(terminal_x_weight, zero_u_weight,
                                         x_target);
    problem.AddTerminalCost(std::move(terminal_cost));
    problem.AddCandidateInitSolution(std::move(init_solution));
    Solver<1, 1>::Config config;
    int verbose_level = 2;
    config.verbose_level = verbose_level;
    Solver<1, 1> solver(config);
    Solver<1, 1>::Solution solution = solver.Solve(problem);
    EXPECT_TRUE(solution.is_solved);
    EXPECT_EQ(num_steps, solution.u_traj.size());
    EXPECT_EQ(num_steps + 1, solution.x_traj.size());
    // We should reach the final target state value
    EXPECT_NEAR(x_target(0), solution.x_traj.back()(0), 1e-2);
    // The state trajectory should gradually get close to the target state
    for (int i = 0; i < static_cast<int>(solution.x_traj.size()) - 1; ++i) {
      EXPECT_LE(solution.x_traj[i](0), solution.x_traj[i + 1](0));
    }
    if (kLogOutput) {
      std::cout << "x_traj: ";
      for (const auto& val : solution.x_traj) {
        std::cout << val << ", ";
      }
      std::cout << "\n";
    }
  }
  {
    // Same weight, but use a different initial solution.
    Problem<1, 1> problem(x0);
    Problem<1, 1>::VectorOfVecU init_solution;
    for (int i = 0; i < num_steps; ++i) {
      std::unique_ptr<CostFunction<1, 1>> cost =
          std::make_unique<QuadCost<1, 1>>(x_weight, u_weight, x_target);
      std::unique_ptr<DynamicModel<1, 1>> model =
          std::make_unique<FirstOrderModel>(step_size);
      problem.AddStep(std::move(cost), std::move(model));
      init_solution.emplace_back(1.0);
    }
    auto zero_u_weight = Eigen::Matrix<double, 1, 1>::Zero();
    auto terminal_x_weight = Eigen::Matrix<double, 1, 1>::Constant(10);
    std::unique_ptr<CostFunction<1, 1>> terminal_cost =
        std::make_unique<QuadCost<1, 1>>(terminal_x_weight, zero_u_weight,
                                         x_target);
    problem.AddTerminalCost(std::move(terminal_cost));
    problem.AddCandidateInitSolution(std::move(init_solution));
    Solver<1, 1>::Config config;
    int verbose_level = 2;
    config.verbose_level = verbose_level;
    Solver<1, 1> solver(config);
    Solver<1, 1>::Solution solution = solver.Solve(problem);
    EXPECT_TRUE(solution.is_solved);
    EXPECT_EQ(num_steps, solution.u_traj.size());
    EXPECT_EQ(num_steps + 1, solution.x_traj.size());
    // We should reach the final target state value
    EXPECT_NEAR(x_target(0), solution.x_traj.back()(0), 1e-2);
    // The state trajectory should gradually get close to the target state
    for (int i = 0; i < static_cast<int>(solution.x_traj.size()) - 1; ++i) {
      EXPECT_LE(solution.x_traj[i](0), solution.x_traj[i + 1](0));
    }
    if (kLogOutput) {
      std::cout << "x_traj: ";
      for (const auto& val : solution.x_traj) {
        std::cout << val << ", ";
      }
      std::cout << "\n";
    }
  }
}

TEST(SolverTest, SecondOrderSystem) {
  const int num_steps = 20;
  const double step_size = 0.1;
  Eigen::Matrix<double, 2, 1> x_weight;
  x_weight << 1.0, 0.1;
  auto u_weight = Eigen::Matrix<double, 1, 1>::Constant(0.1);
  Eigen::Matrix<double, 2, 1> x_target;
  x_target << 10.0, 0.0;
  auto x0 = Eigen::Matrix<double, 2, 1>::Zero();
  auto zero_u_weight = Eigen::Matrix<double, 1, 1>::Zero();
  Eigen::Matrix<double, 2, 1> terminal_x_weight;
  terminal_x_weight << 100.0, 1.0;

  {
    Problem<2, 1> problem(x0);
    Problem<2, 1>::VectorOfVecU init_solution;
    for (int i = 0; i < num_steps; ++i) {
      std::unique_ptr<CostFunction<2, 1>> cost =
          std::make_unique<QuadCost<2, 1>>(x_weight, u_weight, x_target);
      std::unique_ptr<DynamicModel<2, 1>> model =
          std::make_unique<SecondOrderModel>(step_size);
      problem.AddStep(std::move(cost), std::move(model));
      init_solution.emplace_back(0.0);
    }
    std::unique_ptr<CostFunction<2, 1>> terminal_cost =
        std::make_unique<QuadCost<2, 1>>(terminal_x_weight, zero_u_weight,
                                         x_target);
    problem.AddTerminalCost(std::move(terminal_cost));
    problem.AddCandidateInitSolution(std::move(init_solution));
    Solver<2, 1>::Config config;
    int verbose_level = 2;
    config.verbose_level = verbose_level;
    Solver<2, 1> solver(config);
    Solver<2, 1>::Solution solution = solver.Solve(problem);
    EXPECT_TRUE(solution.is_solved);
    EXPECT_EQ(num_steps, solution.u_traj.size());
    EXPECT_EQ(num_steps + 1, solution.x_traj.size());
    // We should reach the final target state value
    EXPECT_NEAR(x_target(0), solution.x_traj.back()(0), 1e-1);
    // The state trajectory should gradually get close to the target state
    for (int i = 0; i < static_cast<int>(solution.x_traj.size()) - 1; ++i) {
      EXPECT_LE(solution.x_traj[i](0), solution.x_traj[i + 1](0));
    }
    if (kLogOutput) {
      std::cout << "x_traj: ";
      for (const auto& val : solution.x_traj) {
        std::cout << "(" << val(0) << ", " << val(1) << "), ";
      }
      std::cout << "\n";

      std::cout << "u_traj: ";
      for (const auto& val : solution.u_traj) {
        std::cout << val(0) << ", ";
      }
      std::cout << "\n";
    }
  }

  {
    // Same weight, but use a different initial solution.
    Problem<2, 1> problem(x0);
    Problem<2, 1>::VectorOfVecU init_solution;
    for (int i = 0; i < num_steps; ++i) {
      std::unique_ptr<CostFunction<2, 1>> cost =
          std::make_unique<QuadCost<2, 1>>(x_weight, u_weight, x_target);
      std::unique_ptr<DynamicModel<2, 1>> model =
          std::make_unique<SecondOrderModel>(step_size);
      problem.AddStep(std::move(cost), std::move(model));
      init_solution.emplace_back(1.0);
    }
    std::unique_ptr<CostFunction<2, 1>> terminal_cost =
        std::make_unique<QuadCost<2, 1>>(terminal_x_weight, zero_u_weight,
                                         x_target);
    problem.AddTerminalCost(std::move(terminal_cost));
    problem.AddCandidateInitSolution(std::move(init_solution));
    Solver<2, 1>::Config config;
    int verbose_level = 2;
    config.verbose_level = verbose_level;
    Solver<2, 1> solver(config);
    Solver<2, 1>::Solution solution = solver.Solve(problem);
    EXPECT_TRUE(solution.is_solved);
    EXPECT_EQ(num_steps, solution.u_traj.size());
    EXPECT_EQ(num_steps + 1, solution.x_traj.size());
    // We should reach the final target state value
    EXPECT_NEAR(x_target(0), solution.x_traj.back()(0), 1e-1);
    // The state trajectory should gradually get close to the target state
    for (int i = 0; i < static_cast<int>(solution.x_traj.size()) - 1; ++i) {
      EXPECT_LE(solution.x_traj[i](0), solution.x_traj[i + 1](0));
    }
    if (kLogOutput) {
      std::cout << "x_traj: ";
      for (const auto& val : solution.x_traj) {
        std::cout << "(" << val(0) << ", " << val(1) << "), ";
      }
      std::cout << "\n";

      std::cout << "u_traj: ";
      for (const auto& val : solution.u_traj) {
        std::cout << val(0) << ", ";
      }
      std::cout << "\n";
    }
  }

  {
    // Same weight, but set a control_limit of u >= 0.
    Problem<2, 1> problem(x0);
    (*problem.mutable_min_u())(0) = 0.0;
    Problem<2, 1>::VectorOfVecU init_solution;
    for (int i = 0; i < num_steps; ++i) {
      std::unique_ptr<CostFunction<2, 1>> cost =
          std::make_unique<QuadCost<2, 1>>(x_weight, u_weight, x_target);
      std::unique_ptr<DynamicModel<2, 1>> model =
          std::make_unique<SecondOrderModel>(step_size);
      problem.AddStep(std::move(cost), std::move(model));
      init_solution.emplace_back(0.0);
    }
    std::unique_ptr<CostFunction<2, 1>> terminal_cost =
        std::make_unique<QuadCost<2, 1>>(terminal_x_weight, zero_u_weight,
                                         x_target);
    problem.AddTerminalCost(std::move(terminal_cost));
    problem.AddCandidateInitSolution(std::move(init_solution));
    Solver<2, 1>::Config config;
    int verbose_level = 2;
    config.verbose_level = verbose_level;
    Solver<2, 1> solver(config);
    Solver<2, 1>::Solution solution = solver.Solve(problem);
    EXPECT_TRUE(solution.is_solved);
    EXPECT_EQ(num_steps, solution.u_traj.size());
    EXPECT_EQ(num_steps + 1, solution.x_traj.size());
    // We should reach the final target state value
    EXPECT_NEAR(x_target(0), solution.x_traj.back()(0), 2e-1);
    // The state trajectory should gradually get close to the target state
    for (int i = 0; i < static_cast<int>(solution.x_traj.size()) - 1; ++i) {
      EXPECT_LE(solution.x_traj[i](0), solution.x_traj[i + 1](0));
    }
    // The control actions should fulfill the limit set before
    for (const auto& u : solution.u_traj) {
      EXPECT_GE(u(0), 0.0);
    }
    if (kLogOutput) {
      std::cout << "x_traj: ";
      for (const auto& val : solution.x_traj) {
        std::cout << "(" << val(0) << ", " << val(1) << "), ";
      }
      std::cout << "\n";

      std::cout << "u_traj: ";
      for (const auto& val : solution.u_traj) {
        std::cout << val(0) << ", ";
      }
      std::cout << "\n";
    }
  }

  {
    // Same weight, but set a control_limit of u >= 0 and a different initial
    // solution.
    Problem<2, 1> problem(x0);
    (*problem.mutable_min_u())(0) = 0.0;
    Problem<2, 1>::VectorOfVecU init_solution;
    for (int i = 0; i < num_steps; ++i) {
      std::unique_ptr<CostFunction<2, 1>> cost =
          std::make_unique<QuadCost<2, 1>>(x_weight, u_weight, x_target);
      std::unique_ptr<DynamicModel<2, 1>> model =
          std::make_unique<SecondOrderModel>(step_size);
      problem.AddStep(std::move(cost), std::move(model));
      init_solution.emplace_back(200.0);
    }
    std::unique_ptr<CostFunction<2, 1>> terminal_cost =
        std::make_unique<QuadCost<2, 1>>(terminal_x_weight, zero_u_weight,
                                         x_target);
    problem.AddTerminalCost(std::move(terminal_cost));
    problem.AddCandidateInitSolution(std::move(init_solution));
    Solver<2, 1>::Config config;
    int verbose_level = 2;
    config.verbose_level = verbose_level;
    Solver<2, 1> solver(config);
    Solver<2, 1>::Solution solution = solver.Solve(problem);
    EXPECT_TRUE(solution.is_solved);
    EXPECT_EQ(num_steps, solution.u_traj.size());
    EXPECT_EQ(num_steps + 1, solution.x_traj.size());
    // We should reach the final target state value
    EXPECT_NEAR(x_target(0), solution.x_traj.back()(0), 2e-1);
    // The state trajectory should gradually get close to the target state
    for (int i = 0; i < static_cast<int>(solution.x_traj.size()) - 1; ++i) {
      EXPECT_LE(solution.x_traj[i](0), solution.x_traj[i + 1](0));
    }
    // The control actions should fulfill the limit set before
    for (const auto& u : solution.u_traj) {
      EXPECT_GE(u(0), 0.0);
    }
    if (kLogOutput) {
      std::cout << "x_traj: ";
      for (const auto& val : solution.x_traj) {
        std::cout << "(" << val(0) << ", " << val(1) << "), ";
      }
      std::cout << "\n";

      std::cout << "u_traj: ";
      for (const auto& val : solution.u_traj) {
        std::cout << val(0) << ", ";
      }
      std::cout << "\n";
    }
  }
}

TEST(SolverTest, ProjectedNewton) {
  Solver<2, 3>::Config config;
  Solver<2, 3> solver(config);
  Solver<2, 3>::IterationMeta meta;
  Solver<2, 3>::MatUU H_uu_ff_inv = Solver<2, 3>::MatUU::Zero();
  {
    // Minimize a^2 + b^2 + 4 * c^2 - 2 * a,
    // s.t.     -1 <= -4 + a <= 1,
    //          -3 <=  5 + b <= 3,
    //          -2 <= -1 + c <= 2
    Solver<2, 3>::MatUU H_uu;
    // clang-format off
    H_uu << 2.0,  0.0,  0.0,
            0.0,  2.0,  0.0,
            0.0,  0.0,  8.0;
    // clang-format on
    Solver<2, 3>::VecU q_u;
    q_u << -2.0, 0.0, 0.0;
    Solver<2, 3>::VecU u;
    u << -4.0, 5.0, -1.0;
    Solver<2, 3>::ActionLimit action_limit;
    action_limit.lower_limits.emplace_back(Solver<2, 3>::ElementaryActionLimit{
        .index_in_action = 0, .limit_value = -1.0});
    action_limit.lower_limits.emplace_back(Solver<2, 3>::ElementaryActionLimit{
        .index_in_action = 1, .limit_value = -3.0});
    action_limit.lower_limits.emplace_back(Solver<2, 3>::ElementaryActionLimit{
        .index_in_action = 2, .limit_value = -2.0});
    action_limit.upper_limits.emplace_back(Solver<2, 3>::ElementaryActionLimit{
        .index_in_action = 0, .limit_value = 1.0});
    action_limit.upper_limits.emplace_back(Solver<2, 3>::ElementaryActionLimit{
        .index_in_action = 1, .limit_value = 3.0});
    action_limit.upper_limits.emplace_back(Solver<2, 3>::ElementaryActionLimit{
        .index_in_action = 2, .limit_value = 2.0});
    const base::Optional<Solver<2, 3>::VecU> test_du =
        solver.OptimizeActionModification(H_uu, q_u, u, action_limit,
                                          &H_uu_ff_inv, &meta);
    Solver<2, 3>::VecU expect_du;
    expect_du << 3.0, -2.0, 0.0;
    Solver<2, 3>::MatUU expect_H_uu_ff_inv;
    // clang-format off
    expect_H_uu_ff_inv << 1.0,  0.0,  0.0,
                              0.0,  1.0,  0.0,
                              0.0,  0.0,  0.125;
    // clang-format on
    EXPECT_TRUE(test_du);
    EXPECT_NEAR((*test_du - expect_du).norm(), 0.0, 1e-6);
    EXPECT_NEAR((H_uu_ff_inv - expect_H_uu_ff_inv).norm(), 0.0, 1e-6);
    EXPECT_NEAR((H_uu_ff_inv - meta.H_uu_ff_inv_map[3]).norm(), 0.0, 1e-6);
    EXPECT_EQ(std::count(meta.clamped_action_indexes.begin(),
                         meta.clamped_action_indexes.end(), 0),
              1);
    EXPECT_EQ(std::count(meta.clamped_action_indexes.begin(),
                         meta.clamped_action_indexes.end(), 1),
              1);
    EXPECT_EQ(std::count(meta.clamped_action_indexes.begin(),
                         meta.clamped_action_indexes.end(), 2),
              0);
    EXPECT_EQ(meta.final_quadratic_iter_num, 0);
  }
  {
    // Minimize a^2 + b^2 + 4 * c^2 - 2 * a,
    // s.t.     -4 + a >= -1
    //           3 + c <=  2
    Solver<2, 3>::MatUU H_uu;
    // clang-format off
    H_uu << 2.0,  0.0,  0.0,
            0.0,  2.0,  0.0,
            0.0,  0.0,  8.0;
    // clang-format on
    Solver<2, 3>::VecU q_u;
    q_u << -2.0, 0.0, 0.0;
    Solver<2, 3>::VecU u;
    u << -4.0, 5.0, 3.0;
    Solver<2, 3>::ActionLimit action_limit;
    action_limit.lower_limits.emplace_back(Solver<2, 3>::ElementaryActionLimit{
        .index_in_action = 0, .limit_value = -1.0});
    action_limit.upper_limits.emplace_back(Solver<2, 3>::ElementaryActionLimit{
        .index_in_action = 2, .limit_value = 2.0});
    const base::Optional<Solver<2, 3>::VecU> test_du =
        solver.OptimizeActionModification(H_uu, q_u, u, action_limit,
                                          &H_uu_ff_inv, &meta);
    Solver<2, 3>::VecU expect_du;
    expect_du << 3.0, 0.0, -1.0;
    Solver<2, 3>::MatUU expect_H_uu_ff_inv;
    // clang-format off
    expect_H_uu_ff_inv << 1.0,  0.0,  0.0,
                              0.0,  0.5,  0.0,
                              0.0,  0.0,  1.0;
    // clang-format on
    EXPECT_TRUE(test_du);
    EXPECT_NEAR((*test_du - expect_du).norm(), 0.0, 1e-6);
    EXPECT_NEAR((H_uu_ff_inv - expect_H_uu_ff_inv).norm(), 0.0, 1e-6);
    EXPECT_NEAR((H_uu_ff_inv - meta.H_uu_ff_inv_map[5]).norm(), 0.0, 1e-6);
    EXPECT_EQ(std::count(meta.clamped_action_indexes.begin(),
                         meta.clamped_action_indexes.end(), 0),
              1);
    EXPECT_EQ(std::count(meta.clamped_action_indexes.begin(),
                         meta.clamped_action_indexes.end(), 1),
              0);
    EXPECT_EQ(std::count(meta.clamped_action_indexes.begin(),
                         meta.clamped_action_indexes.end(), 2),
              1);
    EXPECT_EQ(meta.final_quadratic_iter_num, 0);
  }
  {
    // Minimize a^2 + b^2 + 4 * c^2 - 2 * a,
    // s.t.     -4 + a <= 1
    Solver<2, 3>::MatUU H_uu;
    // clang-format off
    H_uu << 2.0,  0.0,  0.0,
            0.0,  2.0,  0.0,
            0.0,  0.0,  8.0;
    // clang-format on
    Solver<2, 3>::VecU q_u;
    q_u << -2.0, 0.0, 0.0;
    Solver<2, 3>::VecU u;
    u << -4.0, 5.0, 3.0;
    Solver<2, 3>::ActionLimit action_limit;
    action_limit.upper_limits.emplace_back(Solver<2, 3>::ElementaryActionLimit{
        .index_in_action = 0, .limit_value = 1.0});
    const base::Optional<Solver<2, 3>::VecU> test_du =
        solver.OptimizeActionModification(H_uu, q_u, u, action_limit,
                                          &H_uu_ff_inv, &meta);
    Solver<2, 3>::VecU expect_du;
    expect_du << 1.0, 0.0, 0.0;
    Solver<2, 3>::MatUU expect_H_uu_ff_inv;
    // clang-format off
    expect_H_uu_ff_inv << 0.5,  0.0,  0.0,
                              0.0,  0.5,  0.0,
                              0.0,  0.0,  0.125;
    // clang-format on
    EXPECT_TRUE(test_du);
    EXPECT_NEAR((*test_du - expect_du).norm(), 0.0, 1e-6);
    EXPECT_NEAR((H_uu_ff_inv - expect_H_uu_ff_inv).norm(), 0.0, 1e-6);
    EXPECT_NEAR((H_uu_ff_inv - meta.H_uu_ff_inv_map[0]).norm(), 0.0, 1e-6);
    EXPECT_EQ(std::count(meta.clamped_action_indexes.begin(),
                         meta.clamped_action_indexes.end(), 0),
              0);
    EXPECT_EQ(std::count(meta.clamped_action_indexes.begin(),
                         meta.clamped_action_indexes.end(), 1),
              0);
    EXPECT_EQ(std::count(meta.clamped_action_indexes.begin(),
                         meta.clamped_action_indexes.end(), 2),
              0);
    EXPECT_EQ(meta.final_quadratic_iter_num, 1);
  }
  {
    // Minimize a^2 + a * b + b^2 + 4 * c^2,
    // s.t.     -4 + c >= -1
    Solver<2, 3>::MatUU H_uu;
    // clang-format off
    H_uu << 2.0,  1.0,  0.0,
            1.0,  2.0,  0.0,
            0.0,  0.0,  8.0;
    // clang-format on
    Solver<2, 3>::VecU q_u;
    q_u << 0.0, 0.0, 0.0;
    Solver<2, 3>::VecU u;
    u << 1.0, 2.0, -4.0;
    Solver<2, 3>::ActionLimit action_limit;
    action_limit.lower_limits.emplace_back(Solver<2, 3>::ElementaryActionLimit{
        .index_in_action = 2, .limit_value = -1.0});
    const base::Optional<Solver<2, 3>::VecU> test_du =
        solver.OptimizeActionModification(H_uu, q_u, u, action_limit,
                                          &H_uu_ff_inv, &meta);
    Solver<2, 3>::VecU expect_du;
    expect_du << 0.0, 0.0, 3.0;
    Solver<2, 3>::MatUU expect_H_uu_ff_inv;
    // clang-format off
    expect_H_uu_ff_inv <<  2.0 / 3.0,  -1.0 / 3.0,  0.0,
                              -1.0 / 3.0,   2.0 / 3.0,  0.0,
                                     0.0,         0.0,  1.0;
    // clang-format on
    EXPECT_TRUE(test_du);
    EXPECT_NEAR((*test_du - expect_du).norm(), 0.0, 1e-6);
    EXPECT_NEAR((H_uu_ff_inv - expect_H_uu_ff_inv).norm(), 0.0, 1e-6);
    EXPECT_NEAR((H_uu_ff_inv - meta.H_uu_ff_inv_map[4]).norm(), 0.0, 1e-6);
    EXPECT_EQ(std::count(meta.clamped_action_indexes.begin(),
                         meta.clamped_action_indexes.end(), 0),
              0);
    EXPECT_EQ(std::count(meta.clamped_action_indexes.begin(),
                         meta.clamped_action_indexes.end(), 1),
              0);
    EXPECT_EQ(std::count(meta.clamped_action_indexes.begin(),
                         meta.clamped_action_indexes.end(), 2),
              1);
    EXPECT_EQ(meta.final_quadratic_iter_num, 0);
  }
}

}  // namespace ilqr
}  // namespace planning
}  // namespace neodrive
