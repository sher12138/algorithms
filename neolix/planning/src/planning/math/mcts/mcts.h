#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <random>
#include <vector>

namespace neodrive {
namespace planning {
namespace mcts {

/**
 * @brief Children of this class should represent game states
 *
 * A game state is the representation of a single point in the game. For
 * instance in chess, it should at least store all pieces and their locations.
 */
class State {
  friend std::ostream& operator<<(std::ostream& strm, State& s) {
    s.Print(strm);
    return strm;
  };

 protected:
  /**
   * @brief Print a human-readable representation of this state
   *
   * Used for debugging MCTS. Implementations should print a human-readable
   * representation of themselves to the provided stream strm. This
   * representation will also be used when outputting a .dot file using the
   * writeDotFile function from graphviz.h.
   *
   * @param strm The stream to print to
   */
  virtual void Print(std::ostream& strm) { strm << this; };

 public:
  virtual ~State() = default;
};

/**
 * @brief Implementations of this class should represent an action a player can
 * execute on a State.
 *
 * An action is something that acts on a State and results in another. For
 * example in chess an action could be to move the queen to g5.
 *
 * <b>Action must implement a copy constructor.</b>
 *
 * @tparam T The State type this Action can be executed on
 */
template <class T>
class Action {
  friend std::ostream& operator<<(std::ostream& strm, Action<T>& a) {
    a.Print(strm);
    return strm;
  };

 protected:
  virtual void Print(std::ostream& strm) { strm << this; };

 public:
  /**
   * @brief Apply this Action on the given State
   *
   * Should transform the given state to a new one according to this action. For
   * example in chess, calling execute could move the king one square.
   *
   * @note Cloning the state is not required
   * @param state The state to execute on
   */
  virtual void Execute(T& state) = 0;

  virtual ~Action() = default;
};

/**
 * @brief Base class for strategies
 *
 * A strategy is a behaviour that can generate an Action depending on a State.
 */
template <class T>
class Strategy {
 protected:
  /** The state a PlayoutStrategy or ExpansionStrategy will act on  */
  T* state;

 public:
  explicit Strategy(T* state) : state(state) {}

  virtual ~Strategy() = default;
};

/**
 * @brief A strategy that lazily generates child states given the parent state
 *
 * This strategy generates actions that are used in the expansion stage of MCTS.
 *
 * @note Implementing classes must have a constructor taking only one parameter
 * of type State
 *
 * @tparam T The type of State this ExpansionStrategy can generate Actions for
 * @tparam A The type of Actions that will be generated
 */
template <class T, class A>
class ExpansionStrategy : public Strategy<T> {
 public:
  explicit ExpansionStrategy(T* state) : Strategy<T>(state) {}

  /**
   * @brief Generate the next action in the sequence of possible ones
   *
   * Generate a action that can be performed on Strategy#state and which has not
   * been returned before by this instance of ExpansionStrategy.
   *
   * @return An Action that has not been returned before, or nullptr if no such
   * Action exists
   */
  virtual A GenerateNext() = 0;

  /**
   * @return True if generateNext() can generate a new Action
   */
  virtual bool CanGenerateNext() const = 0;
};

/**
 * @brief Generate random actions
 *
 * This strategy generates random actions that are used in the playout stage of
 * MCTS.
 *
 * @note Implementing classes must have a constructor taking only one parameter
 * of type State
 *
 * @tparam T The type of State this PlayoutStrategy can generate Actions for
 * @tparam A The type of Actions that will be generated
 */
template <class T, class A>
class PlayoutStrategy : public Strategy<T> {
 public:
  explicit PlayoutStrategy(T* state) : Strategy<T>(state) {}

  /**
   * @brief Generate a random action
   *
   * Generate a random Action that can be performed on Strategy#state.
   *
   * @param action the action to store the result in
   */
  virtual void GenerateRandom(A& action) = 0;
};

/**
 * @brief Adjusts a score being backpropagated
 *
 * When backpropagating a score through the tree it can be adjusted by this
 * class. Backpropagation::updateScore() is called before Node#update() and the
 * result of Backpropagation::updateScore() is passed to Node::update() instead
 * of the score resulting from Scoring.
 *
 * This is useful for e.g. multiplayer games. For example in chess, the score
 * for the current player should not be adjusted while the score for the enemy
 * player should be inverted (a win for the current player is a loss for the
 * enemy player).
 *
 * @tparam T The State type this Backpropagation can calculate updated scores
 * for
 */
template <class T>
class Backpropagation {
 public:
  /**
   * @param state The state the score is currently being updated for
   * @param backpropScore The score being backpropagated resulting from
   * Scoring::score()
   * @return An updated score for the current state
   */
  virtual float UpdateScore(const T& state, float backpropScore) = 0;

  virtual ~Backpropagation() = default;
};

/**
 * @brief check if a state is terminal
 *
 * Checks if a state is terminal, i.e. the end of the game.
 *
 * @tparam T The State type this TeminationCheck can check
 */
template <class T>
class TerminationCheck {
 public:
  /**
   * @return True if the given state can not haven any children, i.e. the end of
   * the game is reached
   */
  virtual bool IsTerminal(const T& state) = 0;

  virtual ~TerminationCheck() = default;
};

/**
 * @brief Calculates the score of a terminal state
 *
 * Calculate the score of a terminal (i.e. end-of-game) state. A score is
 * usually a number between 0 and 1 where 1 is the best possible score. A score
 * is calculated at the end of the playout stage and is then backpropagated.
 * During backpropagation scores can be updated using Backpropagation.
 *
 * @tparam T The State type this Scoring can calculate scores for
 */
template <class T>
class Scoring {
 public:
  /**
   * @brief Calculate a score for a terminal state
   *
   * A score should be high when the state represents a good end result for the
   * current player and low when the end result is poor.
   *
   * @return A score for the given state
   */
  virtual float Score(const T& state) = 0;

  virtual ~Scoring() = default;
};

/**
 * @brief Class used in the internal data structure of MCTS
 *
 * A Node contains all information needed to generate children. It keeps track
 * of its score and the number of times it has been visited. Furthermore it is
 * used to generate new nodes according to the ExpansionStrategy E.
 *
 * @tparam T The State type that is stored in a node
 * @tparam A The type of Action taken to get to this node
 * @tparam E The ExpansionStrategy to use when generating new nodes
 */
template <class T, class A, class E>
class Node {
 public:
  /**
   * @brief Create a new node in the search tree
   *
   * This constructor initializes the nodes and creates a new instance of the
   * ExpansionStrategy passed as template parameter E.
   *
   * @param id An identifier unique to the tree this node is in
   * @param data The state stored in this node
   * @param parent The parent node
   * @param action The action taken to get to this node from the parent node
   */
  Node(unsigned int id, T data, std::shared_ptr<Node<T, A, E>> parent, A action)
      : id_(id),
        data_(std::move(data)),
        parent_(parent),
        action_(std::move(action)),
        expansion_(&this->data_) {}

  /**
   * @return The unique ID of this node
   */
  unsigned int GetID() const { return id_; }

  /**
   * @return The State associated with this Node
   */
  const T& GetData() const { return data_; }

  /**
   * @return This Node's parent or nullptr if no parent exists (this Node is the
   * root)
   */
  std::shared_ptr<Node<T, A, E>> GetParent() const { return parent_; }

  /**
   * @return All children of this Node
   */
  const std::vector<std::shared_ptr<Node<T, A, E>>>& GetChildren() const {
    return children_;
  }

  /**
   * @return The Action to execute on the parent's State to get from the
   * parent's State to this Node's State.
   */
  const A& GetAction() const { return action_; }

  /**
   * @return A new action if there are any remaining, nullptr if not
   */
  A GenerateNextAction() { return expansion_.GenerateNext(); }

  /**
   * @brief Add a child to this Node's children
   * @param child The child to add
   */
  void AddChild(const std::shared_ptr<Node<T, A, E>>& child) {
    children_.push_back(child);
  }

  /**
   * @brief Checks this Node's ActionGenerator if there are more Actions to be
   * generated.
   * @return True if it is still possible to add children
   */
  bool ShouldExpand() const {
    return children_.empty() || expansion_.CanGenerateNext();
  }

  /**
   * @brief Update this Node's score and increment the number of visits.
   * @param score
   */
  void Update(float score) {
    this->score_sum_ += score;
    num_visits_++;
  }

  /**
   * @return The total score divided by the number of visits.
   */
  float GetAvgScore() const { return score_sum_ / num_visits_; }

  /**
   * @return The number of times updateScore(score) was called
   */
  int GetNumVisits() const { return num_visits_; }

 private:
  unsigned int id_;
  T data_;
  std::shared_ptr<Node<T, A, E>> parent_;
  std::vector<std::shared_ptr<Node<T, A, E>>> children_;
  /** Action done to get from the parent to this node */
  A action_;
  E expansion_;
  int num_visits_ = 0;
  float score_sum_ = 0.0F;
};

template <class S, class T>
using strategy_factory = std::unique_ptr<S> (*)(T*);

/**
 * @brief AI search technique for finding the best Action give a certain State
 *
 * The MCTS algorithm has four stages: selection, expansion, playout and
 * backpropagation. This class represents the general framework for executing
 * these stages and uses a number of user-implemented classes which implement
 * the game rules.
 *
 * In the selection stage, MCTS uses the UCT formula to select the best node (or
 * randomly if a node has not been visited often enough, see
 * MCTS::SetMinVisits()) until it finds a node that still has nodes left to be
 * expanded. The UCT formula has one parameter, see MCTS::SetC(). When PROG_HIST
 * is defined, the progressive history heuristic is used to influence the
 * selection based on the success of an action during the playout stage.
 * MCTS::SetW() is used to set the W parameter for progressive history.
 *
 * In the expansion stage an action is requested from the ExpansionStrategy and
 * a node is expanded using that action. When a node is not visited at least T
 * times, expansion is skipped (see MCTS::SetMinT()).
 *
 * In the playout stage, the PlayoutStrategy is used to generate moves until the
 * end of the game is reached. When a terminal state (the end of the game) is
 * encoutered, the score is calculated using Scoring.
 *
 * In the backpropagation stage, Node::Update() is called for each node from the
 * node expanded in the expansion stage to the root node. The score passed to
 * Node::Update() is the one from the call to Scoring::Score() passed to
 * Backpropagation::UpdateScore() for each call to Node::Update().
 *
 * The time that MCTS is allowed to search van be set by MCTS::SetTime().
 *
 * @tparam T The State type this MCTS operates on
 * @tparam A The Action type this MCTS operates on
 * @tparam E The ExpansionStrategy this MCTS uses
 * @tparam P The PlayoutStrategy this MCTS uses
 */
template <class T, class A, class E, class P>
class MCTS {
 public:
  /**
   * @note backprop, termination and scoring will be deleted by this MCTS
   * instance
   */
  MCTS(const T& root_data, strategy_factory<P, T> playout_strategy_factory,
       Backpropagation<T>* back_prop, TerminationCheck<T>* termination,
       Scoring<T>* scoring)
      : back_prop_(back_prop),
        termination_(termination),
        scoring_(scoring),
        root_(std::make_shared<Node<T, A, E>>(0, root_data, nullptr, A())),
        playout_strategy_factory_(playout_strategy_factory) {}

  MCTS(const MCTS& other) = default;
  MCTS(MCTS&& other) noexcept = default;

  MCTS<T, A, E, P>& operator=(const MCTS<T, A, E, P>& other) = default;
  MCTS<T, A, E, P>& operator=(MCTS<T, A, E, P>&& other) noexcept = default;

  /**
   * @brief Runs the MCTS algorithm and searches for the best Action
   *
   * @return The Action found by MCTS
   */
  A CalculateAction() {
    Search();

    // If no expansion took place, simply execute a random action
    auto& children = root_->GetChildren();
    if (children.empty()) {
      A action;
      T state(root_->GetData());
      auto playout = P(&state);
      playout.GenerateRandom(action);
      return action;
    }

    auto best =
        std::max_element(children.begin(), children.end(),
                         [](const std::shared_ptr<Node<T, A, E>>& child1,
                            const std::shared_ptr<Node<T, A, E>>& child2) {
                           return child1->GetAvgScore() < child2->GetAvgScore();
                         });

    return (*best)->GetAction();
  }

  /**
   * Set the allowed computation time in milliseconds
   * @param time In milliseconds
   */
  void SetTime(int time) {
    this->allowed_computation_time_ = std::chrono::milliseconds(time);
  }

  /**
   * @brief Set the c_ parameter of the UCT formula
   * @param new_c The C parameter
   */
  void SetC(float new_c) { this->c_ = new_c; }

  /**
   * @brief Set the minimal number of visits until a node is expanded
   * @param new_min_t the minimal number of visits
   */
  void SetMinT(float new_min_t) { this->min_t_ = new_min_t; }

  /**
   * Set the minimum number of visits until UCT is used instead of random
   * selection during the selection stage.
   * @param new_min_visits The minimal number of visits
   */
  void SetMinVisits(int new_min_visits) { this->min_visits_ = new_min_visits; }

  /**
   * Set the minimum number of iterations required before calculateAction()
   * returns.
   *
   * MCTS will go over time, set using setTime(int), if this number of
   * iterations is not reached.
   *
   * @param iterations The minimum number of iterations
   */
  void SetMinIterations(int iterations) { this->min_iterations_ = iterations; }

  /**
   * Get the root of the MCTS tree. Useful for printing.
   * @see writeDotFile()
   * @return The root of the MCTS tree
   */
  Node<T, A, E>& GetRoot() { return root_; }

  ~MCTS() {
    delete back_prop_;
    delete termination_;
    delete scoring_;
  }

 private:
  void Search() {
    std::chrono::system_clock::time_point old =
        std::chrono::system_clock::now();

    while (std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now() - old) <
               allowed_computation_time_ ||
           iterations_ < min_iterations_) {
      iterations_++;

      /**
       * Selection
       */
      std::shared_ptr<Node<T, A, E>> selected = root_;
      while (!selected->ShouldExpand()) selected = Select(*selected);

      if (termination_->IsTerminal(selected->GetData())) {
        BackProp(*selected, scoring_->Score(selected->GetData()));
        continue;
      }

      /**
       * Expansion
       */
      std::shared_ptr<Node<T, A, E>> expanded;
      int num_visits = selected->GetNumVisits();
      if (num_visits >= min_t_) {
        expanded = ExpandNext(selected);
      } else {
        expanded = selected;
      }

      /**
       * Simulation
       */
      Simulate(*expanded);
    }
  }

  /** Selects the best child node at the given node */
  std::shared_ptr<Node<T, A, E>> Select(const Node<T, A, E>& node) {
    std::shared_ptr<Node<T, A, E>> best = nullptr;
    float best_score = -std::numeric_limits<float>::max();

    auto& children = node.GetChildren();

    // Select randomly if the Node has not been visited often enough
    if (node.GetNumVisits() < min_visits_) {
      std::uniform_int_distribution<uint> distribution(0, children.size() - 1);
      return children[distribution(generator_)];
    }

    // Use the UCT formula for selection
    for (auto& n : children) {
      float score =
          n->GetAvgScore() +
          c_ * (float)sqrt(log(node.GetNumVisits()) / n->GetNumVisits());

      if (score > best_score) {
        best_score = score;
        best = n;
      }
    }

    return best;
  }
  /** Get the next Action for the given Node, execute and add the new Node to
   * the tree. */
  std::shared_ptr<Node<T, A, E>> ExpandNext(
      const std::shared_ptr<Node<T, A, E>>& node) {
    T expanded_data(node->GetData());
    auto action = node->GenerateNextAction();
    action.Execute(expanded_data);
    auto new_node = std::make_shared<Node<T, A, E>>(
        ++current_node_id_, expanded_data, node, action);
    node->AddChild(new_node);
    return new_node;
  }

  /** Simulate until the stopping condition is reached. */
  void Simulate(Node<T, A, E>& node) {
    T state(node.GetData());

    A action;
    // Check if the end of the game is reached and generate the next state if
    // not
    while (!termination_->IsTerminal(state)) {
      auto playout = playout_strategy_factory_(&state);
      playout->GenerateRandom(action);
      action.Execute(state);
    }

    // Score the leaf node (end of the game)
    float s = scoring_->Score(state);

    BackProp(node, s);
  }

  /** Backpropagate a score through the tree */
  void BackProp(Node<T, A, E>& node, float score) {
    node.Update(back_prop_->UpdateScore(node.GetData(), score));

    std::shared_ptr<Node<T, A, E>> current = node.GetParent();
    while (current) {
      current->Update(back_prop_->UpdateScore(current->GetData(), score));
      current = current->GetParent();
    }
  }

 private:
  /** Default thinking time in milliseconds */
  const int DEFAULT_TIME = 500;

  /** MCTS can go over time if it has less than this amount of iterations */
  const int DEFAULT_MIN_ITERATIONS = 0;

  /** Default C for the UCT formula */
  static constexpr float DEFAULT_C = 0.5;

  /** Minimum number of visits until a Node will be expanded */
  const int DEFAULT_MIN_T = 5;

  /** Default number of visits until a node can be selected using UCT instead of
   * randomly */
  const int DEFAULT_MIN_VISITS = 5;

  Backpropagation<T>* back_prop_;
  TerminationCheck<T>* termination_;
  Scoring<T>* scoring_;

  std::shared_ptr<Node<T, A, E>> root_;

  /** The time MCTS is allowed to search */
  std::chrono::milliseconds allowed_computation_time_ =
      std::chrono::milliseconds(DEFAULT_TIME);

  /** MCTS can go over time if it has less than this amount of iterations */
  int min_iterations_ = DEFAULT_MIN_ITERATIONS;

  /** Tunable bias parameter for node selection */
  float c_ = DEFAULT_C;

  /** Minimum number of visits until a Node will be expanded */
  int min_t_ = DEFAULT_MIN_T;

  /** Minimum number of visits until a Node will be selected using the UCT
   * formula, below this number random selection is used */
  int min_visits_ = DEFAULT_MIN_VISITS;

  /** Variable to assign IDs to a node */
  unsigned int current_node_id_ = 0;

  /** The number of search iterations so far */
  unsigned int iterations_ = 0;

  /** Random generator used in node selection */
  std::mt19937 generator_;

  /** A function generating playout strategies */
  strategy_factory<P, T> playout_strategy_factory_;
};

}  // namespace mcts
}  // namespace planning
}  // namespace neodrive
