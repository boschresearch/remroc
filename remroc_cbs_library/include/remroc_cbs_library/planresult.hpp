
// This source code is from libMultiRobotPlanning
//   (https://github.com/whoenig/libMultiRobotPlanning)
// Copyright (c) 2017 whoenig
// This source code is licensed under the MIT license found in the
// 3rd-party-licenses.txt file in the root directory of this source tree.
#pragma once

#include <vector>

namespace remroc_cbs_library {

/*! \brief Represents the path for an agent

    This class is used to store the result of a planner for a single agent.
    It has both the ordered list of states that need to be traversed as well as
   the ordered
    list of actions together with their respective costs

    \tparam State Custom state for the search. Needs to be copy'able
    \tparam Action Custom action for the search. Needs to be copy'able
    \tparam Cost Custom Cost type (integer or floating point types)
*/
template <typename State, typename Action, typename Cost>
struct PlanResult {
  //! states and their gScore
  std::vector<std::pair<State, Cost> > states;
  //! actions and their cost
  std::vector<std::pair<Action, Cost> > actions;
  //! actual cost of the result
  Cost cost;
  //! lower bound of the cost (for suboptimal solvers)
  Cost fmin;
};

}  // namespace libMultiRobotPlanning
