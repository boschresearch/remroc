
// This source code is from libMultiRobotPlanning
//   (https://github.com/whoenig/libMultiRobotPlanning)
// Copyright (c) 2017 whoenig
// This source code is licensed under the MIT license found in the
// 3rd-party-licenses.txt file in the root directory of this source tree.

#pragma once

namespace remroc_cbs_library {

/*! \brief Represents state transations

    This class represents a transition from a start state applying an action
   with the given cost.

    \tparam State Custom state for the search. Needs to be copy'able
    \tparam Action Custom action for the search. Needs to be copy'able
    \tparam Cost Custom Cost type (integer or floating point types)
*/
template <typename State, typename Action, typename Cost>
struct Neighbor {
  Neighbor(const State& state, const Action& action, Cost cost)
      : state(state), action(action), cost(cost) {}

  //! neighboring state
  State state;
  //! action to get to the neighboring state
  Action action;
  //! cost to get to the neighboring state
  Cost cost;
};

}  // namespace libMultiRobotPlanning
