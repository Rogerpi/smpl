/// \author: Roger Pi

#ifndef SMPL_DUAL_ARM_PLANNER_H
#define SMPL_DUAL_ARM_PLANNER_H

// standard includes
#include <assert.h>
#include <algorithm>
#include <functional>

// system includes
#include <sbpl/heuristics/heuristic.h>
#include <sbpl/planners/planner.h>
#include <memory>
#include <vector>

// project includes
#include <smpl/graph/robot_planning_space.h>
#include <smpl/graph/manip_lattice.h>
#include <smpl/time.h>

namespace smpl
{
// TODO: BRIEF

class DualArmPlanner : public SBPLPlanner
{
public:
  // parameters for controlling how long the search runs
  // TODO: Handle multiple planners appropiately
  struct TimeParameters
  {
    bool bounded;
    bool improve;
    enum TimingType
    {
      EXPANSIONS,
      TIME,
      USER
    } type;
    int max_expansions_init;
    int max_expansions;
    clock::duration max_allowed_time_init;
    clock::duration max_allowed_time;

    std::function<bool()> timed_out_fun;
  };

  DualArmPlanner(SBPLPlanner* planner_first,
    RobotHeuristic* heuristic_first,
    RobotPlanningSpace* space_first,
    SBPLPlanner* planner_dual,
    RobotHeuristic* heuristic_dual,
    RobotPlanningSpace* space_dual);

  /// \name Required Functions from SBPLPlanner
  ///@{
  int replan(double allowed_time_secs, std::vector<int>* solution) override;
  int replan(double allowed_time_secs, std::vector<int>* solution, int* solcost) override;
  int set_goal(int state_id) override;
  int set_start(int state_id) override;
  int force_planning_from_scratch() override;
  int set_search_mode(bool bSearchUntilFirstSolution) override;
  void costs_changed(const StateChangeQuery& stateChange) override;
  ///@}

private:

  // TODO dual: Temporary viz
  auto makePathVisualization(
    const std::vector<RobotState>& path) const
    -> std::vector<visual::Marker>;

  auto makePathGoalVisualization(
    const std::vector<RobotState>& path) const
    -> std::vector<visual::Marker>;

  ManipLattice* m_space;
  MultiTipRobotInterface* m_multitip_robot;


  RobotHeuristic* m_heuristic_first;
  RobotPlanningSpace* m_space_first;
  SBPLPlanner* m_planner_dual;
  RobotHeuristic* m_heuristic_dual;
  RobotPlanningSpace* m_space_dual;
  SBPLPlanner* m_planner_first;


  TimeParameters m_time_params;

  // std::vector<SearchState*> m_states;

  int m_start_state_id;  // graph state id for the start state
  int m_goal_state_id;   // graph state id for the goal state
};

}  // namespace smpl

#endif  // SMPL_DUAL_ARM_PLANNER_H