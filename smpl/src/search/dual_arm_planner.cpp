/// \author Roger Pi

#include <smpl/search/dual_arm_planner.h>

#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/debug/visualize.h>
#include <smpl/debug/marker.h>

namespace smpl
{
static const char* SLOG = "dual";

auto DualArmPlanner::makePathVisualization(
    const std::vector<RobotState>& path) const
    -> std::vector<visual::Marker>
{
    std::vector<visual::Marker> ma;

    if (path.empty()) {
        return ma;
    }
    //TODO dual: using checker single
    auto cinc = 1.0f / float(path.size());
    for (size_t i = 0; i < path.size(); ++i) {
        auto markers = m_space_first->collisionChecker()->getCollisionModelVisualization(path[i]);

        for (auto& marker : markers) {
            auto r = 0.1f;
            auto g = cinc * (float)(path.size() - (i + 1));
            auto b = cinc * (float)i;
            marker.color = visual::Color{ r, g, b, 1.0f };
        }

        for (auto& m : markers) {
            ma.push_back(std::move(m));
        }
    }

    for (size_t i = 0; i < ma.size(); ++i) {
        auto& marker = ma[i];
        marker.ns = "trajectory_first";
        marker.id = i;
    }

    return ma;
}

auto DualArmPlanner::makePathGoalVisualization(
    const std::vector<RobotState>& path) const
    -> std::vector<visual::Marker>
{
    std::vector<visual::Marker> ma;

    if (path.empty()) {
        return ma;
    }
    //TODO dual: using checker single
    auto cinc = 1.0f / float(path.size());
    for (size_t i = 0; i < path.size(); ++i) {
        auto markers = m_space_first->collisionChecker()->getCollisionModelVisualization(path[i]);

        for (auto& marker : markers) {
            auto r = 0.5f;
            auto g = cinc * (float)(path.size() - (i + 1));
            auto b = 0.0f;
            marker.color = visual::Color{ r, g, b, 1.0f };
        }

        for (auto& m : markers) {
            ma.push_back(std::move(m));
        }
    }

    for (size_t i = 0; i < ma.size(); ++i) {
        auto& marker = ma[i];
        marker.ns = "trajectory_goals_first";
        marker.id = i;
    }

    return ma;
}

DualArmPlanner::DualArmPlanner(SBPLPlanner* planner_first,
    RobotHeuristic* heuristic_first,
    RobotPlanningSpace* space_first,
    SBPLPlanner* planner_dual,
    RobotHeuristic* heuristic_dual,
    RobotPlanningSpace* space_dual)
  : SBPLPlanner(),
    m_planner_first(planner_first),
    m_heuristic_first(heuristic_first),
    m_space_first(space_first),
    m_planner_dual(planner_dual),
    m_heuristic_dual(heuristic_dual),
    m_space_dual(space_dual)
{
  m_time_params.bounded = true;
  m_time_params.improve = true;
  m_time_params.type = TimeParameters::TIME;
  m_time_params.max_expansions_init = 0;
  m_time_params.max_expansions = 0;
  m_time_params.max_allowed_time_init = clock::duration::zero();
  m_time_params.max_allowed_time = clock::duration::zero();

/*
  m_multitip_robot = m_space->robot()->getExtension<smpl::MultiTipRobotInterface>();

  if (!m_multitip_robot)
  {
    SMPL_FATAL("RobotPlanningSpace does not implement MultiTipRobotInterface");
  }
  */
}


enum ReplanResultCode
{
  SUCCESS = 0,
  PARTIAL_SUCCESS,
  START_NOT_SET,
  GOAL_NOT_SET,
  TIMED_OUT,
  EXHAUSTED_OPEN_LIST
};

int DualArmPlanner::replan(double allowed_time_secs, std::vector<int>* solution)
{
  int cost;
  return replan(allowed_time_secs, solution, &cost);
}

int DualArmPlanner::replan(double allowed_time_secs, std::vector<int>* solution, int* solcost)
{
  SMPL_DEBUG_NAMED(SLOG, "Find path to goal");

  if (m_start_state_id < 0)
  {
    SMPL_ERROR_NAMED(SLOG, "Dual Start state not set");
    return !START_NOT_SET;
  }
  if (m_goal_state_id < 0)
  {
    SMPL_ERROR_NAMED(SLOG, "Dual Goal state not set");
    return !GOAL_NOT_SET;
  }

  int cost_1, cost_2;

  //Set 1st arm current tip
  //m_multitip_robot->setPlanningLink("eca_link4");

  //Disable 2nd arm
  /*
  std::vector<std::string> disabled_links;
  disabled_links.push_back("reach_gripper1");
  disabled_links.push_back("reach_gripper2");
  disabled_links.push_back("reach_link4");
  disabled_links.push_back("reach_link3");
  disabled_links.push_back("reach_link2");
  disabled_links.push_back("reach_link1");
  disabled_links.push_back("reach_base");
  disabled_links.push_back("extension");
  disabled_links.push_back("pantilt_end");
  disabled_links.push_back("pantilt_pan");
  m_space->collisionChecker()->setDisabledLinks(disabled_links);
  m_space->collisionChecker()->useDisabledLinks(true);
  */


  std::vector<int>* solution_aux = new std::vector<int>();
  int res1 = m_planner_first->replan(allowed_time_secs, solution_aux, &cost_1);
  //TODO: Check if  valid

  //Get path
  std::vector<RobotState> path_1, goals_1;
  m_space_first->extractPathDual(*solution_aux, path_1, goals_1);
  SV_SHOW_INFO_NAMED("trajectory1", makePathVisualization(path_1));
  SV_SHOW_INFO_NAMED("trajectory1", makePathGoalVisualization(goals_1));
  SMPL_INFO("FIRST CHAIN SOLVED!!!!");
  ///std::getchar();

  /*
  
  // Add path to planning space for mp
  m_space->clearStates();
  m_space->initPartialPathMP(path_1);
  m_space->setStart(path_1[0]);
  //heuristic updateStart
  /*
  smpl::GoalConstraint goal2;
  goal2.type = smpl::GoalType::XYZ_RPY_GOAL;
  goal2.pose = m_goal.poses[2];
  goal2.xyz_tolerance = m_goal.xyz_tolerance;
  goal2.rpy_tolerance = m_goal.rpy_tolerance;
  m_space->setGoal(goal2);
  */
 
 /*
  //disabled_links.clear();
  m_space->collisionChecker()->setDisabledLinks(disabled_links);
  m_space->collisionChecker()->useDisabledLinks(false);

  m_multitip_robot->setPlanningLink(m_planner_tips[1]);
*/

  SMPL_INFO("INIT SECOND PLANNER");
  SMPL_INFO_STREAM("space 1st: " << m_space_first << " space 2nd: " << m_space_dual);
  SMPL_INFO_STREAM("planner 1st: " << m_planner_first << " plan 2nd: " << m_planner_dual);

  static_cast<ManipLattice*>(m_space_dual)->initPartialPathMP(path_1, goals_1);
  m_planner_dual->wait_debug_ = true; //tmp
  int res2 = m_planner_dual->replan(allowed_time_secs, solution, &cost_2);

  SMPL_ERROR("FINISHED DUAL ARM PLANNER");
  *solcost = cost_1 + cost_2;
  

  return res2;
}

int DualArmPlanner::set_goal(int goal_state_id)
{
  m_planner_first->set_goal(goal_state_id);
  m_planner_dual->set_goal(goal_state_id);
  return 1;
}
int DualArmPlanner::set_start(int start_state_id)
{
  m_planner_first->set_start(start_state_id);
  m_planner_dual->set_start(start_state_id);
  m_start_state_id = start_state_id;
  return 1;
}
int DualArmPlanner::force_planning_from_scratch()
{
  
  m_planner_first->force_planning_from_scratch();
  m_planner_dual->force_planning_from_scratch();
  
  return 0;
}
int DualArmPlanner::set_search_mode(bool bSearchUntilFirstSolution)
{
  // TODO:
  return 0;
}
void DualArmPlanner::costs_changed(const StateChangeQuery& stateChange)
{
  force_planning_from_scratch();
}

}  // namespace smpl