
#ifndef SMPL_DUAL_PLANNER_INTERFACE_H
#define SMPL_DUAL_PLANNER_INTERFACE_H

// standard includes
#include <map>
#include <memory>
#include <string>
#include <vector>

// system includes
#include <Eigen/Dense>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit_msgs/MotionPlanResponse.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <sbpl/headers.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <visualization_msgs/MarkerArray.h>

// project includes
#include <smpl/collision_checker.h>
#include <smpl/forward.h>
#include <smpl/occupancy_grid.h>
#include <smpl/planning_params.h>
#include <smpl/robot_model.h>
#include <smpl/debug/marker.h>
#include <smpl/graph/robot_planning_space.h>
#include <smpl/heuristic/robot_heuristic.h>

class SBPLPlanner;

namespace smpl {

using PlanningSpaceFactory = std::function<
        std::unique_ptr<RobotPlanningSpace>(
                RobotModel*, CollisionChecker*, const PlanningParams&)>;

using HeuristicFactory = std::function<
        std::unique_ptr<RobotHeuristic>(
                RobotPlanningSpace*, const PlanningParams&)>;

using PlannerFactory = std::function<
        std::unique_ptr<SBPLPlanner>(
                RobotPlanningSpace*, RobotHeuristic*, const PlanningParams&)>;

using GoalConstraints = std::vector<moveit_msgs::Constraints>;

class DualPlannerInterface
{
public:

    DualPlannerInterface(
        RobotModel* robot_dual,
        CollisionChecker* checker_dual,
        RobotModel* robot_first,
        CollisionChecker* checker_first,
        OccupancyGrid* grid);

    ~DualPlannerInterface();

    bool init(const PlanningParams& params);

    bool solve(
        const moveit_msgs::PlanningScene& planning_scene,
        const moveit_msgs::MotionPlanRequest& req,
        moveit_msgs::MotionPlanResponse& res);

    static
    bool SupportsGoalConstraints(
        const GoalConstraints& constraints,
        std::string& why);

    bool canServiceRequest(
        const moveit_msgs::MotionPlanRequest& req,
        moveit_msgs::MotionPlanResponse& res) const;

    auto space_dual() const -> const RobotPlanningSpace* { return m_pspace_dual.get(); }
    auto space_first() const -> const RobotPlanningSpace* { return m_pspace_first.get(); }

    auto search() const -> const SBPLPlanner* { return m_planner.get(); }

    using heuristic_iterator =
            std::map<std::string, std::unique_ptr<RobotHeuristic>>::const_iterator;

    auto heuristics_dual() const -> std::pair<heuristic_iterator, heuristic_iterator> {
        return std::make_pair(begin(m_heuristics_dual), end(m_heuristics_dual));
    }
    auto heuristics_first() const -> std::pair<heuristic_iterator, heuristic_iterator> {
        return std::make_pair(begin(m_heuristics_first), end(m_heuristics_first));
    }

    /// @brief Return planning statistics from the last call to solve.
    ///
    /// Possible keys to statistics include:
    ///     "initial solution planning time"
    ///     "initial epsilon"
    ///     "initial solution expansions"
    ///     "final epsilon planning time"
    ///     "final epsilon"
    ///     "solution epsilon"
    ///     "expansions"
    ///     "solution cost"
    ///
    /// @return The statistics
    auto getPlannerStats() -> std::map<std::string, double>;

    /// \name Visualization
    ///@{

    auto getBfsWallsVisualization(const std::map<std::string, std::unique_ptr<RobotHeuristic>>& heuristics, 
                                  const std::string& label) const -> visual::Marker;
    auto getBfsValuesVisualization(const std::map<std::string, std::unique_ptr<RobotHeuristic>>& heuristics,
                                   const std::string& label) const -> visual::Marker;


    auto makePathVisualization(const std::vector<RobotState>& path) const
        -> std::vector<visual::Marker>;
    ///@}

protected:

    RobotModel* m_robot_dual;
    CollisionChecker* m_checker_dual;
    RobotModel* m_robot_first;
    CollisionChecker* m_checker_first;
    OccupancyGrid* m_grid;

    ForwardKinematicsInterface* m_fk_iface_dual;
    ForwardKinematicsInterface* m_fk_iface_first;

    PlanningParams m_params;

    // params
    bool m_initialized;

    std::map<std::string, PlanningSpaceFactory> m_space_factories;
    std::map<std::string, HeuristicFactory> m_heuristic_factories;
    std::map<std::string, PlannerFactory> m_planner_factories;

    // planner components

    std::unique_ptr<RobotPlanningSpace> m_pspace_dual;
    std::unique_ptr<RobotPlanningSpace> m_pspace_first;
    std::map<std::string, std::unique_ptr<RobotHeuristic>> m_heuristics_dual;
    std::map<std::string, std::unique_ptr<RobotHeuristic>> m_heuristics_first;
    std::unique_ptr<SBPLPlanner> m_planner;

    std::unique_ptr<SBPLPlanner> m_planner_dual;
    std::unique_ptr<SBPLPlanner> m_planner_first;
    // m_dualplanner;

    int m_sol_cost;

    std::string m_planner_id;

    // Set start configuration
    bool setGoal(const GoalConstraints& v_goal_constraints);
    bool setStart(const moveit_msgs::RobotState& state);

    // Retrieve plan from sbpl
    bool plan(double allowed_time, std::vector<RobotState>& path);

    bool parsePlannerID(
        const std::string& planner_id,
        std::vector<std::string>& space_name,
        std::vector<std::string>& heuristic_name,
        std::vector<std::string>& search_name) const;

    bool reinitPlanner(const std::string& planner_id);

    bool reinitPlanner_dual(const std::string& space_name, const std::string& heuristic_name, const std::string& search_name);
    bool reinitPlanner_first(const std::string& space_name, const std::string& heuristic_name, const std::string& search_name);
 

    void postProcessPath(std::vector<RobotState>& path) const;


};

} // namespace smpl

#endif
