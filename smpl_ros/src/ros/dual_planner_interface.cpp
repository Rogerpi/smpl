
#include <smpl/ros/dual_planner_interface.h>

// standard includes
#include <assert.h>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <utility>

// system includes
#include <Eigen/Dense>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <leatherman/utils.h>
#include <sbpl/planners/mhaplanner.h>
#include <smpl/angles.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/debug/visualize.h>
#include <smpl/heuristic/bfs_heuristic.h>
#include <smpl/heuristic/egraph_bfs_heuristic.h>
#include <smpl/heuristic/multi_frame_bfs_heuristic.h>
#include <smpl/heuristic/multi_bfs_heuristic.h>
#include <smpl/heuristic/bfs_with_orientation_heuristic.h>
#include <smpl/post_processing.h>
#include <smpl/stl/memory.h>
#include <smpl/time.h>
#include <smpl/types.h>
#include <trajectory_msgs/JointTrajectory.h>

// project includes
#include <smpl/ros/factories.h>

namespace smpl {

const char* DPI_LOGGER = "dual";

static
bool HasVisibilityConstraints(const moveit_msgs::Constraints& constraints)
{
    return !constraints.visibility_constraints.empty();
}

static
bool HasJointConstraints(const moveit_msgs::Constraints& constraints)
{
    return !constraints.joint_constraints.empty();
}

static
bool HasPositionConstraints(const moveit_msgs::Constraints& constraints)
{
    return !constraints.position_constraints.empty();
}

static
bool HasOrientationConstraints(const moveit_msgs::Constraints& constraints)
{
    return !constraints.orientation_constraints.empty();
}


// a set of constraints representing a goal pose for two different lnks
static
bool IsDualPoseConstraint(const moveit_msgs::Constraints& constraints)
{
    return !HasVisibilityConstraints(constraints) &&
            !HasJointConstraints(constraints) &&
            HasPositionConstraints(constraints) &&
            HasOrientationConstraints(constraints) &&
            constraints.position_constraints.size() == 2 &&
            constraints.orientation_constraints.size() == 2 &&
            constraints.position_constraints[0].link_name ==
                    constraints.orientation_constraints[0].link_name &&
            constraints.position_constraints[1].link_name ==
                    constraints.orientation_constraints[1].link_name &&
            constraints.position_constraints[0].link_name !=
                    constraints.position_constraints[1].link_name;

};

// a set of constraints representing multiple goal poses for different links
static
bool IsDualPoseGoal(const GoalConstraints& goal_constraints)
{
    return goal_constraints.size() == 1 &&
            IsDualPoseConstraint(goal_constraints.front());
};


DualPlannerInterface::DualPlannerInterface(
    RobotModel* robot_dual,
    CollisionChecker* checker_dual,
    RobotModel* robot_first,
    CollisionChecker* checker_first,
    OccupancyGrid* grid)
:
    m_robot_dual(robot_dual),
    m_checker_dual(checker_dual),
    m_robot_first(robot_first),
    m_checker_first(checker_first),
    m_grid(grid),
    m_fk_iface_dual(nullptr),
    m_fk_iface_first(nullptr),
    m_params(),
    m_initialized(false),
    m_pspace_dual(),
    m_pspace_first(),
    m_heuristics_dual(),
    m_heuristics_first(),
    m_planner(),
    m_sol_cost(INFINITECOST),
    m_planner_id()
{
    if (m_robot_first) {
        m_fk_iface_first = m_robot_first->getExtension<ForwardKinematicsInterface>();
    }
    else{
        SMPL_ERROR_STREAM("Robot first not set");
    }
    //TODO dual: is planner link properly set?
    if (m_robot_dual) {
        m_fk_iface_dual = m_robot_dual->getExtension<ForwardKinematicsInterface>();
    }
    else{
        SMPL_ERROR_STREAM("Robot second not set");
    }

    ////////////////////////////////////
    // Setup Planning Space Factories //
    ////////////////////////////////////

    m_space_factories["manip"] = [this](
        RobotModel* r,
        CollisionChecker* c,
        const PlanningParams& p)
    {
        return MakeManipLattice(r, c, p, m_grid);
    };

    m_space_factories["manip_lattice_egraph"] = [this](
        RobotModel* r,
        CollisionChecker* c,
        const PlanningParams& p)
    {
        return MakeManipLatticeEGraph(r, c, p, m_grid);
    };

    m_space_factories["workspace"] = [this](
        RobotModel* r,
        CollisionChecker* c,
        const PlanningParams& p)
    {
        return MakeWorkspaceLattice(r, c, p, m_grid);
    };

    m_space_factories["workspace_egraph"] = [this](
        RobotModel* r,
        CollisionChecker* c,
        const PlanningParams& p)
    {
        return MakeWorkspaceLatticeEGraph(r, c, p, m_grid);
    };

    m_space_factories["adaptive_workspace_lattice"] = [this](
        RobotModel* r,
        CollisionChecker* c,
        const PlanningParams& p)
    {
        return MakeAdaptiveWorkspaceLattice(r, c, p, m_grid);
    };

    ///////////////////////////////
    // Setup Heuristic Factories //
    ///////////////////////////////

    m_heuristic_factories["mbfs"] = [this](
        RobotPlanningSpace* space,
        const PlanningParams& p)
    {
        return MakeMultiBFSHeuristic(space, p, m_grid);
    };
    
    m_heuristic_factories["mfbfs"] = [this](
        RobotPlanningSpace* space,
        const PlanningParams& p)
    {
        return MakeMultiFrameBFSHeuristic(space, p, m_grid);
    };

    m_heuristic_factories["bfs"] = [this](
        RobotPlanningSpace* space,
        const PlanningParams& p)
    {
        return MakeBFSHeuristic(space, p, m_grid);
    };

    m_heuristic_factories["euclid"] = MakeEuclidDistHeuristic;

    m_heuristic_factories["joint_distance"] = MakeJointDistHeuristic;

    m_heuristic_factories["bfs_egraph"] = [this](
        RobotPlanningSpace* space,
        const PlanningParams& p)
    {
        return MakeDijkstraEgraphHeuristic3D(space, p, m_grid);
    };

    m_heuristic_factories["joint_distance_egraph"] = MakeJointDistEGraphHeuristic;

    m_heuristic_factories["bfs_rpy"] = [this](
        RobotPlanningSpace* space,
        const PlanningParams& p)
    {
        return MakeBFSWithOrientationHeuristic(space, p, m_grid);
    };

    /////////////////////////////
    // Setup Planner Factories //
    /////////////////////////////

    m_planner_factories["arastar"] = MakeARAStar;
    m_planner_factories["awastar"] = MakeAWAStar;
    m_planner_factories["mhastar"] = MakeMHAStar;
    m_planner_factories["larastar"] = MakeLARAStar;
    m_planner_factories["egwastar"] = MakeEGWAStar;
    m_planner_factories["padastar"] = MakePADAStar;
    m_planner_factories["marastar"] = MakeMARAStar;
}

DualPlannerInterface::~DualPlannerInterface()
{
}

bool DualPlannerInterface::init(const PlanningParams& params)
{
    SMPL_INFO_NAMED(DPI_LOGGER, "Initialize dual planner interface");

    SMPL_INFO_NAMED(DPI_LOGGER, "  Shortcut Path: %s", params.shortcut_path ? "true" : "false");
    SMPL_INFO_NAMED(DPI_LOGGER, "  Shortcut Type: %s", to_string(params.shortcut_type).c_str());
    SMPL_INFO_NAMED(DPI_LOGGER, "  Interpolate Path: %s", params.interpolate_path ? "true" : "false");

    if (!m_robot_dual) {
        SMPL_ERROR("Dual Robot Model given to Arm Planner Interface must be non-null");
        return false;
    }
    if (!m_robot_first) {
        SMPL_ERROR("First Robot Model given to Arm Planner Interface must be non-null");
        return false;
    }

    if (!m_checker_dual) {
        SMPL_ERROR("Dual Collision Checker given to Arm Planner Interface must be non-null");
        return false;
    }
    if (!m_checker_first) {
        SMPL_ERROR("First Collision Checker given to Arm Planner Interface must be non-null");
        return false;
    }

    if (!m_grid) {
        SMPL_ERROR("Occupancy Grid given to Arm Planner Interface must be non-null");
        return false;
    }

    if (params.cost_per_cell < 0) {
        return false;
    }

    m_params = params;

    m_initialized = true;

    SMPL_INFO_NAMED(DPI_LOGGER, "Initialized dual planner interface");
    return m_initialized;
}

static
void ClearMotionPlanResponse(
    const moveit_msgs::MotionPlanRequest& req,
    moveit_msgs::MotionPlanResponse& res)
{
//    res.trajectory_start.joint_state;
//    res.trajectory_start.multi_dof_joint_state;
//    res.trajectory_start.attached_collision_objects;
    res.trajectory_start.is_diff = false;
    res.group_name = req.group_name;
    res.trajectory.joint_trajectory.header.seq = 0;
    res.trajectory.joint_trajectory.header.stamp = ros::Time(0);
    res.trajectory.joint_trajectory.header.frame_id = "";
    res.trajectory.joint_trajectory.joint_names.clear();
    res.trajectory.joint_trajectory.points.clear();
    res.trajectory.multi_dof_joint_trajectory.header.seq = 0;
    res.trajectory.multi_dof_joint_trajectory.header.stamp = ros::Time(0);
    res.trajectory.multi_dof_joint_trajectory.header.frame_id = "";
    res.trajectory.multi_dof_joint_trajectory.joint_names.clear();
    res.trajectory.multi_dof_joint_trajectory.points.clear();
    res.planning_time = 0.0;
    res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
}

static
bool IsPathValid(CollisionChecker* checker, const std::vector<RobotState>& path)
{
    for (size_t i = 1; i < path.size(); ++i) {
        if (!checker->isStateToStateValid(path[i - 1], path[i])) {
            SMPL_ERROR_STREAM("path between " << path[i - 1] << " and " << path[i] << " is invalid (" << i - 1 << " -> " << i << ")");
            return false;
        }
    }
    return true;
}

static
void ConvertJointVariablePathToJointTrajectory(
    RobotModel* robot,
    const std::vector<RobotState>& path,
    const std::string& joint_state_frame,
    const std::string& multi_dof_joint_state_frame,
    moveit_msgs::RobotTrajectory& traj)
{
    SMPL_INFO("Convert Variable Path to Robot Trajectory");

    traj.joint_trajectory.header.frame_id = joint_state_frame;
    traj.multi_dof_joint_trajectory.header.frame_id = multi_dof_joint_state_frame;

    traj.joint_trajectory.joint_names.clear();
    traj.joint_trajectory.points.clear();
    traj.multi_dof_joint_trajectory.joint_names.clear();
    traj.multi_dof_joint_trajectory.points.clear();

    // fill joint names header for both single- and multi-dof joint trajectories
    auto& variable_names = robot->getPlanningJoints();
    for (auto& var_name : variable_names) {
        std::string joint_name;
        if (IsMultiDOFJointVariable(var_name, &joint_name)) {
            auto it = std::find(
                    begin(traj.multi_dof_joint_trajectory.joint_names),
                    end(traj.multi_dof_joint_trajectory.joint_names),
                    joint_name);
            if (it == end(traj.multi_dof_joint_trajectory.joint_names)) {
                // avoid duplicates
                traj.multi_dof_joint_trajectory.joint_names.push_back(joint_name);
            }
        } else {
            traj.joint_trajectory.joint_names.push_back(var_name);
        }
    }

    SMPL_INFO("  Path includes %zu single-dof joints and %zu multi-dof joints",
            traj.joint_trajectory.joint_names.size(),
            traj.multi_dof_joint_trajectory.joint_names.size());

    // empty or number of points in the path
    if (!traj.joint_trajectory.joint_names.empty()) {
        traj.joint_trajectory.points.resize(path.size());
    }
    // empty or number of points in the path
    if (!traj.multi_dof_joint_trajectory.joint_names.empty()) {
        traj.multi_dof_joint_trajectory.points.resize(path.size());
    }

    for (size_t pidx = 0; pidx < path.size(); ++pidx) {
        auto& point = path[pidx];

        for (size_t vidx = 0; vidx < point.size(); ++vidx) {
            auto& var_name = variable_names[vidx];

            std::string joint_name, local_name;
            if (IsMultiDOFJointVariable(var_name, &joint_name, &local_name)) {
                auto& p = traj.multi_dof_joint_trajectory.points[pidx];
                p.transforms.resize(traj.multi_dof_joint_trajectory.joint_names.size());

                auto it = std::find(
                        begin(traj.multi_dof_joint_trajectory.joint_names),
                        end(traj.multi_dof_joint_trajectory.joint_names),
                        joint_name);
                if (it == end(traj.multi_dof_joint_trajectory.joint_names)) continue;

                auto tidx = std::distance(begin(traj.multi_dof_joint_trajectory.joint_names), it);

                if (local_name == "x" ||
                    local_name == "trans_x")
                {
                    p.transforms[tidx].translation.x = point[vidx];
                } else if (local_name == "y" ||
                    local_name == "trans_y")
                {
                    p.transforms[tidx].translation.y = point[vidx];
                } else if (local_name == "trans_z") {
                    p.transforms[tidx].translation.z = point[vidx];
                } else if (local_name == "theta") {
                    Eigen::Quaterniond q(Eigen::AngleAxisd(point[vidx], Eigen::Vector3d::UnitZ()));
                    tf::quaternionEigenToMsg(q, p.transforms[tidx].rotation);
                } else if (local_name == "rot_w") {
                    p.transforms[tidx].rotation.w = point[vidx];
                } else if (local_name == "rot_x") {
                    p.transforms[tidx].rotation.x = point[vidx];
                } else if (local_name == "rot_y") {
                    p.transforms[tidx].rotation.y = point[vidx];
                } else if (local_name == "rot_z") {
                    p.transforms[tidx].rotation.z = point[vidx];
                } else {
                    SMPL_WARN("Unrecognized multi-dof local variable name '%s'", local_name.c_str());
                    continue;
                }
            } else {
                auto& p = traj.joint_trajectory.points[pidx];
                p.positions.resize(traj.joint_trajectory.joint_names.size());

                auto it = std::find(
                        begin(traj.joint_trajectory.joint_names),
                        end(traj.joint_trajectory.joint_names),
                        var_name);
                if (it == end(traj.joint_trajectory.joint_names)) continue;

                auto posidx = std::distance(begin(traj.joint_trajectory.joint_names), it);

                p.positions[posidx] = point[vidx];
            }
        }
    }
}

static
void ProfilePath(RobotModel* robot, trajectory_msgs::JointTrajectory& traj)
{
    if (traj.points.empty()) {
        return;
    }

    auto& joint_names = traj.joint_names;

    for (size_t i = 1; i < traj.points.size(); ++i) {
        auto& prev_point = traj.points[i - 1];
        auto& curr_point = traj.points[i];

        // find the maximum time required for any joint to reach the next
        // waypoint
        double max_time = 0.0;
        for (size_t jidx = 0; jidx < joint_names.size(); ++jidx) {
            auto from_pos = prev_point.positions[jidx];
            auto to_pos = curr_point.positions[jidx];
            auto vel = robot->velLimit(jidx);
            if (vel <= 0.0) {
                continue;
            }
            auto t = 0.0;
            if (robot->isContinuous(jidx)) {
                t = angles::shortest_angle_dist(from_pos, to_pos) / vel;
            } else {
                t = fabs(to_pos - from_pos) / vel;
            }

            max_time = std::max(max_time, t);
        }

        curr_point.time_from_start = prev_point.time_from_start + ros::Duration(max_time);
    }
}

static
void RemoveZeroDurationSegments(trajectory_msgs::JointTrajectory& traj)
{
    if (traj.points.empty()) {
        return;
    }

    // filter out any duplicate points
    // TODO: find out where these are happening
    size_t end_idx = 1; // current end of the non-filtered range
    for (size_t i = 1; i < traj.points.size(); ++i) {
        auto& prev = traj.points[end_idx - 1];
        auto& curr = traj.points[i];
        if (curr.time_from_start != prev.time_from_start) {
            SMPL_INFO("Move index %zu into %zu", i, end_idx);
            if (end_idx != i) {
                traj.points[end_idx] = std::move(curr);
            }
            end_idx++;
        }
    }
    traj.points.resize(end_idx);
}

static
bool WritePath(
    RobotModel* robot,
    const moveit_msgs::RobotState& ref,
    const moveit_msgs::RobotTrajectory& traj,
    const std::string& path)
{
    boost::filesystem::path p(path);

    try {
        if (!boost::filesystem::exists(p)) {
            SMPL_INFO("Create plan output directory %s", p.native().c_str());
            boost::filesystem::create_directory(p);
        }

        if (!boost::filesystem::is_directory(p)) {
            SMPL_ERROR("Failed to log path. %s is not a directory", path.c_str());
            return false;
        }
    } catch (const boost::filesystem::filesystem_error& ex) {
        SMPL_ERROR("Failed to create plan output directory %s", p.native().c_str());
        return false;
    }

    std::stringstream ss_filename;
    auto now = clock::now();
    ss_filename << "path_" << now.time_since_epoch().count();
    p /= ss_filename.str();

    std::ofstream ofs(p.native());
    if (!ofs.is_open()) {
        return false;
    }

    SMPL_INFO("Log path to %s", p.native().c_str());

    // write header
    for (size_t vidx = 0; vidx < robot->jointVariableCount(); ++vidx) {
        auto& var_name = robot->getPlanningJoints()[vidx];
        ofs << var_name; // TODO: sanitize variable name for csv?
        if (vidx != robot->jointVariableCount() - 1) {
            ofs << ',';
        }
    }
    ofs << '\n';

    auto wp_count = std::max(
            traj.joint_trajectory.points.size(),
            traj.multi_dof_joint_trajectory.points.size());
    for (size_t widx = 0; widx < wp_count; ++widx) {
        // fill the complete robot state
        moveit_msgs::RobotState state = ref;

        if (widx < traj.joint_trajectory.points.size()) {
            auto& wp = traj.joint_trajectory.points[widx];
            auto joint_count = traj.joint_trajectory.joint_names.size();
            for (size_t jidx = 0; jidx < joint_count; ++jidx) {
                auto& joint_name = traj.joint_trajectory.joint_names[jidx];
                auto vp = wp.positions[jidx];
                auto it = std::find(
                        begin(state.joint_state.name),
                        end(state.joint_state.name),
                        joint_name);
                if (it != end(state.joint_state.name)) {
                    auto tvidx = std::distance(begin(state.joint_state.name), it);
                    state.joint_state.position[tvidx] = vp;
                }
            }
        }
        if (widx < traj.multi_dof_joint_trajectory.points.size()) {
            auto& wp = traj.multi_dof_joint_trajectory.points[widx];
            auto joint_count = traj.multi_dof_joint_trajectory.joint_names.size();
            for (size_t jidx = 0; jidx < joint_count; ++jidx) {
                auto& joint_name = traj.multi_dof_joint_trajectory.joint_names[jidx];
                auto& t = wp.transforms[jidx];
                auto it = std::find(
                        begin(state.multi_dof_joint_state.joint_names),
                        end(state.multi_dof_joint_state.joint_names),
                        joint_name);
                if (it != end(state.multi_dof_joint_state.joint_names)) {
                    size_t tvidx = std::distance(begin(state.multi_dof_joint_state.joint_names), it);
                    state.multi_dof_joint_state.transforms[tvidx] = t;
                }
            }
        }

        // write the planning variables out to file
        for (size_t vidx = 0; vidx < robot->jointVariableCount(); ++vidx) {
            auto& var_name = robot->getPlanningJoints()[vidx];

            std::string joint_name, local_name;
            if (IsMultiDOFJointVariable(var_name, &joint_name, &local_name)) {
                auto it = std::find(
                        begin(state.multi_dof_joint_state.joint_names),
                        end(state.multi_dof_joint_state.joint_names),
                        joint_name);
                if (it == end(state.multi_dof_joint_state.joint_names)) continue;

                auto jidx = std::distance(begin(state.multi_dof_joint_state.joint_names), it);
                auto& transform = state.multi_dof_joint_state.transforms[jidx];
                double pos;
                if (local_name == "x" ||
                    local_name == "trans_x")
                {
                    pos = transform.translation.x;
                } else if (local_name == "y" ||
                    local_name == "trans_y")
                {
                    pos = transform.translation.y;
                } else if (local_name == "trans_z") {
                    pos = transform.translation.z;
                } else if (local_name == "theta") {
                    // this list just gets larger:
                    // from sbpl_collision_checking, MoveIt, Bullet, leatherman
                    double s_squared = 1.0 - transform.rotation.w * transform.rotation.w;
                    if (s_squared < 10.0 * std::numeric_limits<double>::epsilon()) {
                        pos = 0.0;
                    } else {
                        double s = 1.0 / sqrt(s_squared);
                        pos = (acos(transform.rotation.w) * 2.0) * transform.rotation.x * s;
                    }
                } else if (local_name == "rot_w") {
                    pos = transform.rotation.w;
                } else if (local_name == "rot_x") {
                    pos = transform.rotation.x;
                } else if (local_name == "rot_y") {
                    pos = transform.rotation.y;
                } else if (local_name == "rot_z") {
                    pos = transform.rotation.z;
                } else {
                    SMPL_WARN("Unrecognized multi-dof local variable name '%s'", local_name.c_str());
                    continue;
                }

                ofs << pos;
            } else {
                auto it = std::find(
                        begin(state.joint_state.name),
                        end(state.joint_state.name),
                        var_name);
                if (it == end(state.joint_state.name)) continue;

                auto tvidx = std::distance(begin(state.joint_state.name), it);
                auto vp = state.joint_state.position[tvidx];
                ofs << vp;
            }

            if (vidx != robot->jointVariableCount() - 1) {
                ofs << ',';
            }
        }
        ofs << '\n';
    }

    return true;
}

bool DualPlannerInterface::solve(
    // TODO: this planning scene is probably not being used in any meaningful way
    const moveit_msgs::PlanningScene& planning_scene,
    const moveit_msgs::MotionPlanRequest& req,
    moveit_msgs::MotionPlanResponse& res)
{
    SMPL_INFO("Called solve");
    ClearMotionPlanResponse(req, res);

    if (!m_initialized) {
        SMPL_ERROR("RP: Not initialized");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        return false;
    }

    if (!canServiceRequest(req, res)) {
        SMPL_ERROR("RP: Can't Service");
        return false;
    }

    if (req.goal_constraints.empty()) {
        SMPL_WARN_NAMED(DPI_LOGGER, "No goal constraints in request!");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
        return true;
    }

    // TODO: lazily reinitialize planner when algorithm changes
    if (!reinitPlanner(req.planner_id)) {
        SMPL_ERROR("RP: Can't reinit");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
        return false;
    }

    res.trajectory_start = planning_scene.robot_state;
    SMPL_INFO_NAMED(DPI_LOGGER, "Allowed Time (s): %0.3f", req.allowed_planning_time);

    std::vector<RobotState> path;
    auto then = clock::now();


    if (!setGoal(req.goal_constraints)) {
        SMPL_ERROR("Failed to set goal");
        res.planning_time = to_seconds(clock::now() - then);
        res.error_code.val = moveit_msgs::MoveItErrorCodes::GOAL_IN_COLLISION;
        return false;
    }

    if (!setStart(req.start_state)) {
        SMPL_ERROR("Failed to set initial configuration of robot");
        res.planning_time = to_seconds(clock::now() - then);
        res.error_code.val = moveit_msgs::MoveItErrorCodes::START_STATE_IN_COLLISION;
        return false;
    }

    if (!plan(req.allowed_planning_time, path)) {
        SMPL_ERROR("Failed to plan within alotted time frame (%0.2f seconds, %d expansions)", req.allowed_planning_time, m_planner->get_n_expands());
        res.planning_time = to_seconds(clock::now() - then);
        res.error_code.val = moveit_msgs::MoveItErrorCodes::PLANNING_FAILED;
        return false;
    }

    res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;


    SMPL_DEBUG_NAMED(DPI_LOGGER, "planner path:");
    for (size_t pidx = 0; pidx < path.size(); ++pidx) {
        auto& point = path[pidx];
        SMPL_DEBUG_STREAM_NAMED(DPI_LOGGER, "  " << pidx << ": " << point);
    }

    /////////////////////
    // smooth the path //
    /////////////////////

    auto check_planned_path = true;
    // TODO dual: If checker dual has acm modified this should be reverted
    if (check_planned_path && !IsPathValid(m_checker_dual, path)) {
        SMPL_ERROR("Planned path is invalid");
    }

    SV_SHOW_INFO_NAMED("trajectory", makePathVisualization(path));
    postProcessPath(path);
    
    SMPL_DEBUG_NAMED(DPI_LOGGER, "smoothed path:");
    for (size_t pidx = 0; pidx < path.size(); ++pidx) {
        auto& point = path[pidx];
        SMPL_DEBUG_STREAM_NAMED(DPI_LOGGER, "  " << pidx << ": " << point);
    }

    ConvertJointVariablePathToJointTrajectory(
            m_robot_dual,
            path,
            req.start_state.joint_state.header.frame_id,
            req.start_state.multi_dof_joint_state.header.frame_id,
            res.trajectory);
    res.trajectory.joint_trajectory.header.stamp = ros::Time::now();

    if (!m_params.plan_output_dir.empty()) {
        WritePath(m_robot_dual, res.trajectory_start, res.trajectory, m_params.plan_output_dir);
    }

    ProfilePath(m_robot_dual, res.trajectory.joint_trajectory);
//    RemoveZeroDurationSegments(traj);

    res.planning_time = to_seconds(clock::now() - then);
    return true;
}

static
bool ExtractJointStateGoal(
    RobotModel* model,
    const GoalConstraints& v_goal_constraints,
    GoalConstraint& goal)
{
    auto& goal_constraints = v_goal_constraints.front();

    SMPL_INFO_NAMED(DPI_LOGGER, "Set goal configuration");

    RobotState sbpl_angle_goal(model->jointVariableCount(), 0);
    RobotState sbpl_angle_tolerance(model->jointVariableCount(), angles::to_radians(3.0));

    if (goal_constraints.joint_constraints.size() < model->jointVariableCount()) {
        SMPL_WARN_NAMED(DPI_LOGGER, "Insufficient joint constraints specified (%zu < %zu)!", goal_constraints.joint_constraints.size(), model->jointVariableCount());
//        return false;
    }
    if (goal_constraints.joint_constraints.size() > model->jointVariableCount()) {
        SMPL_WARN_NAMED(DPI_LOGGER, "Excess joint constraints specified (%zu > %zu)!", goal_constraints.joint_constraints.size(), model->jointVariableCount());
//        return false;
    }

    for (size_t i = 0; i < model->jointVariableCount(); ++i) {
        auto& variable_name = model->getPlanningJoints()[i];

        auto jit = std::find_if(
                begin(goal_constraints.joint_constraints),
                end(goal_constraints.joint_constraints),
                [&](const moveit_msgs::JointConstraint& constraint) {
                    return constraint.joint_name == variable_name;
                });
        if (jit == end(goal_constraints.joint_constraints)) {
            SMPL_WARN("Assume goal position 1 for joint '%s'", variable_name.c_str());
            sbpl_angle_goal[i] = 1.0;
            sbpl_angle_tolerance[i] = 0.1;
        } else {
            sbpl_angle_goal[i] = jit->position;
            sbpl_angle_tolerance[i] = std::min(
                    std::fabs(jit->tolerance_above), std::fabs(jit->tolerance_below));
            SMPL_INFO_NAMED(DPI_LOGGER, "Joint %zu [%s]: goal position: %.3f, goal tolerance: %.3f", i, variable_name.c_str(), sbpl_angle_goal[i], sbpl_angle_tolerance[i]);
        }
    }

    goal.type = GoalType::JOINT_STATE_GOAL;
    goal.angles = sbpl_angle_goal;
    goal.angle_tolerances = sbpl_angle_tolerance;

    auto* fk_iface = model->getExtension<ForwardKinematicsInterface>();

    // TODO: really need to reevaluate the necessity of the planning link
    if (fk_iface) {
        goal.pose = fk_iface->computeFK(goal.angles);
    } else {
        goal.pose = Eigen::Isometry3d::Identity();
    }

    return true;
}

static
bool ExtractGoalPoseFromGoalConstraints(
    const moveit_msgs::Constraints& constraints,
    Eigen::Isometry3d& goal_pose)
{
    assert(!constraints.position_constraints.empty() &&
           !constraints.orientation_constraints.empty());

    // TODO: where is it enforced that the goal position/orientation constraints
    // should be for the planning link?

    auto& position_constraint = constraints.position_constraints.front();
    auto& orientation_constraint = constraints.orientation_constraints.front();

    if (position_constraint.constraint_region.primitive_poses.empty()) {
        SMPL_WARN_NAMED(DPI_LOGGER, "Conversion from goal constraints to goal pose requires at least one primitive shape pose associated with the position constraint region");
        return false;
    }

    auto& bounding_primitive = position_constraint.constraint_region.primitives.front();
    auto& primitive_pose = position_constraint.constraint_region.primitive_poses.front();

    // undo the translation
    Eigen::Isometry3d T_planning_eef; // T_planning_off * T_off_eef;
    tf::poseMsgToEigen(primitive_pose, T_planning_eef);
    Eigen::Vector3d eef_pos(T_planning_eef.translation());

    Eigen::Quaterniond eef_orientation;
    tf::quaternionMsgToEigen(orientation_constraint.orientation, eef_orientation);

    goal_pose = Eigen::Translation3d(eef_pos) * eef_orientation;
    return true;
}

static
bool ExtractDualGoalPoseFromGoalConstraints(
    const moveit_msgs::Constraints& constraints,
    Eigen::Isometry3d& goal_pose1, Eigen::Isometry3d& goal_pose2)
{
    assert(!constraints.position_constraints.empty() &&
           !constraints.orientation_constraints.empty());

    // TODO: where is it enforced that the goal position/orientation constraints
    // should be for the planning link?

    auto& position_constraint1 = constraints.position_constraints.front();
    auto& orientation_constraint1 = constraints.orientation_constraints.front();
    auto& position_constraint2 = constraints.position_constraints[1];
    auto& orientation_constraint2 = constraints.orientation_constraints[1];

    if (position_constraint1.constraint_region.primitive_poses.empty()) {
        SMPL_WARN_NAMED(DPI_LOGGER, "Conversion from goal1 constraints to goal pose requires at least one primitive shape pose associated with the position constraint region");
        return false;
    }

    if (position_constraint2.constraint_region.primitive_poses.empty()) {
        SMPL_WARN_NAMED(DPI_LOGGER, "Conversion from goal2 constraints to goal pose requires at least one primitive shape pose associated with the position constraint region");
        return false;
    }

    auto& bounding_primitive1 = position_constraint1.constraint_region.primitives.front();
    auto& primitive_pose1 = position_constraint1.constraint_region.primitive_poses.front();
    auto& bounding_primitive2 = position_constraint2.constraint_region.primitives.front();
    auto& primitive_pose2 = position_constraint2.constraint_region.primitive_poses.front();

    // undo the translation
    Eigen::Isometry3d T_planning_eef1; // T_planning_off * T_off_eef;
    tf::poseMsgToEigen(primitive_pose1, T_planning_eef1);
    Eigen::Vector3d eef_pos1(T_planning_eef1.translation());

    Eigen::Quaterniond eef_orientation1;
    tf::quaternionMsgToEigen(orientation_constraint1.orientation, eef_orientation1);

    goal_pose1 = Eigen::Translation3d(eef_pos1) * eef_orientation1;

    Eigen::Isometry3d T_planning_eef2; // T_planning_off * T_off_eef;
    tf::poseMsgToEigen(primitive_pose2, T_planning_eef2);
    Eigen::Vector3d eef_pos2(T_planning_eef2.translation());

    Eigen::Quaterniond eef_orientation2;
    tf::quaternionMsgToEigen(orientation_constraint2.orientation, eef_orientation2);

    goal_pose2 = Eigen::Translation3d(eef_pos2) * eef_orientation2;
    return true;
}


static
bool ExtractGoalToleranceFromGoalConstraints(
    const moveit_msgs::Constraints& goal_constraints,
    double tol[6])
{
    if (!goal_constraints.position_constraints.empty() &&
        !goal_constraints.position_constraints.front().constraint_region.primitives.empty())
    {
        auto& position_constraint = goal_constraints.position_constraints.front();
        auto& constraint_primitive = position_constraint.constraint_region.primitives.front();
        auto& dims = constraint_primitive.dimensions;
        switch (constraint_primitive.type) {
        case shape_msgs::SolidPrimitive::BOX:
            tol[0] = dims[shape_msgs::SolidPrimitive::BOX_X];
            tol[1] = dims[shape_msgs::SolidPrimitive::BOX_Y];
            tol[2] = dims[shape_msgs::SolidPrimitive::BOX_Z];
            break;
        case shape_msgs::SolidPrimitive::SPHERE:
            tol[0] = dims[shape_msgs::SolidPrimitive::SPHERE_RADIUS];
            tol[1] = dims[shape_msgs::SolidPrimitive::SPHERE_RADIUS];
            tol[2] = dims[shape_msgs::SolidPrimitive::SPHERE_RADIUS];
            break;
        case shape_msgs::SolidPrimitive::CYLINDER:
            tol[0] = dims[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
            tol[1] = dims[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
            tol[2] = dims[shape_msgs::SolidPrimitive::CYLINDER_RADIUS];
            break;
        case shape_msgs::SolidPrimitive::CONE:
            tol[0] = dims[shape_msgs::SolidPrimitive::CONE_RADIUS];
            tol[1] = dims[shape_msgs::SolidPrimitive::CONE_RADIUS];
            tol[2] = dims[shape_msgs::SolidPrimitive::CONE_RADIUS];
            break;
        }
    } else {
        tol[0] = tol[1] = tol[2] = 0.0;
    }

    if (!goal_constraints.orientation_constraints.empty()) {
        auto& orientation_constraints = goal_constraints.orientation_constraints;
        auto& orientation_constraint = orientation_constraints.front();
        tol[3] = orientation_constraint.absolute_x_axis_tolerance;
        tol[4] = orientation_constraint.absolute_y_axis_tolerance;
        tol[5] = orientation_constraint.absolute_z_axis_tolerance;
    } else {
        tol[3] = tol[4] = tol[5] = 0.0;
    }
    return true;
}

static
bool ExtractPositionGoal(
    const GoalConstraints& v_goal_constraints,
    GoalConstraint& goal)
{
    return false;
}

static
bool ExtractDualPoseGoal(
    const GoalConstraints& v_goal_constraints,
    GoalConstraint& goal1, GoalConstraint& goal2)
{
    assert(!v_goal_constraints.empty());
    auto& goal_constraints = v_goal_constraints.front();

    SMPL_INFO_NAMED(DPI_LOGGER, "Setting goal position");

    Eigen::Isometry3d goal_pose1, goal_pose2;
    if (!ExtractDualGoalPoseFromGoalConstraints(goal_constraints, goal_pose1, goal_pose2)) {
        SMPL_WARN_NAMED(DPI_LOGGER, "Failed to extract goal pose from goal constraints");
        return false;
    }

    goal1.type = GoalType::XYZ_RPY_GOAL;
    goal1.pose = goal_pose1;

    goal2.type = GoalType::XYZ_RPY_GOAL;
    goal2.pose = goal_pose2;

    //TODO: Using first goal tolerance only
    double sbpl_tolerance[6] = { 0.0 };
    if (!ExtractGoalToleranceFromGoalConstraints(goal_constraints, sbpl_tolerance)) {
        SMPL_WARN_NAMED(DPI_LOGGER, "Failed to extract goal tolerance from goal constraints");
        return false;
    }

    goal1.xyz_tolerance[0] = goal2.xyz_tolerance[0] = sbpl_tolerance[0];
    goal1.xyz_tolerance[1] = goal2.xyz_tolerance[1] = sbpl_tolerance[1];
    goal1.xyz_tolerance[2] = goal2.xyz_tolerance[2] = sbpl_tolerance[2];
    goal1.rpy_tolerance[0] = goal2.xyz_tolerance[3] = sbpl_tolerance[3];
    goal1.rpy_tolerance[1] = goal2.xyz_tolerance[4] = sbpl_tolerance[4];
    goal1.rpy_tolerance[2] = goal2.xyz_tolerance[5] = sbpl_tolerance[5];
    return true;
}


// Convert the set of input goal constraints to an SMPL goal type and update
// the goal within the graph, the heuristic, and the search.
bool DualPlannerInterface::setGoal(const GoalConstraints& v_goal_constraints)
{
     GoalConstraint goal1, goal2;

    if (IsDualPoseGoal(v_goal_constraints)) {
        SMPL_INFO_NAMED(DPI_LOGGER, "Planning to dual pose!");
        if (!ExtractDualPoseGoal(v_goal_constraints, goal1, goal2)) {
            SMPL_ERROR("Failed to set goals position");
            return false;
        }

        SMPL_INFO_NAMED(DPI_LOGGER, "New Goal");
        double yaw, pitch, roll;
        angles::get_euler_zyx(goal1.pose.rotation(), yaw, pitch, roll);
        SMPL_INFO_NAMED(DPI_LOGGER, "    pose: (x: %0.3f, y: %0.3f, z: %0.3f, Y: %0.3f, P: %0.3f, R: %0.3f)",goal1.pose.translation()[0],goal1.pose.translation()[1],goal1.pose.translation()[2], yaw, pitch, roll);
        SMPL_INFO_NAMED(DPI_LOGGER, "    tolerance: (dx: %0.3f, dy: %0.3f, dz: %0.3f, dR: %0.3f, dP: %0.3f, dY: %0.3f)",goal1.xyz_tolerance[0],goal1.xyz_tolerance[1],goal1.xyz_tolerance[2],goal1.rpy_tolerance[0], goal1.rpy_tolerance[1], goal1.rpy_tolerance[2]);
        angles::get_euler_zyx(goal2.pose.rotation(), yaw, pitch, roll);
        SMPL_INFO_NAMED(DPI_LOGGER, "    pose: (x: %0.3f, y: %0.3f, z: %0.3f, Y: %0.3f, P: %0.3f, R: %0.3f)",goal2.pose.translation()[0],goal2.pose.translation()[1],goal2.pose.translation()[2], yaw, pitch, roll);
        SMPL_INFO_NAMED(DPI_LOGGER, "    tolerance: (dx: %0.3f, dy: %0.3f, dz: %0.3f, dR: %0.3f, dP: %0.3f, dY: %0.3f)",goal2.xyz_tolerance[0],goal2.xyz_tolerance[1],goal2.xyz_tolerance[2],goal2.rpy_tolerance[0], goal2.rpy_tolerance[1], goal2.rpy_tolerance[2]);
       
    } else {
        SMPL_ERROR("invalid goal type!");
        return false;
    }

    // set sbpl environment goal
    //TODO dual: order matters?
    goal1.name = "goal1"; // FOR VIZ
    if (!m_pspace_first->setGoal(goal1)) {
        SMPL_ERROR("Failed to set goal 1");
        return false;
    }

    goal2.name = "goal2";
    if (!m_pspace_dual->setGoal(goal2)) {
        SMPL_ERROR("Failed to set goal 2");
        return false;
    }
    
    

    for (auto& h : m_heuristics_first) {
        h.second->updateGoal(goal1);
    }
    for (auto& h : m_heuristics_dual) {
        h.second->updateGoal(goal2);
    }

    // set planner goal
    //TODO dual: how to handle this
    auto goal_id_dual = m_pspace_dual->getGoalStateID();
    auto goal_id_first = m_pspace_first->getGoalStateID();
    SMPL_INFO_STREAM("RP: goal id first " << goal_id_first << " dual " << goal_id_dual); 

    if (goal_id_dual == -1 || goal_id_first == -1) {
        SMPL_ERROR("No goal state has been set");
        return false;
    }

    if (m_planner->set_goal(goal_id_first) == 0) {
        SMPL_ERROR("Failed to set planner goal state");
        return false;
    }

    return true;

}

// Convert the input robot state to an SMPL robot state and update the start
// state in the graph, heuristic, and search.
// TODO dual: I definetly don't know how to handle this
bool DualPlannerInterface::setStart(const moveit_msgs::RobotState& state)
{
    SMPL_INFO_NAMED(DPI_LOGGER, "set dual start configuration");

    // TODO: Ideally, the RobotModel should specify joints rather than variables
    if (!state.multi_dof_joint_state.joint_names.empty()) {
        auto& mdof_joint_names = state.multi_dof_joint_state.joint_names;
        for (auto& joint_name : m_robot_dual->getPlanningJoints()) {
            auto it = std::find(begin(mdof_joint_names), end(mdof_joint_names), joint_name);
            if (it != end(mdof_joint_names)) {
                SMPL_WARN_NAMED(DPI_LOGGER, "planner does not currently support planning for multi-dof joints. found '%s' in planning joints", joint_name.c_str());
            }
        }
    }

    std::stringstream ss;
    ss << "Joints::";
    for (unsigned int i = 0; i < state.joint_state.name.size(); i++){
        ss << " name: " << state.joint_state.name[i] << " v: " << state.joint_state.position[i];
    }
    SMPL_INFO_STREAM(ss.str());
    ///std::getchar();

    RobotState initial_positions;
    std::vector<std::string> missing;
    if (!leatherman::getJointPositions(
            state.joint_state,
            state.multi_dof_joint_state,
            m_robot_dual->getPlanningJoints(),
            initial_positions,
            missing))
    {
        SMPL_WARN_STREAM("start state is missing planning joints: " << missing);

        moveit_msgs::RobotState fixed_state = state;
        for (auto& variable : missing) {
            SMPL_WARN("  Assume position 0.0 for joint variable '%s'", variable.c_str());
            fixed_state.joint_state.name.push_back(variable);
            fixed_state.joint_state.position.push_back(0.0);
        }

        if (!leatherman::getJointPositions(
                fixed_state.joint_state,
                fixed_state.multi_dof_joint_state,
                m_robot_dual->getPlanningJoints(),
                initial_positions,
                missing))
        {
            return false;
        }

//        return false;
    }

    SMPL_INFO_STREAM_NAMED(DPI_LOGGER, "  joint variables: " << initial_positions);

    if (!m_pspace_dual->setStart(initial_positions)) {
        SMPL_ERROR("Failed to set start state dual");
        return false;
    }
    if (!m_pspace_first->setStart(initial_positions)) {
        SMPL_ERROR("Failed to set start state first");
        return false;
    }

    auto start_id_dual = m_pspace_dual->getStartStateID();
    if (start_id_dual == -1) {
        SMPL_ERROR("No start state has been set: dual");
        return false;
    }
    auto start_id_first = m_pspace_first->getStartStateID();
    if (start_id_first == -1) {
        SMPL_ERROR("No start state has been set: first");
        return false;
    }

    for (auto& h : m_heuristics_dual) {
        h.second->updateStart(initial_positions);
    }
    for (auto& h : m_heuristics_first) {
        h.second->updateStart(initial_positions);
    }

    //TODO dual: how to handle this
    SMPL_INFO_STREAM("RP: start id first " << start_id_first << " dual " << start_id_dual); 
    if (m_planner->set_start(start_id_first) == 0) {
        SMPL_ERROR("Failed to set start state");
        return false;
    }
    return true;
}


bool DualPlannerInterface::plan(double allowed_time, std::vector<RobotState>& path)
{
    // NOTE: this should be done after setting the start/goal in the environment
    // to allow the heuristic to tailor the visualization to the current
    // scenario

    SMPL_INFO("VIZ DUAL");
    SV_SHOW_INFO_NAMED("bfs_walls", getBfsWallsVisualization(m_heuristics_first, "bfs_walls1"));
    SV_SHOW_INFO_NAMED("bfs_values", getBfsValuesVisualization(m_heuristics_first, "bfs_values1"));
    SV_SHOW_INFO_NAMED("bfs_walls", getBfsWallsVisualization(m_heuristics_dual, "bfs_walls2"));
    SV_SHOW_INFO_NAMED("bfs_values", getBfsValuesVisualization(m_heuristics_dual, "bfs_values2"));

        
    

    SMPL_WARN_NAMED(DPI_LOGGER, "Planning!!!!!");
    bool b_ret = false;
    std::vector<int> solution_state_ids;

    // reinitialize the search space
    m_planner->force_planning_from_scratch();

    // plan
    b_ret = m_planner->replan(allowed_time, &solution_state_ids, &m_sol_cost);

    // check if an empty plan was received.
    if (b_ret && solution_state_ids.size() <= 0) {
        SMPL_WARN_NAMED(DPI_LOGGER, "Path returned by the planner is empty?");
        b_ret = false;
    }

    // if a path is returned, then pack it into msg form
    if (b_ret && (solution_state_ids.size() > 0)) {
        SMPL_INFO_NAMED(DPI_LOGGER, "Planning succeeded");
        SMPL_INFO_NAMED(DPI_LOGGER, "  Num Expansions (Initial): %d", m_planner->get_n_expands_init_solution());
        SMPL_INFO_NAMED(DPI_LOGGER, "  Num Expansions (Final): %d", m_planner->get_n_expands());
        SMPL_INFO_NAMED(DPI_LOGGER, "  Epsilon (Initial): %0.3f", m_planner->get_initial_eps());
        SMPL_INFO_NAMED(DPI_LOGGER, "  Epsilon (Final): %0.3f", m_planner->get_solution_eps());
        SMPL_INFO_NAMED(DPI_LOGGER, "  Time (Initial): %0.3f", m_planner->get_initial_eps_planning_time());
        SMPL_INFO_NAMED(DPI_LOGGER, "  Time (Final): %0.3f", m_planner->get_final_eps_planning_time());
        SMPL_INFO_NAMED(DPI_LOGGER, "  Path Length (states): %zu", solution_state_ids.size());
        SMPL_INFO_NAMED(DPI_LOGGER, "  Solution Cost: %d", m_sol_cost);

        path.clear();
        std::vector<RobotState> goals;
        // Todo dual: pspace?
        if (!m_pspace_dual->extractPathDual(solution_state_ids, path, goals)) {
            SMPL_ERROR("Failed to convert state id path to joint variable path");
            return false;
        }
        //TODO dual: temorary add first goal
        path.push_back(goals[0]);
    }
    return b_ret;
}

/// Test if a particular set of goal constraints it supported.
///
/// This tests whether, in general, any planning algorithm supported by this
/// interface can support a particular set of constraints. Certain planning
/// algorithms may not be able to handle a given set of constraints. This
/// method also cannot check for validity of constraints against a particular
/// robot model at this step. In particular, it cannot assert that the a set
/// of joint constraints lists a constraint for each joint, which is currently
/// required.
bool DualPlannerInterface::SupportsGoalConstraints(
    const GoalConstraints& goal_constraints,
    std::string& why)
{
    if (goal_constraints.empty()) {
        return true;
    }

    if (!IsDualPoseGoal(goal_constraints))
    {
        why = "goal constraints are not supported";
        return false;
    }

    return true;
}

bool DualPlannerInterface::canServiceRequest(
    const moveit_msgs::MotionPlanRequest& req,
    moveit_msgs::MotionPlanResponse& res) const
{
    // check for an empty start state
    // TODO: generalize this to "missing necessary state information"
    if (req.start_state.joint_state.position.empty()) {
        SMPL_ERROR("No start state given. Unable to plan.");
        res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_ROBOT_STATE;
        return false;
    }

    std::string why;
    if (!SupportsGoalConstraints(req.goal_constraints, why)) {
        SMPL_ERROR("Goal constraints not supported (%s)", why.c_str());
        res.error_code.val = moveit_msgs::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
        return false;
    }

    return true;
}

auto DualPlannerInterface::getPlannerStats() -> std::map<std::string, double>
{
    std::map<std::string, double> stats;
    stats["initial solution planning time"] = m_planner->get_initial_eps_planning_time();
    stats["initial epsilon"] = m_planner->get_initial_eps();
    stats["initial solution expansions"] = m_planner->get_n_expands_init_solution();
    stats["final epsilon planning time"] = m_planner->get_final_eps_planning_time();
    stats["final epsilon"] = m_planner->get_final_epsilon();
    stats["solution epsilon"] = m_planner->get_solution_eps();
    stats["expansions"] = m_planner->get_n_expands();
    stats["solution cost"] = m_sol_cost;
    return stats;
}

auto DualPlannerInterface::makePathVisualization(
    const std::vector<RobotState>& path) const
    -> std::vector<visual::Marker>
{
    std::vector<visual::Marker> ma;

    if (path.empty()) {
        return ma;
    }
    //TODO dual: using checker dual which might have acm modified
    auto cinc = 1.0f / float(path.size());
    for (size_t i = 0; i < path.size(); ++i) {
        auto markers = m_checker_dual->getCollisionModelVisualization(path[i]);

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
        marker.ns = "trajectory";
        marker.id = i;
    }

    return ma;
}

auto DualPlannerInterface::getBfsWallsVisualization(const std::map<std::string, std::unique_ptr<RobotHeuristic>>& heuristics,
     const std::string& label) const -> visual::Marker
{
if (heuristics.empty()) {
        return visual::Marker{ };
    }

    auto first = heuristics.begin();

    if (auto* hbfs = dynamic_cast<BfsHeuristic*>(first->second.get())) {
        return hbfs->getWallsVisualization(label);
    } else if (auto* hmfbfs = dynamic_cast<MultiFrameBfsHeuristic*>(first->second.get())) {
        return hmfbfs->getWallsVisualization();
    } else if (auto* mfbfs = dynamic_cast<MultiBfsHeuristic*>(first->second.get())) {
        return mfbfs->getWallsVisualization();
    } else if (auto* debfs = dynamic_cast<DijkstraEgraphHeuristic3D*>(first->second.get())) {
        return debfs->getWallsVisualization();
    } else {
        return visual::Marker{ };
    }
}


auto DualPlannerInterface::getBfsValuesVisualization(const std::map<std::string, std::unique_ptr<RobotHeuristic>>& heuristics, const std::string& label) const -> visual::Marker
{
    if (heuristics.empty()) {
        return visual::Marker{ };
    }

    auto first = heuristics.begin();

    if (auto* hbfs = dynamic_cast<BfsHeuristic*>(first->second.get())) {
        return hbfs->getValuesVisualization(label);
    } else if (auto* hmfbfs = dynamic_cast<MultiFrameBfsHeuristic*>(first->second.get())) {
        return hmfbfs->getValuesVisualization();
    } else if (auto* mfbfs = dynamic_cast<MultiBfsHeuristic*>(first->second.get())) {
        return mfbfs->getValuesVisualization();
    } else if (auto* debfs = dynamic_cast<DijkstraEgraphHeuristic3D*>(first->second.get())) {
        return debfs->getValuesVisualization();
    } else {
        return visual::Marker{ };
    }
}

//TODO dual: real parse
bool DualPlannerInterface::parsePlannerID(
    const std::string& planner_id,
    std::vector<std::string>& space_name,
    std::vector<std::string>& heuristic_name,
    std::vector<std::string>& search_name) const
    {
        space_name.resize(2);
        space_name[0] = "manip";
        space_name[1] = "manip";

        heuristic_name.resize(2);
        heuristic_name[0] = "bfs";
        heuristic_name[1] = "bfs";

        search_name.resize(2);
        search_name[0] = "arastar";
        search_name[1] = "arastar";

        return true;
    }

bool DualPlannerInterface::reinitPlanner_dual(const std::string& space_name, const std::string& heuristic_name, const std::string& search_name){
    auto psait = m_space_factories.find(space_name);
    if (psait == end(m_space_factories)) {
        SMPL_ERROR("Unrecognized planning space name '%s'", space_name.c_str());
        return false;
    }

    smpl::PlanningParams pp = m_params;
    pp.addParam("use_multiple_ik_solutions", false);

    m_pspace_dual = psait->second(m_robot_dual, m_checker_dual, pp);
    if (!m_pspace_dual) {
        SMPL_ERROR("Failed to build planning space '%s'", space_name.c_str());
        return false;
    }

    auto hait = m_heuristic_factories.find(heuristic_name);
    if (hait == end(m_heuristic_factories)) {
        SMPL_ERROR("Unrecognized heuristic name '%s'", heuristic_name.c_str());
        return false;
    }

    auto heuristic = hait->second(m_pspace_dual.get(), pp);
    if (!heuristic) {
        SMPL_ERROR("Failed to build heuristic '%s'", heuristic_name.c_str());
        return false;
    }

    // initialize heuristics
    m_heuristics_dual.clear();
    m_heuristics_dual.insert(std::make_pair(heuristic_name, std::move(heuristic)));

    for (auto& entry : m_heuristics_dual) {
        m_pspace_dual->insertHeuristic(entry.second.get());
    }

    SMPL_INFO("Heuristic inserted");
    

    auto pait = m_planner_factories.find(search_name);
    if (pait == end(m_planner_factories)) {
        SMPL_ERROR("Unrecognized search name '%s'", search_name.c_str());
        return false;
    }

    auto first_heuristic = begin(m_heuristics_dual);
    m_planner_dual = pait->second(m_pspace_dual.get(), first_heuristic->second.get(), pp);
    if (!m_planner_dual) {
        SMPL_ERROR("Failed to build planner '%s'", search_name.c_str());
        return false;
    }
    return true;
}


bool DualPlannerInterface::reinitPlanner_first(const std::string& space_name, const std::string& heuristic_name, const std::string& search_name){
    auto psait = m_space_factories.find(space_name);
    if (psait == end(m_space_factories)) {
        SMPL_ERROR("Unrecognized planning space name '%s'", space_name.c_str());
        return false;
    }

    smpl::PlanningParams pp = m_params;
    std::string mprim_f =  "/home/roger/catkin_ws/src/iauv_planning/girona500_dual_arm_config/config/girona500_first.mprim";
    pp.addParam("mprim_filename", mprim_f);
    m_pspace_first = psait->second(m_robot_first, m_checker_first, pp);
    if (!m_pspace_first) {
        SMPL_ERROR("Failed to build planning space '%s'", space_name.c_str());
        return false;
    }

    auto hait = m_heuristic_factories.find(heuristic_name);
    if (hait == end(m_heuristic_factories)) {
        SMPL_ERROR("Unrecognized heuristic name '%s'", heuristic_name.c_str());
        return false;
    }

    auto heuristic = hait->second(m_pspace_first.get(), pp);
    if (!heuristic) {
        SMPL_ERROR("Failed to build heuristic '%s'", heuristic_name.c_str());
        return false;
    }

    // initialize heuristics
    m_heuristics_first.clear();
    m_heuristics_first.insert(std::make_pair(heuristic_name, std::move(heuristic)));

    for (auto& entry : m_heuristics_first) {
        m_pspace_first->insertHeuristic(entry.second.get());
    }

    SMPL_INFO("Heuristic inserted");
    

    auto pait = m_planner_factories.find(search_name);
    if (pait == end(m_planner_factories)) {
        SMPL_ERROR("Unrecognized search name '%s'", search_name.c_str());
        return false;
    }

    auto first_heuristic = begin(m_heuristics_first);
    m_planner_first = pait->second(m_pspace_first.get(), first_heuristic->second.get(), pp);
    if (!m_planner_first) {
        SMPL_ERROR("Failed to build planner '%s'", search_name.c_str());
        return false;
    }
    return true;
}

bool DualPlannerInterface::reinitPlanner(const std::string& planner_id){
    
    SMPL_INFO_NAMED(DPI_LOGGER, "Initialize dual planner");

    std::vector<std::string> search_name;
    std::vector<std::string> heuristic_name;
    std::vector<std::string> space_name;
    if (!parsePlannerID(planner_id, space_name, heuristic_name, search_name)) {
        SMPL_ERROR("Failed to parse planner setup");
        return false;
    }
    

    for(unsigned int i = 0; i < 2; i++){
        SMPL_INFO_NAMED(DPI_LOGGER, " -> Planning Space: %s", space_name[i].c_str());
        SMPL_INFO_NAMED(DPI_LOGGER, " -> Heuristic: %s", heuristic_name[i].c_str());
        SMPL_INFO_NAMED(DPI_LOGGER, " -> Search: %s", search_name[i].c_str());
    }
    bool ok = true;
    ok &= reinitPlanner_first(space_name[0], heuristic_name[0], search_name[0]);
    ok &= reinitPlanner_dual(space_name[1], heuristic_name[1], search_name[1]);
    if ( !ok){
        return false;
    }
  
    SMPL_INFO("Make dual planner");
    m_planner = MakeDualPlanner(m_planner_first.get(),begin(m_heuristics_first)->second.get(),m_pspace_first.get(),
                                m_planner_dual.get(),begin(m_heuristics_dual)->second.get(),m_pspace_dual.get(),
                                m_params);
    SMPL_INFO("Done make dual planner");
    m_planner_id = planner_id;
    return true;
}

void DualPlannerInterface::postProcessPath(std::vector<RobotState>& path) const
{
    //Todo dual: robot and checker
    // shortcut path
    if (m_params.shortcut_path) {
        if (!InterpolatePath(*m_checker_dual, path)) {
            SMPL_WARN_NAMED(DPI_LOGGER, "Failed to interpolate planned path with %zu waypoints before shortcutting.", path.size());
            std::vector<RobotState> ipath = path;
            path.clear();
            ShortcutPath(m_robot_dual, m_checker_dual, ipath, path, m_params.shortcut_type);
        } else {
            std::vector<RobotState> ipath = path;
            path.clear();
            ShortcutPath(m_robot_dual, m_checker_dual, ipath, path, m_params.shortcut_type);
        }
    }

    // interpolate path
    if (m_params.interpolate_path) {
        if (!InterpolatePath(*m_checker_dual, path)) {
            SMPL_WARN_NAMED(DPI_LOGGER, "Failed to interpolate trajectory");
        }
    }
}

} // namespace smpl
