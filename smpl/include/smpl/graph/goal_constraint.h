#ifndef SMPL_GOAL_CONSTRAINT_H
#define SMPL_GOAL_CONSTRAINT_H

// standard includes
#include <vector>

// system includes
#include <Eigen/StdVector>

// project includes
#include <smpl/spatial.h>
#include <smpl/types.h>

namespace smpl {

enum GoalType
{
    INVALID_GOAL_TYPE = -1,
    XYZ_GOAL,
    XYZ_RPY_GOAL,
    JOINT_STATE_GOAL,
    MULTIPLE_POSE_GOAL,
    USER_GOAL_CONSTRAINT_FN,
    NUMBER_OF_GOAL_TYPES
};

using GoalConstraintFn = bool (*)(void* user, const RobotState& state);

struct GoalConstraint
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Relevant for joint state goals
    RobotState angles;
    std::vector<double> angle_tolerances;

    // Relevant for workspace goals
    Affine3 pose;               // goal pose of the planning link

    std::vector<Affine3, Eigen::aligned_allocator<Affine3>> poses;

    double xyz_tolerance[3];            // (x, y, z) tolerance
    double rpy_tolerance[3];            // (R, P, Y) tolerance

    GoalConstraintFn check_goal = NULL;
    void* check_goal_user = NULL;

    GoalType type;                      // type of goal constraint
    std::string name;
};

} // namespace smpl

#endif

