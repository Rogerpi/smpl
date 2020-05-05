#include <smpl_moveit_interface/interface/robot_command_model.h>

#include <ros/console.h>

namespace sbpl_interface {

static const char* LOG = "robot_command_model";

RobotCommandModel::~RobotCommandModel() { }

// Load a new RobotModel into the RobotCommandModel. Emits the robotLoaded()
// signal, sets the managed RobotState to its default values, and emits a
// robotStateChanged() signal.
bool RobotCommandModel::load(const moveit::core::RobotModelConstPtr& robot)
{
    if (robot == NULL) {
        return false;
    }

    m_robot_model = robot;
    m_robot_state.reset(new moveit::core::RobotState(robot));
    m_group_states.resize(robot->getJointModelGroups().size(), -1);
    Q_EMIT robotLoaded();

    m_robot_state->setToDefaultValues();

    updateAndNotify();

    return true;
}

auto RobotCommandModel::getRobotModel() const
    -> const moveit::core::RobotModelConstPtr&
{
    return m_robot_model;
}

auto RobotCommandModel::getRobotState() const -> const moveit::core::RobotState*
{
    return m_robot_state.get();
}

static
int GetJointModelGroupIndex(
    const moveit::core::RobotModel* robot_model,
    const moveit::core::JointModelGroup* group)
{
    for (auto i = 0; i < robot_model->getJointModelGroups().size(); ++i) {
        auto* g = robot_model->getJointModelGroups()[i];
        if (g == group) {
            return i;
        }
    }
    return -1;
}

auto RobotCommandModel::getGroupStateName(
    const moveit::core::JointModelGroup* group) const
    -> const std::string&
{
    if (m_robot_model == NULL) {
        ROS_DEBUG_NAMED(LOG, "No robot model loaded");
        return m_empty;
    }

    auto group_index = GetJointModelGroupIndex(m_robot_model.get(), group);
    if (group_index < 0) {
        ROS_DEBUG_NAMED(LOG, "Joint group '%s' not found", group->getName().c_str());
        return m_empty;
    }

    auto state_index = m_group_states[group_index];
    if (state_index < 0) {
        ROS_DEBUG_NAMED(LOG, "No group state set");
        return m_empty;
    }

    ROS_DEBUG_NAMED(LOG, "Group state is '%s'", group->getDefaultStateNames()[state_index].c_str());
    return group->getDefaultStateNames()[state_index];
}

// if the variable has changed, invalidate all groups that it is a part of
void InvalidateGroups(
    RobotCommandModel* model,
    int vindex,
    double old_value,
    double new_value)
{
    if (old_value != new_value) {
        auto* joint = model->m_robot_model->getJointOfVariable(vindex);
        for (auto group : model->m_robot_model->getJointModelGroups()) {
            if (group->hasJointModel(joint->getName())) {
                auto index = GetJointModelGroupIndex(model->m_robot_model.get(), group);
                if (model->m_group_states[index] != -1) {
                    ROS_DEBUG_NAMED(LOG, "Reset state of group %s", group->getName().c_str());
                }
                model->m_group_states[index] = -1;
            }
        }
    }
}

void InvalidateGroups(
    RobotCommandModel* model,
    const double* old_state,
    const double* new_state)
{
    for (auto i = 0; i < model->m_robot_state->getVariableCount(); ++i) {
        InvalidateGroups(model, i, old_state[i], new_state[i]);
    }
}

void RobotCommandModel::setVariablePositions(const double* position)
{
    auto same = std::equal(
            position, position + m_robot_state->getVariableCount(),
            m_robot_state->getVariablePositions());
    if (!same) {
        ROS_DEBUG_NAMED(LOG, "Set variable positions from array");
        InvalidateGroups(this, m_robot_state->getVariablePositions(), position);
        m_robot_state->setVariablePositions(position);
        updateAndNotify();
    }
}

void RobotCommandModel::setVariablePositions(const std::vector<double>& position)
{
    auto same = std::equal(
            begin(position), end(position),
            m_robot_state->getVariablePositions());
    if (!same) {
        ROS_DEBUG_NAMED(LOG, "Set variable positions from vector");
        InvalidateGroups(this, m_robot_state->getVariablePositions(), position.data());
        m_robot_state->setVariablePositions(position);
        updateAndNotify();
    }
}

void RobotCommandModel::setVariablePositions(
    const std::map<std::string, double>& variable_map)
{
    // detect changes to state
    auto same = true;
    for (auto& e : variable_map) {
        if (m_robot_state->getVariablePosition(e.first) != e.second) {
            same = false;
            break;
        }
    }

    if (!same) {
        ROS_DEBUG_NAMED(LOG, "Set variable positions from map");
        m_robot_state->setVariablePositions(variable_map);
        updateAndNotify();
    }
}

void RobotCommandModel::setVariablePositions(
    const std::map<std::string, double>& variable_map,
    std::vector<std::string>& missing_variables)
{
    // TODO: detect changes to state
    ROS_DEBUG_NAMED(LOG, "Set variable positions from map and report missing");
    m_robot_state->setVariablePositions(variable_map, missing_variables);
    updateAndNotify();
}

void RobotCommandModel::setVariablePositions(
    const std::vector<std::string>& variable_names,
    const std::vector<double>& variable_position)
{
    assert(variable_names.size() == variable_position.size());
    auto same = true;
    for (auto i = 0; i < variable_names.size(); ++i) {
        auto& name = variable_names[i];
        auto& position = variable_position[i];
        if (m_robot_state->getVariablePosition(name) != position) {
            same = false;
            break;
        }
    }
    if (!same) {
        ROS_DEBUG_NAMED(LOG, "Set variable positions from name/position pairs");
        m_robot_state->setVariablePositions(variable_names, variable_position);
        updateAndNotify();
    }
}

void RobotCommandModel::setVariablePosition(
    const std::string& variable,
    double value)
{
    auto index = m_robot_model->getVariableIndex(variable);
    if (m_robot_state->getVariablePosition(index) != value) {
        ROS_DEBUG_NAMED(LOG, "Set position of variable '%s' to %f", variable.c_str(), value);
        InvalidateGroups(this, index, m_robot_state->getVariablePosition(index), value);
        m_robot_state->setVariablePosition(variable, value);
        updateAndNotify();
    }
}

void RobotCommandModel::setVariablePosition(int index, double value)
{
    if (m_robot_state->getVariablePosition(index) != value) {
        ROS_DEBUG_NAMED(LOG, "Set position of variable %d to %f", index, value);
        InvalidateGroups(this, index, m_robot_state->getVariablePosition(index), value);
        m_robot_state->setVariablePosition(index, value);
        updateAndNotify();
    }
}

bool RobotCommandModel::setFromIK(
    const moveit::core::JointModelGroup* group,
    const Eigen::Isometry3d& pose,
    unsigned int attempts, //Remove
    double timeout,
    const moveit::core::GroupStateValidityCallbackFn& constraint,
    const kinematics::KinematicsQueryOptions& options)
{
    // TODO: detect changes to state
    auto res = m_robot_state->setFromIK(group, pose, timeout, constraint, options);
    if (res) {
        ROS_DEBUG_NAMED(LOG, "Set positions of joint group '%s' via IK", group->getName().c_str());
        updateAndNotify();
    }
    return res;
}

static
int GetDefaultStateIndex(
    const moveit::core::JointModelGroup* group,
    const std::string& name)
{
    auto it = std::find(
            begin(group->getDefaultStateNames()),
            end(group->getDefaultStateNames()),
            name);

    if (it != end(group->getDefaultStateNames())) {
        return std::distance(begin(group->getDefaultStateNames()), it);
    }

    return -1;
}

bool RobotCommandModel::setToDefaultValues(
    const moveit::core::JointModelGroup* group,
    const std::string& name)
{
    // detect changes to state
    std::map<std::string, double> default_positions;
    if (!group->getVariableDefaultPositions(name, default_positions)) {
        return false;
    }

    auto same = true;
    for (auto& e : default_positions) {
        auto cpos = m_robot_state->getVariablePosition(e.first);
        if (cpos != e.second) { same = false; break; }
    }

    if (same) {
        // even if the variables haven't changed, we may have to explicitly
        // set the active group state
        auto gindex = GetJointModelGroupIndex(m_robot_model.get(), group);
        auto sindex = GetDefaultStateIndex(group, name);
        ROS_DEBUG_NAMED(LOG, "Update group state %d -> %d", m_group_states[gindex], sindex);
        m_group_states[gindex] = sindex;
        return true;
    }

    if (m_robot_state->setToDefaultValues(group, name)) {
        auto gindex = GetJointModelGroupIndex(m_robot_model.get(), group);
        auto sindex = GetDefaultStateIndex(group, name);
        ROS_DEBUG_NAMED(LOG, "Update group state %d -> %d", m_group_states[gindex], sindex);
        m_group_states[gindex] = sindex;

        ROS_DEBUG_NAMED(LOG, "Set positions of joint group '%s' to default values '%s'", group->getName().c_str(), name.c_str());
        updateAndNotify();
        return true;
    }
    return false;
}

void RobotCommandModel::setJointPositions(
    const moveit::core::JointModel* joint,
    const Eigen::Isometry3d& joint_transform)
{
    // TODO: detect changes to state
    m_robot_state->setJointPositions(joint, joint_transform);
    updateAndNotify();
}

void RobotCommandModel::setJointGroupPositions(
    const moveit::core::JointModelGroup* group,
    const std::vector<double>& positions)
{
    // TODO: detect changes to state
    ROS_DEBUG_NAMED(LOG, "Set positions of joint group '%s'", group->getName().c_str());
    m_robot_state->setJointGroupPositions(group, positions);
    updateAndNotify();
}

void RobotCommandModel::updateAndNotify()
{
    m_robot_state->update();
    Q_EMIT robotStateChanged();
}

} // namespace sbpl_interface
