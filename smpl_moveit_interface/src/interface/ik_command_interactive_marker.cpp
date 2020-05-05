#include <smpl_moveit_interface/interface/ik_command_interactive_marker.h>

#include <cmath>
#include <cstdlib>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>

#include <smpl_moveit_interface/interface/robot_command_model.h>
#include <smpl_moveit_interface/interface/utils.h>

namespace sbpl_interface {

static const char* LOG = "ik_command_interactive_marker";

IKCommandInteractiveMarker::IKCommandInteractiveMarker(RobotCommandModel* model)
    : m_im_server("phantom_controls")
{
    assert(model != NULL);
    m_model = model;
    connect(m_model, SIGNAL(robotLoaded()), this, SLOT(updateRobotModel()));
    connect(m_model, SIGNAL(robotStateChanged()), this, SLOT(updateRobotState()));
}

void IKCommandInteractiveMarker::setActiveJointGroup(const std::string& group_name)
{
    if (group_name != m_active_group_name) {
        m_active_group_name = group_name;
        reinitInteractiveMarkers();
        updateInteractiveMarkers();
        Q_EMIT updateActiveJointGroup(group_name);
    }
}

static std::string MarkerNameFromTipName(const std::string& tip_name)
{
    return tip_name + "_controls";
}

static std::string TipNameFromMarkerName(const std::string& marker_name)
{
    return marker_name.substr(0, marker_name.rfind("_control"));
}

void IKCommandInteractiveMarker::updateRobotModel()
{
    m_marker_scale_cache.clear();
    reinitInteractiveMarkers();
}

void IKCommandInteractiveMarker::updateRobotState()
{
    updateInteractiveMarkers();
}

// Given a state, for each revolute joint, find the equivalent 2*pi joint
// position that is nearest (numerically) to a reference state
void CorrectIKSolution(
    moveit::core::RobotState& state,
    const moveit::core::JointModelGroup* group,
    const moveit::core::RobotState& ref_state)
{
    for (auto gvidx = 0; gvidx < group->getVariableCount(); ++gvidx) {
        ROS_DEBUG_NAMED(LOG, "Check variable '%s' for bounded revoluteness", group->getVariableNames()[gvidx].c_str());

        auto vidx = group->getVariableIndexList()[gvidx];
        auto* joint = state.getRobotModel()->getJointOfVariable(vidx);
        if (joint->getType() != moveit::core::JointModel::REVOLUTE ||
            !joint->getVariableBounds()[0].position_bounded_)
        {
            continue;
        }

        ROS_DEBUG_NAMED(LOG, "  Normalize variable '%s'", group->getVariableNames()[gvidx].c_str());

        auto spos = state.getVariablePosition(vidx);
        auto vdiff = ref_state.getVariablePosition(gvidx) - spos;
        auto twopi_hops = (int)std::abs(vdiff / (2.0 * M_PI));

        ROS_DEBUG_NAMED(LOG, " -> seed pos: %f", ref_state.getVariablePosition(gvidx));
        ROS_DEBUG_NAMED(LOG, " ->  sol pos: %f", spos);
        ROS_DEBUG_NAMED(LOG, " ->    vdiff: %f", vdiff);
        ROS_DEBUG_NAMED(LOG, " -> num hops: %d", twopi_hops);

        auto dir = std::copysign(1.0, vdiff);
        auto npos = spos + (2.0 * M_PI) * twopi_hops * dir;
        if (std::abs(npos - ref_state.getVariablePosition(gvidx)) > M_PI) {
            npos += 2.0 * M_PI * dir;
        }

        ROS_DEBUG_NAMED(LOG, " ->     npos: %f", npos);

        if (twopi_hops) {
            ROS_DEBUG_NAMED(LOG, " -> Attempt to normalize variable '%s' to %f from %f", group->getVariableNames()[gvidx].c_str(), npos, spos);
        } else {
            ROS_DEBUG_NAMED(LOG, "No hops necessary");
        }

        state.setVariablePosition(vidx, npos);
        if (!state.satisfiesBounds(joint)) {
            ROS_WARN_NAMED(LOG, "normalized value for '%s' out of bounds",  group->getVariableNames()[gvidx].c_str());
            state.setVariablePosition(vidx, spos);
        }
    }
}

void IKCommandInteractiveMarker::processInteractiveMarkerFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr& msg)
{
    ROS_DEBUG_NAMED(LOG, "Interactive marker feedback");
    ROS_DEBUG_NAMED(LOG, "  Marker: %s", msg->marker_name.c_str());
    ROS_DEBUG_NAMED(LOG, "  Control: %s", msg->control_name.c_str());
    ROS_DEBUG_NAMED(LOG, "  Event Type: %u", (unsigned)msg->event_type);

    if (msg->event_type == visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE) {
        auto* robot_state = m_model->getRobotState();
        auto* group = robot_state->getJointModelGroup(m_active_group_name);
        if (!group) {
            ROS_ERROR_NAMED(LOG, "Failed to retrieve joint group '%s'", m_active_group_name.c_str());
            return;
        }

        // run ik from this tip link
        Eigen::Isometry3d wrist_pose;
        tf::poseMsgToEigen(msg->pose, wrist_pose);

        moveit::core::RobotState ik_state(*robot_state);
        if (ik_state.setFromIK(group, wrist_pose)) {
            // correct solution to be closer to seed state
            CorrectIKSolution(ik_state, group, *robot_state);
            m_model->setVariablePositions(ik_state.getVariablePositions());
        } else {
            // TODO: anything special here?
        }
    }
}

static
auto normalized(geometry_msgs::Quaternion q) -> geometry_msgs::Quaternion
{
    auto len = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    auto qq = q;
    qq.w /= len;
    qq.x /= len;
    qq.y /= len;
    qq.z /= len;
    return qq;
}

static
auto MakeNormalizedQuaternion(double w, double x, double y, double z)
    -> geometry_msgs::Quaternion
{
    geometry_msgs::Quaternion q;
    q.w = w;
    q.x = x;
    q.y = y;
    q.z = z;
    return normalized(q);
}

// Get the largest coordinate (x, y, or z) of the bounding box, posed at $pose.
static
auto GetMaxCoordOBB(const Eigen::Isometry3d& origin, const double size[3]) -> double
{
    double half[3] = { 0.5 * size[0], 0.5 * size[1], 0.5 * size[2] };
    Eigen::Vector3d corners[8] =
    {
        Eigen::Vector3d(-half[0], -half[1], -half[2]),
        Eigen::Vector3d(-half[0], -half[1],  half[2]),
        Eigen::Vector3d(-half[0],  half[1], -half[2]),
        Eigen::Vector3d(-half[0],  half[1],  half[2]),
        Eigen::Vector3d( half[0], -half[1], -half[2]),
        Eigen::Vector3d( half[0], -half[1],  half[2]),
        Eigen::Vector3d( half[0],  half[1], -half[2]),
        Eigen::Vector3d( half[0],  half[1], -half[2]),
    };

    auto s = 0.0;
    for (auto v = 0; v < 8; ++v) {
        auto corner = Eigen::Vector3d(origin * corners[v]);
        s = std::max(s, std::fabs(corner.x()));
        s = std::max(s, std::fabs(corner.y()));
        s = std::max(s, std::fabs(corner.z()));
    }
    return s;
}

static
auto ComputeMarkerScale(const moveit::core::LinkModel* link) -> float
{
    auto sqrd = [](double t) { return t * t; };

    auto scale = 0.0f;

    ROS_INFO_NAMED(LOG, "Scan %zu shapes to determine marker scale", link->getShapes().size());

    for (auto i = 0; i < link->getShapes().size(); ++i) {
        auto& shape = link->getShapes()[i];
        auto& origin = link->getCollisionOriginTransforms()[i];

        switch (shape->type) {
        case shapes::BOX:
        {
            auto* box = static_cast<const shapes::Box*>(shape.get());
            scale = std::max(scale, (float)GetMaxCoordOBB(origin, box->size));
            break;
        }
        case shapes::CONE:
        {
            auto* cone = static_cast<const shapes::Cone*>(shape.get());
            double size[3] = { 2.0 * cone->radius, 2.0 * cone->radius, cone->length };
            scale = std::max(scale, (float)GetMaxCoordOBB(origin, size));
            break;
        }
        case shapes::CYLINDER:
        {
            auto* cylinder = static_cast<const shapes::Cylinder*>(shape.get());

            double size[3] = { 2.0 * cylinder->radius, 2.0 * cylinder->radius, cylinder->length };
            scale = std::max(scale, (float)GetMaxCoordOBB(origin, size));
            break;
        }
        case shapes::MESH:
        {
            // NOTE: any chance there are vertices not referenced by the
            // available triangles that we are counting here? That would be
            // most unfortunate.

            auto* mesh = static_cast<const shapes::Mesh*>(shape.get());

            auto s = 0.0;
            for (auto vidx = 0; vidx < mesh->vertex_count; ++vidx) {
                auto vxi = 3 * vidx + 0;
                auto vyi = 3 * vidx + 1;
                auto vzi = 3 * vidx + 2;

                auto v = Eigen::Vector3d(
                        mesh->vertices[vxi],
                        mesh->vertices[vyi],
                        mesh->vertices[vzi]);
                v = origin * v;

                s = std::max(s, std::fabs(v.x()));
                s = std::max(s, std::fabs(v.y()));
                s = std::max(s, std::fabs(v.z()));
            }

            scale = std::max(scale, (float)s);
            break;
        }
        case shapes::SPHERE:
        {
            auto* sphere = static_cast<const shapes::Sphere*>(shape.get());
            auto s = sphere->radius;
            double size[3] = { 2.0 * s, 2.0 * s, 2.0 * s };
            scale = std::max(scale, (float)GetMaxCoordOBB(origin, size));
            break;
        }
        case shapes::OCTREE:
        case shapes::PLANE:
        case shapes::UNKNOWN_SHAPE:
            ROS_WARN_NAMED(LOG, "Unsupported shape for determining ik marker scale");
            break;
        }
    }

    if (scale == 0.0f) {
        return 1.0f;
    } else {
        return 2.0f * scale;
    }
}

// This gets called whenever the robot model or active joint group changes.
void IKCommandInteractiveMarker::reinitInteractiveMarkers()
{
    auto& robot_model = m_model->getRobotModel();

    ROS_INFO_NAMED(LOG, "Setup Interactive Markers for Robot");

    ROS_INFO_NAMED(LOG, " -> Remove any existing markers");
    m_im_server.clear();
    m_int_marker_names.clear();

    bool have_robot = (bool)robot_model;
    bool have_active_group = !m_active_group_name.empty();
    if (!have_robot || !have_active_group) {
        if (!have_robot) {
            ROS_INFO_NAMED(LOG, "No robot model to initialize interactive markers from");
        }
        if (!have_active_group) {
            ROS_INFO_NAMED(LOG, "No active joint group to initialize interactive markers from");
        }
        m_im_server.applyChanges(); // TODO: defer idiom here
        return;
    }

    auto* jg = robot_model->getJointModelGroup(m_active_group_name);
    if (!jg) {
        ROS_INFO_NAMED(LOG, "Failed to retrieve joint group '%s'", m_active_group_name.c_str());
        m_im_server.applyChanges();
        return;
    }

    auto tips = GetTipLinks(*jg);

    for (auto* tip_link : tips) {
        ROS_INFO_NAMED(LOG, "Adding interactive marker for controlling pose of link %s", tip_link->getName().c_str());

        visualization_msgs::InteractiveMarker tip_marker;
        tip_marker.header.frame_id = robot_model->getModelFrame();

        tip_marker.pose.orientation =
                MakeNormalizedQuaternion(1.0, 0.0, 0.0, 0.0);
        tip_marker.pose.position.x = 0.0;
        tip_marker.pose.position.y = 0.0;
        tip_marker.pose.position.z = 0.0;

        tip_marker.name = MarkerNameFromTipName(tip_link->getName());
        tip_marker.description = "ik control of link " + tip_link->getName();

        // determine the size of the marker
        auto scale = 1.0f;
        auto it = m_marker_scale_cache.find(tip_link->getName());
        if (it != end(m_marker_scale_cache)) {
            scale = it->second;
        } else {
            scale = ComputeMarkerScale(tip_link);
            ROS_INFO_NAMED(LOG, "Marker scale for link '%s' = %f", tip_link->getName().c_str(), scale);
            m_marker_scale_cache[tip_link->getName()] = scale;
        }
        tip_marker.scale = scale;

        visualization_msgs::InteractiveMarkerControl dof_control;
        dof_control.orientation_mode =
                visualization_msgs::InteractiveMarkerControl::INHERIT;
        dof_control.always_visible = false;
//        dof_control.description = "pose_control";

        dof_control.orientation = MakeNormalizedQuaternion(1.0, 1.0, 0.0, 0.0);

        dof_control.name = "rotate_x";
        dof_control.interaction_mode =
                visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        tip_marker.controls.push_back(dof_control);

        dof_control.name = "move_x";
        dof_control.interaction_mode =
                visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        tip_marker.controls.push_back(dof_control);

        dof_control.orientation =
                MakeNormalizedQuaternion(1.0, 0.0, 1.0, 0.0);

        dof_control.name = "rotate_z";
        dof_control.interaction_mode =
                visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        tip_marker.controls.push_back(dof_control);

        dof_control.name = "move_z";
        dof_control.interaction_mode =
                visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        tip_marker.controls.push_back(dof_control);

        dof_control.orientation = MakeNormalizedQuaternion(1.0, 0.0, 0.0, 1.0);

        dof_control.name = "rotate_y";
        dof_control.interaction_mode =
                visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
        tip_marker.controls.push_back(dof_control);

        dof_control.name = "move_y";
        dof_control.interaction_mode =
                visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        tip_marker.controls.push_back(dof_control);

        auto feedback_fn = [this](
            const visualization_msgs::InteractiveMarkerFeedbackConstPtr& msg)
        {
            return this->processInteractiveMarkerFeedback(msg);
        };
        m_im_server.insert(tip_marker, feedback_fn);
        m_int_marker_names.push_back(tip_marker.name);
    }

    m_im_server.applyChanges();
}

void IKCommandInteractiveMarker::updateInteractiveMarkers()
{
    auto* robot_state = m_model->getRobotState();
    assert(robot_state != NULL);
    for (auto& marker_name : m_int_marker_names) {
        // stuff the current pose
        std::string tip_link_name = TipNameFromMarkerName(marker_name);
        auto& T_model_tip = robot_state->getGlobalLinkTransform(tip_link_name);

        geometry_msgs::Pose tip_pose;
        tf::poseEigenToMsg(T_model_tip, tip_pose);

        // update the pose of the interactive marker
        std_msgs::Header header;
        header.frame_id = m_model->getRobotModel()->getModelFrame();
        header.stamp = ros::Time(0);
        if (!m_im_server.setPose(marker_name, tip_pose, header)) {
            ROS_INFO_NAMED(LOG, "Failed to set pose of interactive marker '%s'", marker_name.c_str());
        }
    }

    m_im_server.applyChanges();
}

} // namespace sbpl_interface
