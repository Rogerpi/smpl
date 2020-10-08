#include "sbpl_dualplanner_manager.h"

// system includes
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>
#include <smpl/stl/memory.h>

#include "sbpl_dualplanning_context.h"

static const char* xmlTypeToString(XmlRpc::XmlRpcValue::Type type)
{
    switch (type) {
    case XmlRpc::XmlRpcValue::TypeInvalid:
        return "Invalid";
    case XmlRpc::XmlRpcValue::TypeBoolean:
        return "Boolean";
    case XmlRpc::XmlRpcValue::TypeInt:
        return "Int";
    case XmlRpc::XmlRpcValue::TypeDouble:
        return "Double";
    case XmlRpc::XmlRpcValue::TypeString:
        return "String";
    case XmlRpc::XmlRpcValue::TypeDateTime:
        return "DateTime";
    case XmlRpc::XmlRpcValue::TypeBase64:
        return "Base64";
    case XmlRpc::XmlRpcValue::TypeArray:
        return "Array";
    case XmlRpc::XmlRpcValue::TypeStruct:
        return "Struct";
    default:
        return "Unrecognized";
    }
}

namespace sbpl_interface {

// pp = planning plugin
static const char* PP_LOGGER = "dualplanning";

static
void LogMotionPlanRequest(const MotionPlanRequest& req)
{
    ROS_DEBUG_NAMED(PP_LOGGER, "Motion Plan Request");

    ROS_DEBUG_NAMED(PP_LOGGER, "  workspace_parameters");
    ROS_DEBUG_NAMED(PP_LOGGER, "    header");
    ROS_DEBUG_NAMED(PP_LOGGER, "      seq: %u", req.workspace_parameters.header.seq);
    ROS_DEBUG_STREAM_NAMED(PP_LOGGER, "      stamp: " << req.workspace_parameters.header.stamp);
    ROS_DEBUG_NAMED(PP_LOGGER, "      frame_id: \"%s\"", req.workspace_parameters.header.frame_id.c_str());
    ROS_DEBUG_NAMED(PP_LOGGER, "    min_corner");
    ROS_DEBUG_NAMED(PP_LOGGER, "      x: %f", req.workspace_parameters.min_corner.x);
    ROS_DEBUG_NAMED(PP_LOGGER, "      y: %f", req.workspace_parameters.min_corner.y);
    ROS_DEBUG_NAMED(PP_LOGGER, "      z: %f", req.workspace_parameters.min_corner.z);
    ROS_DEBUG_NAMED(PP_LOGGER, "    max_corner");
    ROS_DEBUG_NAMED(PP_LOGGER, "      x: %f", req.workspace_parameters.max_corner.x);
    ROS_DEBUG_NAMED(PP_LOGGER, "      y: %f", req.workspace_parameters.max_corner.y);
    ROS_DEBUG_NAMED(PP_LOGGER, "      z: %f", req.workspace_parameters.max_corner.z);

    ROS_DEBUG_NAMED(PP_LOGGER, "  start_state");
    ROS_DEBUG_NAMED(PP_LOGGER, "    joint_state:");
    auto& joint_state = req.start_state.joint_state;
    for (size_t jidx = 0; jidx < joint_state.name.size(); ++jidx) {
        ROS_DEBUG_NAMED(PP_LOGGER, "      { name: %s, position: %0.3f }", joint_state.name[jidx].c_str(), joint_state.position[jidx]);
    }
    ROS_DEBUG_NAMED(PP_LOGGER, "    multi_dof_joint_state");
    auto& multi_dof_joint_state = req.start_state.multi_dof_joint_state;
    ROS_DEBUG_NAMED(PP_LOGGER, "      header: { seq: %d, stamp: %0.3f, frame_id: \"%s\" }",
            multi_dof_joint_state.header.seq,
            multi_dof_joint_state.header.stamp.toSec(),
            multi_dof_joint_state.header.frame_id.c_str());
    for (size_t jidx = 0; jidx < multi_dof_joint_state.joint_names.size(); ++jidx) {
        auto& joint_name = multi_dof_joint_state.joint_names[jidx];
        auto& transform = multi_dof_joint_state.transforms[jidx];
        ROS_DEBUG_NAMED(PP_LOGGER, "      { joint_names: %s, transform: (%f, %f, %f, %f, %f, %f, %f) }",
                joint_name.c_str(),
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w);
    }

    ROS_DEBUG_NAMED(PP_LOGGER, "    attached_collision_objects: %zu", req.start_state.attached_collision_objects.size());
    ROS_DEBUG_NAMED(PP_LOGGER, "    is_diff: %s", req.start_state.is_diff ? "true" : "false");

    ROS_DEBUG_NAMED(PP_LOGGER, "  goal_constraints: %zu", req.goal_constraints.size());
    for (size_t cind = 0; cind < req.goal_constraints.size(); ++cind) {
        auto& constraints = req.goal_constraints[cind];

        // joint constraints
        ROS_DEBUG_NAMED(PP_LOGGER, "    joint_constraints: %zu", constraints.joint_constraints.size());
        for (size_t jcind = 0; jcind < constraints.joint_constraints.size(); ++jcind) {
            auto& joint_constraint = constraints.joint_constraints[jcind];
            ROS_DEBUG_NAMED(PP_LOGGER, "      joint_name: %s, position: %0.3f, tolerance_above: %0.3f, tolerance_below: %0.3f, weight: %0.3f",
                    joint_constraint.joint_name.c_str(),
                    joint_constraint.position,
                    joint_constraint.tolerance_above,
                    joint_constraint.tolerance_below,
                    joint_constraint.weight);
        }

        // position constraints
        ROS_DEBUG_NAMED(PP_LOGGER, "    position_constraints: %zu", constraints.position_constraints.size());
        for (size_t pcind = 0; pcind < constraints.position_constraints.size(); ++pcind) {
            auto& pos_constraint = constraints.position_constraints[pcind];
            ROS_DEBUG_NAMED(PP_LOGGER, "      header: { frame_id: %s, seq: %u, stamp: %0.3f }", pos_constraint.header.frame_id.c_str(), pos_constraint.header.seq, pos_constraint.header.stamp.toSec());
            ROS_DEBUG_NAMED(PP_LOGGER, "      link_name: %s", pos_constraint.link_name.c_str());
            ROS_DEBUG_NAMED(PP_LOGGER, "      target_point_offset: (%0.3f, %0.3f, %0.3f)", pos_constraint.target_point_offset.x, pos_constraint.target_point_offset.y, pos_constraint.target_point_offset.z);
            ROS_DEBUG_NAMED(PP_LOGGER, "      constraint_region:");
            ROS_DEBUG_NAMED(PP_LOGGER, "        primitives: %zu", pos_constraint.constraint_region.primitives.size());
            for (size_t pind = 0; pind < pos_constraint.constraint_region.primitives.size(); ++pind) {
                auto& prim = pos_constraint.constraint_region.primitives[pind];
                auto& pose = pos_constraint.constraint_region.primitive_poses[pind];
                ROS_DEBUG_NAMED(PP_LOGGER, "          { type: %d, pose: { position: (%0.3f, %0.3f, %0.3f), orientation: (%0.3f, %0.3f, %0.3f, %0.3f) } }", prim.type, pose.position.x, pose.position.y, pose.position.y, pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
            }
            ROS_DEBUG_NAMED(PP_LOGGER, "        meshes: %zu", pos_constraint.constraint_region.meshes.size());
        }

        // orientation constarints
        ROS_DEBUG_NAMED(PP_LOGGER, "    orientation_constraints: %zu", constraints.orientation_constraints.size());
        for (size_t ocind = 0; ocind < constraints.orientation_constraints.size(); ++ocind) {
            auto& rot_constraint = constraints.orientation_constraints[ocind];
            ROS_DEBUG_NAMED(PP_LOGGER, "      header: { frame_id: %s, seq: %u, stamp: %0.3f }", rot_constraint.header.frame_id.c_str(), rot_constraint.header.seq, rot_constraint.header.stamp.toSec());
            ROS_DEBUG_NAMED(PP_LOGGER, "      orientation: (%0.3f, %0.3f, %0.3f, %0.3f)", rot_constraint.orientation.w, rot_constraint.orientation.x, rot_constraint.orientation.y, rot_constraint.orientation.z);
            ROS_DEBUG_NAMED(PP_LOGGER, "      link_name: %s", rot_constraint.link_name.c_str());
            ROS_DEBUG_NAMED(PP_LOGGER, "      absolute_x_axis_tolerance: %0.3f", rot_constraint.absolute_x_axis_tolerance);
            ROS_DEBUG_NAMED(PP_LOGGER, "      absolute_y_axis_tolerance: %0.3f", rot_constraint.absolute_y_axis_tolerance);
            ROS_DEBUG_NAMED(PP_LOGGER, "      absolute_z_axis_tolerance: %0.3f", rot_constraint.absolute_z_axis_tolerance);
            ROS_DEBUG_NAMED(PP_LOGGER, "      weight: %0.3f", rot_constraint.weight);
        }

        // visibility constraints
        ROS_DEBUG_NAMED(PP_LOGGER, "    visibility_constraints: %zu", constraints.visibility_constraints.size());
    }

    ROS_DEBUG_NAMED(PP_LOGGER, "  path_constraints");
    ROS_DEBUG_NAMED(PP_LOGGER, "    joint_constraints: %zu", req.path_constraints.joint_constraints.size());
    ROS_DEBUG_NAMED(PP_LOGGER, "    position_constraints: %zu", req.path_constraints.position_constraints.size());
    ROS_DEBUG_NAMED(PP_LOGGER, "    orientation_constraints: %zu", req.path_constraints.orientation_constraints.size());
    ROS_DEBUG_NAMED(PP_LOGGER, "    visibility_constraints: %zu", req.path_constraints.visibility_constraints.size());

    ROS_DEBUG_NAMED(PP_LOGGER, "  trajectory_constraints");
    for (size_t cind = 0; cind < req.trajectory_constraints.constraints.size(); ++cind) {
        auto& constraints = req.trajectory_constraints.constraints[cind];
        ROS_DEBUG_NAMED(PP_LOGGER, "    joint_constraints: %zu", constraints.joint_constraints.size());
        ROS_DEBUG_NAMED(PP_LOGGER, "    position_constraints: %zu", constraints.position_constraints.size());
        ROS_DEBUG_NAMED(PP_LOGGER, "    orientation_constraints: %zu", constraints.orientation_constraints.size());
        ROS_DEBUG_NAMED(PP_LOGGER, "    visibility_constraints: %zu", constraints.visibility_constraints.size());
    }

    ROS_DEBUG_NAMED(PP_LOGGER, "  planner_id: %s", req.planner_id.c_str());
    ROS_DEBUG_NAMED(PP_LOGGER, "  group_name: %s", req.group_name.c_str());
    ROS_DEBUG_NAMED(PP_LOGGER, "  num_planning_attempts: %d", req.num_planning_attempts);
    ROS_DEBUG_NAMED(PP_LOGGER, "  allowed_planning_time: %f", req.allowed_planning_time);
    ROS_DEBUG_NAMED(PP_LOGGER, "  max_velocity_scaling_factor: %f", req.max_velocity_scaling_factor);
}

static
void LogPlanningScene(const planning_scene::PlanningScene& scene)
{
    ROS_INFO_NAMED(PP_LOGGER, "Planning Scene");
    ROS_INFO_NAMED(PP_LOGGER, "  Ptr: %p", &scene);
    ROS_INFO_NAMED(PP_LOGGER, "  Name: %s", scene.getName().c_str());
    ROS_INFO_NAMED(PP_LOGGER, "  Has Parent: %s", scene.getParent() ? "true" : "false");
    ROS_INFO_NAMED(PP_LOGGER, "  Has Robot Model: %s", scene.getRobotModel() ? "true" : "false");
    ROS_INFO_NAMED(PP_LOGGER, "  Planning Frame: %s", scene.getPlanningFrame().c_str());
    ROS_INFO_NAMED(PP_LOGGER, "  Active Collision Detector Name: %s", scene.getActiveCollisionDetectorName().c_str());
    ROS_INFO_NAMED(PP_LOGGER, "  Has World: %s", scene.getWorld() ? "true" : "false");
    if (scene.getWorld()) {
        ROS_INFO_NAMED(PP_LOGGER, "    size:  %zu", scene.getWorld()->size());
        ROS_INFO_NAMED(PP_LOGGER, "    Object IDs: %zu", scene.getWorld()->getObjectIds().size());
        auto wbegin = scene.getWorld()->begin();
        auto wend = scene.getWorld()->end();
        for (auto oit = wbegin; oit != wend; ++oit) {
            auto& object_id = oit->first;
            ROS_INFO_NAMED(PP_LOGGER, "      %s", object_id.c_str());
        }
    }
    ROS_INFO_NAMED(PP_LOGGER, "  Has Collision World: %s", scene.getCollisionWorld() ? "true" : "false");
    ROS_INFO_NAMED(PP_LOGGER, "  Has Collision Robot: %s", scene.getCollisionRobot() ? "true" : "false");
    ROS_INFO_NAMED(PP_LOGGER, "  Current State:");

    auto& current_state = scene.getCurrentState();
    for (size_t vind = 0; vind < current_state.getVariableCount(); ++vind) {
        ROS_INFO_NAMED(PP_LOGGER, "    %s: %0.3f", current_state.getVariableNames()[vind].c_str(), current_state.getVariablePosition(vind));
    }

//    ROS_INFO_NAMED(PP_LOGGER, "Allowed collision matrix");
//    scene.getAllowedCollisionMatrix().print(std::cout);
}

// key/value pairs for parameters to pass down to planner
using PlannerSettings = std::map<std::string, std::string>;

// config name -> (string -> string)
using PlannerSettingsMap = std::map<std::string, PlannerSettings>;

static
bool XMLToString(XmlRpc::XmlRpcValue& value, std::string& out)
{
    switch (value.getType()) {
    case XmlRpc::XmlRpcValue::TypeString: {
        std::string string_param = (std::string)value;
        out = string_param;
    }   break;
    case XmlRpc::XmlRpcValue::TypeBoolean: {
        bool bool_param = (bool)value;
        out = bool_param ? "true" : "false";
    }   break;
    case XmlRpc::XmlRpcValue::TypeInt: {
        int int_param = (int)value;
        out = std::to_string(int_param);
    }   break;
    case XmlRpc::XmlRpcValue::TypeDouble: {
        double double_param = (double)value;
        out = std::to_string(double_param);
    }   break;
    case XmlRpc::XmlRpcValue::TypeArray: {
        std::stringstream ss;
        for (int i = 0; i < value.size(); ++i) {
            XmlRpc::XmlRpcValue& arr_value = value[i];
            switch (arr_value.getType()) {
            case XmlRpc::XmlRpcValue::TypeBoolean:
                ss << (bool)arr_value;
                break;
            case XmlRpc::XmlRpcValue::TypeInt:
                ss << (int)arr_value;
                break;
            case XmlRpc::XmlRpcValue::TypeDouble:
                ss << (double)arr_value;
                break;
            case XmlRpc::XmlRpcValue::TypeString:
                ss << (std::string)arr_value;
                break;
            default:
                ROS_ERROR_NAMED(PP_LOGGER, "Unsupported array member type (%s)", xmlTypeToString(arr_value.getType()));
                return false;
            }

            if (i != value.size() - 1) {
                ss << ' ';
            }
        }
        out = ss.str();
        return true;
    }   break;
    case XmlRpc::XmlRpcValue::TypeStruct: {
        std::stringstream ss;
        int i = 0;
        for (auto it = value.begin(); it != value.end(); ++it) {
            ss << it->first << ' ';
            XmlRpc::XmlRpcValue& struct_value = it->second;
            switch (struct_value.getType()) {
            case XmlRpc::XmlRpcValue::TypeBoolean:
                ss << (bool)struct_value;
                break;
            case XmlRpc::XmlRpcValue::TypeInt:
                ss << (int)struct_value;
                break;
            case XmlRpc::XmlRpcValue::TypeDouble:
                ss << (double)struct_value;
                break;
            case XmlRpc::XmlRpcValue::TypeString:
                ss << (std::string)struct_value;
                break;
            default:
                ROS_ERROR_NAMED(PP_LOGGER, "Unsupported struct member type (%s)", xmlTypeToString(struct_value.getType()));
                return false;
            }

            if (i != value.size() - 1) {
                ss << ' ';
            }

            ++i;
        }

        out = ss.str();
        return true;
    }   break;
    default: {
        return false;
    }   break;
    }

    return true;
}

static
bool LoadSettingsMap(
    const ros::NodeHandle& nh,
    const std::string& param_name,
    PlannerSettingsMap& settings)
{
    if (!nh.hasParam(param_name)) {
        return true;
    }

    PlannerSettingsMap planner_configs;

    XmlRpc::XmlRpcValue search_configs_cfg;
    if (!nh.getParam(param_name, search_configs_cfg)) {
        ROS_ERROR_NAMED(PP_LOGGER, "Failed to retrieve '%s'", param_name.c_str());
        return false;
    }

    // planner_configs should be a mapping of planner configuration names to
    // another struct which is a mapping of parameter names (strings) to
    // parameter values (type known per-parameter)
    if (search_configs_cfg.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
        ROS_ERROR_NAMED(PP_LOGGER, "'planner_configs' section should be a map of planner configuration names to planner configurations (found type '%s')", xmlTypeToString(search_configs_cfg.getType()));
        return false;
    }

    for (auto it = search_configs_cfg.begin();
        it != search_configs_cfg.end();
        ++it)
    {
        const std::string& planner_config_name = it->first;
        XmlRpc::XmlRpcValue& planner_settings_cfg = it->second;

        ROS_DEBUG_NAMED(PP_LOGGER, "Read configuration for '%s'", planner_config_name.c_str());

        if (planner_settings_cfg.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            ROS_ERROR_NAMED(PP_LOGGER, "Planner configuration should be a map of parameter names to values");
            return false;
        }

        PlannerSettings planner_settings;
        for (auto iit = planner_settings_cfg.begin();
            iit != planner_settings_cfg.end();
            ++iit)
        {
            const std::string& planner_setting_name = iit->first;
            XmlRpc::XmlRpcValue& planner_setting = iit->second;
            ROS_DEBUG_NAMED(PP_LOGGER, "Read value for parameter '%s'", planner_setting_name.c_str());
            if (!XMLToString(planner_setting, planner_settings[planner_setting_name])) {
                ROS_ERROR_NAMED(PP_LOGGER, "Unsupported parameter type");
            }
            // planner settings filled if no error above
        }

        planner_configs[planner_config_name] = planner_settings;
    }

    settings = std::move(planner_configs);
    return true;
}

/// Load the mapping from planner configuration name to planner configuration
/// settings.
static
bool LoadPlannerConfigurationMapping(
    const ros::NodeHandle& nh,
    const moveit::core::RobotModel& model,
    PlannerConfigurationMap* pcm_out)
{
    // New Behavior! Instead of requiring sane config, we'll instead warn on
    // all errors and parse as many existing configurations as possible. This
    // can be useful for debugging. It also allows restarting move_group using
    // a different planner plugin without clearing parameters, which causes
    // configurations from other planner plugins to persist on the param server
    // if they do not exist for this plugin.
    auto ignore_errors = true;

    PlannerSettingsMap search_settings;
    if (!LoadSettingsMap(nh, "search_configs", search_settings)) {
        ROS_ERROR_NAMED(PP_LOGGER, "Failed to load search settings");
        return false;
    }
    PlannerSettingsMap heuristic_settings;
    if (!LoadSettingsMap(nh, "heuristic_configs", heuristic_settings)) {
        ROS_ERROR_NAMED(PP_LOGGER, "Failed to load heuristic settings");
        return false;
    }
    PlannerSettingsMap graph_settings;
    if (!LoadSettingsMap(nh, "graph_configs", graph_settings)) {
        ROS_ERROR_NAMED(PP_LOGGER, "Failed to load graph settings");
        return false;
    }
    PlannerSettingsMap shortcut_settings;
    if (!LoadSettingsMap(nh, "shortcut_configs", shortcut_settings)) {
        ROS_ERROR_NAMED(PP_LOGGER, "Failed to load shortcut settings");
        return false;
    }

    ROS_DEBUG_NAMED(PP_LOGGER, "Successfully loaded planner settings");

    PlannerConfigurationMap pcm;

    const char* req_group_params[] = { };

    for (auto& group_name : model.getJointModelGroupNames()) {
        if (!nh.hasParam(group_name)) {
            ROS_WARN_NAMED(PP_LOGGER, "No planning configuration for joint group '%s'", group_name.c_str());
            continue;
        }

        ROS_DEBUG_NAMED(PP_LOGGER, "Read configuration for joint group '%s'", group_name.c_str());

        // read group_name -> group config
        XmlRpc::XmlRpcValue joint_group_cfg;
        if (!nh.getParam(group_name, joint_group_cfg)) {
            ROS_ERROR_NAMED(PP_LOGGER, "Failed to retrieve '%s'", group_name.c_str());
            return false;
        }

        if (joint_group_cfg.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            ROS_WARN_NAMED(PP_LOGGER, "'%s' should be a map of group names to group settings", group_name.c_str());
            if (ignore_errors) continue; // next planning group
            return false;
        }

        ROS_DEBUG_NAMED(PP_LOGGER, "Create (group, planner) configurations");

        // read array of planner configurations
        if (!joint_group_cfg.hasMember("planner_configs")) {
            ROS_WARN("No planner configurations specified for group '%s'", group_name.c_str());
            continue;
        }

        auto& planner_configs_cfg = joint_group_cfg["planner_configs"];
        if (planner_configs_cfg.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
            ROS_WARN_NAMED(PP_LOGGER, "'planner_configs' element of '%s' should be a map of names to planner configurations (actual: %s)", group_name.c_str(), xmlTypeToString(planner_configs_cfg.getType()));
            if (ignore_errors) continue; // next planning group
            return false;
        }

        for (auto pcit = planner_configs_cfg.begin(); pcit != planner_configs_cfg.end(); ++pcit) {
            auto& pc_name = pcit->first;
            auto& planner_config = pcit->second;
            if (planner_config.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
                ROS_WARN_NAMED(PP_LOGGER, "planner config should be a map from configuration name to configuration");
                if (ignore_errors) continue; // next configuration
                return false;
            }

            const char *required_keys[] =
            {
                "search_config",
                "heuristic_config",
                "graph_config",
                "shortcut_config"
            };

            if (std::any_of(
                    required_keys,
                    required_keys + sizeof(required_keys) / sizeof(const char*),
                    [&](const char *key)
                    {
                        return !planner_config.hasMember(key);
                    }))
            {
                ROS_WARN_NAMED(PP_LOGGER, "planner config lacks required keys");
                for (int i = 0; i < sizeof(required_keys) / sizeof(const char*); ++i) {
                    const char* key = required_keys[i];
                    ROS_WARN_NAMED(PP_LOGGER, "Has '%s': %s", key, planner_config.hasMember(key) ? "true" : "false");
                }
                if (ignore_errors) continue; // next configuration
                return false;
            }

            PlannerConfigurationSettings pcs;
            pcs.name = group_name + "[" + pc_name + "]";
            pcs.group = group_name;

            for (auto mit = planner_config.begin(); mit != planner_config.end(); ++mit) {
                if (mit->second.getType() != XmlRpc::XmlRpcValue::TypeString) {
                    ROS_WARN_NAMED(PP_LOGGER, "planner configuration value is not a string");
                    continue;
                }
                const std::string &cfgkey(mit->first);
                const std::string &cfgval(mit->second);
                // squash search, heuristic, graph, and shortcut configurations
                // into combined config
                if (cfgkey == "graph_config") {
                    auto it = graph_settings.find(cfgval);
                    if (it == graph_settings.end()) {
                        ROS_WARN_NAMED(PP_LOGGER, "No graph settings exist for configuration '%s'", cfgval.c_str());
                        continue;
                    }
                    pcs.config.insert(it->second.begin(), it->second.end());
                } else if (cfgkey == "heuristic_config") {
                    auto it = heuristic_settings.find(cfgval);
                    if (it == heuristic_settings.end()) {
                        ROS_WARN_NAMED(PP_LOGGER, "No heuristic settings exist for configuration '%s'", cfgval.c_str());
                        continue;
                    }
                    pcs.config.insert(it->second.begin(), it->second.end());
                } else if (cfgkey == "search_config") {
                    auto it = search_settings.find(cfgval);
                    if (it == search_settings.end()) {
                        ROS_WARN_NAMED(PP_LOGGER, "No search settings exist for configuration '%s'", cfgval.c_str());
                        continue;
                    }
                    pcs.config.insert(it->second.begin(), it->second.end());
                } else if (cfgkey == "shortcut_config") {
                    auto it = shortcut_settings.find(cfgval);
                    if (it == shortcut_settings.end()) {
                        ROS_WARN_NAMED(PP_LOGGER, "No shortcut settings exist for configuration '%s'", cfgval.c_str());
                        continue;
                    }
                    pcs.config.insert(it->second.begin(), it->second.end());
                } else {
                    // anything else goes straight into the configuration map
                    pcs.config.insert({ cfgkey, cfgval });
                }
            }

            pcm[pcs.name] = pcs;
        }

        ROS_DEBUG_NAMED(PP_LOGGER, "Create group configuration");

        // Gather required parameters for the group, regardless of planner configuration
        PlannerSettings req_settings;
        if (std::all_of(
                req_group_params,
                req_group_params + sizeof(req_group_params) / sizeof(const char*),
                [&](const char* param_name)
                {
                    if (!joint_group_cfg.hasMember(param_name)) {
                        ROS_WARN_NAMED(PP_LOGGER, "Group '%s' lacks parameter '%s'", group_name.c_str(), param_name);
                        return false;
                    }

                    ROS_DEBUG_NAMED(PP_LOGGER, "Convert parameter '%s' to string representation", param_name);
                    XmlRpc::XmlRpcValue& param = joint_group_cfg[param_name];
                    if (!XMLToString(param, req_settings[param_name])) {
                        ROS_ERROR_NAMED(PP_LOGGER, "Unsupported parameter type");
                        return false;
                    }

                    ROS_DEBUG_NAMED(PP_LOGGER, "Converted parameter to '%s'", req_settings[param_name].c_str());
                    return true;
                }))
        {
            PlannerConfigurationSettings pcs;
            pcs.name = group_name;
            pcs.group = group_name;
            pcs.config = req_settings;
            pcm[pcs.name] = pcs;
        }
    }

    *pcm_out = std::move(pcm);
    return true;
}

// retrive an already-initialized model for a given group
// Roger: This will be invoked twice. One for the first subgroup (eca_and_auv)
// and one for the whole group. 
auto GetModelForGroup(
    SBPLDualPlannerManager* manager,
    const std::string& group_name, const std::string& ik_group_name = std::string())
    -> MoveItRobotModel*
{
    auto it = manager->m_sbpl_models.find(group_name);
    if (it != end(manager->m_sbpl_models)) {
        ROS_DEBUG_NAMED(PP_LOGGER, "Use existing SBPL Robot Model for group '%s'", group_name.c_str());
        return it->second.get();
    }

    auto model = smpl::make_unique<MoveItRobotModel>();
    if (!model->init(manager->m_robot_model, group_name, ik_group_name)) {
        ROS_WARN_NAMED(PP_LOGGER, "Failed to initialize SBPL Robot Model");
        return NULL;
    }

    ROS_INFO_NAMED(PP_LOGGER, "Created SBPL Robot Model for group '%s'", group_name.c_str());
    auto ent = manager->m_sbpl_models.insert(std::make_pair(group_name, std::move(model)));
    return ent.first->second.get();
}

auto GetPlanningContextForPlanner(
    SBPLDualPlannerManager* manager,
    MoveItRobotModel* model_dual, // Whole model
    MoveItRobotModel* model_first, // Model of eca_arm_and_auv
    const std::string& planner_id)
    -> SBPLDualPlanningContextPtr
{
    SBPLDualPlanningContextPtr null_context;
    ROS_INFO_STREAM("Planner id: " << planner_id);
    auto it = manager->m_contexts.find(planner_id);
    if (it != end(manager->m_contexts)) {
        return it->second;
    }

    auto context = SBPLDualPlanningContextPtr(new SBPLDualPlanningContext(
            model_dual, model_first, "sbpl_dualplanning_context", model_dual->planningGroupName()));

    // find a configuration for this group + planner_id
    auto& configs = manager->getPlannerConfigurations();

    // merge parameters from global group parameters and parameters for the
    // selected planning configuration
    auto all_params = std::map<std::string, std::string>();
    for (auto& config : configs) {
        auto& name = config.first;
        auto& settings = config.second;
        auto& group_name = model_dual->planningGroupName();
        ROS_INFO_STREAM("NAME " << name << " settings " << settings.name << " group " << group_name);
        if (name == group_name) {
            ROS_INFO_STREAM("1 ");
            for(auto & s : settings.config){
                ROS_INFO_STREAM("first " << s.first << " second "<< s.second);
            }
            all_params.insert(begin(settings.config), end(settings.config));
        } else if (name == planner_id) {
            ROS_INFO_STREAM("2 ");
            for(auto & s : settings.config){
                ROS_INFO_STREAM("first " << s.first << " second "<< s.second);
            }
            all_params.insert(begin(settings.config), end(settings.config));
        }
    }

    if (!context->init(all_params)) {
        ROS_ERROR_NAMED(PP_LOGGER, "Failed to initialize SBPL Planning Context");
        return null_context;
    }

    manager->m_contexts.insert(std::make_pair(planner_id, context));
    return context;
}

auto SelectPlanningLinks(
    const SBPLDualPlannerManager* manager,
    const MotionPlanRequest& req)
    -> std::pair<std::string, std::string>
{
    if (req.goal_constraints.empty()) {
        return std::make_pair(std::string(), std::string()); // doesn't matter, we'll bail out soon
    }

    auto& goal_constraint = req.goal_constraints.front();
    // should've received one pose constraint for two links, o/w
    // canServiceRequest would have complained (TODO, commented temorarily)
    if (!goal_constraint.position_constraints.size() == 2) {
        auto& position_constraint1 = goal_constraint.position_constraints[0];
        auto& position_constraint2 = goal_constraint.position_constraints[1];
        return std::make_pair(position_constraint1.link_name, position_constraint2.link_name);
    }

    return std::make_pair(std::string(), std::string()); // well...nothing we can do about this
}

SBPLDualPlannerManager::SBPLDualPlannerManager() :
    Base(),
    m_robot_model(),
    m_viz()
{
    ROS_DEBUG_NAMED(PP_LOGGER, "Constructed SBPL Dual Planner Manager");
    smpl::viz::set_visualizer(&m_viz);
}

SBPLDualPlannerManager::~SBPLDualPlannerManager()
{
    ROS_DEBUG_NAMED(PP_LOGGER, "Destructed SBPL Dual Planner Manager");
    if (smpl::viz::visualizer() == &m_viz) {
        smpl::viz::unset_visualizer();
    }
}

bool SBPLDualPlannerManager::initialize(
    const robot_model::RobotModelConstPtr& model,
    const std::string& ns)
{
    ROS_INFO_NAMED(PP_LOGGER, "Initialize SBPL Dual Planner Manager");
    ROS_INFO_NAMED(PP_LOGGER, "  Robot Model: %s", model->getName().c_str());
    ROS_INFO_NAMED(PP_LOGGER, "  Namespace: %s", ns.c_str());

    m_robot_model = model;

    ros::NodeHandle nh(ns);
    PlannerConfigurationMap pcm;
    if (!LoadPlannerConfigurationMapping(nh, *model, &pcm)) {
        ROS_ERROR_NAMED(PP_LOGGER, "Failed to load planner configurations");
        return false;
    }

    setPlannerConfigurations(pcm);

    ROS_INFO_NAMED(PP_LOGGER, "Initialized SBPL Dual Planner Manager");
    return true;
}

auto SBPLDualPlannerManager::getDescription() const -> std::string
{
    return "Search-Based Planning Algorithms";
}

void SBPLDualPlannerManager::getPlanningAlgorithms(
    std::vector<std::string>& algs) const
{
    auto& configs = getPlannerConfigurations();
    for (auto& entry : configs) {
        if (entry.first.find('[') != std::string::npos) {
            algs.push_back(entry.first);
        }
    }
}

auto SBPLDualPlannerManager::getPlanningContext(
    const planning_scene::PlanningSceneConstPtr& planning_scene,
    const MotionPlanRequest& req,
    moveit_msgs::MoveItErrorCodes& error_code) const
    -> PlanningContextPtr
{
    ROS_DEBUG_NAMED(PP_LOGGER, "Get SBPL Dual Planning Context");

    LogMotionPlanRequest(req);

    PlanningContextPtr null_context;

    if (!canServiceRequest(req)) {
        ROS_WARN_NAMED(PP_LOGGER, "Unable to service request");
        return null_context;
    }

    if (!planning_scene) {
        ROS_WARN_NAMED(PP_LOGGER, "Planning Scene is null");
        return null_context;
    }

    ////////////////////////////////////////
    // Initialize/Update SBPL Robot Model //
    ////////////////////////////////////////

    auto* mutable_me = const_cast<SBPLDualPlannerManager*>(this);
    auto* sbpl_model_dual = GetModelForGroup(mutable_me, req.group_name, "bravo_arm");
    if (!sbpl_model_dual) {
        ROS_WARN_NAMED(PP_LOGGER, "No SBPL Robot Model available for group '%s'", req.group_name.c_str());
        return null_context;
    }
    // TODO dual: group as param
    std::string first_model = "eca_arm_and_auv";
    auto* sbpl_model_first = GetModelForGroup(mutable_me, first_model);
    if (!sbpl_model_first) {
        ROS_WARN_NAMED(PP_LOGGER, "No SBPL Robot Model available for group '%s'", first_model.c_str());
        return null_context;
    }
 
    auto planning_links = SelectPlanningLinks(this, req);
    if (planning_links.first.empty()) { // Only one check shoud be enough
        ROS_INFO_NAMED(PP_LOGGER, "Clear the planning link");
    } else {
        ROS_INFO_NAMED(PP_LOGGER, "Set planning links to '%s' and '%s", planning_links.first.c_str(), planning_links.second.c_str());
    }

    /* Let's avoid this for now as we can't set only one planning link
    if (!sbpl_model->setPlanningLink(planning_link)) {
        ROS_ERROR_NAMED(PP_LOGGER, "Failed to set planning link to '%s'", planning_link.c_str());
        return null_context;
    }
    */

    bool res = true;
    res &= sbpl_model_first->setPlanningScene(planning_scene);
    res &= sbpl_model_first->setPlanningFrame(planning_scene->getPlanningFrame());
    res &= sbpl_model_dual->setPlanningScene(planning_scene);
    res &= sbpl_model_dual->setPlanningFrame(planning_scene->getPlanningFrame());
    //TODO dual: ee as param. This is not set automatically as this model is not a chain
    res &= sbpl_model_dual->setPlanningLink("bravo_jaw_link");
    if (!res) {
        ROS_WARN_NAMED(PP_LOGGER, "Failed to set SBPL Robot Model's planning scene or planning frame");
        return null_context;
    }

    /////////////////////////////////////////////
    // Initialize/Update SBPL Planning Context //
    /////////////////////////////////////////////

#if 0
    LogPlanningScene(*planning_scene);
#endif

    auto sbpl_context = GetPlanningContextForPlanner(
            mutable_me, sbpl_model_dual, sbpl_model_first, req.planner_id);

    ///ROS_WARN("before set planning scene");
    sbpl_context->setPlanningScene(planning_scene);
    ///ROS_WARN("before set motion plan request");
    sbpl_context->setMotionPlanRequest(req);
    ///ROS_WARN("after set motion plan request");
    ///std::getchar();

    return std::move(sbpl_context);
}

bool SBPLDualPlannerManager::canServiceRequest(const MotionPlanRequest& req) const
{
    ROS_DEBUG_NAMED(PP_LOGGER, "SBPLDualPlannerManager::canServiceRequest()");

    if (req.allowed_planning_time < 0.0) {
        ROS_WARN_NAMED(PP_LOGGER, "allowed_planning_time must be non-negative");
        return false;
    }

    // check for a configuration for the requested group
    auto pcit = getPlannerConfigurations().find(req.group_name);
    if (pcit == getPlannerConfigurations().end()) {
        ROS_WARN_NAMED(PP_LOGGER, "No planner configuration found for group '%s'", req.group_name.c_str());
        return false;
    }

    std::vector<std::string> available_algs;
    getPlanningAlgorithms(available_algs);
    if (std::find(
            available_algs.begin(), available_algs.end(), req.planner_id) ==
                    available_algs.end())
    {
        ROS_WARN_NAMED(PP_LOGGER, "No configuration found for the '%s' algorithm", req.planner_id.c_str());
    }

    // guard against unsupported constraints in the underlying interface
    /* TODO
    std::string why;
    if (!smpl::PlannerInterface::SupportsGoalConstraints(
            req.goal_constraints, why))
    {
        ROS_ERROR_NAMED(PP_LOGGER, "goal constraints not supported (%s)", why.c_str());
        return false;
    }
    */

    if (!req.trajectory_constraints.constraints.empty()) {
        ROS_WARN_NAMED(PP_LOGGER, "SBPL Dual planner does not support trajectory constraints");
        return false;
    }

    // TODO: check start state for existence of state for our robot model

    // TODO: check for existence of workspace parameters frame? Would this make
    // this call tied to an explicit planning scene?
    if (req.workspace_parameters.header.frame_id.empty()) {
        ROS_WARN_NAMED(PP_LOGGER, "SBPL Dual planner requires workspace parameters to have a valid frame");
        return false;
    }

    // check for positive workspace volume
    auto& min_corner = req.workspace_parameters.min_corner;
    auto& max_corner = req.workspace_parameters.max_corner;
    if (min_corner.x > max_corner.x ||
        min_corner.y > max_corner.y ||
        min_corner.z > max_corner.z)
    {
        std::stringstream reasons;
        if (min_corner.x > max_corner.x) {
            reasons << "negative length";
        }
        if (min_corner.y > max_corner.y) {
            reasons << (reasons.str().empty() ? "" : " ") << "negative width";
        }
        if (min_corner.z > max_corner.z) {
            reasons << (reasons.str().empty() ? "" : " ") << "negative height";
        }
        ROS_WARN_NAMED(PP_LOGGER, "SBPL Dual planner requires valid workspace (%s)", reasons.str().c_str());
        return false;
    }

    return true;
}

void SBPLDualPlannerManager::setPlannerConfigurations(
    const PlannerConfigurationMap& pcs)
{
    Base::setPlannerConfigurations(pcs);

    ROS_DEBUG_NAMED(PP_LOGGER, "Planner Configurations");
    for (auto& entry : pcs) {
        ROS_DEBUG_NAMED(PP_LOGGER, "  %s: { name: %s, group: %s }", entry.first.c_str(), entry.second.name.c_str(), entry.second.group.c_str());
        for (auto& e : entry.second.config) {
            ROS_DEBUG_NAMED(PP_LOGGER, "    %s: %s", e.first.c_str(), e.second.c_str());
        }
    }
}

} // namespace sbpl_interface

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(sbpl_interface::SBPLDualPlannerManager, planning_interface::PlannerManager);
