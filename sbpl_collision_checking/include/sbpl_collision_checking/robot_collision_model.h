////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2011, Benjamin Cohen, Andrew Dornbush
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Benjamin Cohen
/// \author Andrew Dornbush

#ifndef sbpl_collision_robot_collision_model_h
#define sbpl_collision_robot_collision_model_h

// standard includes
#include <memory>
#include <ostream>
#include <string>
#include <vector>

// system includes
#include <Eigen/Dense>
#include <geometric_shapes/shapes.h>
#include <urdf_model/model.h>

// project includes
#include <sbpl_collision_checking/base_collision_models.h>
#include <sbpl_collision_checking/collision_model_config.h>
#include <sbpl_collision_checking/debug.h>
#include <sbpl_collision_checking/shapes.h>
#include <sbpl_collision_checking/types.h>

namespace smpl {
namespace collision {

typedef Eigen::Isometry3d (*JointTransformFunction)(
    const Eigen::Isometry3d& origin,
    const Eigen::Vector3d& axis,
    double* jvals);

class RobotCollisionModelImpl;

class RobotCollisionModel;
typedef std::shared_ptr<RobotCollisionModel> RobotCollisionModelPtr;
typedef std::shared_ptr<const RobotCollisionModel> RobotCollisionModelConstPtr;

enum JointType
{
    FIXED,
    REVOLUTE,
    PRISMATIC,
    CONTINUOUS,
    PLANAR,
    FLOATING
};

/// \brief Represents the collision model of the robot used for planning.
class RobotCollisionModel
{
public:

    static auto Load(
        const ::urdf::ModelInterface& urdf,
        const CollisionModelConfig& config)
        -> RobotCollisionModelPtr;

    RobotCollisionModel() = default;

    bool init(
        const ::urdf::ModelInterface& urdf,
        const CollisionModelConfig& config);

    ~RobotCollisionModel();

    /// \name Robot Model - General Information
    ///@{
    auto   name() const -> const std::string&;
    auto   modelFrame() const -> const std::string&;
    ///@}

    /// \name Robot Model - Variable Information
    ///@{
    size_t jointVarCount() const;
    auto   jointVarNames() const -> const std::vector<std::string>&;

    bool   hasJointVar(const std::string& joint_name) const;
    int    jointVarIndex(const std::string& joint_name) const;
    auto   jointVarName(int jidx) const -> const std::string&;

    bool   jointVarIsContinuous(const std::string& joint_name) const;
    bool   jointVarHasPositionBounds(const std::string& joint_name) const;
    double jointVarMinPosition(const std::string& joint_name) const;
    double jointVarMaxPosition(const std::string& joint_name) const;

    bool   jointVarIsContinuous(int jidx) const;
    bool   jointVarHasPositionBounds(int jidx) const;
    double jointVarMinPosition(int jidx) const;
    double jointVarMaxPosition(int jidx) const;

    int    jointVarJointIndex(int vidx) const;
    ///@}

    /// \name Robot Model - Joint Information
    ///@{
    size_t jointCount() const;
    auto   jointName(int jidx) const -> const std::string&;

    int    jointParentLinkIndex(int jidx) const;
    int    jointChildLinkIndex(int jidx) const;

    auto   jointOrigin(int jidx) const -> const Eigen::Isometry3d&;
    auto   jointAxis(int jidx) const -> const Eigen::Vector3d&;
    auto   jointTransformFn(int jidx) const -> JointTransformFunction;

    int    jointVarIndexFirst(int jidx) const;
    int    jointVarIndexLast(int jidx) const;

    auto   jointType(int jidx) const -> JointType;

    bool   isDescendantJoint(int jidx, int pjidx) const;
    ///@}

    /// \name Robot Model - Link Information
    ///@{
    size_t linkCount() const;
    auto   linkNames() const -> const std::vector<std::string>&;

    bool   hasLink(const std::string& link_name) const;
    int    linkIndex(const std::string& link_name) const;
    auto   linkName(int lidx) const -> const std::string&;

    int    linkParentJointIndex(int lidx) const;
    auto   linkChildJointIndices(int lidx) const -> const std::vector<int>&;
    ///@}

    /// \name Collision Model
    ///@{
    size_t sphereModelCount() const;

    bool   hasSpheresModel(const std::string& link_name) const;
    bool   hasSpheresModel(int lidx) const;
    size_t spheresModelCount() const;
    auto   spheresModel(int smidx) const -> const CollisionSpheresModel&;

    bool   hasVoxelsModel(const std::string& link_name) const;
    bool   hasVoxelsModel(int lidx) const;
    size_t voxelsModelCount() const;
    auto   voxelsModel(int vmidx) const -> const CollisionVoxelsModel&;

    int    linkSpheresModelIndex(int lidx) const;

    size_t groupCount() const;
    auto   group(int gidx) const -> const CollisionGroupModel&;
    bool   hasGroup(const std::string& group_name) const;
    int    groupIndex(const std::string& group_name) const;
    auto   groupName(int gidx) const -> const std::string&;

    auto   groupLinkIndices(const std::string& group_name) const ->
            const std::vector<int>&;
    auto   groupLinkIndices(int gidx) const -> const std::vector<int>&;

    double maxLeafSphereRadius() const;
    double maxSphereRadius() const;
    ///@}

private:

    // cached after initialization to allow regeneration of references
    CollisionModelConfig m_config;

    /// \name Robot Model
    ///@{
    std::string                             m_name;
    std::string                             m_model_frame;

    std::vector<std::string>                m_jvar_names;
    std::vector<bool>                       m_jvar_continuous;
    std::vector<bool>                       m_jvar_has_position_bounds;
    std::vector<double>                     m_jvar_min_positions;
    std::vector<double>                     m_jvar_max_positions;
    hash_map<std::string, int>              m_jvar_name_to_index;
    std::vector<int>                        m_jvar_joint_indices;

    std::vector<std::string>                m_joint_names;
    Isometry3dVector                        m_joint_origins;
    std::vector<Eigen::Vector3d>            m_joint_axes;
    std::vector<std::pair<int, int>>        m_joint_var_indices;
    std::vector<JointTransformFunction>     m_joint_transforms;
    std::vector<JointType>                  m_joint_types;
    std::vector<int>                        m_joint_parent_links;
    std::vector<int>                        m_joint_child_links;

    std::vector<bool>                       m_desc_joint_matrix;

    std::vector<std::string>                m_link_names;
    std::vector<int>                        m_link_parent_joints;
    std::vector<std::vector<int>>           m_link_children_joints;
    hash_map<std::string, int>              m_link_name_to_index;
    ///@}

    /// \name Collision Model
    ///@{

    // storage for collision shapes found in the urdf model
    std::vector<std::vector<double>>            m_vertex_buffers;
    std::vector<std::vector<std::uint32_t>>     m_index_buffers;
    std::vector<BoxShape> m_box_shapes;
    std::vector<CylinderShape> m_cylinder_shapes;
    std::vector<SphereShape> m_sphere_shapes;
    std::vector<MeshShape> m_mesh_shapes;

    // map from link to collision shapes; links appear in the order they
    // are specified in the urdf
    std::vector<LinkCollisionGeometry> m_link_geometries;

    // one entry for each link that has a spheres model
    std::vector<CollisionSpheresModel>  m_spheres_models;

    // one entry for each link that has a voxels model
    std::vector<CollisionVoxelsModel>   m_voxels_models;

    // one entry for each group specified through config
    std::vector<CollisionGroupModel>    m_group_models;
    hash_map<std::string, int>          m_group_name_to_index;

    // per-link references to corresponding spheres and voxels models
    std::vector<const CollisionSpheresModel*> m_link_spheres_models;
    std::vector<const CollisionVoxelsModel*>  m_link_voxels_models;

    ///@}

// TODO:
//    bool hasSpheresModelOverride(const std::string& link_name, const std::string& group_name) const;
//    bool hasSpheresModelOverride(const std::string& link_name, int gidx) const;
//    bool hasSpheresModelOverride(int lidx, const std::string& group_name) const;
//    bool hasSpheresModelOverride(int lidx, int gidx) const;

    // this function will take care of appending joint information independent
    // of the type of joint including, name, origin, axis, and offsets into
    // joint variable array
    void addJoint(const ::urdf::Joint& joint);

    // each of these functions is responsible for appending its corresponding
    // per-variable names, flags, and bounds, the type of the joint, and the
    // function used for computing joint transforms
    void addFixedJoint(const ::urdf::Joint& joint);
    void addRevoluteJoint(const ::urdf::Joint& joint);
    void addPrismaticJoint(const ::urdf::Joint& joint);
    void addContinuousJoint(const ::urdf::Joint& joint);
    void addPlanarJoint(const ::urdf::Joint& joint);
    void addFloatingJoint(const ::urdf::Joint& joint);

    bool initRobotModel(
        const ::urdf::ModelInterface& urdf,
        const WorldJointConfig& config);

    bool initCollisionModel(
        const ::urdf::ModelInterface& urdf,
        const CollisionModelConfig& config);

    bool initCollisionShapes(const ::urdf::ModelInterface& urdf);
    bool createCollisionShape(const ::urdf::Collision& collision);

    bool expandGroups(
        const std::vector<CollisionGroupConfig>& groups,
        std::vector<CollisionGroupConfig>& expanded_groups) const;

    bool generateSphereModels(
        int urdf_link_index,
        double radius,
        std::vector<CollisionSphereModel>& spheres) const;

    bool generateBoundingSpheres(
        const CollisionGeometry* geom,
        double radius,
        std::vector<CollisionSphereModel>& spheres) const;

    bool checkCollisionModelConfig(const CollisionModelConfig& config);

    bool checkCollisionModelReferences() const;

    Eigen::Isometry3d poseUrdfToEigen(const ::urdf::Pose& p) const;

    bool voxelizeLink(
        const ::urdf::ModelInterface& urdf,
        const std::string& link_name,
        CollisionVoxelsModel& model) const;

    bool voxelizeCollisionElement(
        const ::urdf::Collision& collision,
        double res,
        std::vector<Eigen::Vector3d>& voxels) const;

    bool voxelizeGeometry(
        const ::urdf::Geometry& geom,
        const Eigen::Isometry3d& pose,
        double res,
        std::vector<Eigen::Vector3d>& voxels) const;
};

auto LoadRobotCollisionModel(
    const ::urdf::ModelInterface& urdf,
    const CollisionModelConfig& config)
    -> std::unique_ptr<RobotCollisionModel>;

inline
const std::string& RobotCollisionModel::name() const
{
    return m_name;
}

inline
const std::string& RobotCollisionModel::modelFrame() const
{
    return m_model_frame;
}

inline
size_t RobotCollisionModel::jointVarCount() const
{
    return m_jvar_names.size();
}

inline
const std::vector<std::string>& RobotCollisionModel::jointVarNames() const
{
    return m_jvar_names;
}

inline
bool RobotCollisionModel::hasJointVar(const std::string& joint_name) const
{
    return m_jvar_name_to_index.find(joint_name) != m_jvar_name_to_index.end();
}

inline
int RobotCollisionModel::jointVarIndex(const std::string& joint_name) const
{
    auto it = m_jvar_name_to_index.find(joint_name);
    ASSERT_RANGE(it != m_jvar_name_to_index.end());
    assert(it->second >= 0 && it->second < m_jvar_names.size());
    return it->second;
}

inline
const std::string& RobotCollisionModel::jointVarName(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_jvar_names, jidx);
    return m_jvar_names[jidx];
}

inline
bool RobotCollisionModel::jointVarIsContinuous(
    const std::string& joint_name) const
{
    const int jidx = jointVarIndex(joint_name);
    return m_jvar_continuous[jidx];
}

inline
bool RobotCollisionModel::jointVarHasPositionBounds(
    const std::string& joint_name) const
{
    const int jidx = jointVarIndex(joint_name);
    return m_jvar_has_position_bounds[jidx];
}

inline
double RobotCollisionModel::jointVarMinPosition(
    const std::string& joint_name) const
{
    const int jidx = jointVarIndex(joint_name);
    return m_jvar_min_positions[jidx];
}

inline
double RobotCollisionModel::jointVarMaxPosition(
    const std::string& joint_name) const
{
    const int jidx = jointVarIndex(joint_name);
    return m_jvar_max_positions[jidx];
}

inline
int RobotCollisionModel::jointVarJointIndex(int vidx) const
{
    return m_jvar_joint_indices[vidx];
}

inline
bool RobotCollisionModel::jointVarIsContinuous(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_jvar_continuous, jidx);
    return m_jvar_continuous[jidx];
}

inline
bool RobotCollisionModel::jointVarHasPositionBounds(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_jvar_has_position_bounds, jidx);
    return m_jvar_has_position_bounds[jidx];
}

inline
double RobotCollisionModel::jointVarMinPosition(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_jvar_min_positions, jidx);
    return m_jvar_min_positions[jidx];
}

inline
double RobotCollisionModel::jointVarMaxPosition(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_jvar_max_positions, jidx);
    return m_jvar_max_positions[jidx];
}

inline
size_t RobotCollisionModel::jointCount() const
{
    return m_joint_transforms.size();
}

inline
const std::string& RobotCollisionModel::jointName(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_joint_names, jidx);
    return m_joint_names[jidx];
}

inline
int RobotCollisionModel::jointParentLinkIndex(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_joint_parent_links, jidx);
    return m_joint_parent_links[jidx];
}

inline
int RobotCollisionModel::jointChildLinkIndex(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_joint_child_links, jidx);
    return m_joint_child_links[jidx];
}

inline
const Eigen::Isometry3d& RobotCollisionModel::jointOrigin(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_joint_origins, jidx);
    return m_joint_origins[jidx];
}

inline
const Eigen::Vector3d& RobotCollisionModel::jointAxis(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_joint_axes, jidx);
    return m_joint_axes[jidx];
}

inline
JointTransformFunction RobotCollisionModel::jointTransformFn(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_joint_transforms, jidx);
    return m_joint_transforms[jidx];
}

inline
int RobotCollisionModel::jointVarIndexFirst(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_joint_var_indices, jidx);
    return m_joint_var_indices[jidx].first;
}

inline
int RobotCollisionModel::jointVarIndexLast(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_joint_var_indices, jidx);
    return m_joint_var_indices[jidx].second;
}

inline
JointType RobotCollisionModel::jointType(int jidx) const
{
    ASSERT_VECTOR_RANGE(m_joint_types, jidx);
    return m_joint_types[jidx];
}

inline
bool RobotCollisionModel::isDescendantJoint(int jidx, int pjidx) const
{
    return m_desc_joint_matrix[jidx * jointCount() + pjidx];
}

inline
size_t RobotCollisionModel::linkCount() const
{
    return m_link_names.size();
}

inline
const std::vector<std::string>& RobotCollisionModel::linkNames() const
{
    return m_link_names;
}

inline
bool RobotCollisionModel::hasLink(const std::string& link_name) const
{
    return m_link_name_to_index.find(link_name) != m_link_name_to_index.end();
}

inline
int RobotCollisionModel::linkIndex(const std::string& link_name) const
{
    auto it = m_link_name_to_index.find(link_name);
    ASSERT_RANGE(it != m_link_name_to_index.end());
    assert(it->second >= 0 && it->second < m_link_names.size());
    return it->second;
}

inline
const std::string& RobotCollisionModel::linkName(int lidx) const
{
    ASSERT_VECTOR_RANGE(m_link_names, lidx);
    return m_link_names[lidx];
}

inline
int RobotCollisionModel::linkParentJointIndex(int lidx) const
{
    ASSERT_VECTOR_RANGE(m_link_parent_joints, lidx);
    return m_link_parent_joints[lidx];
}

inline
const std::vector<int>& RobotCollisionModel::linkChildJointIndices(
    int lidx) const
{
    ASSERT_VECTOR_RANGE(m_link_children_joints, lidx);
    return m_link_children_joints[lidx];
}

inline
size_t RobotCollisionModel::sphereModelCount() const
{
    size_t sphere_count = 0;
    for (const auto& spheres_model : m_spheres_models) {
        sphere_count += spheres_model.spheres.size();
    }
    return sphere_count;
}

inline
bool RobotCollisionModel::hasSpheresModel(const std::string& link_name) const
{
    const int lidx = linkIndex(link_name);
    return m_link_spheres_models[lidx] != nullptr;
}

inline
bool RobotCollisionModel::hasSpheresModel(int lidx) const
{
    ASSERT_VECTOR_RANGE(m_link_spheres_models, lidx);
    return m_link_spheres_models[lidx] != nullptr;
}

inline
size_t RobotCollisionModel::spheresModelCount() const
{
    return m_spheres_models.size();
}

inline
const CollisionSpheresModel& RobotCollisionModel::spheresModel(
    int smidx) const
{
    ASSERT_VECTOR_RANGE(m_spheres_models, smidx);
    return m_spheres_models[smidx];
}

inline
bool RobotCollisionModel::hasVoxelsModel(const std::string& link_name) const
{
    const int lidx = linkIndex(link_name);
    return m_link_voxels_models[lidx] != nullptr;
}

inline
bool RobotCollisionModel::hasVoxelsModel(int lidx) const
{
    ASSERT_VECTOR_RANGE(m_link_voxels_models, lidx);
    return m_link_voxels_models[lidx];
}

inline
size_t RobotCollisionModel::voxelsModelCount() const
{
    return m_voxels_models.size();
}

inline
const CollisionVoxelsModel& RobotCollisionModel::voxelsModel(int vmidx) const
{
    ASSERT_RANGE(vmidx >= 0 && vmidx < m_voxels_models.size());
    return m_voxels_models[vmidx];
}

inline
int RobotCollisionModel::linkSpheresModelIndex(int lidx) const
{
    ASSERT_VECTOR_RANGE(m_link_spheres_models, lidx);
    const CollisionSpheresModel* ss = m_link_spheres_models[lidx];
    if (ss) {
        return std::distance(m_spheres_models.data(), ss);
    } else {
        return -1;
    }
}
inline
size_t RobotCollisionModel::groupCount() const
{
    return m_group_models.size();
}

inline
const CollisionGroupModel& RobotCollisionModel::group(int gidx) const
{
    ASSERT_VECTOR_RANGE(m_group_models, gidx);
    return m_group_models[gidx];
}

inline
bool RobotCollisionModel::hasGroup(const std::string& group_name) const
{
    return m_group_name_to_index.find(group_name) !=
            m_group_name_to_index.end();
}

inline
int RobotCollisionModel::groupIndex(const std::string& group_name) const
{
    auto it = m_group_name_to_index.find(group_name);
    ASSERT_RANGE(it != m_group_name_to_index.end());
    assert(it->second >= 0 && it->second < m_group_models.size());
    return it->second;
}

inline
const std::string& RobotCollisionModel::groupName(int gidx) const
{
    ASSERT_VECTOR_RANGE(m_group_models, gidx);
    return m_group_models[gidx].name;
}

inline
const std::vector<int>& RobotCollisionModel::groupLinkIndices(
    const std::string& group_name) const
{
    const int gidx = groupIndex(group_name);
    return m_group_models[gidx].link_indices;
}

inline
const std::vector<int>& RobotCollisionModel::groupLinkIndices(int gidx) const
{
    ASSERT_VECTOR_RANGE(m_group_models, gidx);
    return m_group_models[gidx].link_indices;
}

inline
double RobotCollisionModel::maxSphereRadius() const
{
    double d = 0.0;
    for (const auto& spheres : m_spheres_models) {
        d = std::max(d, spheres.spheres.maxRadius());
    }
    return d;
}

inline
double RobotCollisionModel::maxLeafSphereRadius() const
{
    double d = 0.0;
    for (const auto& spheres : m_spheres_models) {
        d = std::max(d, spheres.spheres.maxLeafRadius());
    }
    return d;
}

} // namespace collision
} // namespace smpl

#endif
