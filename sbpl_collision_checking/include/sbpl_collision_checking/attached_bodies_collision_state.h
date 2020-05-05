////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
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

/// \author Andrew Dornbush

#ifndef sbpl_collision_attached_bodies_collision_state_h
#define sbpl_collision_attached_bodies_collision_state_h

// standard includes
#include <memory>
#include <string>
#include <vector>

// system includes
#include <Eigen/Dense>
#include <leatherman/viz.h>
#include <visualization_msgs/MarkerArray.h>

// project includes
#include <sbpl_collision_checking/attached_bodies_collision_model.h>
#include <sbpl_collision_checking/base_collision_states.h>
#include <sbpl_collision_checking/robot_collision_state.h>

namespace smpl {
namespace collision {

/// Const member functions of this class are not thread-safe but multiple
/// instances may be created.
class AttachedBodiesCollisionState
{
public:

    AttachedBodiesCollisionState(
        const AttachedBodiesCollisionModel* model,
        RobotCollisionState* state);

    const AttachedBodiesCollisionModel* model() const;
    RobotCollisionState* state();

    ///\name Attached Bodies State
    ///@{
    auto attachedBodyTransform(const std::string& link_name) const
            -> const Eigen::Isometry3d&;
    auto attachedBodyTransform(int abidx) const -> const Eigen::Isometry3d&;

    bool attachedBodyTransformDirty(const std::string& id) const;
    bool attachedBodyTransformDirty(int abidx) const;

    bool updateAttachedBodyTransforms();
    bool updateAttachedBodyTransform(const std::string& id);
    bool updateAttachedBodyTransform(int abidx);
    ///@}

    /// \name Attached Bodies Collision State
    ///@{
    auto voxelsState(int vsidx) const -> const CollisionVoxelsState&;
    bool voxelsStateDirty(int vsidx) const;
    bool updateVoxelsStates();
    bool updateVoxelsState(int vsidx);

    int attachedBodySpheresStateIndex(int abidx) const;

    auto spheresState(int ssidx) const -> const CollisionSpheresState&;

    auto sphereState(const SphereIndex& sidx) const
            -> const CollisionSphereState&;
    bool sphereStateDirty(const SphereIndex& sidx) const;
    bool updateSphereStates();
    bool updateSphereStates(int ssidx);
    bool updateSphereState(const SphereIndex& sidx);

    auto groupSpheresStateIndices(const std::string& group_name) const
            -> const std::vector<int>&;
    auto groupSpheresStateIndices(int gidx) const -> const std::vector<int>&;

    auto groupOutsideVoxelsStateIndices(const std::string& group_name) const
            -> const std::vector<int>&;
    auto groupOutsideVoxelsStateIndices(int gidx) const
            -> const std::vector<int>&;
    ///@}

    auto getVisualization() const -> visualization_msgs::MarkerArray;
    auto getVisualization(const std::string& group_name) const
            -> visualization_msgs::MarkerArray;
    auto getVisualization(int gidx) const -> visualization_msgs::MarkerArray;

private:

    struct AttachedBodyState
    {
        CollisionVoxelsState* voxels_state = nullptr;
        CollisionSpheresState* spheres_state = nullptr;
    };

    const AttachedBodiesCollisionModel*     m_model;
    RobotCollisionState*                    m_state;

    std::vector<CollisionSpheresState>      m_spheres_states;
    std::vector<int>                        m_voxels_state_versions;
    std::vector<CollisionVoxelsState>       m_voxels_states;
    std::vector<CollisionGroupState>        m_group_states;

    // version number to stay in sync with parent AttachedBodiesCollisionModel
    int m_version;

    hash_map<int, AttachedBodyState> m_attached_bodies_states;

    void reinitCollisionState() const;
    void reinitCollisionState();

    int attachedBodyTransformVersion(int abidx) const;
};

typedef std::shared_ptr<AttachedBodiesCollisionState> AttachedBodiesCollisionStatePtr;
typedef std::shared_ptr<const AttachedBodiesCollisionState> AttachedBodiesCollisionStateConstPtr;

inline
const AttachedBodiesCollisionModel* AttachedBodiesCollisionState::model() const
{
    return m_model;
}

inline
RobotCollisionState* AttachedBodiesCollisionState::state()
{
    return m_state;
}

inline
const Eigen::Isometry3d& AttachedBodiesCollisionState::attachedBodyTransform(
    const std::string& id) const
{
    reinitCollisionState();
    const int abidx = m_model->attachedBodyIndex(id);
    const int lidx = m_model->attachedBodyLinkIndex(abidx);
    return m_state->linkTransform(lidx);
}

inline
const Eigen::Isometry3d& AttachedBodiesCollisionState::attachedBodyTransform(
    int abidx) const
{
    reinitCollisionState();
    const int lidx = m_model->attachedBodyLinkIndex(abidx);
    return m_state->linkTransform(lidx);
}

inline
bool AttachedBodiesCollisionState::attachedBodyTransformDirty(
    const std::string& id) const
{
    reinitCollisionState();
    const int abidx = m_model->attachedBodyIndex(id);
    const int lidx = m_model->attachedBodyLinkIndex(abidx);
    return m_state->linkTransformDirty(lidx);
}

inline
bool AttachedBodiesCollisionState::attachedBodyTransformDirty(
    int abidx) const
{
    reinitCollisionState();
    const int lidx = m_model->attachedBodyLinkIndex(abidx);
    return m_state->linkTransformDirty(lidx);
}

inline
bool AttachedBodiesCollisionState::updateAttachedBodyTransforms()
{
    reinitCollisionState();
    return m_state->updateLinkTransforms();
}

inline
bool AttachedBodiesCollisionState::updateAttachedBodyTransform(
    const std::string& id)
{
    reinitCollisionState();
    const int abidx = m_model->attachedBodyIndex(id);
    const int lidx = m_model->attachedBodyLinkIndex(abidx);
    return m_state->updateLinkTransform(lidx);
}

inline
bool AttachedBodiesCollisionState::updateAttachedBodyTransform(int abidx)
{
    reinitCollisionState();
    const int lidx = m_model->attachedBodyLinkIndex(abidx);
    return m_state->updateLinkTransform(lidx);
}

inline
const CollisionVoxelsState& AttachedBodiesCollisionState::voxelsState(
    int vsidx) const
{
    reinitCollisionState();
    ASSERT_VECTOR_RANGE(m_voxels_states, vsidx);
    return m_voxels_states[vsidx];
}

inline
bool AttachedBodiesCollisionState::voxelsStateDirty(int vsidx) const
{
    reinitCollisionState();
    ASSERT_VECTOR_RANGE(m_voxels_states, vsidx);
    int abidx = m_voxels_states[vsidx].model->link_index;
    int body_version = attachedBodyTransformVersion(abidx);
    return attachedBodyTransformDirty(abidx) || m_voxels_state_versions[vsidx] != body_version;
}

inline
bool AttachedBodiesCollisionState::updateVoxelsStates()
{
    reinitCollisionState();
    bool updated = false;
    for (size_t vsidx = 0; vsidx < m_voxels_states.size(); ++vsidx) {
        updated |= updateVoxelsState(vsidx);
    }
    return updated;
}

inline
int AttachedBodiesCollisionState::attachedBodySpheresStateIndex(int abidx) const
{
    reinitCollisionState();
    const CollisionSpheresState* ss =
            m_attached_bodies_states.at(abidx).spheres_state;
    if (ss) {
        return std::distance(m_spheres_states.data(), ss);
    } else {
        return -1;
    }
}

inline
const CollisionSpheresState& AttachedBodiesCollisionState::spheresState(
    int ssidx) const
{
    reinitCollisionState();
    ASSERT_VECTOR_RANGE(m_spheres_states, ssidx);
    return m_spheres_states[ssidx];
}

inline
const CollisionSphereState& AttachedBodiesCollisionState::sphereState(
    const SphereIndex& sidx) const
{
    reinitCollisionState();
    ASSERT_VECTOR_RANGE(m_spheres_states, sidx.ss);
    ASSERT_VECTOR_RANGE(m_spheres_states[sidx.ss].spheres, sidx.s);
    return m_spheres_states[sidx.ss].spheres[sidx.s];
}

inline
bool AttachedBodiesCollisionState::sphereStateDirty(
    const SphereIndex& sidx) const
{
    reinitCollisionState();
    ASSERT_VECTOR_RANGE(m_spheres_states, sidx.ss);
    const CollisionSpheresState& spheres_state = m_spheres_states[sidx.ss];
    const int bidx = spheres_state.model->link_index;
    int body_version = attachedBodyTransformVersion(bidx);
    return attachedBodyTransformDirty(bidx) || spheres_state.spheres[sidx.s].version != body_version;
}

inline
bool AttachedBodiesCollisionState::updateSphereStates()
{
    reinitCollisionState();
    bool updated = false;
    for (size_t ssidx = 0; ssidx < m_spheres_states.size(); ++ssidx) {
        updated |= updateSphereStates(ssidx);
    }
    return updated;
}

inline
bool AttachedBodiesCollisionState::updateSphereStates(int ssidx)
{
    reinitCollisionState();
    bool updated = false;
    const CollisionSpheresState& spheres_state = m_spheres_states[ssidx];
    for (size_t sidx = 0; sidx < spheres_state.spheres.size(); ++sidx) {
        updated |= updateSphereState(SphereIndex(ssidx, sidx));
    }
    return updated;
}

inline
const std::vector<int>&
AttachedBodiesCollisionState::groupSpheresStateIndices(
    const std::string& group_name) const
{
    reinitCollisionState();
    const int gidx = m_model->groupIndex(group_name);
    return m_group_states[gidx].spheres_indices;
}

inline
const std::vector<int>&
AttachedBodiesCollisionState::groupSpheresStateIndices(int gidx) const
{
    reinitCollisionState();
    ASSERT_VECTOR_RANGE(m_group_states, gidx);
    return m_group_states[gidx].spheres_indices;
}

inline
const std::vector<int>&
AttachedBodiesCollisionState::groupOutsideVoxelsStateIndices(
    const std::string& group_name) const
{
    reinitCollisionState();
    const int gidx = m_model->groupIndex(group_name);
    return m_group_states[gidx].voxels_indices;
}

inline
const std::vector<int>&
AttachedBodiesCollisionState::groupOutsideVoxelsStateIndices(int gidx) const
{
    reinitCollisionState();
    ASSERT_VECTOR_RANGE(m_group_states, gidx);
    return m_group_states[gidx].voxels_indices;
}

inline
visualization_msgs::MarkerArray
AttachedBodiesCollisionState::getVisualization() const
{
    reinitCollisionState();
    return visualization_msgs::MarkerArray();
}

inline
visualization_msgs::MarkerArray
AttachedBodiesCollisionState::getVisualization(
    const std::string& group_name) const
{
    reinitCollisionState();
    return visualization_msgs::MarkerArray();
}

inline
visualization_msgs::MarkerArray
AttachedBodiesCollisionState::getVisualization(int gidx) const
{
    reinitCollisionState();
    const CollisionGroupState& group_state = m_group_states[gidx];

    std::vector<std::vector<double>> spheres;
    std::vector<double> rad;

    size_t sphere_count = 0;
    for (int ssidx : group_state.spheres_indices) {
        const CollisionSpheresState& spheres_state = m_spheres_states[ssidx];
        sphere_count += spheres_state.spheres.size();
    }

    spheres.reserve(sphere_count);
    rad.reserve(sphere_count);

    for (int ssidx : group_state.spheres_indices) {
        const CollisionSpheresState& spheres_state = m_spheres_states[ssidx];
        for (const CollisionSphereState& sphere_state : spheres_state.spheres) {
            if (!sphere_state.isLeaf()) {
                continue;
            }

            std::vector<double> sphere(4, 0.0);
            sphere[0] = sphere_state.pos.x();
            sphere[1] = sphere_state.pos.y();
            sphere[2] = sphere_state.pos.z();
            sphere[3] = sphere_state.model->radius;
            spheres.push_back(std::move(sphere));
            rad.push_back(sphere_state.model->radius);
        }
    }

    const int hue = 90;
    return ::viz::getSpheresMarkerArray(spheres, rad, hue, "", "attached_bodies_model", 0);
}

inline
void AttachedBodiesCollisionState::reinitCollisionState() const
{
    AttachedBodiesCollisionState* mm =
            const_cast<AttachedBodiesCollisionState*>(this);
    return mm->reinitCollisionState();
}

inline
int AttachedBodiesCollisionState::attachedBodyTransformVersion(int abidx) const
{
    int lidx = m_model->attachedBodyLinkIndex(abidx);
    return m_state->linkTransformVersion(lidx);
}

} // namespace collision
} // namespace smpl

#endif
