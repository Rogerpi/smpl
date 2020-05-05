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

#ifndef sbpl_collision_self_collision_model_h
#define sbpl_collision_self_collision_model_h

// standard includes
#include <memory>
#include <string>

// system includes
#include <smpl/forward.h>
#include <smpl/occupancy_grid.h>

// project includes
#include <sbpl_collision_checking/allowed_collisions_interface.h>
#include <sbpl_collision_checking/attached_bodies_collision_model.h>
#include <sbpl_collision_checking/attached_bodies_collision_state.h>
#include <sbpl_collision_checking/robot_collision_model.h>
#include <sbpl_collision_checking/robot_motion_collision_model.h>
#include <sbpl_collision_checking/robot_collision_state.h>
#include <sbpl_collision_checking/types.h>

namespace smpl {
namespace collision {

#define SCDL_USE_META_TREE 0

SBPL_CLASS_FORWARD(SelfCollisionModel);
class SelfCollisionModel
{
public:

    SelfCollisionModel(
        OccupancyGrid* grid,
        const RobotCollisionModel* rcm,
        const AttachedBodiesCollisionModel* ab_model);

    ~SelfCollisionModel();

    const AllowedCollisionMatrix& allowedCollisionMatrix() const;
    void updateAllowedCollisionMatrix(const AllowedCollisionMatrix& acm);
    void setAllowedCollisionMatrix(const AllowedCollisionMatrix& acm);

    void setPadding(double padding);

    void setWorldToModelTransform(const Eigen::Isometry3d& transform);

    bool checkCollision(
        const RobotCollisionState& state,
        const AttachedBodiesCollisionState& ab_state,
        const int gidx,
        double& dist);

    bool checkCollision(
        const RobotCollisionState& state,
        const AttachedBodiesCollisionState& ab_state,
        const AllowedCollisionsInterface& aci,
        const int gidx,
        double& dist);

    bool checkMotionCollision(
        RobotCollisionState& state,
        AttachedBodiesCollisionState& ab_state,
        const RobotMotionCollisionModel& rmcm,
        const std::vector<double>& start,
        const std::vector<double>& finish,
        const int gidx,
        double& dist);

    bool checkMotionCollision(
        RobotCollisionState& state,
        AttachedBodiesCollisionState& ab_state,
        const AllowedCollisionsInterface& aci,
        const RobotMotionCollisionModel& rmcm,
        const std::vector<double>& start,
        const std::vector<double>& finish,
        const int gidx,
        double& dist);

    double collisionDistance(
        const RobotCollisionState& state,
        const AttachedBodiesCollisionState& ab_state,
        const int gidx);

    double collisionDistance(
        const RobotCollisionState& state,
        const AttachedBodiesCollisionState& ab_state,
        const AllowedCollisionsInterface& aci,
        const int gidx);

    bool collisionDetails(
        const RobotCollisionState& state,
        const AttachedBodiesCollisionState& ab_state,
        const int gidx,
        CollisionDetails& details);

    bool collisionDetails(
        const RobotCollisionState& state,
        const AttachedBodiesCollisionState& ab_state,
        const AllowedCollisionsInterface& aci,
        const int gidx,
        CollisionDetails& details);

    // TODO: contacts checks
    // TODO: detailed checks

private:

    OccupancyGrid*                          m_grid;
    const RobotCollisionModel*              m_rcm;
    const AttachedBodiesCollisionModel*     m_abcm;

    RobotCollisionState                     m_rcs;
    AttachedBodiesCollisionState            m_abcs;

    // cached group information updated when a collision check for a different
    // group is made
    int                                     m_gidx;
    std::vector<int>                        m_voxels_indices;
    std::vector<int>                        m_ab_voxels_indices;

    // cached set of spheres state pairs that should be checked for self
    // collisions when using the internal allowed collision matrix
    std::vector<std::pair<int, int>>        m_checked_spheres_states;
    std::vector<std::pair<int, int>>        m_checked_attached_body_spheres_states;
    std::vector<std::pair<int, int>>        m_checked_attached_body_robot_spheres_states;

    AllowedCollisionMatrix                  m_acm;
    double                                  m_padding;

    // queue storage for sphere hierarchy traversal
    using SpherePair =
            std::pair<const CollisionSphereState*, const CollisionSphereState*>;
    std::vector<SpherePair> m_q;
    std::vector<const CollisionSphereState*>    m_vq;

    std::vector<Eigen::Vector3d> m_v_rem;
    std::vector<Eigen::Vector3d> m_v_ins;

#if SCDL_USE_META_TREE
    // cached group information for building meta trees
    typedef hash_map<const CollisionSphereModel*, const CollisionSphereState*> ModelStateMap;
    ModelStateMap                               m_model_state_map;
    std::vector<CollisionSphereModel>           m_root_models;
    std::vector<const CollisionSphereModel*>    m_root_model_pointers;
    CollisionSpheresModel                       m_meta_model;
    CollisionSpheresState                       m_meta_state;
#endif

    void initAllowedCollisionMatrix();

    bool checkCommonInputs(
        const RobotCollisionState& state,
        const AttachedBodiesCollisionState& ab_state,
        const int gidx) const;

    void prepareState(int gidx, const double* state);
    void updateGroup(int gidx);
    void copyState(const double* state);
    void updateVoxelsStates();

    // update the state of outside-group voxels and check for collisions between
    // spheres and occupied voxels
    bool checkRobotVoxelsStateCollisions(double& dist);
    bool checkAttachedBodyVoxelsStateCollisions(double& dist);

    // check for collisions between inside-group spheres
    bool checkRobotSpheresStateCollisions(double& dist);
    bool checkRobotSpheresStateCollisions(
        const AllowedCollisionsInterface& aci,
        double& dist);
    bool checkAttachedBodySpheresStateCollisions(double& dist);
    bool checkAttachedBodySpheresStateCollisions(
        const AllowedCollisionsInterface& aci,
        double& dist);
    bool checkRobotAttachedBodySpheresStateCollisions(double& dist);
    bool checkRobotAttachedBodySpheresStateCollisions(
        const AllowedCollisionsInterface& aci,
        double& dist);

    bool checkSpheresStateCollision(
        RobotCollisionState& stateA,
        RobotCollisionState& stateB,
        int ss1i,
        int ss2i,
        const CollisionSpheresState& ss1,
        const CollisionSpheresState& ss2,
        double& dist);

    template <typename StateTypeA, typename StateTypeB>
    bool checkSpheresStateCollision(
        StateTypeA& stateA,
        StateTypeB& stateB,
        const int ss1i, const int ss2i,
        const CollisionSpheresState& ss1,
        const CollisionSpheresState& ss2,
        double& dist);

    void updateCheckedSpheresIndices();
    void updateRobotCheckedSphereIndices();
    void updateRobotAttachedBodyCheckedSphereIndices();
    void updateAttachedBodyCheckedSphereIndices();

    void updateMetaSphereTrees();

    double robotVoxelsCollisionDistance();
    double robotSpheresCollisionDistance();
    double robotSpheresCollisionDistance(const AllowedCollisionsInterface& aci);

    double attachedBodyVoxelsCollisionDistance();
    double attachedBodySpheresCollisionDistance();
    double attachedBodySpheresCollisionDistance(const AllowedCollisionsInterface& aci);

    double spheresStateCollisionDistance(
        const int ss1i, const int ss2i,
        const CollisionSpheresState& ss1,
        const CollisionSpheresState& ss2);

    double sphereDistance(
        const CollisionSphereState& s1,
        const CollisionSphereState& s2) const;

    bool getRobotVoxelsStateCollisionDetails(CollisionDetails& details);
    bool getAttachedBodyVoxelsStateCollisionDetails(CollisionDetails& details);

    bool getRobotSpheresStateCollisionDetails(CollisionDetails& details);
    bool getRobotSpheresStateCollisionDetails(
        const AllowedCollisionsInterface& aci,
        CollisionDetails& details);

    bool getAttachedBodySpheresStateCollisionDetails(CollisionDetails& details);
    bool getAttachedBodySpheresStateCollisionDetails(
        const AllowedCollisionsInterface& aci,
        CollisionDetails& details);

    bool getRobotAttachedBodySpheresStateCollisionDetails(
        CollisionDetails& details);
    bool getRobotAttachedBodySpheresStateCollisionDetails(
        const AllowedCollisionsInterface& aci,
        CollisionDetails& details);
};

} // namespace collision
} // namespace smpl

#endif
