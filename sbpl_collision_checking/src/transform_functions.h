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

#ifndef sbpl_collision_transform_functions_h
#define sbpl_collision_transform_functions_h

#include <Eigen/Dense>

#define SBPL_COLLISION_SPECIALIZED_JOINT_TRANSFORMS 1

namespace smpl {
namespace collision {

////////////////////////////////////////
// Compute*JointTransform Definitions //
////////////////////////////////////////

Eigen::Isometry3d ComputeRevoluteJointTransform(
    const Eigen::Isometry3d& origin,
    const Eigen::Vector3d& axis,
    double* jvals);

Eigen::Isometry3d ComputeRevoluteJointTransformX(
    const Eigen::Isometry3d& origin,
    const Eigen::Vector3d& axis,
    double* jvals);

Eigen::Isometry3d ComputeRevoluteJointTransformY(
    const Eigen::Isometry3d& origin,
    const Eigen::Vector3d& axis,
    double* jvals);

Eigen::Isometry3d ComputeRevoluteJointTransformZ(
    const Eigen::Isometry3d& origin,
    const Eigen::Vector3d& axis,
    double* jvals);

Eigen::Isometry3d ComputeContinuousJointTransform(
    const Eigen::Isometry3d& origin,
    const Eigen::Vector3d& axis,
    double* jvals);

Eigen::Isometry3d ComputePrismaticJointTransform(
    const Eigen::Isometry3d& origin,
    const Eigen::Vector3d& axis,
    double* jvals);

Eigen::Isometry3d ComputeFloatingJointTransform(
    const Eigen::Isometry3d& origin,
    const Eigen::Vector3d& axis,
    double* jvals);

Eigen::Isometry3d ComputePlanarJointTransform(
    const Eigen::Isometry3d& origin,
    const Eigen::Vector3d& axis,
    double* jvals);

Eigen::Isometry3d ComputeFixedJointTransform(
    const Eigen::Isometry3d& origin,
    const Eigen::Vector3d& axis,
    double* jvals);

////////////////////////////////////////
// Compute*JointTransform Definitions //
////////////////////////////////////////

inline
Eigen::Isometry3d ComputeRevoluteJointTransform(
    const Eigen::Isometry3d& o,
    const Eigen::Vector3d& axis,
    double* jvals)
{
    return o * Eigen::AngleAxisd(jvals[0], axis);
}

inline
Eigen::Isometry3d ComputeRevoluteJointTransformX(
    const Eigen::Isometry3d& o,
    const Eigen::Vector3d& axis,
    double* jvals)
{
#if SBPL_COLLISION_SPECIALIZED_JOINT_TRANSFORMS
    Eigen::Transform<double, 3, Eigen::Isometry> t;
    double cth = cos(jvals[0]);
    double sth = sin(jvals[0]);
    t(0,0) = o(0,0);
    t(1,0) = o(1,0);
    t(2,0) = o(2,0);
    t(3,0) = 0.0;

    t(0,1) = cth * o(0,1) + sth * o(0,2);
    t(1,1) = cth * o(1,1) + sth * o(1,2);
    t(2,1) = cth * o(2,1) + sth * o(2,2);
    t(3,1) = 0.0;

    t(0,2) = cth * o(0,2) - sth * o(0,1);
    t(1,2) = cth * o(1,2) - sth * o(1,1);
    t(2,2) = cth * o(2,2) - sth * o(2,1);
    t(3,0) = 0.0;

    t(0,3) = o(0,3);
    t(1,3) = o(1,3);
    t(2,3) = o(2,3);
    t(3,3) = 1.0;
    return t;
#else
    return ComputeRevoluteJointTransform(o, axis, jvals);
#endif
}

inline
Eigen::Isometry3d ComputeRevoluteJointTransformY(
    const Eigen::Isometry3d& o,
    const Eigen::Vector3d& axis,
    double* jvals)
{
#if SBPL_COLLISION_SPECIALIZED_JOINT_TRANSFORMS
    Eigen::Transform<double, 3, Eigen::Isometry> t;
    double cth = cos(jvals[0]);
    double sth = sin(jvals[0]);
    t(0,0) = cth * o(0,0) - sth * o(0,2);
    t(1,0) = cth * o(1,0) - sth * o(1,2);
    t(2,0) = cth * o(2,0) - sth * o(2,2);
    t(3,0) = 0.0;

    t(0,1) = o(0,1);
    t(1,1) = o(1,1);
    t(2,1) = o(2,1);
    t(3,1) = 0.0;

    t(0,2) = sth * o(0,0) + cth * o(0,2);
    t(1,2) = sth * o(1,0) + cth * o(1,2);
    t(2,2) = sth * o(2,0) + cth * o(2,2);
    t(3,0) = 0.0;

    t(0,3) = o(0,3);
    t(1,3) = o(1,3);
    t(2,3) = o(2,3);
    t(3,3) = 1.0;
    return t;
#else
    return ComputeRevoluteJointTransform(o, axis, jvals);
#endif
}

inline
Eigen::Isometry3d ComputeRevoluteJointTransformZ(
    const Eigen::Isometry3d& o,
    const Eigen::Vector3d& axis,
    double* jvals)
{
#if SBPL_COLLISION_SPECIALIZED_JOINT_TRANSFORMS
    Eigen::Transform<double, 3, Eigen::Isometry> t;
    double cth = cos(jvals[0]);
    double sth = sin(jvals[0]);
    t(0,0) = o(0, 0) * cth + o(0, 1) * sth;
    t(1,0) = o(1, 0) * cth + o(1, 1) * sth;
    t(2,0) = o(2, 0) * cth + o(2, 1) * sth;
    t(3,0) = 0.0;

    t(0,1) = o(0, 1) * cth - o(0, 0) * sth;
    t(1,1) = o(1, 1) * cth - o(1, 0) * sth;
    t(2,1) = o(2, 1) * cth - o(2, 0) * sth;
    t(3,1) = 0.0;

    t(0,2) = o(0, 2);
    t(1,2) = o(1, 2);
    t(2,2) = o(2, 2);
    t(3,0) = 0.0;

    t(0,3) = o(0, 3);
    t(1,3) = o(1, 3);
    t(2,3) = o(2, 3);
    t(3,3) = 1.0;
    return t;
#else
    return ComputeRevoluteJointTransform(o, axis, jvals);
#endif
}

inline
Eigen::Isometry3d ComputeContinuousJointTransform(
    const Eigen::Isometry3d& o,
    const Eigen::Vector3d& axis,
    double* jvals)
{
    return o * Eigen::AngleAxisd(jvals[0], axis);
}

inline
Eigen::Isometry3d ComputePrismaticJointTransform(
    const Eigen::Isometry3d& o,
    const Eigen::Vector3d& axis,
    double* jvals)
{
    return o * Eigen::Translation3d(jvals[0] * axis);
}

inline
Eigen::Isometry3d ComputePrismaticJointTransformX(
    const Eigen::Isometry3d& o,
    const Eigen::Vector3d& axis,
    double* jvals)
{
    return o * Eigen::Translation3d(jvals[0], 0.0, 0.0);
}

inline
Eigen::Isometry3d ComputePrismaticJointTransformY(
    const Eigen::Isometry3d& o,
    const Eigen::Vector3d& axis,
    double* jvals)
{
    return o * Eigen::Translation3d(0.0, jvals[0], 0.0);
}

inline
Eigen::Isometry3d ComputePrismaticJointTransformZ(
    const Eigen::Isometry3d& o,
    const Eigen::Vector3d& axis,
    double* jvals)
{
    return o * Eigen::Translation3d(0.0, 0.0, jvals[0]);
}

inline
Eigen::Isometry3d ComputeFloatingJointTransform(
    const Eigen::Isometry3d& o,
    const Eigen::Vector3d& axis,
    double* jvals)
{
    // TODO: be mindful that quaternion values here may not be normalized
    return o *
            Eigen::Translation3d(Eigen::Vector3d(jvals[0], jvals[1], jvals[2])) *
            Eigen::Quaterniond(jvals[6], jvals[3], jvals[4], jvals[5]);
}

inline
Eigen::Isometry3d ComputePlanarJointTransform(
    const Eigen::Isometry3d& o,
    const Eigen::Vector3d& axis,
    double* jvals)
{
    return o *
            Eigen::Translation3d(Eigen::Vector3d(jvals[0], jvals[1], 0.0)) *
            Eigen::AngleAxisd(jvals[2], Eigen::Vector3d::UnitZ());
}

inline
Eigen::Isometry3d ComputeFixedJointTransform(
    const Eigen::Isometry3d& o,
    const Eigen::Vector3d& axis,
    double* jvals)
{
    return o;
}

} // namespace collision
} // namespace smpl

#endif
