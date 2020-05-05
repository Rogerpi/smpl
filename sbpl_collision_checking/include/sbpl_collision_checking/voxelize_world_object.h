#ifndef SBPL_COLLISION_CHECKING_VOXELIZE_WORLD_OBJECT_H
#define SBPL_COLLISION_CHECKING_VOXELIZE_WORLD_OBJECT_H

#include <vector>

#include <Eigen/Dense>
#include <geometric_shapes/shapes.h>
#include <moveit/collision_detection/world.h>

namespace smpl {
namespace collision {

bool VoxelizeObject(
    const collision_detection::World::Object& object,
    double res,
    const Eigen::Vector3d& go,
    std::vector<std::vector<Eigen::Vector3d>>& all_voxels);

bool VoxelizeObject(
    const collision_detection::World::Object& object,
    double res,
    const Eigen::Vector3d& go,
    const Eigen::Vector3d& gmin,
    const Eigen::Vector3d& gmax,
    std::vector<std::vector<Eigen::Vector3d>>& all_voxels);

bool VoxelizeShape(
    const shapes::Shape& shape,
    const Eigen::Isometry3d& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels);

bool VoxelizeShape(
    const shapes::Shape& shape,
    const Eigen::Isometry3d& pose,
    double res,
    const Eigen::Vector3d& go,
    const Eigen::Vector3d& gmin,
    const Eigen::Vector3d& gmax,
    std::vector<Eigen::Vector3d>& voxels);

bool VoxelizeSphere(
    const shapes::Sphere& sphere,
    const Eigen::Isometry3d& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels);

bool VoxelizeCylinder(
    const shapes::Cylinder& cylinder,
    const Eigen::Isometry3d& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels);

bool VoxelizeCone(
    const shapes::Cone& cone,
    const Eigen::Isometry3d& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels);

bool VoxelizeBox(
    const shapes::Box& box,
    const Eigen::Isometry3d& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels);

bool VoxelizePlane(
    const shapes::Plane& plane,
    const Eigen::Isometry3d& pose,
    double res,
    const Eigen::Vector3d& go,
    const Eigen::Vector3d& gmin,
    const Eigen::Vector3d& gmax,
    std::vector<Eigen::Vector3d>& voxels);

bool VoxelizeMesh(
    const shapes::Mesh& mesh,
    const Eigen::Isometry3d& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels);

bool VoxelizeOcTree(
    const shapes::OcTree& octree,
    const Eigen::Isometry3d& pose,
    double res,
    const Eigen::Vector3d& go,
    std::vector<Eigen::Vector3d>& voxels);

} // namespace collision
} // namespace smpl

#endif
