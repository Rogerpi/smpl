#ifndef SBPL_COLLISION_CHECKING_SHAPES_H
#define SBPL_COLLISION_CHECKING_SHAPES_H

#include <cstdint>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <octomap/OcTree.h>
#include <visualization_msgs/Marker.h>

#include <sbpl_collision_checking/types.h>

namespace smpl {
namespace collision {

enum class ShapeType
{
    Sphere,
    Cylinder,
    Cone,
    Box,
    Plane,
    Mesh,
    OcTree
};

inline const char* to_cstring(ShapeType type) {
    switch (type) {
    case ShapeType::Sphere:
        return "Sphere";
    case ShapeType::Cylinder:
        return "Cylinder";
    case ShapeType::Cone:
        return "Cone";
    case ShapeType::Box:
        return "Box";
    case ShapeType::Plane:
        return "Plane";
    case ShapeType::Mesh:
        return "Mesh";
    case ShapeType::OcTree:
        return "OcTree";
    }
}

struct CollisionShape {
    ShapeType type;
};

struct SphereShape : public CollisionShape {
    double radius = 0.0;

    SphereShape() { type = ShapeType::Sphere; }
    SphereShape(double r) { type = ShapeType::Sphere; radius = r; }
};

struct CylinderShape : public CollisionShape {
    double radius = 0.0;
    double height = 0.0;

    CylinderShape() { type = ShapeType::Cylinder; }

    CylinderShape(double r, double h) {
        type = ShapeType::Cylinder; radius = r; height = h;
    }
};

struct ConeShape : public CollisionShape {
    double radius = 0.0;
    double height = 0.0;

    ConeShape() { type = ShapeType::Cone; }

    ConeShape(double r, double h) {
        type = ShapeType:: Cone; radius = r; height = h;
    }
};

struct BoxShape : public CollisionShape {
    double size[3] = { 0.0, 0.0, 0.0 };

    BoxShape() { type = ShapeType::Box; }

    BoxShape(double size_x, double size_y, double size_z) {
        type = ShapeType::Box;
        size[0] = size_x; size[1] = size_y; size[2] = size_z;
    }
};

struct PlaneShape : public CollisionShape {
    double a = 0.0, b = 0.0, c = 0.0, d = 0.0;

    PlaneShape() { type = ShapeType::Plane; }

    PlaneShape(double aa, double bb, double cc, double dd) {
        type = ShapeType::Plane; a = aa; b = bb; c = cc; d = dd;
    }
};

struct MeshShape : public CollisionShape {
    double* vertices = nullptr;
    size_t vertex_count = 0;

    std::uint32_t* triangles = nullptr;
    size_t triangle_count = 0;

    MeshShape() { type = ShapeType::Mesh; }
};

struct OcTreeShape : public CollisionShape {
    const octomap::OcTree* octree = nullptr;
    OcTreeShape() { type = ShapeType::OcTree; }
    OcTreeShape(const octomap::OcTree* o) { type = ShapeType::OcTree; octree = o; }
};

struct CollisionObject {
    std::string id;
    std::vector<CollisionShape*> shapes;
    AlignedVector<Eigen::Isometry3d> shape_poses;
};

/// Helper struct to represent geometry attached to a robot link. Bundles
/// together a reference to the collision shape and its fixed offset from
/// the link's origin.
struct CollisionGeometry {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    const CollisionShape* shape;
    Eigen::Isometry3d offset;
};

/// Another helper struct to store all collision geometry attached to a robot
/// link.
struct LinkCollisionGeometry {
    std::vector<CollisionGeometry> geometries;
};

} // namespace collision
} // namespace smpl

#endif
