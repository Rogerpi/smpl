#ifndef SMPL_MULTI_BFS_HEURISTIC_H
#define SMPL_MULTI_BFS_HEURISTIC_H

// standard includes
#include <memory>

// project includes
#include <smpl/occupancy_grid.h>
#include <smpl/bfs3d/bfs3d.h>
#include <smpl/debug/marker.h>
#include <smpl/heuristic/robot_heuristic.h>

namespace smpl {
 
//Similar implementation as multi frame bfs. Base bfs around a region instead of a single point.
class MultiBfsHeuristic : public RobotHeuristic
{
public:

    virtual ~MultiBfsHeuristic();

    bool init(RobotPlanningSpace* space, const OccupancyGrid* grid);

    double inflationRadius() const { return m_inflation_radius; }
    void setInflationRadius(double radius);
    double baseInflationRadius() const { return m_base_inflation_radius; }
    void setBaseInflationRadius(double radius);
    
    int costPerCell() const { return m_cost_per_cell; }
    void setCostPerCell(int cost);

    auto grid() const -> const OccupancyGrid* { return m_grid; }

    auto getWallsVisualization() const -> visual::Marker;
    auto getValuesVisualization() -> visual::Marker;

    /// \name Required Public Functions from RobotHeuristic
    ///@{
    double getMetricStartDistance(double x, double y, double z) override;
    double getMetricGoalDistance(double x, double y, double z) override;
    ///@}

    /// \name Required Public Functions from Extension
    ///@{
    Extension* getExtension(size_t class_code) override;
    ///@}

    /// \name Reimplemented Public Functions from RobotPlanningSpaceObserver
    ///@{
    void updateGoal(const GoalConstraint& goal) override;
    ///@}

    /// \name Required Public Functions from Heuristic
    ///@{
    int GetGoalHeuristic(int state_id) override;
    int GetStartHeuristic(int state_id) override;
    int GetFromToHeuristic(int from_id, int to_id) override; 
    int GetGoalHeuristic(int state_id, int planning_group) override;
    ///@}

    double getMetricGoalDistance(double x, double y, double z, bool use_ee);
    int GetGoalHeuristic(int state_id, bool use_ee);

private:

    const OccupancyGrid* m_grid = nullptr;

    PointProjectionExtension* m_pp = nullptr;
    ExtractRobotStateExtension* m_ers = nullptr;
    ForwardKinematicsInterface* m_fk_iface = nullptr;

    std::unique_ptr<BFS_3D> m_base_bfs;
    std::unique_ptr<BFS_3D> m_ee_bfs;

    double m_inflation_radius = 0.0;
    double m_base_inflation_radius = 0.0;
    int m_cost_per_cell = 1;

    std::vector<double> goal_config;

    void syncGridAndBfs();
    int getBfsCostToGoal(const BFS_3D& bfs, int x, int y, int z) const;
};

} // namespace smpl

#endif
