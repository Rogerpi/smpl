#ifndef SMPL_BFS_WITH_ORIENTATION_HEURISTIC_H
#define SMPL_BFS_WITH_ORIENTATION_HEURISTIC_H

// standard includes
#include <memory>

// project includes
#include <smpl/occupancy_grid.h>
#include <smpl/bfs3d/bfs3d.h>
#include <smpl/debug/marker.h>
#include <smpl/heuristic/robot_heuristic.h>

namespace smpl {

class BfsRPYHeuristic : public RobotHeuristic
{
public:

    virtual ~BfsRPYHeuristic();

    bool init(RobotPlanningSpace* space, const OccupancyGrid* grid);

    double inflationRadius() const { return m_inflation_radius; }
    void setInflationRadius(double radius);
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
    ///@}

private:

    GoalConstraint m_goal;

    const OccupancyGrid* m_grid = nullptr;

    std::unique_ptr<BFS_3D> m_bfs;
    PoseProjectionExtension* m_pp = nullptr;

    double m_inflation_radius = 0.0;
    int m_cost_per_cell = 1;

    struct CellCoord
    {
        int x, y, z;
        CellCoord() = default;
        CellCoord(int x, int y, int z) : x(x), y(y), z(z) { }
    };
    std::vector<CellCoord> m_goal_cells;

    void syncGridAndBfs();
    int getBfsCostToGoal(const BFS_3D& bfs, int x, int y, int z) const;
};

} // namespace smpl

#endif
