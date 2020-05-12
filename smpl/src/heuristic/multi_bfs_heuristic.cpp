#include <smpl/heuristic/multi_bfs_heuristic.h>

// project includes
#include <smpl/bfs3d/bfs3d.h>
#include <smpl/console/console.h>
#include <smpl/debug/visualize.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/debug/colors.h>

namespace smpl {

static const char* LOG = "heuristic.multi_bfs";

MultiBfsHeuristic::~MultiBfsHeuristic()
{
    // empty to allow forward declaration of BFS_3D
}

bool MultiBfsHeuristic::init(RobotPlanningSpace* space, const OccupancyGrid* grid)
{
    if (!grid) {
        return false;
    }

    if (!RobotHeuristic::init(space)) {
        return false;
    }

    m_grid = grid;

    m_pp = space->getExtension<PointProjectionExtension>();
    if (m_pp) {
        SMPL_INFO_NAMED(LOG, "Got Point Projection Extension!");
    }
    m_ers = space->getExtension<ExtractRobotStateExtension>();
    if (m_ers) {
        SMPL_INFO_NAMED(LOG, "Got Extract Robot State Extension!");
    }
    m_fk_iface = space->robot()->getExtension<ForwardKinematicsInterface>();
    if (m_fk_iface) {
        SMPL_INFO_NAMED(LOG, "Got Forward Kinematics Interface!");
    }
    syncGridAndBfs();
    return true;
}

void MultiBfsHeuristic::setInflationRadius(double radius)
{
    m_inflation_radius = radius;
}

void MultiBfsHeuristic::setBaseInflationRadius(double radius)
{
    m_base_inflation_radius = radius;
}

void MultiBfsHeuristic::setCostPerCell(int cost_per_cell)
{
    m_cost_per_cell = cost_per_cell;
}

void MultiBfsHeuristic::updateGoal(const GoalConstraint& goal)
{
    int gx, gy, gz, base_gx, base_gy, base_gz;
    grid()->worldToGrid(
            goal.pose.translation()[0],
            goal.pose.translation()[1],
            goal.pose.translation()[2],
            gx, gy, gz);


    SMPL_ERROR_NAMED(LOG, "Setting the BFS heuristic goal (%d, %d, %d)", gx, gy, gz);
    SMPL_ERROR_STREAM("Origin_arm "<<gx<<","<<gy<<","<<gz);
    
    if(goal.angles.empty())
    {
        int listSize = ((double)(1.8/grid()->resolution()))+1;
        std::vector<int> inputGoals(pow(listSize,3)*3);
        std::vector<Eigen::Vector3d> centers;
        SMPL_ERROR_STREAM("Goal Region Size "<<listSize<<" resolution "<<grid()->resolution()<<" total size "<<inputGoals.size());
        int idx = 0;
        int xSign = 1,ySign = 1, zSign = 1;
        for(int i=0;i<listSize;i++)
        {
            for(int j=0;j<listSize;j++)
               for(int k=0;k<listSize;k++)
                {   
                    if(i>listSize/2)
                        xSign = -(i-std::ceil(listSize/2));
                    else
                        xSign = i;
                    if(j>listSize/2)
                        ySign = -(j-std::ceil(listSize/2));
                    else
                        ySign = j;
                    if(k>listSize/2)
                        zSign = -(k-std::ceil(listSize/2));
                    else
                        zSign = k;
                    inputGoals[idx]  = gx + xSign;
                    inputGoals[idx+1] = gy + ySign;
                    inputGoals[idx+2] = gz + zSign;
                    
                    Eigen::Vector3d p;
                    grid()->gridToWorld( inputGoals[idx],  inputGoals[idx+1],  inputGoals[idx+2], p.x(), p.y(), p.z());
                    centers.push_back(p);
                    idx+=3; 
                }
        }

        visual::Color color(0.93f, 0.4f, 0.58f, 0.4f);     
        SV_SHOW_INFO (visual::MakeCubesMarker(
                centers,
                grid()->resolution(),
                color,
                grid()->getReferenceFrame(),
                "bfs_base_goals"));

        int numGoals = m_base_bfs->run< std::vector<int>::iterator >(inputGoals.begin(),inputGoals.end());
        if (!numGoals)
        {
            SMPL_ERROR_NAMED(LOG, "Grid Base Heuristic goal is out of BFS bounds");
        }
        else
        {
            SMPL_ERROR_STREAM("Number of base goal added "<<numGoals);
        }
    }
    else
    {
        goal_config = goal.angles;
        /*SMPL_ERROR_STREAM("Goal config is "<<goal_config[0]<<","<<goal_config[1]<<","
            <<goal_config[2]<<","<<goal_config[3]<<","<<goal_config[4]<<","<<goal_config[5]<<","
            <<goal_config[6]<<","<<goal_config[7]);*/
        grid()->worldToGrid(
                goal.angles[0], goal.angles[1], 5,//goal.angles[2],
                base_gx, base_gy, base_gz);

        SMPL_ERROR_STREAM("Base Goal :"<<goal.angles[0]<<","<<goal.angles[1]<<","<<5//goal.angles[2]
            <<","<<base_gx<<","<<base_gy<<","<<base_gz);
        

        if (!m_base_bfs->inBounds(base_gx, base_gy, base_gz))
        {
            SMPL_ERROR_NAMED(LOG, "Base Heuristic goal is out of BFS bounds");
        }

        m_base_bfs->run(base_gx,base_gy,base_gz);
    }

    if (!m_ee_bfs->inBounds(gx, gy, gz))
    {
        SMPL_ERROR_NAMED(LOG, "Arm Heuristic goal is out of BFS bounds");
    }
    
    m_ee_bfs->run(gx, gy, gz);
        

    if (!m_pp) {
        return ;
    }
}

double MultiBfsHeuristic::getMetricStartDistance(double x, double y, double z)
{
    int start_id = planningSpace()->getStartStateID();
    if (!m_pp) {
        return 0.0;
    }

    Eigen::Vector3d p;
    if (!m_pp->projectToPoint(planningSpace()->getStartStateID(), p)) {
        return 0.0;
    }

    int sx, sy, sz;
    grid()->worldToGrid(p.x(), p.y(), p.z(), sx, sy, sz);

    int gx, gy, gz;
    grid()->worldToGrid(x, y, z, gx, gy, gz);

    // compute the manhattan distance to the start cell
    const int dx = sx - gx;
    const int dy = sy - gy;
    const int dz = sz - gz;
    return grid()->resolution() * (abs(dx) + abs(dy) + abs(dz));
}

double MultiBfsHeuristic::getMetricGoalDistance(double x, double y, double z)
{
    int gx, gy, gz;
    grid()->worldToGrid(x, y, z, gx, gy, gz);
    if (!m_ee_bfs->inBounds(gx, gy, gz)) {
        return (double)BFS_3D::WALL * grid()->resolution();
    } else {
        return (double)m_ee_bfs->getDistance(gx, gy, gz) * grid()->resolution();
    }
}

double MultiBfsHeuristic::getMetricGoalDistance(double x, double y, double z, bool use_ee)
{
    SMPL_WARN("Get metric goal distance by group not implemented yet");
    /*int gx, gy, gz;
    if(planning_group==GroupType::BASE)
    {
        grid()->worldToGrid(x, y, z, gx, gy, gz);
        if (!m_base_bfs->inBounds(gx, gy, gz)) {
            return (double)BFS_3D::WALL * grid()->resolution();
        } else {
            return (double)m_base_bfs->getDistance(gx, gy, gz) * grid()->resolution();
        }
    }
    else
    {
        grid()->worldToGrid(x, y, z, gx, gy, gz);
        if (!m_bfs->inBounds(gx, gy, gz)) {
            return (double)BFS_3D::WALL * grid()->resolution();
        } else {
            return (double)m_bfs->getDistance(gx, gy, gz) * grid()->resolution();
        }
    }*/
}

Extension* MultiBfsHeuristic::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<RobotHeuristic>()) {
        return this;
    }
    return nullptr;
}

int MultiBfsHeuristic::GetGoalHeuristic(int state_id)
{
    if (!m_pp) {
        return 0;
    }

    Eigen::Vector3d p;
    if (!m_pp->projectToPoint(state_id, p)) {
        return 0;
    }

    Eigen::Vector3i dp;
    grid()->worldToGrid(p.x(), p.y(), p.z(), dp.x(), dp.y(), dp.z());
    return getBfsCostToGoal(*m_ee_bfs, dp.x(), dp.y(), dp.z());
}


int MultiBfsHeuristic::GetGoalHeuristic(int state_id, bool use_ee)
{
    if (!m_pp) {
        return 0;
    }

    Eigen::Vector3d p;
    if(use_ee) // Arm
    {   
        if (!m_pp->projectToPoint(state_id, p)) {
            return 0;
        }
    }
    else // Base
    {
        if (state_id==0)
            return 0;

        //else if (!m_pp->projectToBasePoint(state_id, p)) { 
        // From Dina's implementation, projectToBasePoint extracts the base pose from an state,
        // unless its the goal state, where an IK solution is found and base pose is extracted from there
        // For now, I discard the goal state case.
        const RobotState& state = m_ers->extractState(state_id);
        p[0] = state[0];
        p[1] = state[1];
        p[2] = state[2];
    }
    
    Eigen::Vector3i dp;
    grid()->worldToGrid(p.x(), p.y(), p.z(), dp.x(), dp.y(), dp.z());

    SMPL_DEBUG_STREAM("Getting distance heursitic for "<< use_ee? "Arm":"Base");
    SMPL_DEBUG_STREAM("get heursitic for grid point "<<dp.x()<<","<<dp.y()<<","<<dp.z());
    SMPL_DEBUG_STREAM("get heursitic for world point "<<p.x()<<","<<p.y()<<","<<p.z());
    
   double cost = use_ee ? getBfsCostToGoal(*m_ee_bfs, dp.x(), dp.y(), dp.z()) 
                        : getBfsCostToGoal(*m_base_bfs, dp.x(), dp.y(), dp.z());

   
    if (!m_pp->projectToPoint(planningSpace()->getStartStateID(), p)) {
        return 0.0;
    }

    int sx, sy, sz;
    grid()->worldToGrid(p.x(), p.y(), p.z(), sx, sy, sz);

// Dina implemented here some clearance penalty

SMPL_INFO_STREAM("Total final heuristic Cost is "<<cost);
//std::getchar();
SMPL_INFO_STREAM("====================================================================");
        
   return cost;
}

int MultiBfsHeuristic::GetStartHeuristic(int state_id)
{
    SMPL_WARN_ONCE("MultiBfsHeuristic::GetStartHeuristic unimplemented");
    return 0;
}

int MultiBfsHeuristic::GetFromToHeuristic(int from_id, int to_id)
{
    if (to_id == planningSpace()->getGoalStateID()) {
        return GetGoalHeuristic(from_id);
    }
    else {
        SMPL_WARN_ONCE("MultiBfsHeuristic::GetFromToHeuristic unimplemented for arbitrary state pair");
        return 0;
    }
}

auto MultiBfsHeuristic::getWallsVisualization() const -> visual::Marker
{
    std::vector<Eigen::Vector3d> centers;
    int dimX = grid()->numCellsX();
    int dimY = grid()->numCellsY();
    int dimZ = grid()->numCellsZ();
    for (int x = 0; x < dimX; x++) {
    for (int y = 0; y < dimY; y++) {
    for (int z = 0; z < dimZ; z++) {
        if (m_base_bfs->isWall(x, y, z)) {
            Eigen::Vector3d p;
            grid()->gridToWorld(x, y, z, p.x(), p.y(), p.z());
            centers.push_back(p);
        }
    }
    }
    }

    SMPL_ERROR_NAMED(LOG, "BFS Visualization contains %zu points", centers.size());

    visual::Color color;
    color.r = 100.0f / 255.0f;
    color.g = 149.0f / 255.0f;
    color.b = 238.0f / 255.0f;
    color.a = 1.0f;
    return visual::MakeCubesMarker(
            centers,
            grid()->resolution(),
            color,
            grid()->getReferenceFrame(),
            "bfs_walls");

}

auto MultiBfsHeuristic::getValuesVisualization() -> visual::Marker
{
    SMPL_WARN("MultiBfsHeuristis getValuesVisualization commented up temporarily");
    return visual::Marker();

    /*
    if (m_goal_x < 0 || m_goal_y < 0 || m_goal_z < 0) {
        return visual::MakeEmptyMarker();
    }

    if (m_ee_bfs->isWall(m_goal_x, m_goal_y, m_goal_z)) {
        return visual::MakeEmptyMarker();
    }

    // hopefully this doesn't screw anything up too badly...this will flush the
    // bfs to a little past the start, but this would be done by the search
    // hereafter anyway
    int start_heur = GetGoalHeuristic(planningSpace()->getStartStateID());
    if (start_heur == Infinity) {
        return visual::MakeEmptyMarker();
    }

    SMPL_INFO("Start cell heuristic: %d", start_heur);

    const int max_cost = (int)(1.1 * start_heur);

    SMPL_INFO("Get visualization of cells up to cost %d", max_cost);

    // ...and this will also flush the bfs...

    const size_t max_points = 4 * 4096;

    std::vector<Eigen::Vector3d> points;
    std::vector<visual::Color> colors;

    struct CostCell
    {
        int x, y, z, g;
    };
    std::queue<CostCell> cells;
    Grid3<bool> visited(grid()->numCellsX(), grid()->numCellsY(), grid()->numCellsZ(), false);
    visited(m_goal_x, m_goal_y, m_goal_z) = true;
    cells.push({m_goal_x, m_goal_y, m_goal_z, 0});
    while (!cells.empty()) {
        CostCell c = cells.front();
        cells.pop();

        if (c.g > max_cost || points.size() >= max_points) {
            break;
        }

        {
            double cost_pct = (double)c.g / (double)max_cost;

            visual::Color color = visual::MakeColorHSV(300.0 - 300.0 * cost_pct);

            auto clamp = [](double d, double lo, double hi) {
                if (d < lo) {
                    return lo;
                } else if (d > hi) {
                    return hi;
                } else {
                    return d;
                }
            };

            color.r = clamp(color.r, 0.0f, 1.0f);
            color.g = clamp(color.g, 0.0f, 1.0f);
            color.b = clamp(color.b, 0.0f, 1.0f);

            Eigen::Vector3d p;
            grid()->gridToWorld(c.x, c.y, c.z, p.x(), p.y(), p.z());
            if (std::fabs(p.z() -  3.73) < 1) {

                            points.push_back(p);

            colors.push_back(color);
            } 
        }

//        visited(c.x, c.y, c.z) = true;

        const int d = m_cost_per_cell * m_ee_bfs->getDistance(c.x, c.y, c.z);

        for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
        for (int dz = -1; dz <= 1; ++dz) {
            if (!(dx | dy | dz)) {
                continue;
            }

            int sx = c.x + dx;
            int sy = c.y + dy;
            int sz = c.z + dz;

            // check if neighbor is valid
            if (!m_ee_bfs->inBounds(sx, sy, sz) || m_ee_bfs->isWall(sx, sy, sz)) {
                continue;
            }

            // check if cost can be improved
            if (visited(sx, sy, sz)) {
                continue;
            }

            visited(sx, sy, sz) = true;

            int dd = m_cost_per_cell * m_ee_bfs->getDistance(sx, sy, sz);
            cells.push({sx, sy, sz, dd});
        }
        }
        }
    }

     return visual::MakeCubesMarker(
            std::move(points),
            0.5 * grid()->resolution(),
            std::move(colors),
            grid()->getReferenceFrame(),
            "bfs_values");
    */
}

void MultiBfsHeuristic::syncGridAndBfs()
{
    const int xc = grid()->numCellsX();
    const int yc = grid()->numCellsY();
    const int zc = grid()->numCellsZ();
    m_base_bfs.reset(new BFS_3D(xc, yc, zc));
    m_ee_bfs.reset(new BFS_3D(xc, yc, zc));

    const int cell_count = xc * yc * zc;
    int wall_count = 0, base_wall_count = 0;
    for (int x = 0; x < xc; ++x) {
        for (int y = 0; y < yc; ++y) {
            for (int z = 0; z < zc; ++z) {
                const double radius = m_inflation_radius;
                if (grid()->getDistance(x, y, z) <= radius) {
                    m_ee_bfs->setWall(x, y, z);
                    ++wall_count;
                }
                if(grid()->getDistance(x, y, z) <= m_base_inflation_radius)
                {
                    m_base_bfs->setWall(x,y,z);
                    ++base_wall_count;
                }
            }
        }
    }

    SMPL_ERROR_NAMED(LOG, "%d/%d (%0.3f%%) walls in the bfs heuristic", wall_count, cell_count, 100.0 * (double)wall_count / cell_count);
    SMPL_ERROR_NAMED(LOG, "%d/%d (%0.3f%%) walls in the base bfs heuristic of radius (%0.3f)", base_wall_count, cell_count, 100.0 * (double)base_wall_count / cell_count, m_base_inflation_radius);
}

int MultiBfsHeuristic::getBfsCostToGoal(const BFS_3D& bfs, int x, int y, int z) const
{
    if (!bfs.inBounds(x, y, z)) {
        return Infinity;
    }
    else if (bfs.getDistance(x, y, z) == BFS_3D::WALL) {
        return Infinity;
    }
    else 
    {
        return m_cost_per_cell * bfs.getDistance(x, y, z);
    }
}

} // namespace smpl
