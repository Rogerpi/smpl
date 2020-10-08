////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2010, Benjamin Cohen, Andrew Dornbush
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

#include <smpl/graph/manip_lattice_action_space.h>

// standard includes
#include <limits>
#include <numeric>

// project includes
#include <smpl/angles.h>
#include <smpl/console/console.h>
#include <smpl/graph/manip_lattice.h>
#include <smpl/heuristic/robot_heuristic.h>

namespace smpl {

bool ManipLatticeActionSpace::init(ManipLattice* space)
{
    if (!ActionSpace::init(space)) {
        return false;
    }

    // NOTE: other default thresholds will be set in readParameters, with
    // default values specified in PlanningParams
    m_mprim_thresh[MotionPrimitive::Type::LONG_DISTANCE] =
            std::numeric_limits<double>::infinity();

    clear();

    RobotModel* robot = planningSpace()->robot();

    m_fk_iface = robot->getExtension<ForwardKinematicsInterface>();
    m_ik_iface = robot->getExtension<InverseKinematicsInterface>();

    if (!m_fk_iface) {
        SMPL_WARN("Manip Lattice Action Set requires Forward Kinematics Interface");
    }

    if (!m_ik_iface) {
        SMPL_WARN("Manip Lattice Action Set recommends Inverse Kinematics Interface");
    }

    useMultipleIkSolutions(false);
    useAmp(MotionPrimitive::SNAP_TO_XYZ, false);
    useAmp(MotionPrimitive::SNAP_TO_RPY, false);
    useAmp(MotionPrimitive::SNAP_TO_XYZ_RPY, false);
    useAmp(MotionPrimitive::SHORT_DISTANCE, false);
    ampThresh(MotionPrimitive::SNAP_TO_XYZ, 0.2);
    ampThresh(MotionPrimitive::SNAP_TO_RPY, 0.2);
    ampThresh(MotionPrimitive::SNAP_TO_XYZ_RPY, 0.2);
    ampThresh(MotionPrimitive::SHORT_DISTANCE, 0.2);
    //ampThresh(MotionPrimitive::SUCCESSOR, 0.0);

    return true;
}

/// \brief Load motion primitives from file.
///
/// Read in the discrete variable deltas for each motion. If short distance
/// motion primitives are included in the file, they are also enabled.
///
/// Action Set File Format
///
/// Motion_Primitives(degrees): <i actions> <j planning joint variables> <k short distance motion primitives>
/// dv11         dv12        ... dv1m
/// ...
/// dv(i-k)1     dv(i-k)2    ... dv(i-k)m
/// dv(i-k+1)1   dv(i-k+1)2  ... dv(i-k+1)m
/// ...
/// dvi1         dvi2        ... dvim
bool ManipLatticeActionSpace::load(const std::string& action_filename)
{
    SMPL_INFO_STREAM("Load mp " << action_filename);
    FILE* fCfg = fopen(action_filename.c_str(), "r");
    if (!fCfg) {
        SMPL_ERROR("Failed to open action set file. (file: '%s')", action_filename.c_str());
        return false;
    }

    char sTemp[1024] = { 0 };
    int nrows = 0;
    int ncols = 0;
    int short_mprims = 0;

    // read and check header
    if (fscanf(fCfg, "%1023s", sTemp) < 1) {
        SMPL_ERROR("Parsed string has length < 1.");
    }

    if (strcmp(sTemp, "Motion_Primitives(degrees):") != 0) {
        SMPL_ERROR("First line of motion primitive file should be 'Motion_Primitives(degrees):'. Please check your file. (parsed string: %s)\n", sTemp);
        return false;
    }

    // read number of actions
    if (fscanf(fCfg, "%d", &nrows) < 1) {
        SMPL_ERROR("Parsed string has length < 1.");
        return false;
    }

    // read length of joint array
    if (fscanf(fCfg, "%d", &ncols) < 1) {
        SMPL_ERROR("Parsed string has length < 1.");
        return false;
    }

    // read number of short distance motion primitives
    if (fscanf(fCfg, "%d", &short_mprims) < 1) {
        SMPL_ERROR("Parsed string has length < 1.");
        return false;
    }

    if (short_mprims == nrows) {
        SMPL_WARN("# of motion prims == # of short distance motion prims. No long distance motion prims set.");
    }

    std::vector<double> mprim(ncols - 2, 0); // Group & weight params

    bool have_short_dist_mprims = short_mprims > 0;
    if (have_short_dist_mprims) {
        useAmp(MotionPrimitive::SHORT_DISTANCE, true);
    }

    ManipLattice* lattice = static_cast<ManipLattice*>(planningSpace());

    for (int i = 0; i < nrows; ++i) {
        // read joint delta
        for (int j = 0; j < ncols - 2; ++j) {
            double d;
            if (fscanf(fCfg, "%lf", &d) < 1)  {
                SMPL_ERROR("Parsed string has length < 1.");
                return false;
            }
            if (feof(fCfg)) {
                SMPL_ERROR("End of parameter file reached prematurely. Check for newline.");
                return false;
            }
            mprim[j] = d * lattice->resolutions()[j];
            SMPL_DEBUG("Got %0.3f deg -> %0.3f rad", d, mprim[j]);
        }

        int group;
        if(fscanf(fCfg, "%i", &group)<1)
        {
            SMPL_ERROR("No planning group defined for this MP!");
            group = smpl::GroupType::ANY;
        }
        double weight;
        if(fscanf(fCfg, "%lf", &weight)<1)
        {
            SMPL_ERROR("No weight defiend for this MP!");
            weight = 1.0;
        }
        SMPL_INFO_STREAM("Read weight " << weight);
        
        if (i < (nrows - short_mprims)) {
            addMotionPrim(mprim, group, weight, false);
        } else {
            addMotionPrim(mprim, group, weight, true);
        }
    }

    fclose(fCfg);
    return true;
}

/// \brief Add a long or short distance motion primitive to the action set
/// \param mprim The angle delta for each joint, in radians
/// \param short_dist true = short distance; false = long distance
/// \param add_converse Whether to add the negative of this motion primitive
///     to the action set
void ManipLatticeActionSpace::addMotionPrim(
    const std::vector<double>& mprim,
    int group, double weight,
    bool short_dist_mprim,
    bool add_converse)
{
    MotionPrimitive m;

    if (short_dist_mprim) {
        m.type = MotionPrimitive::SHORT_DISTANCE;
    } else {
        m.type = MotionPrimitive::LONG_DISTANCE;
    }

    m.action.push_back(mprim);
    m.group = smpl::GroupType(group);
    m.weight = weight;
    m_mprims.push_back(m);
    SMPL_INFO_STREAM("Argument weight: " << weight);

    if (add_converse) {
        for (RobotState& state : m.action) {
            for (size_t i = 0; i < state.size(); ++i) {
                state[i] *= -1.0;
            }
        }
        m_mprims.push_back(m);
    }
}

void ManipLatticeActionSpace::addMotionPrim(
    const std::vector<double>& mprim,
    bool short_dist_mprim,
    bool add_converse)
{
    addMotionPrim(mprim, GroupType::ANY, 1, short_dist_mprim, add_converse);
}

/*
void ManipLatticeActionSpace::addSuccsMotionPrim(
    const std::vector<int>& joint_idxs,
    const RobotState& parent,
    const RobotState& succ)
{
  auto it = m_msuccs.find(joint_idxs);
  if(it == m_msuccs.end()){
      std::map<RobotState, std::vector<MotionPrimitive>> succs_map;
      std::vector<MotionPrimitive> prims;
      prims.resize(1);
      prims[0].type = prims[0].SUCCESSOR;
      prims[0].group = smpl::GroupType::ANY;
      prims[0].weight = 1; // TODO

      std::vector<double> motion = succ;
      for(unsigned int i = 0; i < succ.size(); i++){
          motion[i] -= parent[i];
      }

      prims[0].action.push_back(motion);
      succs_map.insert(std::make_pair(parent, prims));
      m_msuccs.insert(std::make_pair(joint_idxs, succs_map));
    
  }
  else{
      //HEREEEEEEEEEEEEEEEEEEEEEE
      //TODO: Check is not repeated
      it->second.push_back(succ);
  }
}
*/


/// \brief Remove long and short motion primitives and disable adaptive motions.
///
/// Thresholds for short distance and adaptive motions are retained
void ManipLatticeActionSpace::clear()
{
    m_mprims.clear();

    // add all amps to the motion primitive set
    MotionPrimitive mprim;

    mprim.type = MotionPrimitive::SNAP_TO_RPY;
    mprim.action.clear();               // TODO:
    mprim.group = smpl::GroupType::ANY; // I don't like this for multiple end-effectors
    mprim.weight = 0.5;                 // These MP should be parsed from file
    m_mprims.push_back(mprim);

    mprim.type = MotionPrimitive::SNAP_TO_XYZ;
    mprim.action.clear();
    mprim.group = smpl::GroupType::ANY; 
    mprim.weight = 0.5;
    m_mprims.push_back(mprim);

    mprim.type = MotionPrimitive::SNAP_TO_XYZ_RPY;
    mprim.action.clear();
     mprim.group = smpl::GroupType::ANY; 
    mprim.weight = 0.5;
    m_mprims.push_back(mprim);

    /*
    mprim.type = MotionPrimitive::SUCCESSOR;
    mprim.action.clear();
    mprim.group = smpl::GroupType::ANY;
    mprim.weight = 1.0;
    m_mprims.push_back(mprim);
    */

    for (int i = 0; i < MotionPrimitive::NUMBER_OF_MPRIM_TYPES; ++i) {
        m_mprim_enabled[i] = (i == MotionPrimitive::Type::LONG_DISTANCE);
    }
}

int ManipLatticeActionSpace::longDistCount() const
{
    return std::accumulate(
            begin(), end(), 0,
            [](int count, const MotionPrimitive& prim)
            {
                return count +
                        (prim.type == MotionPrimitive::LONG_DISTANCE ? 1 : 0);
            });
}

int ManipLatticeActionSpace::shortDistCount() const
{
    return std::accumulate(
            begin(), end(), 0,
            [](int count, const MotionPrimitive& prim)
            {
                return count +
                        (prim.type == MotionPrimitive::SHORT_DISTANCE ? 1 : 0);
            });
}

bool ManipLatticeActionSpace::useAmp(MotionPrimitive::Type type) const
{
    return m_mprim_enabled[type];
}

bool ManipLatticeActionSpace::useMultipleIkSolutions() const
{
    return m_use_multiple_ik_solutions;
}

bool ManipLatticeActionSpace::useLongAndShortPrims() const
{
    return m_use_long_and_short_dist_mprims;
}

double ManipLatticeActionSpace::ampThresh(MotionPrimitive::Type type) const
{
    return m_mprim_thresh[type];
}

void ManipLatticeActionSpace::useAmp(MotionPrimitive::Type type, bool enable)
{
    m_mprim_enabled[type] = enable;
}

void ManipLatticeActionSpace::useMultipleIkSolutions(bool enable)
{
    m_use_multiple_ik_solutions = enable;
}

void ManipLatticeActionSpace::useLongAndShortPrims(bool enable)
{
    m_use_long_and_short_dist_mprims = enable;
}

void ManipLatticeActionSpace::ampThresh(
    MotionPrimitive::Type type,
    double thresh)
{
    if (type != MotionPrimitive::LONG_DISTANCE) {
        m_mprim_thresh[type] = thresh;
    }
}

auto ManipLatticeActionSpace::getStartGoalDistances(const RobotState& state)
    -> std::pair<double, double>
{
    if (!m_fk_iface) {
        return std::make_pair(0.0, 0.0);
    }

    auto pose = m_fk_iface->computeFK(state);

    if (planningSpace()->numHeuristics() > 0) {
        RobotHeuristic* h = planningSpace()->heuristic(0);
        double start_dist = h->getMetricStartDistance(
                pose.translation()[0],
                pose.translation()[1],
                pose.translation()[2]);
        double goal_dist = h->getMetricGoalDistance(
                pose.translation()[0],
                pose.translation()[1],
                pose.translation()[2]);
        return std::make_pair(start_dist, goal_dist);
    } else {
        return std::make_pair(0.0, 0.0);
    }
}

bool ManipLatticeActionSpace::apply(
    const RobotState& parent,
    std::vector<Action>& actions)
{
    double goal_dist, start_dist;
    std::tie(start_dist, goal_dist) = getStartGoalDistances(parent);

    for (auto& prim : m_mprims) {
        (void)getAction(parent, goal_dist, start_dist, prim, actions);
    }

    if (actions.empty()) {
        SMPL_WARN_ONCE("No motion primitives specified");
    }

    return true;
}

bool ManipLatticeActionSpace::apply(
    const RobotState& parent,
    std::vector<Action>& actions, ActionsWeight& weights, int group)
{
    double goal_dist, start_dist;
    std::tie(start_dist, goal_dist) = getStartGoalDistances(parent);

    std::vector<Action> act;
    for (auto& prim : m_mprims) {
        if (group == GroupType::ANY || prim.group == GroupType::ANY || prim.group == group){
            act.clear();
            if(getAction(parent, goal_dist, start_dist, prim, act)){
                actions.insert(actions.end(), act.begin(), act.end());
                for(auto& _ : act)
                    weights.push_back(prim.weight);
            }
        }
    }
    if (actions.empty()) {
        SMPL_WARN("No motion primitives specified (group)");
    }
    return true;
}

bool ManipLatticeActionSpace::getAction(
    const RobotState& parent,
    double goal_dist,
    double start_dist,
    const MotionPrimitive& mp,
    std::vector<Action>& actions)
{
    SMPL_INFO_STREAM("goal dist " << goal_dist);
    SMPL_INFO_STREAM("TYPE: " << mp.type << " ENABLED " << (m_mprim_enabled[mp.type] ? "true" : "false") << "THR: " <<  m_mprim_thresh[mp.type]);
    if (!mprimActive(start_dist, goal_dist, mp.type)) {
        return false;
    }

    GoalType goal_type = planningSpace()->goal().type;
    auto& goal_pose = planningSpace()->goal().pose;
    switch (mp.type) {
    case MotionPrimitive::LONG_DISTANCE:  // fall-through
    case MotionPrimitive::SHORT_DISTANCE:
    {
        Action action;
        if (!applyMotionPrimitive(parent, mp, action)) {
            return false;
        }
        actions.push_back(std::move(action));
        return true;
    }
    case MotionPrimitive::SNAP_TO_RPY:
    {
        SMPL_INFO("SNAP RPY ");
        //std::getchar();
        return computeIkAction(
                parent,
                goal_pose,
                goal_dist,
                ik_option::RESTRICT_XYZ,
                actions);
    }
    case MotionPrimitive::SNAP_TO_XYZ:
    {
        SMPL_INFO("SNAP XYZ ");
        //std::getchar();
        return computeIkAction(
                parent,
                goal_pose,
                goal_dist,
                ik_option::RESTRICT_RPY,
                actions);
    }
    case MotionPrimitive::SNAP_TO_XYZ_RPY:
    {
        SMPL_INFO("SNAP XYZRPY ");
        ///std::getchar();
        if (planningSpace()->goal().type != GoalType::JOINT_STATE_GOAL) {
            if(computeIkAction(
                    parent,
                    goal_pose,
                    goal_dist,
                    ik_option::UNRESTRICTED,
                    actions)){
                        SMPL_INFO("IK WORK");
                        return true;
                    }
                    else{
                        SMPL_INFO("IK DOES NOT WORLK");
                        return false;
                    }
        }

        // goal is 7dof; instead of computing IK, use the goal itself as the IK
        // solution
        Action action = { planningSpace()->goal().angles };
        actions.push_back(std::move(action));
        SBPL_ERROR("7DOF goal as solution. Is this valid for Girona500?"); //TODO: Check 

        return true;
    }
    /*
    case MotionPrimitive::SUCCESSOR: //For now hardcoded successor part
    {
        RobotState sub_parent = RobotState(8);
        sub_parent[0] = parent[0];
        sub_parent[1] = parent[1];
        sub_parent[2] = parent[2];
        sub_parent[3] = parent[3];
        sub_parent[4] = parent[4];
        sub_parent[5] = parent[5];
        sub_parent[6] = parent[6];
        sub_parent[7] = parent[7];
        auto it = m_msuccs.find(sub_parent);
        if (it == m_msuccs.end()){
            SMPL_ERROR("Reached state out of successor primitives");
            return false;
        }
        unsigned int succ_size = it->second.size();
        for(unsigned int i = 0; i < succ_size; i++){
            Action action = Action(1);
            action[1] = parent;
            action[1][0] = it->second[i][0];
            action[1][1] = it->second[i][1];
            action[1][2] = it->second[i][2];
            action[1][3] = it->second[i][3];
            action[1][4] = it->second[i][4];
            action[1][5] = it->second[i][5];
            action[1][6] = it->second[i][6];
            action[1][7] = it->second[i][7];
            actions.push_back(action);
        }
        SMPL_ERROR_STREAM("SUCCESSOR MP : " << succ_size);
        return true;
    }
    */
    default:
        SMPL_ERROR("Motion Primitives of type '%d' are not supported.", mp.type);
        return false;
    }
}

bool ManipLatticeActionSpace::applyMotionPrimitive(
    const RobotState& state,
    const MotionPrimitive& mp,
    Action& action)
{
    action = mp.action;
    for (size_t i = 0; i < action.size(); ++i) {
        if (action[i].size() != state.size()) {
            return false;
        }

        // Converting MPs from AUV frame to world frame
        Eigen::Matrix2d worldToBody;
        worldToBody(0,0) = cos(state[3]);
        worldToBody(0,1) = -sin(state[3]);
        worldToBody(1,0) = sin(state[3]);
        worldToBody(1,1) = cos(state[3]);
        Eigen::Vector2d actionInWorldFrame = worldToBody*Eigen::Vector2d(action[i][0],action[i][1]);
        action[i][0] = actionInWorldFrame[0];
        action[i][1] = actionInWorldFrame[1];

        //Apply MP
        for (size_t j = 0; j < action[i].size(); ++j) {
            action[i][j] = action[i][j] + state[j];
        }
    }
    return true;
}

bool ManipLatticeActionSpace::computeIkAction(
    const RobotState& state,
    const Affine3& goal,
    double dist_to_goal,
    ik_option::IkOption option,
    std::vector<Action>& actions)
{
    if (!m_ik_iface) {
        SMPL_ERROR("NO IK INTERFACE...");
        return false;
    }

    if (m_use_multiple_ik_solutions) {
        SMPL_INFO("RP: MULTIPLE");
        //get actions for multiple ik solutions
        std::vector<RobotState> solutions;
        if (!m_ik_iface->computeIK(goal, state, solutions, option)) {
            return false;
        }
        for (auto& solution : solutions) {
            Action action = { std::move(solution) };
            actions.push_back(std::move(action));
        }
    } else {
        //get single action for single ik solution
        RobotState ik_sol;
        if (!m_ik_iface->computeIK(goal, state, ik_sol)) {
            SMPL_ERROR("IK FAILED");
            return false;
        }

        Action action = { std::move(ik_sol) };
        actions.push_back(std::move(action));
    }

    return true;
}

bool ManipLatticeActionSpace::mprimActive(
    double start_dist,
    double goal_dist,
    MotionPrimitive::Type type) const
{
    // TODO: this seems a awkward..."short distance" motion primitives should be
    // the default since "long distance" primitives are usually implemented as
    // an optimization in regions far from the start or goal, and often we
    // always need "short distance" motion primitives near the start and goal.
    // -Andrew
    if (type == MotionPrimitive::LONG_DISTANCE) {
        if (m_use_long_and_short_dist_mprims) {
            return true;
        }
        const bool near_goal =
                goal_dist <= m_mprim_thresh[MotionPrimitive::SHORT_DISTANCE];
        const bool near_start =
                start_dist <= m_mprim_thresh[MotionPrimitive::SHORT_DISTANCE];
        const bool near_endpoint = near_goal;// || near_start;
        return !(m_mprim_enabled[MotionPrimitive::SHORT_DISTANCE] && near_endpoint);
    } else if (type == MotionPrimitive::SHORT_DISTANCE) {
        if (m_use_long_and_short_dist_mprims) {
            return m_mprim_enabled[type];
        }
        const bool near_goal = goal_dist <= m_mprim_thresh[type];
        const bool near_start = start_dist <= m_mprim_thresh[type];
        const bool near_endpoint = near_goal;// || near_start;
        return m_mprim_enabled[type] && near_endpoint;
    } else {
        return m_mprim_enabled[type] && goal_dist <= m_mprim_thresh[type];
    }
}

} // namespace smpl
