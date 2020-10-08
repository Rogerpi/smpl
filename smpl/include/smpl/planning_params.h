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

#ifndef SMPL_PLANNING_PARAMS_H
#define SMPL_PLANNING_PARAMS_H

// standard includes
#include <stdexcept>
#include <string>
#include <unordered_map>

// system includes
#include <boost/variant.hpp>

#define G_LOG "graph"
#define G_EXPANSIONS_LOG G_LOG ".expansions"
#define G_SUCCESSORS_LOG G_LOG ".successors"
#define H_LOG "heuristic"
#define S_LOG "search"
#define S_EXPANSIONS_LOG SEARCH_LOG ".expansions"
#define S_SUCCESSORS_LOG SEARCH_LOG ".successors"

namespace smpl {

enum ShortcutType
{
    INVALID_SHORTCUT_TYPE = -1,
    JOINT_SPACE,
    JOINT_POSITION_VELOCITY_SPACE,
    EUCLID_SPACE,
    NUM_SHORTCUT_TYPES
};

std::string to_string(ShortcutType type);

struct ParameterException : public std::runtime_error
{
    ParameterException(const std::string& what) : std::runtime_error(what) { }
    ParameterException(const char* what) : std::runtime_error(what) { }
};

using Parameter = boost::variant<bool, int, double, std::string>;

class PlanningParams
{
public:

    static const int DefaultCostPerCell = 100;

    // post processing parameters
    static const bool DefaultShortcutPath = false;
    static const bool DefaultInterpolatePath = false;
    static const ShortcutType DefaultShortcutType = ShortcutType::JOINT_SPACE;

    // TODO: visualization parameters

    PlanningParams();

    /// \name Heuristic
    ///@{
    int cost_per_cell;             ///< uniform cost of cells in heuristic
    ///@}

    /// \name Post-Processing
    ///@{
    bool shortcut_path;
    bool interpolate_path;
    ShortcutType shortcut_type;
    ///@}

    /// \name Logging
    ///@{
    std::string plan_output_dir;
    ///@}

    void addParam(const std::string& name, bool val);
    void addParam(const std::string& name, int val);
    void addParam(const std::string& name, double val);
    void addParam(const std::string& name, const std::string& val);

    void param(const std::string& name, bool& val, bool def) const;
    void param(const std::string& name, int& val, int def) const;
    void param(const std::string& name, double& val, double def) const;
    void param(
        const std::string& name,
        std::string& val,
        const std::string& def) const;

    bool getParam(const std::string& name, bool& val) const;
    bool getParam(const std::string& name, int& val) const;
    bool getParam(const std::string& name, double& val) const;
    bool getParam(const std::string& name, std::string& val) const;

    bool hasParam(const std::string& name) const;

    void print() const;

private:

    std::unordered_map<std::string, Parameter> params;

    bool m_warn_defaults;

    void convertToBool(const Parameter& p, bool& val) const;
    void convertToInt(const Parameter& p, int& val) const;
    void convertToDouble(const Parameter& p, double& val) const;
    void convertToString(const Parameter& p, std::string& val) const;
};

} // namespace smpl

#endif

