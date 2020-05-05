#include <smpl_moveit_interface/interface/joint_variable_command_widget.h>

// system includes
#include <Eigen/Dense>
#include <QGridLayout>
#include <QWidget>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <ros/console.h>
#include <smpl/angles.h>
#include <smpl/console/nonstd.h>

#include <smpl_moveit_interface/interface/robot_command_model.h>

namespace sbpl_interface {

static const char *LOG = "joint_variable_command_widget";

//////////////////////
// Helper Functions //
//////////////////////

/// Test if a variable is a single-dof angular variable
///
/// Angle variables are treated differently by displaying their values in
/// degrees while the underlying value is expressed in radians
static bool IsVariableAngle(
    const moveit::core::RobotModel& robot_model,
    int vidx)
{
    auto* jm = robot_model.getJointOfVariable(vidx);

    auto& var_name = robot_model.getVariableNames()[vidx];

    auto& var_bounds = jm->getVariableBounds(var_name);

    return (jm->getType() == moveit::core::JointModel::REVOLUTE ||
        (
            jm->getType() == moveit::core::JointModel::PLANAR &&
            !var_bounds.position_bounded_
        ) ||
        (
            jm->getType() == moveit::core::JointModel::FLOATING &&
            !var_bounds.position_bounded_
        ));
}

static auto CreateRealVariableSpinBox(
    const moveit::core::VariableBounds& bounds)
    -> QDoubleSpinBox*
{
    ROS_DEBUG_NAMED(LOG, "Create real variable spinbox");
    auto* spinbox = new QDoubleSpinBox;
    double min, max, step;
    if (bounds.position_bounded_) {
        // why are floating joint linear variables bounded with infinite bounds?
        if (bounds.min_position_ == -std::numeric_limits<double>::infinity()) {
            min = -100.0;
        } else {
            min = bounds.min_position_;
        }
        if (bounds.max_position_ == std::numeric_limits<double>::infinity()) {
            max = 100.0;
        } else {
            max = bounds.max_position_;
        }

        if (bounds.max_position_ == std::numeric_limits<double>::infinity() ||
            bounds.min_position_ == -std::numeric_limits<double>::infinity())
        {
            step = 0.01;
        } else {
            step = (max - min) / 100.0;
        }
    } else {
        min = -100.0;
        max = 100.0;
        step = 0.01;
    }

    spinbox->setMinimum(min);
    spinbox->setMaximum(max);
    ROS_DEBUG_NAMED(LOG, "  Single step for real variable: %f", step);

    // TODO: compute the number of decimals required to display a
    // change in the joint variable or round the step up to the
    // nearest number of decimals desired to be displayed
    spinbox->setDecimals(3);
    spinbox->setSingleStep(step);
    spinbox->setWrapping(false);
    spinbox->setSuffix("m");
    return spinbox;
}

static auto CreateRollVariableSpinBox() -> QDoubleSpinBox*
{
    ROS_DEBUG_NAMED(LOG, "Create roll variable spinbox");
    // TODO: limit bounds
    auto* spinbox = new QDoubleSpinBox;
    spinbox->setMinimum(-180.0);
    spinbox->setMaximum(180.0);
    spinbox->setSingleStep(1.0);
    spinbox->setWrapping(true);
    spinbox->setSuffix(QChar(0260));
    return spinbox;
}

static auto CreatePitchVariableSpinBox() -> QDoubleSpinBox*
{
    ROS_DEBUG_NAMED(LOG, "Create pitch variable spinbox");
    // TODO: limit bounds
    auto* spinbox = new QDoubleSpinBox;
    spinbox->setMinimum(0.0);
    spinbox->setMaximum(180.0);
    spinbox->setSingleStep(1.0);
    spinbox->setWrapping(true);
    spinbox->setSuffix(QChar(0260));
    return spinbox;
}

static auto CreateYawVariableSpinBox() -> QDoubleSpinBox*
{
    ROS_DEBUG_NAMED(LOG, "Create yaw variable spinbox");
    // TODO: limit bounds
    auto* spinbox = new QDoubleSpinBox;
    spinbox->setMinimum(-180.0);
    spinbox->setMaximum(180.0);
    spinbox->setSingleStep(1.0);
    spinbox->setWrapping(true);
    spinbox->setSuffix(QChar(0260));
    return spinbox;
}

static auto CreateAngleVariableSpinBox() -> QDoubleSpinBox*
{
    ROS_DEBUG_NAMED(LOG, "Create angle variable spinbox");
    auto* spinbox = new QDoubleSpinBox;
    spinbox->setMinimum(0.0);
    spinbox->setMaximum(359.0);
    spinbox->setSingleStep(1.0);
    spinbox->setWrapping(true);
    spinbox->setSuffix(QChar(0260));
    return spinbox;
}

static auto CreateRevoluteVariableSpinBox(
    const moveit::core::VariableBounds& bounds)
    -> QDoubleSpinBox*
{
    if (bounds.position_bounded_) {
        ROS_DEBUG_NAMED(LOG, "Create revolute variable spinbox");
        QDoubleSpinBox* spinbox = new QDoubleSpinBox;
        spinbox->setMinimum(
                smpl::angles::to_degrees(bounds.min_position_));
        spinbox->setMaximum(
                smpl::angles::to_degrees(bounds.max_position_));
        spinbox->setSingleStep(1.0);
        spinbox->setWrapping(false);
        spinbox->setSuffix(QChar(0260));
        return spinbox;
    } else {
        return CreateAngleVariableSpinBox();
    }
}

////////////////////////////////////////
// JointVariableCommandWidget Methods //
////////////////////////////////////////

JointVariableCommandWidget::JointVariableCommandWidget(
    RobotCommandModel* model,
    QWidget* parent)
:
    QWidget(parent),
    m_model(model),
    m_ignore_sync(false)
{
    auto* glayout = new QGridLayout;
    setLayout(glayout);

    m_joint_groups_combo_box = new QComboBox;
    m_joint_group_states_combo_box = new QComboBox;

    updateJointGroupControls();

    connect(m_model, SIGNAL(robotLoaded()), this, SLOT(updateRobotModel()));
    connect(m_model, SIGNAL(robotStateChanged()), this, SLOT(updateRobotState()));

    // TODO: lazily connect these signals when we're certain we have a robot
    // model?

    connect(m_joint_groups_combo_box,
            SIGNAL(currentIndexChanged(const QString&)),
            this,
            SLOT(setJointGroup(const QString&)));
    connect(m_joint_group_states_combo_box,
            SIGNAL(currentIndexChanged(const QString&)),
            this,
            SLOT(setNamedState(const QString&)));
}

JointVariableCommandWidget::~JointVariableCommandWidget()
{
    // TODO: are the spinboxes/labels that are not currently attached to the
    // widget deleted when this entire widget is deleted? Probably not, and in
    // that case we may have to delete them manually
}

// Display the joint variable controls for the active joint group. Joint variable
// controls will be displayed if the following conditions are met:
//
// (1) A robot model has been loaded into the associated RobotCommandModel.
// (2) An active joint group is selected. This should always be true if there
//     is at least one group in the robot model.
//
// In this scenario, we assume that the controls have already been set up (when
// we receive a robotLoaded() signal), and we simply need to add the controls to
// the layout and display them.
void JointVariableCommandWidget::updateJointGroupControls()
{
    ROS_DEBUG_NAMED(LOG, "Display joint variable controls for active joint group");
    auto& robot_model = m_model->getRobotModel();
    if (!robot_model) {
        ROS_DEBUG_NAMED(LOG, " -> Robot has not been loaded");
        return;
    }

    if (m_active_joint_group.empty()) {
        ROS_DEBUG_NAMED(LOG, " -> No active joint group set");
        return;
    }

    auto* jmg = robot_model->getJointModelGroup(m_active_joint_group);
    if (!jmg) {
        // NOTE: This should always be true, since we check for existence of
        // the active joint group
        ROS_DEBUG_NAMED(LOG, " -> Joint Group '%s' does not exist within Robot Model '%s'", m_active_joint_group.c_str(), robot_model->getName().c_str());
        return;
    }

    auto* glayout = qobject_cast<QGridLayout*>(layout());
    assert(glayout != NULL);

    ////////////////////////////////////////
    // Clear widgets from from the layout //
    ////////////////////////////////////////

    ROS_DEBUG_NAMED(LOG, "  Remove everything from layout");

    while (!glayout->isEmpty()) {
        auto* item = glayout->takeAt(0);
        if (item->widget()) {
            item->widget()->hide();
            glayout->removeWidget(item->widget());
        } else {
            ROS_WARN("Here there be layouts?!");
            glayout->removeItem(item);
            delete item;
        }
    }

    // Add the master label and group combo box back in.
    glayout->addWidget(new QLabel(tr("Joint Group:")), 0, 0);
    glayout->addWidget(m_joint_groups_combo_box, 0, 1);
    m_joint_groups_combo_box->show(); // this gets hidden above

    // Add the named state combo box in if we have named states for this group.
    if (!jmg->getDefaultStateNames().empty()) {
        glayout->addWidget(new QLabel(tr("Named State:")), 1, 0);
        glayout->addWidget(m_joint_group_states_combo_box, 1, 1);
        m_joint_group_states_combo_box->show();
    }

    /////////////////////////////////
    // Add joint variable controls //
    /////////////////////////////////

    ROS_DEBUG_NAMED(LOG, "  Add joint group controls to layout");

    // gather (label, spinbox) pairs to be visible in the layout
    // TODO: this could probably be made more better since spinboxes are
    // constructed per-joint and should be able to be looked up from the joint
    // they derive from
    std::vector<std::pair<QLabel*, QDoubleSpinBox*>> group_widgets;
    auto& joint_models = jmg->getActiveJointModels();
    for (auto* joint_model : joint_models) {
        auto bidx = joint_model->getFirstVariableIndex();
        for (auto vidx = bidx; vidx < bidx + joint_model->getVariableCount(); ++vidx) {
            auto& var_name = robot_model->getVariableNames()[vidx];
            assert(vidx >= 0 && vidx < m_vind_to_spinbox.size());
            for (auto* spinbox : m_vind_to_spinbox[vidx]) {
                // find the label corresponding to this spinbox
                auto it = std::find(m_spinboxes.begin(), m_spinboxes.end(), spinbox);
                auto sidx = std::distance(m_spinboxes.begin(), it);
                group_widgets.emplace_back(m_labels[sidx], spinbox);
            }
        }
    }

    // erase duplicate entries...see above for insight to why this is needed
    auto uit = std::unique(group_widgets.begin(), group_widgets.end());
    group_widgets.erase(uit, group_widgets.end());

    // add (label, spinbox) widgets to the layout
    auto row = glayout->rowCount();
    for (auto& entry : group_widgets) {
        ROS_DEBUG_NAMED(LOG, "  Add spinbox %p and label %p to layout", entry.first, entry.second);
        glayout->addWidget(entry.first, row, 0);
        glayout->addWidget(entry.second, row, 1);
        entry.first->show();
        entry.second->show();
        ++row;
    }

    ROS_DEBUG_NAMED(LOG, "  Layout contains %d controls", glayout->count());

    ///////////////////////////////////////////////////////////////////////
    // KEEP THESE CALLS BECAUSE THEY USED TO BE THE ONLY WAY TO AUTOMATIC
    // RESIZING WORK AND THEY HAVEN'T BROKEN YET

//    for (int i = 0; i < glayout->count(); ++i) {
//        glayout->itemAt(i)->invalidate();
//    }
//
//    glayout->invalidate();
    layout()->invalidate();
//    glayout->activate();
//    layout()->activate();
//    glayout->update();
    layout()->update();

    //////////////////////////////////////////////////////////////////////
}

// Set the active joint group and named state, and display controls for group
// variables
void JointVariableCommandWidget::setActiveJointGroup(
    const std::string& group_name)
{
    if (m_active_joint_group != group_name) {
        ROS_DEBUG_NAMED(LOG, "Update active joint group '%s' -> '%s'", m_active_joint_group.c_str(), group_name.c_str());
        auto& robot_model = m_model->getRobotModel();
        auto* group = robot_model ?
                robot_model->getJointModelGroup(group_name) : NULL;

        if (robot_model != NULL && group == NULL) {
            m_active_joint_group = "";
        } else {
            m_active_joint_group = group_name;
        }

        auto idx = m_joint_groups_combo_box->findText(QString::fromStdString(group_name));
        m_joint_groups_combo_box->setCurrentIndex(idx);

        // Update the named state selection to reflect the state in the command
        if (group != NULL) {
            // Disconnect signals here. We don't want to change the robot state
            // by selecting a named state without the user explicitly selecting
            // it from the combo box
            disconnect(
                    m_joint_group_states_combo_box,
                    SIGNAL(currentIndexChanged(const QString&)),
                    this,
                    SLOT(setNamedState(const QString&)));

            m_joint_group_states_combo_box->clear();
            for (auto& state_name : group->getDefaultStateNames()) {
                m_joint_group_states_combo_box->addItem(QString::fromStdString(state_name));
            }

            auto named_state = m_model->getGroupStateName(group);
            if (named_state.empty()) {
                m_joint_group_states_combo_box->setCurrentIndex(-1);
            } else {
                auto q_named_state = QString::fromStdString(named_state);
                auto idx = m_joint_group_states_combo_box->findText(q_named_state);
                m_joint_group_states_combo_box->setCurrentIndex(idx);
            }

            connect(m_joint_group_states_combo_box,
                    SIGNAL(currentIndexChanged(const QString&)),
                    this,
                    SLOT(setNamedState(const QString&)));
        }

        updateJointGroupControls();

        Q_EMIT updateActiveJointGroup(m_active_joint_group);
    }
}

void JointVariableCommandWidget::setActiveNamedState(
    const std::string& state_name)
{
    ROS_DEBUG_NAMED(LOG, "Set Active Named State '%s'", state_name.c_str());
    auto& robot_model = m_model->getRobotModel();
    if (robot_model == NULL) return;

    auto* group = robot_model->getJointModelGroup(m_active_joint_group);
    if (group == NULL) return;

    if (m_model->getGroupStateName(group) != state_name) {
        ROS_DEBUG_NAMED(LOG, "Update named state '%s' -> '%s'", m_model->getGroupStateName(group).c_str(), state_name.c_str());

        if (!state_name.empty()) {
            m_model->setToDefaultValues(group, state_name);
        }
    }

    auto idx = m_joint_group_states_combo_box->findText(QString::fromStdString(m_model->getGroupStateName(group)));
    ROS_DEBUG_NAMED(LOG, "Set combo box to item '%s' at index %d", m_model->getGroupStateName(group).c_str(), idx);
    m_joint_group_states_combo_box->setCurrentIndex(idx);
}

/// Update the spinboxes and group selections for a new robot. Creates a new set
/// of spinboxes, associated labels, and creates the bidrectional mapping
/// between spinboxes and the joint variables they control. Also populates the
/// joint group combo box with the available joint groups in the Robot Model.
/// The selected joint group in the combo box will be the first joint group in
/// the Robot Model afterwards
void JointVariableCommandWidget::updateRobotModel()
{
    ROS_DEBUG_NAMED(LOG, "Update JointVariableCommandWidget after Robot Model update");

    auto& robot_model = m_model->getRobotModel();

    // clear current gui
    m_spinbox_to_vind.clear();
    auto var_count = robot_model ? robot_model->getVariableCount() : 0;
    m_vind_to_spinbox.resize(var_count);
    m_spinboxes.clear(); // TODO: disconnect/delete old spinboxes?
    m_labels.clear();

    ROS_DEBUG_NAMED(LOG, "  Remove previous entries from combo box");
    m_joint_groups_combo_box->clear();

    ROS_DEBUG_NAMED(LOG, "  Remove previous named state entries from combo box");
    m_joint_group_states_combo_box->clear();

    if (!robot_model) {
        return;
    }

    ///////////////////////////////////
    // Build joint variable controls //
    ///////////////////////////////////

    ROS_DEBUG_NAMED(LOG, "  Build joint variable controls");

    for (auto* jm : robot_model->getActiveJointModels()) {
        switch (jm->getType()) {
        case moveit::core::JointModel::FLOATING: {
            auto* trans_x_spinbox = CreateRealVariableSpinBox(
                    robot_model->getVariableBounds(jm->getName() + "/trans_x"));
            auto* trans_y_spinbox = CreateRealVariableSpinBox(
                    robot_model->getVariableBounds(jm->getName() + "/trans_y"));
            auto* trans_z_spinbox = CreateRealVariableSpinBox(
                    robot_model->getVariableBounds(jm->getName() + "/trans_y"));
            auto* rot_R_spinbox = CreateRollVariableSpinBox();
            auto* rot_P_spinbox = CreatePitchVariableSpinBox();
            auto* rot_Y_spinbox = CreateYawVariableSpinBox();

            m_spinboxes.push_back(trans_x_spinbox);
            m_spinboxes.push_back(trans_y_spinbox);
            m_spinboxes.push_back(trans_z_spinbox);
            m_spinboxes.push_back(rot_R_spinbox);
            m_spinboxes.push_back(rot_P_spinbox);
            m_spinboxes.push_back(rot_Y_spinbox);
            m_labels.push_back(new QLabel(QString::fromStdString(jm->getName() + "/trans_x")));
            m_labels.push_back(new QLabel(QString::fromStdString(jm->getName() + "/trans_y")));
            m_labels.push_back(new QLabel(QString::fromStdString(jm->getName() + "/trans_z")));
            m_labels.push_back(new QLabel(QString::fromStdString(jm->getName() + "/rot_R")));
            m_labels.push_back(new QLabel(QString::fromStdString(jm->getName() + "/rot_P")));
            m_labels.push_back(new QLabel(QString::fromStdString(jm->getName() + "/rot_Y")));

            m_spinbox_to_vind[trans_x_spinbox] = std::vector<int>(1);
            m_spinbox_to_vind[trans_y_spinbox] = std::vector<int>(1);
            m_spinbox_to_vind[trans_z_spinbox] = std::vector<int>(1);

            // indices for variables (qw, qx, qy, qz) of this joint
            m_spinbox_to_vind[rot_R_spinbox] = std::vector<int>(4);
            m_spinbox_to_vind[rot_P_spinbox] = std::vector<int>(4);
            m_spinbox_to_vind[rot_Y_spinbox] = std::vector<int>(4);

            // map joint variable indices to spinboxes and vice versa
            for (auto vidx = jm->getFirstVariableIndex();
                vidx < jm->getFirstVariableIndex() + jm->getVariableCount();
                ++vidx)
            {
                auto lvidx = vidx - jm->getFirstVariableIndex();
                // check which variable it is (makes no assumption about the
                // order of variables within the joint) but does make
                // assumptions about the names of the variables ...expects
                // standard moveit naming conventions for floating joint
                // variables
                auto& var_name = jm->getLocalVariableNames()[lvidx];
                if (var_name == "trans_x") {
                    m_vind_to_spinbox[vidx] = { trans_x_spinbox };
                    m_spinbox_to_vind[trans_x_spinbox][0] = vidx;
                } else if (var_name == "trans_y") {
                    m_vind_to_spinbox[vidx] = { trans_y_spinbox };
                    m_spinbox_to_vind[trans_y_spinbox][0] = vidx;
                } else if (var_name == "trans_z") {
                    m_vind_to_spinbox[vidx] = { trans_z_spinbox };
                    m_spinbox_to_vind[trans_z_spinbox][0] = vidx;
                } else if (var_name == "rot_w") {
                    m_vind_to_spinbox[vidx] =
                            { rot_R_spinbox, rot_P_spinbox, rot_Y_spinbox };
                    m_spinbox_to_vind[rot_R_spinbox][0] = vidx;
                    m_spinbox_to_vind[rot_P_spinbox][0] = vidx;
                    m_spinbox_to_vind[rot_Y_spinbox][0] = vidx;
                } else if (var_name == "rot_x") {
                    m_vind_to_spinbox[vidx] =
                            { rot_R_spinbox, rot_P_spinbox, rot_Y_spinbox };
                    m_spinbox_to_vind[rot_R_spinbox][1] = vidx;
                    m_spinbox_to_vind[rot_P_spinbox][1] = vidx;
                    m_spinbox_to_vind[rot_Y_spinbox][1] = vidx;
                } else if (var_name == "rot_y") {
                    m_vind_to_spinbox[vidx] =
                            { rot_R_spinbox, rot_P_spinbox, rot_Y_spinbox };
                    m_spinbox_to_vind[rot_R_spinbox][2] = vidx;
                    m_spinbox_to_vind[rot_P_spinbox][2] = vidx;
                    m_spinbox_to_vind[rot_Y_spinbox][2] = vidx;
                } else if (var_name == "rot_z") {
                    m_vind_to_spinbox[vidx] =
                            { rot_R_spinbox, rot_P_spinbox, rot_Y_spinbox };
                    m_spinbox_to_vind[rot_R_spinbox][3] = vidx;
                    m_spinbox_to_vind[rot_P_spinbox][3] = vidx;
                    m_spinbox_to_vind[rot_Y_spinbox][3] = vidx;
                } else {
                    ROS_ERROR("Unrecognized local joint variable name");
                }
            }
        }   break;
        case moveit::core::JointModel::PLANAR: {
            auto* x_spinbox = CreateRealVariableSpinBox(
                    robot_model->getVariableBounds(jm->getName() + "/x"));
            auto* y_spinbox = CreateRealVariableSpinBox(
                    robot_model->getVariableBounds(jm->getName() + "/y"));
            auto* theta_spinbox = CreateAngleVariableSpinBox();

            m_spinboxes.push_back(x_spinbox);
            m_spinboxes.push_back(y_spinbox);
            m_spinboxes.push_back(theta_spinbox);
            m_labels.push_back(new QLabel(QString::fromStdString(jm->getName() + "/x")));
            m_labels.push_back(new QLabel(QString::fromStdString(jm->getName() + "/y")));
            m_labels.push_back(new QLabel(QString::fromStdString(jm->getName() + "/theta")));

            for (auto vidx = jm->getFirstVariableIndex();
                vidx < jm->getFirstVariableIndex() + jm->getVariableCount();
                ++vidx)
            {
                auto lvidx = vidx - jm->getFirstVariableIndex();
                auto& var_name = jm->getLocalVariableNames()[lvidx];
                if (var_name == "x") {
                    m_vind_to_spinbox[vidx] = { x_spinbox };
                    m_spinbox_to_vind[x_spinbox] = { vidx} ;
                } else if (var_name == "y") {
                    m_vind_to_spinbox[vidx] = { y_spinbox };
                    m_spinbox_to_vind[y_spinbox] = { vidx };
                } else if (var_name == "theta") {
                    m_vind_to_spinbox[vidx] = { theta_spinbox };
                    m_spinbox_to_vind[theta_spinbox] = { vidx };
                } else {
                    ROS_ERROR("Unrecognized local joint variable name");
                }
            }
        }   break;
        case moveit::core::JointModel::PRISMATIC: {
            auto* spinbox = CreateRealVariableSpinBox(jm->getVariableBounds()[0]);
            m_spinboxes.push_back(spinbox);
            m_labels.push_back(new QLabel(QString::fromStdString(jm->getName())));
            auto vidx = jm->getFirstVariableIndex();
            m_vind_to_spinbox[vidx] = { spinbox };
            m_spinbox_to_vind[spinbox] = { vidx };
        }   break;
        case moveit::core::JointModel::REVOLUTE: {
            auto* spinbox = CreateRevoluteVariableSpinBox(jm->getVariableBounds()[0]);
            m_spinboxes.push_back(spinbox);
            m_labels.push_back(new QLabel(QString::fromStdString(jm->getName())));
            auto vidx = jm->getFirstVariableIndex();
            m_vind_to_spinbox[vidx] = { spinbox };
            m_spinbox_to_vind[spinbox] = { vidx };
        }   break;
        case moveit::core::JointModel::FIXED:
        case moveit::core::JointModel::UNKNOWN: {
            ROS_ERROR("Invalid joint type %d found in active joint models", (int)jm->getType());
        }   break;
        }
    }

    // Connect all spinboxes to the same slot for setting joint variable
    // positions. We'll later use the sender address to determine which joint
    // variable is modified.
    for (auto* spinbox : m_spinboxes) {
        connect(spinbox,
                SIGNAL(valueChanged(double)),
                this,
                SLOT(setJointVariableFromSpinBox(double)));
    }

    ////////////////////////////////////////
    // Reinitialize Joint Group Combo Box //
    ////////////////////////////////////////

    // NOTE: Update the combo box after the variable spinboxes have been created
    // since setJointGroup will be called by signals emitted by the combo box.

    ROS_DEBUG_NAMED(LOG, "  Add joint group entries to combo box:");
    for (auto& group_name : robot_model->getJointModelGroupNames()) {
        ROS_DEBUG_NAMED(LOG, "    %s", group_name.c_str());
        m_joint_groups_combo_box->addItem(QString::fromStdString(group_name));
    }

    if (m_active_joint_group.empty()) {
        ROS_DEBUG_NAMED(LOG, "  No active joint group selected");
        // clear the default selected
        m_joint_groups_combo_box->setCurrentIndex(-1);
    } else {
        ROS_DEBUG_NAMED(LOG, "  Attempt to select joint group %s", m_active_joint_group.c_str());
        auto* group = robot_model->getJointModelGroup(m_active_joint_group);
        if (group != NULL) {
            ROS_DEBUG_NAMED(LOG, "  Select joint group %s", m_active_joint_group.c_str());
            auto active_joint_group = QString::fromStdString(m_active_joint_group);
            int ajg_idx = m_joint_groups_combo_box->findText(active_joint_group);
            m_joint_groups_combo_box->setCurrentIndex(ajg_idx);
        } else {
            ROS_DEBUG_NAMED(LOG, "  Joint group does not exist");
            m_active_joint_group = "";
            m_joint_groups_combo_box->setCurrentIndex(-1);
        }
    }
}

// Update all spinbox controls to reflect changes to the command state.
void JointVariableCommandWidget::updateRobotState()
{
    ROS_DEBUG_NAMED(LOG, "Update JointVariableCommandWidget after Robot State update");

    if (m_ignore_sync) {
        return;
    }

    ROS_DEBUG_NAMED(LOG, "  Sync spinboxes");
    auto& robot_model = m_model->getRobotModel();

    if (robot_model == NULL) {
        ROS_WARN("Robot not yet loaded");
        return;
    }

    auto* robot_state = m_model->getRobotState();
    assert(robot_state != NULL);

    ////////////////////////////////
    // Update values in spinboxes //
    ////////////////////////////////

    // ugh this needs to be batched
    for (auto* jm : robot_model->getActiveJointModels()) {
        if (jm->getType() == moveit::core::JointModel::FLOATING) {
            // only floating joints have many-to-many joint variable to
            // spinbox mappings
            std::vector<QDoubleSpinBox*> qspinboxes;
            // set the values of the translation variable spinboxes and gather
            // the rotation quaternion values
            double qvars[4];
            auto bidx = jm->getFirstVariableIndex();
            for (auto vi = bidx; vi < bidx + jm->getVariableCount(); ++vi) {
                auto lvi = vi - jm->getFirstVariableIndex();
                auto& var_name = jm->getLocalVariableNames()[lvi];
                if (var_name == "trans_x" ||
                    var_name == "trans_y" ||
                    var_name == "trans_z")
                {
                    assert(m_vind_to_spinbox[vi].size() == 1);
                    auto* spinbox = m_vind_to_spinbox[vi][0];
                    auto value = robot_state->getVariablePosition(vi);
                    if (value != spinbox->value()) {
                        disconnect(
                                spinbox, SIGNAL(valueChanged(double)),
                                this, SLOT(setJointVariableFromSpinBox(double)));
                        spinbox->setValue(value);
                        connect(
                                spinbox, SIGNAL(valueChanged(double)),
                                this, SLOT(setJointVariableFromSpinBox(double)));
                    }
                } else if (var_name == "rot_w") {
                    assert(m_vind_to_spinbox[vi].size() == 3);
                    qspinboxes = m_vind_to_spinbox[vi];
                    qvars[0] = robot_state->getVariablePosition(vi);
                } else if (var_name == "rot_x") {
                    qvars[1] = robot_state->getVariablePosition(vi);
                } else if (var_name == "rot_y") {
                    qvars[2] = robot_state->getVariablePosition(vi);
                } else if (var_name == "rot_z") {
                    qvars[3] = robot_state->getVariablePosition(vi);
                }
            }

            ROS_DEBUG_NAMED(LOG, "  Sync rpy from quaternion (%0.3f, %0.3f, %0.3f, %0.3f)", qvars[0], qvars[1], qvars[2], qvars[3]);
            // simultaneously set the values for the rpy spinboxes
            Eigen::Isometry3d rot(Eigen::Quaterniond(
                    qvars[0], qvars[1], qvars[2], qvars[3]));
            Eigen::Vector3d ypr;
            smpl::angles::get_euler_zyx(rot.rotation(), ypr[0], ypr[1], ypr[2]);

            Eigen::Isometry3d A(
                Eigen::AngleAxisd(ypr[0], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(ypr[1], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(ypr[2], Eigen::Vector3d::UnitX()));

            auto are_diff = A.isApprox(rot, 0.001);

            // round to the nearest degree to try and get more stable results
            auto vY = std::round(smpl::angles::to_degrees(ypr[0]));
            auto vP = std::round(smpl::angles::to_degrees(ypr[1]));
            auto vR = std::round(smpl::angles::to_degrees(ypr[2]));
            // break the cycle
            if ((qspinboxes[0]->value() != vR ||
                qspinboxes[1]->value() != vP ||
                qspinboxes[2]->value() != vY) && are_diff)
            {
                disconnect(qspinboxes[0], SIGNAL(valueChanged(double)),
                        this, SLOT(setJointVariableFromSpinBox(double)));
                disconnect(qspinboxes[1], SIGNAL(valueChanged(double)),
                        this, SLOT(setJointVariableFromSpinBox(double)));
                disconnect(qspinboxes[2], SIGNAL(valueChanged(double)),
                        this, SLOT(setJointVariableFromSpinBox(double)));
                ROS_DEBUG_NAMED(LOG, "sync rpy spinboxes with values (%0.3f, %0.3f, %0.3f)", vR, vP, vY);
                qspinboxes[0]->setValue(vR);
                qspinboxes[1]->setValue(vP);
                qspinboxes[2]->setValue(vY);
                connect(qspinboxes[0], SIGNAL(valueChanged(double)),
                        this, SLOT(setJointVariableFromSpinBox(double)));
                connect(qspinboxes[1], SIGNAL(valueChanged(double)),
                        this, SLOT(setJointVariableFromSpinBox(double)));
                connect(qspinboxes[2], SIGNAL(valueChanged(double)),
                        this, SLOT(setJointVariableFromSpinBox(double)));
            }
        } else {
            auto bidx = jm->getFirstVariableIndex();
            for (auto vi = bidx; vi < bidx + jm->getVariableCount(); ++vi) {
                // all others should have a 1-1 mapping
                assert(m_vind_to_spinbox[vi].size() == 1);
                auto* spinbox = m_vind_to_spinbox[vi][0];
                if (IsVariableAngle(*robot_model, vi)) {
                    auto value = smpl::angles::to_degrees(
                            robot_state->getVariablePosition(vi));
                    if (value != spinbox->value()) {
                        disconnect(spinbox, SIGNAL(valueChanged(double)), this, SLOT(setJointVariableFromSpinBox(double)));
                        spinbox->setValue(value);
                        connect(spinbox, SIGNAL(valueChanged(double)), this, SLOT(setJointVariableFromSpinBox(double)));
                    }
                } else {
                    auto value = robot_state->getVariablePosition(vi);
                    // this check is required because the internal value of
                    // the spinbox may differ from the displayed value.
                    // Apparently, scrolling the spinbox by a step less than
                    // the precision will update the internal value, but
                    // calling setValue will ensure that the internal value
                    // is the same as the value displayed. The absence of
                    // this check can result in not being able to update a
                    // joint variable
                    if (value != spinbox->value()) {
                        disconnect(spinbox, SIGNAL(valueChanged(double)), this, SLOT(setJointVariableFromSpinBox(double)));
                        spinbox->setValue(value);
                        connect(spinbox, SIGNAL(valueChanged(double)), this, SLOT(setJointVariableFromSpinBox(double)));
                    }
                }
            }
        }
    }

    ///////////////////////////////
    // Update active named state //
    ///////////////////////////////

    auto* group = m_model->getRobotModel()->getJointModelGroup(m_active_joint_group);
    if (group != NULL) {
        this->setActiveNamedState(m_model->getGroupStateName(group));
    }

    ROS_DEBUG_NAMED(LOG, "Finished synchronizing spinboxes");
}

void JointVariableCommandWidget::setJointGroup(const QString& group_name)
{
    auto group_name_str = group_name.toStdString();
    setActiveJointGroup(group_name_str);
}

void JointVariableCommandWidget::setNamedState(const QString& state_name)
{
    auto state_name_str = state_name.toStdString();
    setActiveNamedState(state_name_str);
}

/// Update the model variable from a spinbox
void JointVariableCommandWidget::setJointVariableFromSpinBox(double value)
{
    auto* spinbox = qobject_cast<QDoubleSpinBox*>(sender());
    if (!spinbox) {
        ROS_WARN("setJointVariableFromSpinBox not called from a spinbox");
        return;
    }

    assert(m_spinbox_to_vind.find(spinbox) != m_spinbox_to_vind.end());

    auto& vindices = m_spinbox_to_vind[spinbox];

    auto& robot_model = m_model->getRobotModel();

    ROS_DEBUG_STREAM_NAMED(LOG, "Update joint variables " << vindices << " from spinbox " << spinbox << " with value " << value);

    if (vindices.size() == 4) {
        // so much hackery here for quaternion controls
        // need to get the values from the other spinboxes
        auto& rpy_controls = m_vind_to_spinbox[vindices[0]];

        ROS_DEBUG_STREAM_NAMED(LOG, "rpy controls: " << rpy_controls);

        // attempt stability
        auto r = std::round(rpy_controls[0]->value());
        auto p = std::round(rpy_controls[1]->value());
        auto y = std::round(rpy_controls[2]->value());
        ROS_DEBUG_NAMED(LOG, "set rpy values from spinbox values (%0.3f, %0.3f, %0.3f)", r, p, y);
        Eigen::Quaterniond rot(
                Eigen::AngleAxisd(smpl::angles::to_radians(y), Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(smpl::angles::to_radians(p), Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(smpl::angles::to_radians(r), Eigen::Vector3d::UnitX()));
        ROS_DEBUG_NAMED(LOG, "  set quaternion (%0.3f, %0.3f, %0.3f, %0.3f)", rot.w(), rot.x(), rot.y(), rot.z());
        m_ignore_sync = true;
        m_model->setVariablePosition(vindices[0], rot.w());
        m_model->setVariablePosition(vindices[1], rot.x());
        m_model->setVariablePosition(vindices[2], rot.y());
        m_model->setVariablePosition(vindices[3], rot.z());
        m_ignore_sync = false;
    } else if (vindices.size() == 1) {
        // everything else
        if (IsVariableAngle(*robot_model, vindices[0])) {
            // convert to radians and assign
            m_model->setVariablePosition(vindices[0], smpl::angles::to_radians(value));
        } else {
            // assign without conversion
            m_model->setVariablePosition(vindices[0], value);
        }
    } else {
        ROS_ERROR("weird");
    }
}

} // namespace sbpl_interface
