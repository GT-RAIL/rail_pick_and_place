/*!
 * \file GraspCollectionPanel.cpp
 * \brief RViz plugin for grasp collection.
 *
 * The grasp collection panel allows for calling the grasp and store action server with the associated settings.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 6, 2015
 */

// RAIL Pick and Place Tools
#include <rail_pick_and_place_tools/GraspCollectionPanel.h>

// ROS
#include <pluginlib/class_list_macros.h>

// Qt
#include <QBoxLayout>

using namespace std;
using namespace rail::pick_and_place;

GraspCollectionPanel::GraspCollectionPanel(QWidget *parent)
    : rviz::Panel(parent), grasp_and_store_ac_("/rail_grasp_collection/grasp_and_store", true)
{
  // grasp collection options
  QHBoxLayout *first_row_layout = new QHBoxLayout();
  QLabel *options_label = new QLabel("Options:");
  options_label->setAlignment(Qt::AlignRight);
  lift_box_ = new QCheckBox("Lift Object");
  verify_box_ = new QCheckBox("Verify Grasp");
  first_row_layout->addWidget(lift_box_, 0, Qt::AlignLeft);
  first_row_layout->addWidget(verify_box_, 0, Qt::AlignLeft);
  first_row_layout->addStretch();

  // grasp collection input
  QHBoxLayout *second_row_layout = new QHBoxLayout();
  QLabel *name_label = new QLabel("Object name:");
  name_label->setAlignment(Qt::AlignRight);
  name_input_ = new QLineEdit();
  grasp_and_store_button_ = new QPushButton("Grasp");
  second_row_layout->addWidget(name_input_);
  second_row_layout->addWidget(grasp_and_store_button_);

  // organizational layout
  QGridLayout *grid_layout = new QGridLayout();
  grid_layout->addWidget(options_label, 0, 0);
  grid_layout->addWidget(name_label, 1, 0);
  grid_layout->addLayout(first_row_layout, 0, 1);
  grid_layout->addLayout(second_row_layout, 1, 1);

  // grasp collection feedback
  grasp_and_store_status_ = new QLabel("Ready to collect grasp data.");
  grasp_and_store_status_->setAlignment(Qt::AlignCenter);

  // build final layout
  QVBoxLayout *layout = new QVBoxLayout();
  layout->addLayout(grid_layout);
  layout->addWidget(grasp_and_store_status_);

  // connect the grasp button
  QObject::connect(grasp_and_store_button_, SIGNAL(clicked()), this, SLOT(executeGraspAndStore()));

  // set the final layout
  this->setLayout(layout);
}

void GraspCollectionPanel::executeGraspAndStore()
{
  // disable the button first
  grasp_and_store_button_->setEnabled(false);

  // check if the action server exists
  if (!grasp_and_store_ac_.isServerConnected())
  {
    grasp_and_store_status_->setText("Grasp and store action server not found!");
    // make sure to re-enable the button
    grasp_and_store_button_->setEnabled(true);
  } else
  {
    // set options for the goal from the GUI
    rail_pick_and_place_msgs::GraspAndStoreGoal goal;
    goal.lift = lift_box_->isChecked();
    goal.verify = verify_box_->isChecked();
    goal.object_name = name_input_->text().toStdString();
    // send the goal asynchronously
    grasp_and_store_ac_.sendGoal(goal, boost::bind(&GraspCollectionPanel::doneCallback, this, _1, _2),
        actionlib::SimpleActionClient<rail_pick_and_place_msgs::GraspAndStoreAction>::SimpleActiveCallback(),
        boost::bind(&GraspCollectionPanel::feedbackCallback, this, _1));
  }
}

void GraspCollectionPanel::doneCallback(const actionlib::SimpleClientGoalState &state,
    const rail_pick_and_place_msgs::GraspAndStoreResultConstPtr &result)
{
  // check if the action was successful
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED && result->success)
  {
    // built the final status string
    stringstream ss;
    ss << "Grasp demonstration successfully stored with ID " << result->id << ".";
    grasp_and_store_status_->setText(ss.str().c_str());
  }
  else
  {
    // state text should represent what went wrong
    grasp_and_store_status_->setText(state.getText().c_str());
  }

  // re-enable the button
  grasp_and_store_button_->setEnabled(true);
}

void GraspCollectionPanel::feedbackCallback(const rail_pick_and_place_msgs::GraspAndStoreFeedbackConstPtr &feedback)
{
  // simply set the status to the current message
  grasp_and_store_status_->setText(feedback->message.c_str());
}

void GraspCollectionPanel::save(rviz::Config config) const
{
  // first call the super class
  rviz::Panel::save(config);

  // save the state of the check boxes
  config.mapSetValue("LiftEnabled", lift_box_->isChecked());
  config.mapSetValue("VerifyEnabled", verify_box_->isChecked());
}

void GraspCollectionPanel::load(const rviz::Config &config)
{
  // first call the super class
  rviz::Panel::load(config);

  // check the state of each check box
  bool flag;
  lift_box_->setChecked(config.mapGetBool("LiftEnabled", &flag) && flag);
  verify_box_->setChecked(config.mapGetBool("VerifyEnabled", &flag) && flag);
}

// tell pluginlib about this class (must outside of any namespace scope)
PLUGINLIB_EXPORT_CLASS(rail::pick_and_place::GraspCollectionPanel, rviz::Panel)
