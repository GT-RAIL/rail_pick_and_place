/*!
 * \GraspCollectionPanel.cpp
 * \brief Rviz plugin for collecting grasp data.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date March 10, 2015
 */

#include <rail_pick_and_place_tools/GraspCollectionPanel.h>

using namespace std;

namespace rail
{
namespace pick_and_place
{

GraspCollectionPanel::GraspCollectionPanel(QWidget *parent) :
    rviz::Panel(parent),
    ac_grasp_and_store_("rail_grasp_collection/grasp_and_store", true)
{
  //grasp collection options
  QHBoxLayout *first_row_layout = new QHBoxLayout;
  QLabel *options_label = new QLabel("Options:");
  lift_box_ = new QCheckBox("Lift objects");
  verify_box_ = new QCheckBox("Verify grasps");
  first_row_layout->addWidget(lift_box_, 0, Qt::AlignLeft);
  first_row_layout->addWidget(verify_box_, 0, Qt::AlignLeft);
  first_row_layout->addStretch();

  //grasp collection input
  QHBoxLayout *second_row_layout = new QHBoxLayout;
  QLabel *name_label = new QLabel("Object name:");
  name_input_ = new QLineEdit;
  grasp_and_store_button_ = new QPushButton("Grasp");
  second_row_layout->addWidget(name_input_);
  second_row_layout->addWidget(grasp_and_store_button_);

  //organizational layout
  QGridLayout *grid_layout = new QGridLayout;
  grid_layout->addWidget(options_label, 0, 0);
  grid_layout->addWidget(name_label, 1, 0);
  grid_layout->addLayout(first_row_layout, 0, 1);
  grid_layout->addLayout(second_row_layout, 1, 1);

  //grasp collection feedback
  grasp_and_store_status_ = new QLabel("Ready to collect grasp data.");
  grasp_and_store_status_->setAlignment(Qt::AlignRight);

  //build final layout
  QVBoxLayout *layout = new QVBoxLayout;
  layout->addLayout(grid_layout);
  layout->addWidget(grasp_and_store_status_);

  //connect things
  QObject::connect(grasp_and_store_button_, SIGNAL(clicked()), this, SLOT(executeGraspAndStore()));

  setLayout(layout);
}

void GraspCollectionPanel::executeGraspAndStore()
{
  if (!ac_grasp_and_store_.isServerConnected())
  {
    grasp_and_store_status_->setText("No grasp and store action server found!");
    return;
  }
  rail_pick_and_place_msgs::GraspAndStoreGoal grasp_and_store_goal;
  grasp_and_store_goal.lift = lift_box_->isChecked();
  grasp_and_store_goal.verify = verify_box_->isChecked();
  grasp_and_store_goal.object_name = name_input_->text().toStdString();
  ac_grasp_and_store_.sendGoal(grasp_and_store_goal, boost::bind(&GraspCollectionPanel::doneCb, this, _1, _2),
      actionlib::SimpleActionClient<rail_pick_and_place_msgs::GraspAndStoreAction>::SimpleActiveCallback(),
      boost::bind(&GraspCollectionPanel::feedbackCb, this, _1));

  grasp_and_store_button_->setEnabled(false);
}

void GraspCollectionPanel::doneCb(const actionlib::SimpleClientGoalState &state, const rail_pick_and_place_msgs::GraspAndStoreResultConstPtr &result)
{
  if (result->success)
  {
    grasp_and_store_status_->setText("Grasp data successfully collected.");
  }
  else
  {
    grasp_and_store_status_->setText("Grasp failed.");
  }

  grasp_and_store_button_->setEnabled(true);
}

void GraspCollectionPanel::feedbackCb(const rail_pick_and_place_msgs::GraspAndStoreFeedbackConstPtr &feedback)
{
  grasp_and_store_status_->setText(feedback->message.c_str());
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void GraspCollectionPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
  if (lift_box_->isChecked())
    config.mapSetValue("LiftEnabled", 1);
  else
    config.mapSetValue("LiftEnabled", 0);
  if (verify_box_->isChecked())
    config.mapSetValue("VerifyEnabled", 1);
  else
    config.mapSetValue("VerifyEnabled", 0);
}

// Load all configuration data for this panel from the given Config object.
void GraspCollectionPanel::load(const rviz::Config &config)
{
  rviz::Panel::load(config);
  int lift_enabled;
  int verify_enabled;
  if (config.mapGetInt("LiftEnabled", &lift_enabled))
  {
    if (lift_enabled == 1)
      lift_box_->setChecked(true);
    else
      lift_box_->setChecked(false);
  }
  if (config.mapGetInt("VerifyEnabled", &verify_enabled))
  {
    if (verify_enabled == 1)
      verify_box_->setChecked(true);
    else
      verify_box_->setChecked(false);
  }
}

}
}
// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rail::pick_and_place::GraspCollectionPanel,rviz::Panel )
