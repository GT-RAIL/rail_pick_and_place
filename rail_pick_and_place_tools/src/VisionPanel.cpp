/*!
 * \VisionPanel.cpp
 * \brief Rviz plugin for point cloud segmentation and recognition.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date March 11, 2015
 */
#include <rail_pick_and_place_tools/VisionPanel.h>

using namespace std;

namespace rail
{
namespace pick_and_place
{

VisionPanel::VisionPanel(QWidget *parent) :
    rviz::Panel(parent),
    ac_recognize_all_("rail_segmentation/recognize_all", true)
{
  segmentClient = nh_.serviceClient<std_srvs::Empty>("rail_segmentation/segment");

  //buttons and such
  QGridLayout *grid_layout = new QGridLayout;
  QLabel *segment_label = new QLabel(": Automatic object segmentation");
  QLabel *recognize_label = new QLabel(": Recognize all segmented objects");
  segment_button_ = new QPushButton("Segment");
  recognize_all_button_ = new QPushButton("Recognize");
  grid_layout->addWidget(segment_button_, 0, 0);
  grid_layout->addWidget(recognize_all_button_, 1, 0);
  grid_layout->addWidget(segment_label, 0, 1);
  grid_layout->addWidget(recognize_label, 1, 1);

  //action feedback
  action_status_ = new QLabel("Ready to segment and recognize.");

  //build final layout
  QVBoxLayout *layout = new QVBoxLayout;
  layout->addLayout(grid_layout);
  layout->addWidget(action_status_);

  //connect things
  QObject::connect(segment_button_, SIGNAL(clicked()), this, SLOT(executeSegment()));
  QObject::connect(recognize_all_button_, SIGNAL(clicked()), this, SLOT(executeRecognizeAll()));

  setLayout(layout);
}

void VisionPanel::executeSegment()
{
  segment_button_->setEnabled(false);
  recognize_all_button_->setEnabled(false);
  std_srvs::Empty srv;
  if (!segmentClient.call(srv))
  {
    ROS_INFO("Could not call segmentation service.");
    action_status_->setText("Failed to call segmentation service.");
  }
  else
  {
    action_status_->setText("Segmentation complete.");
  }
  segment_button_->setEnabled(true);
  recognize_all_button_->setEnabled(true);
}

void VisionPanel::executeRecognizeAll()
{
  rail_segmentation::RecognizeAllGoal recognize_all_goal;
  ac_recognize_all_.sendGoal(recognize_all_goal, boost::bind(&VisionPanel::doneCb, this, _1, _2),
      actionlib::SimpleActionClient<rail_segmentation::RecognizeAllAction>::SimpleActiveCallback(),
      boost::bind(&VisionPanel::feedbackCb, this, _1));

  segment_button_->setEnabled(false);
  recognize_all_button_->setEnabled(false);
}

void VisionPanel::doneCb(const actionlib::SimpleClientGoalState& state, const rail_segmentation::RecognizeAllResultConstPtr& result)
{
  stringstream ss;
  ss << "Recognized " << result->totalRecognized << " objects.";
  action_status_->setText(ss.str().c_str());

  segment_button_->setEnabled(true);
  recognize_all_button_->setEnabled(true);
}

void VisionPanel::feedbackCb(const rail_segmentation::RecognizeAllFeedbackConstPtr& feedback)
{
  action_status_->setText(feedback->message.c_str());
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void VisionPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}

// Load all configuration data for this panel from the given Config object.
void VisionPanel::load(const rviz::Config &config)
{
  rviz::Panel::load(config);
}

}
}
// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rail::pick_and_place::VisionPanel,rviz::Panel )
