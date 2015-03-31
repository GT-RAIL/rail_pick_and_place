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
    rviz::Panel(parent)
{
  segmentClient = nh_.serviceClient<std_srvs::Empty>("rail_segmentation/segment");

  //buttons and such
  QGridLayout *grid_layout = new QGridLayout;
  QLabel *segment_label = new QLabel(": Automatic object segmentation");
  segment_button_ = new QPushButton("Segment");
  grid_layout->addWidget(segment_button_, 0, 0);
  grid_layout->addWidget(segment_label, 0, 1);

  //action feedback
  action_status_ = new QLabel("Ready to segment.");

  //build final layout
  QVBoxLayout *layout = new QVBoxLayout;
  layout->addLayout(grid_layout);
  layout->addWidget(action_status_);

  //connect things
  QObject::connect(segment_button_, SIGNAL(clicked()), this, SLOT(executeSegment()));

  setLayout(layout);
}

void VisionPanel::executeSegment()
{
  segment_button_->setEnabled(false);
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
