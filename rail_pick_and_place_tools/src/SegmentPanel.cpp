/*!
 * \VisionPanel.cpp
 * \brief Rviz plugin for point cloud segmentation and recognition.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date March 11, 2015
 */

// RAIL Pick and Place Tools
#include "rail_pick_and_place_tools/SegmentPanel.h"

// ROS
#include <pluginlib/class_list_macros.h>
#include <std_srvs/Empty.h>

using namespace std;

namespace rail
{
namespace pick_and_place
{

SegmentPanel::SegmentPanel(QWidget *parent) : rviz::Panel(parent)
{
  // set defaults
  string segment_service("/segmentation/segment");

  // get the segmentation service
  node_.getParam("/rviz/segment_panel/segment_service", segment_service);

  // setup the segmentation service
  segment_srv_ = node_.serviceClient<std_srvs::Empty>(segment_service);

  // final layout
  QVBoxLayout *layout = new QVBoxLayout();

  // main segmentation button
  segment_button_ = new QPushButton("Segment");
  layout->addWidget(segment_button_);

  // service label
  string service_label_text = "Calling on " + segment_service;
  QLabel *segment_label = new QLabel(service_label_text.c_str());
  layout->addWidget(segment_label);

  // service feedback
  segment_status_ = new QLabel("Ready to segment.");
  layout->addWidget(segment_status_);

  // connect the segmentation button
  QObject::connect(segment_button_, SIGNAL(clicked()), this, SLOT(executeSegment()));

  // set the final layout
  this->setLayout(layout);
}

void SegmentPanel::executeSegment()
{
  // disable the button first
  segment_button_->setEnabled(false);

  // check if the service exists
  if (!segment_srv_.exists())
  {
    segment_status_->setText("Failed to call segmentation service.");
  } else
  {
    // attempt to call the service
    std_srvs::Empty srv;
    if (!segment_srv_.call(srv))
    {
      segment_status_->setText("Segmentation failed.");
    }
    else
    {
      segment_status_->setText("Segmentation complete.");
    }
  }

  // re-enable the button
  segment_button_->setEnabled(true);
}

void SegmentPanel::save(rviz::Config config) const
{
  // simply call the super class
  rviz::Panel::save(config);
}

void SegmentPanel::load(const rviz::Config &config)
{
  // simply call the super class
  rviz::Panel::load(config);
}

}
}

// Tell pluginlib about this class (must outside of any namespace scope)
PLUGINLIB_EXPORT_CLASS(rail::pick_and_place::SegmentPanel, rviz::Panel)
