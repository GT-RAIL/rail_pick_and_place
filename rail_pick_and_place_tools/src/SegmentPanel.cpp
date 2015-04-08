/*!
 * \file SegmentPanel.cpp
 * \brief RViz plugin for point cloud segmentation.
 *
 * The segmentation panel allows for sending segmentation service requests.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 6, 2015
 */

// RAIL Pick and Place Tools
#include "rail_pick_and_place_tools/SegmentPanel.h"

// ROS
#include <pluginlib/class_list_macros.h>
#include <std_srvs/Empty.h>

// Qt
#include <QBoxLayout>

using namespace std;
using namespace rail::pick_and_place;

SegmentPanel::SegmentPanel(QWidget *parent) : rviz::Panel(parent)
{
  // set defaults
  string segment_service("/segmentation/segment");

  // get the segmentation service
  node_.getParam("/rviz/segment_panel/segment_service", segment_service);

  // setup the segmentation service
  segment_srv_ = node_.serviceClient<std_srvs::Empty>(segment_service);

  // main segmentation button
  segment_button_ = new QPushButton("Segment");

  // service label
  string service_label_text = "Calling on " + segment_service;
  QLabel *segment_label = new QLabel(service_label_text.c_str());
  segment_label->setAlignment(Qt::AlignCenter);

  // service feedback
  segment_status_ = new QLabel("Ready to segment.");
  segment_status_->setAlignment(Qt::AlignCenter);

  // build the final layout
  QVBoxLayout *layout = new QVBoxLayout();
  layout->addWidget(segment_button_);
  layout->addWidget(segment_label);
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

// tell pluginlib about this class (must outside of any namespace scope)
PLUGINLIB_EXPORT_CLASS(rail::pick_and_place::SegmentPanel, rviz::Panel)
