/*!
 * \VisionPanel.h
 * \brief Rviz plugin for point cloud segmentation and recognition.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date March 11, 2015
 */
#ifndef RAIL_PICK_AND_PLACE_SEGMENT_PANEL_H_
#define RAIL_PICK_AND_PLACE_SEGMENT_PANEL_H_

// ROS
#include <ros/ros.h>
#include <rviz/panel.h>

// Qt
#include <QLabel>
#include <QPushButton>

namespace rail
{
namespace pick_and_place
{

class SegmentPanel : public rviz::Panel
{

// this class uses Qt slots and is a subclass of QObject, so it needs the Q_OBJECT macro
Q_OBJECT

public:
  /**
  * \brief constructor
  * @param parent parent widget
  */
  SegmentPanel(QWidget *parent = NULL);

  /**
  * \brief rviz load function
  * @param config rviz configuration
  */
  virtual void load(const rviz::Config &config);

  /**
  * \brief rviz save function
  * @param config rviz configuration
  */
  virtual void save(rviz::Config config) const;

private:
  /*! The ROS node handle. */
  ros::NodeHandle node_;
  /*! The main segmentation service. */
  ros::ServiceClient segment_srv_;

  /*! The main segmentation button. */
  QPushButton *segment_button_;
  /*! The segmentation service. */
  QLabel *segment_status_;

// used as UI callbacks
private Q_SLOTS:

  /**
  * \brief Call segmentation service and update the interface accordingly
  */
  void executeSegment();
};

}
}

#endif
