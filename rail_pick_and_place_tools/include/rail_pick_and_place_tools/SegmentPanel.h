/*!
 * \file SegmentPanel.h
 * \brief RViz plugin for point cloud segmentation.
 *
 * The segmentation panel allows for sending segmentation service requests.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 6, 2015
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

/*!
 * \class SegmentPanel
 * \brief RViz plugin for point cloud segmentation.
 *
 * The segmentation panel allows for sending segmentation service requests.
 */
class SegmentPanel : public rviz::Panel
{

// this class uses Qt slots and is a subclass of QObject, so it needs the Q_OBJECT macro
Q_OBJECT

public:
  /*!
   * \brief Create a new SegmentPanel.
   *
   * Creates a new SegmentPanel and adds the correct widgets.
   *
   * \param parent The parent widget for this panel (defaults to NULL).
   */
  SegmentPanel(QWidget *parent = NULL);

  /*!
   * \brief Load RViz configuration settings.
   *
   * No settings are saved or loaded for this panel.
   *
   * \param config The RViz configuration settings to load.
   */
  virtual void load(const rviz::Config &config);

  /*!
   * \brief Save RViz configuration settings.
   *
   * No settings are saved or loaded for this panel.
   *
   * \param config The RViz configuration settings to save.
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

  /*!
   * \brief Callback for the segment button.
   *
   * Calls the segmentation service.
   */
  void executeSegment();
};

}
}

#endif
