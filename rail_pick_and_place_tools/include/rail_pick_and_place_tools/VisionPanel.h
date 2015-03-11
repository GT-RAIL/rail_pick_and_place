/*!
 * \VisionPanel.h
 * \brief Rviz plugin for point cloud segmentation and recognition.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date March 11, 2015
 */
#ifndef VISION_PANEL_H
#define VISION_PANEL_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/Empty.h>
#include <rail_segmentation/RecognizeAllAction.h>
#include <rviz/panel.h>

#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

class QLineEdit;

namespace rail
{
namespace pick_and_place
{

class VisionPanel : public rviz::Panel
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
  Q_OBJECT
public:
  /**
  * \brief constructor
  * @param parent parent widget
  */
  VisionPanel(QWidget *parent = 0);

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

protected:
  QPushButton *segment_button_;
  QPushButton *recognize_all_button_;
  QLabel *action_status_;

protected
  Q_SLOTS:

  /**
  * \brief Call segmentation service and update the interface accordingly
  */
  void executeSegment();

  /**
  * \brief Start recognizing all of the segmented point clouds and update the interface accordingly
  */
  void executeRecognizeAll();

private:
  ros::ServiceClient segmentClient;

  actionlib::SimpleActionClient <rail_segmentation::RecognizeAllAction> ac_recognize_all_;

  // The ROS node handle.
  ros::NodeHandle nh_;

  /**
  * \brief Callback for when the recognition action finishes
  * @param state goal state
  * @param result recognize all result
  */
  void doneCb(const actionlib::SimpleClientGoalState& state, const rail_segmentation::RecognizeAllResultConstPtr& result);

  /**
  * \brief Callback for feedback from the recognition action server
  * @param feedback recognize all feedback
  */
  void feedbackCb(const rail_segmentation::RecognizeAllFeedbackConstPtr& feedback);

};

} // end namespace pick_and_place
} // end namespace rail

#endif // VISION_PANEL_H
