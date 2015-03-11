/*!
 * \GraspCollectionPanel.h
 * \brief Rviz plugin for collecting grasp data.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date March 10, 2015
 */

#ifndef GRASP_COLLECTION_PANEL_H
#define GRASP_COLLECTION_PANEL_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <rail_pick_and_place_msgs/GraspAndStoreAction.h>
#include <rviz/panel.h>

#include <QCheckBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>

class QLineEdit;

namespace rail
{
namespace pick_and_place
{

class GraspCollectionPanel : public rviz::Panel
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
  Q_OBJECT
public:
  /**
  * \brief Constructor
  * @param parent parent widget
  */
  GraspCollectionPanel(QWidget *parent = 0);

  /**
  * \brief rviz load function, will load the max model size
  * @param config rviz configuration
  */
  virtual void load(const rviz::Config &config);

  /**
  * \brief rviz save function, will save the max model size
  * @param config rviz configuration
  */
  virtual void save(rviz::Config config) const;

protected:
  QCheckBox *lift_box_;
  QCheckBox *verify_box_;
  QLabel *grasp_and_store_status_;
  QLineEdit *name_input_;
  QPushButton *grasp_and_store_button_;

protected
  Q_SLOTS:

  /**
  * \brief call the grasp and store action server, updating the interface accordingly
  */
  void executeGraspAndStore();

private:
  actionlib::SimpleActionClient <rail_pick_and_place_msgs::GraspAndStoreAction> ac_grasp_and_store_;

  // The ROS node handle.
  ros::NodeHandle nh_;

  /**
  * \brief Callback for when the grasp and store action server finishes
  * @param state goal state
  * @param result grasp and store result
  */
  void doneCb(const actionlib::SimpleClientGoalState& state, const rail_pick_and_place_msgs::GraspAndStoreResultConstPtr& result);

  /**
  * \brief Callback for feedback from the grasp and store action server
  * @param feedback grasp and store feedback
  */
  void feedbackCb(const rail_pick_and_place_msgs::GraspAndStoreFeedbackConstPtr& feedback);

};

} // end namespace pick_and_place
} // end namespace rail

#endif // GRASP_COLLECTION_PANEL_H
