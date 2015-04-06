/*!
 * \GraspCollectionPanel.h
 * \brief Rviz plugin for collecting grasp data.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date March 10, 2015
 */

#ifndef RAIL_PICK_AND_PLACE_GRASP_COLLECTION_PANEL_H_
#define RAIL_PICK_AND_PLACE_GRASP_COLLECTION_PANEL_H_

// ROS
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>
#include <rail_pick_and_place_msgs/GraspAndStoreAction.h>
#include <rviz/panel.h>

// QT
#include <QCheckBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

namespace rail
{
namespace pick_and_place
{

class GraspCollectionPanel : public rviz::Panel
{

// this class uses Qt slots and is a subclass of QObject, so it needs the Q_OBJECT macro
Q_OBJECT

public:
  /**
  * \brief Constructor
  * @param parent parent widget
  */
  GraspCollectionPanel(QWidget *parent = NULL);

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

private:
  /**
  * \brief Callback for when the grasp and store action server finishes
  * @param state goal state
  * @param result grasp and store result
  */
  void doneCallback(const actionlib::SimpleClientGoalState &state,
      const rail_pick_and_place_msgs::GraspAndStoreResultConstPtr &result);

  /**
  * \brief Callback for feedback from the grasp and store action server
  * @param feedback grasp and store feedback
  */
  void feedbackCallback(const rail_pick_and_place_msgs::GraspAndStoreFeedbackConstPtr &feedback);

  /*! The main grasp and store action client */
  actionlib::SimpleActionClient<rail_pick_and_place_msgs::GraspAndStoreAction> grasp_and_store_ac_;

  QCheckBox *lift_box_;
  QCheckBox *verify_box_;
  QLabel *grasp_and_store_status_;
  QLineEdit *name_input_;
  QPushButton *grasp_and_store_button_;

// used as UI callbacks
private
  Q_SLOTS:

  /**
  * \brief call the grasp and store action server, updating the interface accordingly
  */
  void executeGraspAndStore();
};

}
}

#endif
