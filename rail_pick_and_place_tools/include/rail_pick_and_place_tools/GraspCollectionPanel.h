#ifndef GRASP_COLLECTION_PANEL_H
#define GRASP_COLLECTION_PANEL_H

# include <ros/ros.h>
# include <rviz/panel.h>

#include <actionlib/client/simple_action_client.h>
#include <rail_pick_and_place_msgs/GraspAndStoreAction.h>

#include <QCheckBox>
#include <QComboBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QSpinBox>

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
  GraspCollectionPanel(QWidget *parent = 0);

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load(const rviz::Config &config);

  virtual void save(rviz::Config config) const;

protected:
  // List of grasps and models
  QCheckBox *lift_box_;
  QCheckBox *verify_box_;
  QLabel *grasp_and_store_status_;
  QLineEdit *name_input_;
  QPushButton *grasp_and_store_button_;

protected
  Q_SLOTS:
  void

  executeGraspAndStore();

  void displayModel();

  void deselectAll();

private:
  actionlib::SimpleActionClient <rail_pick_and_place_msgs::GraspAndStoreAction> ac_grasp_and_store_;

  // The ROS node handle.
  ros::NodeHandle nh_;

  void updateModelInfo();

  void doneCb(const actionlib::SimpleClientGoalState& state, const rail_pick_and_place_msgs::GraspAndStoreResultConstPtr& result);

  void feedbackCb(const rail_pick_and_place_msgs::GraspAndStoreFeedbackConstPtr& feedback);

};

} // end namespace pick_and_place
} // end namespace rail

#endif // GRASP_COLLECTION_PANEL_H
