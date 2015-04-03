/*!
 * \ModelGenerationPanel.h
 * \brief Rviz plugin for generating object recognition/grasping models.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date March 2, 2015
 */

#ifndef MODEL_GENERATION_PANEL_H
#define MODEL_GENERATION_PANEL_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <graspdb/graspdb.h>
#include <geometry_msgs/PoseArray.h>
#include <rail_pick_and_place_msgs/GenerateModelsAction.h>
#include <rviz/panel.h>
#include <sensor_msgs/PointCloud2.h>

#include <QComboBox>
#include <QGridLayout>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

class QLineEdit;

namespace rail
{
namespace pick_and_place
{

class ModelGenerationPanel : public rviz::Panel
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
  Q_OBJECT
public:
  /**
  * \brief constructor
  * @param parent parent widget
  */
  ModelGenerationPanel(QWidget *parent = 0);

  /**
  * \brief destructor
  */
  ~ModelGenerationPanel();

  /**
  * \brief rviz load function, will load option check box states
  * @param config rviz configuration
  */
  virtual void load(const rviz::Config &config);

  /**
  * \brief rviz save function, will save option check box states
  * @param config rviz configuration
  */
  virtual void save(rviz::Config config) const;

protected:
  QLabel *model_generation_status_;
  QComboBox *object_list_;
  QListWidget *models_list_;
  QSpinBox *model_size_spinbox_;
  QPushButton *generate_button_;
  QPushButton *remove_button_;

protected
  Q_SLOTS:

  /**
  * \brief Start point cloud registration (model generation), and update the interface accordingly
  *
  * Checked individual grasps and merged models will be used as input for the registration graph.
  */
  void executeRegistration();

  /**
  * \brief Display the currently selected individual grasp or object model
  */
  void displayModel();

  /**
  * \brief Remove the currently selected individual grasp or object model
  */
  void removeModel();

  /**
  * \brief Uncheck any checked items in the models list
  */
  void deselectAll();

  /**
  * \brief Read all grasps and models for the currently selected object and populate the models_list_
  * @text the text of the new item displayed in the box
  */
  void populateModelsList(const QString &text);

  void updateObjectNames();

private:
  actionlib::SimpleActionClient <rail_pick_and_place_msgs::GenerateModelsAction> ac_generate_models;

  ros::Publisher display_cloud_pub;
  ros::Publisher display_grasps_pub;

  // The ROS node handle.
  ros::NodeHandle nh_;

  graspdb::Client *graspdb_;
  std::vector<graspdb::GraspDemonstration> current_demonstrations_;
  std::vector<graspdb::GraspModel> current_models_;

  bool okay_;

  /**
  * \brief Read merged models and add new models to the list
  */
  void updateModelInfo();

  /**
  * \brief Callback for the registration action server finishing
  * @param state goal state
  * @param result registration result
  */
  void doneCb(const actionlib::SimpleClientGoalState &state, const rail_pick_and_place_msgs::GenerateModelsResultConstPtr &result);

  /**
  * \brief Callback for feedback from the registration action server
  * @param feedback registration feedback
  */
  void feedbackCb(const rail_pick_and_place_msgs::GenerateModelsFeedbackConstPtr &feedback);

};

} // end namespace pick_and_place
} // end namespace rail

#endif // MODEL_GENERATION_PANEL_H
