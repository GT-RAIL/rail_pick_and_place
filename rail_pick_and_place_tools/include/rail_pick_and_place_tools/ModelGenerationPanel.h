/*!
 * \ModelGenerationPanel.h
 * \brief Rviz plugin for generating object recognition/grasping models.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date March 2, 2015
 */

#ifndef RAIL_PICK_AND_PLACE_MODEL_GENERATION_PANEL_H_
#define RAIL_PICK_AND_PLACE_MODEL_GENERATION_PANEL_H_

// ROS
#include <actionlib/client/simple_action_client.h>
#include <graspdb/graspdb.h>
#include <rail_pick_and_place_msgs/GenerateModelsAction.h>
#include <rail_pick_and_place_msgs/RetrieveGraspDemonstrationAction.h>
#include <rail_pick_and_place_msgs/RetrieveGraspModelAction.h>
#include <ros/ros.h>
#include <rviz/panel.h>

// Qt
#include <QComboBox>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QSpinBox>

namespace rail
{
namespace pick_and_place
{

class ModelGenerationPanel : public rviz::Panel
{

// this class uses Qt slots and is a subclass of QObject, so it needs the Q_OBJECT macro
Q_OBJECT

public:
  /**
  * \brief constructor
  * @param parent parent widget
  */
  ModelGenerationPanel(QWidget *parent = NULL);

  /**
  * \brief destructor
  */
  virtual ~ModelGenerationPanel();

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

private:
  /**
  * \brief Callback for the registration action server finishing
  * @param state goal state
  * @param result registration result
  */
  void doneCallback(const actionlib::SimpleClientGoalState &state, const rail_pick_and_place_msgs::GenerateModelsResultConstPtr &result);

  /**
  * \brief Callback for feedback from the registration action server
  * @param feedback registration feedback
  */
  void feedbackCallback(const rail_pick_and_place_msgs::GenerateModelsFeedbackConstPtr &feedback);

  /*! The grasp database connection. */
  graspdb::Client *graspdb_;

  /*! The public ROS node handle. */
  ros::NodeHandle node_;
  /*! The generate model action client. */
  actionlib::SimpleActionClient<rail_pick_and_place_msgs::GenerateModelsAction> generate_models_ac_;
  /*! The retrieve grasp demonstration action client. */
  actionlib::SimpleActionClient<rail_pick_and_place_msgs::RetrieveGraspDemonstrationAction> retrieve_grasp_ac_;
  /*! The retrieve grasp model action client. */
  actionlib::SimpleActionClient<rail_pick_and_place_msgs::RetrieveGraspModelAction> retrieve_grasp_model_ac_;

  QLabel *model_generation_status_;
  /*! The current list of objects */
  QComboBox *object_list_;
  QListWidget *models_list_;
  QSpinBox *model_size_spin_box_;
  QPushButton *refresh_button_, *select_all_button_, *deselect_all_button_, *generate_models_button_, *delete_button_;

// used as UI callbacks
private
  Q_SLOTS:

  /**
  * \brief Start point cloud registration (model generation), and update the interface accordingly
  *
  * Checked individual grasps and merged models will be used as input for the registration graph.
  */
  void executeGenerateModels();


  /**
  * \brief Remove the currently selected individual grasp or object model
  */
  void deleteModel();

  void selectAll();

  /**
  * \brief Uncheck any checked items in the models list
  */
  void deselectAll();

  /**
  * \brief Read all grasps and models for the currently selected object and populate the models_list_
  * @text the text of the new item displayed in the box
  */
  void populateModelsList(const QString &text);

  void modelSelectionChanged();

  /*!
   * \brief Refresh with the latest object and model lists.
   *
   * Refreshes the interface with the latest object and model lists.
   */
  void refresh();
};

}
}

#endif
