/*!
 * \file ModelGenerationPanel.h
 * \brief RViz plugin for model generation.
 *
 * The model generation panel allows for model generation requests to be made using a selection of grasps and models.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 8, 2015
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

/*!
 * \class ModelGenerationPanel
 * \brief RViz plugin for model generation.
 *
 * The model generation panel allows for model generation requests to be made using a selection of grasps and models.
 */
class ModelGenerationPanel : public rviz::Panel
{

// this class uses Qt slots and is a subclass of QObject, so it needs the Q_OBJECT macro
Q_OBJECT

public:
  /*!
   * \brief Create a new ModelGenerationPanel.
   *
   * Creates a new ModelGenerationPanel and adds the correct widgets and action client connections.
   *
   * \param parent The parent widget for this panel (defaults to NULL).
   */
  ModelGenerationPanel(QWidget *parent = NULL);

  /*!
   * \brief Cleans up a ModelGenerationPanel.
   *
   * Cleans up any connections used by the ModelGenerationPanel.
   */
  virtual ~ModelGenerationPanel();

  /*!
   * \brief Load RViz configuration settings.
   *
   * Load the max model size state if it is set.
   *
   * \param config The RViz configuration settings to load.
   */
  virtual void load(const rviz::Config &config);

  /*!
   * \brief Save RViz configuration settings.
   *
   * Save the max model size state.
   *
   * \param config The RViz configuration settings to save.
   */
  virtual void save(rviz::Config config) const;

private:
  /*!
   * \brief Callback for when the generate models action server finishes.
   *
   * Sets the status message to the corresponding message.
   *
   * \param state The finished goal state.
   * \param result The result of the action client call.
   */
  void doneCallback(const actionlib::SimpleClientGoalState &state,
      const rail_pick_and_place_msgs::GenerateModelsResultConstPtr &result);

  /*!
   * \brief Callback for when the generate models action sends feedback.
   *
   * Sets the status message to the corresponding message.
   *
   * \param feedback The current feedback message.
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

  /*! The status of the generate models action server. */
  QLabel *model_generation_status_;
  /*! The current list of objects. */
  QComboBox *object_list_;
  /*! The list of grasps and models. */
  QListWidget *models_list_;
  /*! The maximum model size spinner. */
  QSpinBox *model_size_spin_box_;
  /*! The various buttons used in the interface. */
  QPushButton *refresh_button_, *select_all_button_, *deselect_all_button_, *generate_models_button_, *delete_button_;

// used as UI callbacks
private
  Q_SLOTS:

  /*!
   * \brief Call the generate models action server.
   *
   * Calls the generate models action server with the given settings and disable the button.
   */
  void executeGenerateModels();

  /*!
   * \brief Remove the currently selected individual grasp or object model.
   *
   * Makes a call to the graspdb to delete the selected model or grasp (if any). A confirmation dialog is given.
   */
  void deleteModel();

  /*!
   * \brief Select all models/grasps.
   *
   * Selects all the current models/grasps in the list.
   */
  void selectAll();

  /*!
   * \brief Deselect all models/grasps.
   *
   * Deselects all the current models/grasps in the list.
   */
  void deselectAll();

  /*!
   * \brief Populates the model list.
   *
   * Populates the models list with the given object's grasps and models.
   */
  void populateModelsList(const QString &text);

  /*!
   * \brief Display a selected model.
   *
   * Makes a call to display the selected model/grasp (if any) and updates the text on the delete button.
   */
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
