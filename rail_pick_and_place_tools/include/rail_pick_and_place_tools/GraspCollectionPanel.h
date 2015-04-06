/*!
 * \file GraspCollectionPanel.h
 * \brief RViz plugin for grasp collection.
 *
 * The grasp collection panel allows for calling the grasp and store action server with the associated settings.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 6, 2015
 */

#ifndef RAIL_PICK_AND_PLACE_GRASP_COLLECTION_PANEL_H_
#define RAIL_PICK_AND_PLACE_GRASP_COLLECTION_PANEL_H_

// ROS
#include <actionlib/client/simple_action_client.h>
#include <rail_pick_and_place_msgs/GraspAndStoreAction.h>
#include <rviz/panel.h>

// Qt
#include <QCheckBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

namespace rail
{
namespace pick_and_place
{

/*!
 * \class GraspCollectionPanel
 * \brief RViz plugin for grasp collection.
 *
 * The grasp collection panel allows for calling the grasp and store action server with the associated settings.
 */
class GraspCollectionPanel : public rviz::Panel
{

// this class uses Qt slots and is a subclass of QObject, so it needs the Q_OBJECT macro
Q_OBJECT

public:
  /*!
   * \brief Create a new GraspCollectionPanel.
   *
   * Creates a new GraspCollectionPanel and adds the correct widgets and action client connections.
   *
   * \param parent The parent widget for this panel (defaults to NULL).
   */
  GraspCollectionPanel(QWidget *parent = NULL);

  /*!
   * \brief Load RViz configuration settings.
   *
   * Load the saved checkbox states if they are set.
   *
   * \param config The RViz configuration settings to load.
   */
  virtual void load(const rviz::Config &config);

  /*!
   * \brief Save RViz configuration settings.
   *
   * Save the checkbox states.
   *
   * \param config The RViz configuration settings to save.
   */
  virtual void save(rviz::Config config) const;

private:
  /*!
   * \brief Callback for when the grasp and store action server finishes.
   *
   * Sets the status message to the corresponding message.
   *
   * \param state The finished goal state.
   * \param result The result of the action client call.
   */
  void doneCallback(const actionlib::SimpleClientGoalState &state,
      const rail_pick_and_place_msgs::GraspAndStoreResultConstPtr &result);

  /*!
   * \brief Callback for when the grasp and store action sends feedback.
   *
   * Sets the status message to the corresponding message.
   *
   * \param feedback The current feedback message.
   */
  void feedbackCallback(const rail_pick_and_place_msgs::GraspAndStoreFeedbackConstPtr &feedback);

  /*! The main grasp and store action client. */
  actionlib::SimpleActionClient<rail_pick_and_place_msgs::GraspAndStoreAction> grasp_and_store_ac_;

  /*! The lift and verify grasp check boxes. */
  QCheckBox *lift_box_, *verify_box_;
  /*! The action server status. */
  QLabel *grasp_and_store_status_;
  /*! The text box for the object name input. */
  QLineEdit *name_input_;
  /*! The main grasp and store button. */
  QPushButton *grasp_and_store_button_;

// used as UI callbacks
private
  Q_SLOTS:

  /*!
   * \brief Call the grasp and store action server.
   *
   * Calls the grasp and store action server with the given settings and disable the button.
   */
  void executeGraspAndStore();
};

}
}

#endif
