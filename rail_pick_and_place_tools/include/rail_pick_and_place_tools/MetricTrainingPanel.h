/*!
 * \file MetricTrainingPanel.h
 * \brief RViz plugin for registration metric training.
 *
 * The metric training panel allows for sending metric training requests and getting user Yes/No feedback.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 8, 2015
 */

#ifndef RAIL_PICK_AND_PLACE_METRIC_TRAINING_PANEL_H_
#define RAIL_PICK_AND_PLACE_METRIC_TRAINING_PANEL_H_

// ROS
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <graspdb/graspdb.h>
#include <rail_pick_and_place_msgs/GetYesNoFeedbackAction.h>
#include <rail_pick_and_place_msgs/TrainMetricsAction.h>
#include <ros/ros.h>
#include <rviz/panel.h>

// Boost
#include <boost/thread/mutex.hpp>

// Qt
#include <QComboBox>
#include <QLabel>
#include <QPushButton>

namespace rail
{
namespace pick_and_place
{

/*!
 * \class MetricTrainingPanel
 * \brief RViz plugin for registration metric training.
 *
 * The metric training panel allows for sending metric training requests and getting user Yes/No feedback.
 */
class MetricTrainingPanel : public rviz::Panel
{

// this class uses Qt slots and is a subclass of QObject, so it needs the Q_OBJECT macro
Q_OBJECT

public:
  /*!
   * \brief Create a new MetricTrainingPanel.
   *
   * Creates a new MetricTrainingPanel and adds the correct widgets.
   *
   * \param parent The parent widget for this panel (defaults to NULL).
   */
  MetricTrainingPanel(QWidget *parent = NULL);

  /*!
   * \brief Cleans up a MetricTrainingPanel.
   *
   * Cleans up any connections used by the MetricTrainingPanel.
   */
  virtual ~MetricTrainingPanel();

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
  /*!
   * \brief Get the yes/no response from the user.
   *
   * Enables the yes/no buttons and waits for the user's response.
   *
   * \param goal The empty goal.
   */
  void getYesNoFeedbackCallback(const rail_pick_and_place_msgs::GetYesNoFeedbackGoalConstPtr &goal);

  /*!
   * \brief Callback for when the train metrics action server finishes.
   *
   * Sets the status message to the corresponding message.
   *
   * \param state The finished goal state.
   * \param result The result of the action client call.
   */
  void doneCallback(const actionlib::SimpleClientGoalState &state,
      const rail_pick_and_place_msgs::TrainMetricsResultConstPtr &result);

  /*!
   * \brief Callback for when the train metrics action sends feedback.
   *
   * Sets the status message to the corresponding message.
   *
   * \param feedback The current feedback message.
   */
  void feedbackCallback(const rail_pick_and_place_msgs::TrainMetricsFeedbackConstPtr &feedback);

  /*! The grasp database connection. */
  graspdb::Client *graspdb_;
  /*! If the user has responded yet and their response. */
  bool responded_, yes_;

  /*! Mutex for locking on the response flag. */
  boost::mutex mutex_;

  /*! The ROS node handle. */
  ros::NodeHandle node_;
  /*! The main user response service. */
  ros::ServiceServer get_yes_no_feedback_srv_;
  /*! The user feedback action server. */
  actionlib::SimpleActionServer<rail_pick_and_place_msgs::GetYesNoFeedbackAction> as_;
  /*! The train metrics action client. */
  actionlib::SimpleActionClient<rail_pick_and_place_msgs::TrainMetricsAction> train_metrics_ac_;

  /*! The current list of objects */
  QComboBox *object_list_;
  /*! The train metrics, refresh, and feedback buttons. */
  QPushButton *train_metrics_button_, *refresh_button_, *yes_button_, *no_button_;
  /*! The train metrics status text. */
  QLabel *train_metrics_status_;

// used as UI callbacks
private Q_SLOTS:

  /*!
   * \brief Callback for the segment button.
   *
   * Calls the segmentation service.
   */
  void executeTrainMetrics();

  /*!
   * \brief Refresh with the latest object list.
   *
   * Refreshes the interface with the latest object list.
   */
  void refresh();

  /*!
   * \brief Sets the value of the yes flag to true.
   *
   * Indicates the user selected "yes".
   */
  void setYesFeedback();

  /*!
   * \brief Sets the value of the yes flag to false.
   *
   * Indicates the user selected "no".
   */
  void setNoFeedback();
};

}
}

#endif
