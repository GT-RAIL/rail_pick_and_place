/*!
 * \file MetricTrainingPanel.cpp
 * \brief RViz plugin for registration metric training.
 *
 * The metric training panel allows for sending metric training requests and getting user Yes/No feedback.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 8, 2015
 */

// RAIL Pick and Place Tools
#include "rail_pick_and_place_tools/MetricTrainingPanel.h"

// ROS
#include <pluginlib/class_list_macros.h>

// Qt
#include <QBoxLayout>

using namespace std;
using namespace rail::pick_and_place;

MetricTrainingPanel::MetricTrainingPanel(QWidget *parent)
    : rviz::Panel(parent), train_metrics_ac_("/metric_trainer/train_metrics", true),
      as_(node_, "/metric_trainer/get_yes_no_feedback", boost::bind(&MetricTrainingPanel::getYesNoFeedbackCallback,
          this, _1), false)
{
  responded_ = false;

  // set defaults
  int port = graspdb::Client::DEFAULT_PORT;
  string host("127.0.0.1");
  string user("ros");
  string password("");
  string db("graspdb");

  // grab any parameters we need
  node_.getParam("/graspdb/host", host);
  node_.getParam("/graspdb/port", port);
  node_.getParam("/graspdb/user", user);
  node_.getParam("/graspdb/password", password);
  node_.getParam("/graspdb/db", db);

  // connect to the grasp database
  graspdb_ = new graspdb::Client(host, port, user, password, db);
  bool okay = graspdb_->connect();

  if (!okay)
  {
    ROS_WARN("Could not connect to grasp database.");
  }

  as_.start();

  // list of current objects
  QHBoxLayout *objects_layout = new QHBoxLayout();
  QLabel *object_name_label = new QLabel("Object:");
  object_name_label->setAlignment(Qt::AlignRight);
  object_list_ = new QComboBox();
  objects_layout->addWidget(object_name_label);
  objects_layout->addWidget(object_list_);
  objects_layout->setAlignment(Qt::AlignCenter);

  // refresh and train buttons
  QHBoxLayout *buttons_layout = new QHBoxLayout();
  refresh_button_ = new QPushButton("Refresh");
  train_metrics_button_ = new QPushButton("Begin Training");
  buttons_layout->addWidget(refresh_button_);
  buttons_layout->addWidget(train_metrics_button_);
  buttons_layout->setAlignment(Qt::AlignCenter);

  // service feedback
  train_metrics_status_ = new QLabel("Ready to train.");
  train_metrics_status_->setAlignment(Qt::AlignCenter);

  // yes/no response buttons
  QHBoxLayout *response_layout = new QHBoxLayout();
  QLabel *response_label = new QLabel("Valid Registration?");
  object_name_label->setAlignment(Qt::AlignRight);
  yes_button_ = new QPushButton("Yes");
  no_button_ = new QPushButton("No");
  // initially disabled (waits for response request)
  yes_button_->setEnabled(false);
  no_button_->setEnabled(false);
  response_layout->addWidget(response_label);
  response_layout->addWidget(yes_button_);
  response_layout->addWidget(no_button_);
  response_layout->setAlignment(Qt::AlignCenter);

  // build the final layout
  QVBoxLayout *layout = new QVBoxLayout();
  layout->addLayout(objects_layout);
  layout->addLayout(buttons_layout);
  layout->addWidget(train_metrics_status_);
  layout->addLayout(response_layout);

  // connect the buttons
  QObject::connect(refresh_button_, SIGNAL(clicked()), this, SLOT(refresh()));
  QObject::connect(train_metrics_button_, SIGNAL(clicked()), this, SLOT(executeTrainMetrics()));
  QObject::connect(yes_button_, SIGNAL(clicked()), this, SLOT(setYesFeedback()));
  QObject::connect(no_button_, SIGNAL(clicked()), this, SLOT(setNoFeedback()));

  // update with the initial state
  this->refresh();

  // set the final layout
  this->setLayout(layout);
}

MetricTrainingPanel::~MetricTrainingPanel()
{
  // cleanup
  as_.shutdown();
  graspdb_->disconnect();
  delete graspdb_;
}

void MetricTrainingPanel::getYesNoFeedbackCallback(const rail_pick_and_place_msgs::GetYesNoFeedbackGoalConstPtr &goal)
{
  // enable the buttons
  yes_button_->setEnabled(true);
  no_button_->setEnabled(true);

  // wait for a response from the user
  ros::Rate rate(10);
  while (ros::ok())
  {
    // lock the flag
    {
      boost::mutex::scoped_lock lock(mutex_);
      if (responded_)
      {
        // reset the flag
        responded_ = false;

        // send the response
        rail_pick_and_place_msgs::GetYesNoFeedbackResult result;
        result.yes = yes_;
        as_.setSucceeded(result);

        // disable the buttons
        yes_button_->setEnabled(false);
        no_button_->setEnabled(false);
        return;
      }
    }
    rate.sleep();
  }
}

void MetricTrainingPanel::setYesFeedback()
{
  // lock the flag
  boost::mutex::scoped_lock lock(mutex_);
  yes_ = true;
  responded_ = true;
}

void MetricTrainingPanel::setNoFeedback()
{
  // lock the flag
  boost::mutex::scoped_lock lock(mutex_);
  yes_ = false;
  responded_ = true;
}

void MetricTrainingPanel::refresh()
{
  // disable the button as we work
  refresh_button_->setEnabled(false);

  // clear the current objects
  object_list_->clear();
  // load from both the demonstration and model lists
  vector<string> object_names;
  graspdb_->getUniqueGraspDemonstrationObjectNames(object_names);
  // sort the names
  sort(object_names.begin(), object_names.end());
  // add each item name
  for (size_t i = 0; i < object_names.size(); i++)
  {
    object_list_->addItem(object_names[i].c_str());
  }

  // re-enable the button
  refresh_button_->setEnabled(true);
}

void MetricTrainingPanel::executeTrainMetrics()
{
  // disable the button first
  train_metrics_button_->setEnabled(false);

  if (!train_metrics_ac_.isServerConnected())
  {
    train_metrics_status_->setText("Train metrics action server not found!");
    // make sure to re-enable the button
    train_metrics_button_->setEnabled(true);
  } else
  {
    // set options for the goal from the GUI
    rail_pick_and_place_msgs::TrainMetricsGoal goal;
    goal.object_name = object_list_->currentText().toStdString();
    // send the goal asynchronously
    train_metrics_ac_.sendGoal(goal, boost::bind(&MetricTrainingPanel::doneCallback, this, _1, _2),
        actionlib::SimpleActionClient<rail_pick_and_place_msgs::TrainMetricsAction>::SimpleActiveCallback(),
        boost::bind(&MetricTrainingPanel::feedbackCallback, this, _1));
  }
}

void MetricTrainingPanel::doneCallback(const actionlib::SimpleClientGoalState &state,
    const rail_pick_and_place_msgs::TrainMetricsResultConstPtr &result)
{
  // check if the action was successful
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED && result->success)
  {
    train_metrics_status_->setText("Metric training finished and saved.");
  }
  else
  {
    // state text should represent what went wrong
    train_metrics_status_->setText(state.getText().c_str());
  }

  // re-enable the button
  train_metrics_button_->setEnabled(true);
}

void MetricTrainingPanel::feedbackCallback(const rail_pick_and_place_msgs::TrainMetricsFeedbackConstPtr &feedback)
{
  // simply set the status to the current message
  train_metrics_status_->setText(feedback->message.c_str());
}

void MetricTrainingPanel::save(rviz::Config config) const
{
  // simply call the super class
  rviz::Panel::save(config);
}

void MetricTrainingPanel::load(const rviz::Config &config)
{
  // simply call the super class
  rviz::Panel::load(config);
}

// tell pluginlib about this class (must outside of any namespace scope)
PLUGINLIB_EXPORT_CLASS(rail::pick_and_place::MetricTrainingPanel, rviz::Panel)
