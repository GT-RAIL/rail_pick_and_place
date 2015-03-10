#ifndef MODEL_GENERATION_PANEL_H
#define MODEL_GENERATION_PANEL_H

# include <ros/ros.h>
# include <rviz/panel.h>

#include <actionlib/client/simple_action_client.h>
#include <rail_recognition/DisplayModel.h>
#include <rail_recognition/GenerateModelsAction.h>
#include <rail_recognition/GetModelNumbers.h>

#include <QComboBox>
#include <QGridLayout>
#include <QLabel>
#include <QListWidget>
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
  ModelGenerationPanel(QWidget *parent = 0);

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load(const rviz::Config &config);

  virtual void save(rviz::Config config) const;

protected:
  // List of grasps and models
  QLabel *model_generation_status_;
  QLabel *busy_feedback_;
  QListWidget *models_list_;
  QSpinBox *model_size_spinbox_;
  QPushButton *generate_button_;

protected
  Q_SLOTS:
  void

  executeRegistration();

  void displayModel();

  void deselectAll();

private:
  actionlib::SimpleActionClient <rail_recognition::GenerateModelsAction> acGenerateModels;

  ros::ServiceClient display_model_client_;
  ros::ServiceClient get_model_numbers_client_;
  //ros::ServiceClient generate_models_client_;

  // The ROS node handle.
  ros::NodeHandle nh_;

  void updateModelInfo();

  void doneCb(const actionlib::SimpleClientGoalState &state, const rail_recognition::GenerateModelsResultConstPtr &result);

  void feedbackCb(const rail_recognition::GenerateModelsFeedbackConstPtr &feedback);

};

} // end namespace pick_and_place
} // end namespace rail

#endif // MODEL_GENERATION_PANEL_H
