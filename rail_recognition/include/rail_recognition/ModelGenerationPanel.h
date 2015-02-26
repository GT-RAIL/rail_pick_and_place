#ifndef MODEL_GENERATION_PANEL_H
#define MODEL_GENERATION_PANEL_H

# include <ros/ros.h>
# include <rviz/panel.h>

#include <rail_recognition/DisplayModel.h>
#include <rail_recognition/GenerateModels.h>
#include <rail_recognition/GetModelNumbers.h>

#include <QComboBox>
#include <QGridLayout>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QSpinBox>
#include <QVBoxLayout>

class QLineEdit;

namespace rail_recognition
{

class ModelGenerationPanel: public rviz::Panel
{
// This class uses Qt slots and is a subclass of QObject, so it needs
// the Q_OBJECT macro.
Q_OBJECT
public:
  ModelGenerationPanel( QWidget* parent = 0 );

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

protected:
  // List of grasps and models
  QListWidget *models_list_;
  QComboBox *models_dropdown_;
  QSpinBox *model_size_spinbox_;

  ros::ServiceClient display_model_client_;
  ros::ServiceClient get_model_numbers_client_;
  ros::ServiceClient generate_models_client_;

  // The ROS node handle.
  ros::NodeHandle nh_;

protected Q_SLOTS:
  void executeRegistration();

  void displayModel();

private:
  void updateModelInfo();



};

} // end namespace rail_recognition

#endif // MODEL_GENERATION_PANEL_H
