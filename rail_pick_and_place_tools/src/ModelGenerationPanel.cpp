/*!
 * \ModelGenerationPanel.cpp
 * \brief Rviz plugin for generating object recognition/grasping models.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date March 2, 2015
 */

#include <rail_pick_and_place_tools/ModelGenerationPanel.h>

using namespace std;

namespace rail
{
namespace pick_and_place
{

ModelGenerationPanel::ModelGenerationPanel(QWidget *parent) :
    rviz::Panel(parent), ac_generate_models("/model_generator/generate_models", true)
{
  //setup connection to grasp database
  // set defaults
  int port = graspdb::Client::DEFAULT_PORT;
  string host("127.0.0.1");
  string user("ros");
  string password("");
  string db("graspdb");

  // grab any parameters we need
  nh_.getParam("/graspdb/host", host);
  nh_.getParam("/graspdb/port", port);
  nh_.getParam("/graspdb/user", user);
  nh_.getParam("/graspdb/password", password);
  nh_.getParam("/graspdb/db", db);

  // connect to the grasp database
  graspdb_ = new graspdb::Client(host, port, user, password, db);
  okay_ = graspdb_->connect();

  if (okay_)
    ROS_INFO("Successfully connected to grasp database.");
  else
    ROS_INFO("Could not connect to grasp database.");

  display_cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("pc_registration/model_cloud", 1);
  display_grasps_pub = nh_.advertise<geometry_msgs::PoseArray>("pc_registration/model_grasps", 1);

  models_list_ = new QListWidget;
  object_list_ = new QComboBox;

  QObject::connect(object_list_, SIGNAL(currentIndexChanged(const QString &)), this, SLOT(populateModelsList(const QString &)));

  updateObjectNames();

  //grasp/model list
  QVBoxLayout *models_layout = new QVBoxLayout;
  QHBoxLayout *object_name_layout = new QHBoxLayout;
  QLabel *object_name_label = new QLabel("Object:");
  object_name_layout->addWidget(object_name_label);
  object_name_layout->addWidget(object_list_);
  QHBoxLayout *models_layout_header = new QHBoxLayout;
  QLabel *list_label = new QLabel("Model List:");
  QPushButton *refresh_button = new QPushButton("Refresh");
  models_layout_header->addWidget(list_label);
  models_layout_header->addWidget(refresh_button);
  QPushButton *deselect_all_button = new QPushButton("Deselect All");
  models_layout->addLayout(models_layout_header);
  models_layout->addLayout(object_name_layout);
  models_layout->addWidget(models_list_);
  models_layout->addWidget(deselect_all_button);

  //model generation instructions
  QVBoxLayout *generate_layout = new QVBoxLayout;
  QLabel *generate_label = new QLabel("Model Generation:");
  generate_label->setAlignment(Qt::AlignTop);
  QLabel *generate_instructions = new QLabel("New models will be generated from the grasps and models selected from the list.");
  generate_instructions->setWordWrap(true);
  generate_layout->addWidget(generate_label);
  generate_layout->addWidget(generate_instructions);

  //max model size setting
  model_size_spinbox_ = new QSpinBox;
  model_size_spinbox_->setRange(2, 20);
  model_size_spinbox_->setSingleStep(1);
  model_size_spinbox_->setValue(6);
  QLabel *model_size_label = new QLabel("Grasps Per Model:");
  QHBoxLayout *model_size_layout = new QHBoxLayout;
  model_size_layout->addWidget(model_size_label);
  model_size_layout->addWidget(model_size_spinbox_);
  generate_layout->addLayout(model_size_layout);

  //generate and display buttons
  generate_button_ = new QPushButton("Generate Models");
  QPushButton *display_button = new QPushButton("Display Model");
  remove_button_ = new QPushButton("Remove Model");
  generate_layout->addWidget(generate_button_);
  generate_layout->addWidget(display_button);
  generate_layout->addWidget(remove_button_);

  //feedback
  model_generation_status_ = new QLabel("Ready to generate models.");

  //build final layout
  QGridLayout *grid_layout = new QGridLayout;
  grid_layout->addLayout(models_layout, 0, 0);
  grid_layout->addLayout(generate_layout, 0, 1);
  grid_layout->setColumnMinimumWidth(0, 150);
  grid_layout->setRowMinimumHeight(0, 185);
  QVBoxLayout *layout = new QVBoxLayout;
  layout->addLayout(grid_layout);
  layout->addWidget(model_generation_status_);

  //connect things
  QObject::connect(refresh_button, SIGNAL(clicked()), this, SLOT(updateObjectNames()));
  QObject::connect(deselect_all_button, SIGNAL(clicked()), this, SLOT(deselectAll()));
  QObject::connect(generate_button_, SIGNAL(clicked()), this, SLOT(executeRegistration()));
  QObject::connect(display_button, SIGNAL(clicked()), this, SLOT(displayModel()));
  QObject::connect(remove_button_, SIGNAL(clicked()), this, SLOT(removeModel()));

  setLayout(layout);
}

ModelGenerationPanel::~ModelGenerationPanel()
{
  graspdb_->disconnect();
}

void ModelGenerationPanel::deselectAll()
{
  for (unsigned int i = 0; i < models_list_->count(); i++)
  {
    if (models_list_->item(i)->checkState() == Qt::Checked)
    {
      models_list_->item(i)->setCheckState(Qt::Unchecked);
    }
  }
}

void ModelGenerationPanel::executeRegistration()
{
  if (!ac_generate_models.isServerConnected())
  {
    model_generation_status_->setText("No model generation action server found!");
    return;
  }
  rail_pick_and_place_msgs::GenerateModelsGoal generate_models_goal;
  for (unsigned int i = 0; i < models_list_->count(); i++)
  {
    if (models_list_->item(i)->checkState() == Qt::Checked)
    {
      string selected_item = models_list_->item(i)->text().toStdString();
      int id = atoi(selected_item.substr(5).c_str());
      if (selected_item.at(0) == 'g')
        generate_models_goal.grasp_demonstration_ids.push_back(id);
      else
        generate_models_goal.grasp_model_ids.push_back(id);
    }
  }
  generate_models_goal.max_model_size = model_size_spinbox_->value();
  ac_generate_models.sendGoal(generate_models_goal, boost::bind(&ModelGenerationPanel::doneCb, this, _1, _2),
      actionlib::SimpleActionClient<rail_pick_and_place_msgs::GenerateModelsAction>::SimpleActiveCallback(),
      boost::bind(&ModelGenerationPanel::feedbackCb, this, _1));

  generate_button_->setEnabled(false);
}

void ModelGenerationPanel::doneCb(const actionlib::SimpleClientGoalState &state, const rail_pick_and_place_msgs::GenerateModelsResultConstPtr &result)
{
  int new_models = result->new_model_ids.size();
  stringstream ss;
  if (new_models == 0)
  {
    ss << "No new models generated.";
  }
  else if (new_models == 1)
  {
    ss << "Added 1 new model.";
  }
  else
  {
    ss << "Added " << new_models << " new models.";
  }
  model_generation_status_->setText(ss.str().c_str());
  updateModelInfo();

  generate_button_->setEnabled(true);
}

void ModelGenerationPanel::feedbackCb(const rail_pick_and_place_msgs::GenerateModelsFeedbackConstPtr &feedback)
{
  model_generation_status_->setText(feedback->message.c_str());
}

void ModelGenerationPanel::displayModel()
{
  if (models_list_->currentItem() == NULL)
  {
    ROS_INFO("No item selected.");
    return;
  }
  int i = models_list_->currentIndex().row() - 1; //the 1 is the adjustment for the demonstrations label
  string selected_item = models_list_->currentItem()->text().toStdString();
  if (selected_item.at(0) == '-')
  {
    ROS_INFO("No item selected.");
    return;
  }

  sensor_msgs::PointCloud2 cloud;
  geometry_msgs::PoseArray poses;
  if (selected_item.at(0) == 'g')
  {
    cloud = current_demonstrations_[i].getPointCloud();
    cloud.header.frame_id = "base_footprint";
    poses.header.frame_id = "base_footprint";
    //poses.header = current_demonstrations_[i].getGraspPose().toROSPoseStampedMessage().header;
    poses.poses.push_back(current_demonstrations_[i].getGraspPose().toROSPoseMessage());
  }
  else
  {
    i -= current_demonstrations_.size() + 1; //the 1 is an adjustment for the models label
    cloud = current_models_[i].getPointCloud();
    cloud.header.frame_id = "base_footprint";
    poses.header.frame_id = "base_footprint";
    for (unsigned int j = 0; j < current_models_[i].getNumGrasps(); j ++)
    {
      poses.poses.push_back(current_models_[i].getGrasp(j).getGraspPose().toROSPoseMessage());
    }
  }

  display_cloud_pub.publish(cloud);
  display_grasps_pub.publish(poses);
}

void ModelGenerationPanel::removeModel()
{
  if (models_list_->currentItem() == NULL)
  {
    ROS_INFO("No item selected.");
    return;
  }

  string selected_item = models_list_->currentItem()->text().toStdString();
  if (selected_item.at(0) == '-')
  {
    ROS_INFO("No item selected.");
    return;
  }

  QMessageBox::StandardButton confirm;
  int id = atoi(selected_item.substr(5).c_str());
  if (selected_item.at(0) == 'g')
  {
    graspdb_->deleteGraspDemonstration(id);
    confirm = QMessageBox::question(this, "Remove Model Confirmation", "Are you sure you want to remove the highlighted grasp demonstration?", QMessageBox::Yes|QMessageBox::No);
    if (confirm == QMessageBox::Yes)
    {
      current_demonstrations_.erase(current_demonstrations_.begin() + models_list_->currentIndex().row() - 1);
      delete models_list_->currentItem();
    }
  }
  else
  {
    graspdb_->deleteGraspModel(id);
    confirm = QMessageBox::question(this, "Remove Model Confirmation", "Are you sure you want to remove the highlighted object model?", QMessageBox::Yes|QMessageBox::No);
    if (confirm == QMessageBox::Yes)
    {
      current_models_.erase(current_models_.begin() + models_list_->currentIndex().row() - current_demonstrations_.size() - 2);
      delete models_list_->currentItem();
    }
  }
}

void ModelGenerationPanel::updateObjectNames()
{
  object_list_->clear();
  vector<string> object_names;
  vector<string> model_names;
  graspdb_->getUniqueGraspDemonstrationObjectNames(object_names);
  graspdb_->getUniqueGraspModelObjectNames(model_names);
  object_names.insert(object_names.end(), model_names.begin(), model_names.end());
  sort(object_names.begin(), object_names.end());
  object_names.erase(unique(object_names.begin(), object_names.end()), object_names.end());
  for (unsigned int i = 0; i < object_names.size(); i ++)
  {
    object_list_->addItem(object_names[i].c_str());
  }
}

void ModelGenerationPanel::populateModelsList(const QString &text)
{
  if (object_list_->count() <= 0)
    return;

  models_list_->clear();
  current_demonstrations_.clear();
  current_models_.clear();
  graspdb_->loadGraspDemonstrationsByObjectName(text.toStdString(), current_demonstrations_);
  graspdb_->loadGraspModelsByObjectName(text.toStdString(), current_models_);

  QListWidgetItem *grasps_label = new QListWidgetItem("--Grasp Demonstrations--", models_list_);
  grasps_label->setFlags(Qt::ItemIsEnabled);
  for (unsigned int i = 0; i < current_demonstrations_.size(); i ++)
  {
    stringstream ss;
    ss << "grasp" << current_demonstrations_[i].getID();
    QListWidgetItem *item = new QListWidgetItem(ss.str().c_str(), models_list_);
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(Qt::Unchecked);
  }

  QListWidgetItem *models_label = new QListWidgetItem("--Object Models--", models_list_);
  models_label->setFlags(Qt::ItemIsEnabled);
  for (unsigned int i = 0; i < current_models_.size(); i ++)
  {
    stringstream ss;
    ss << "model" << current_models_[i].getID();
    QListWidgetItem *item = new QListWidgetItem(ss.str().c_str(), models_list_);
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(Qt::Unchecked);
  }
}

void ModelGenerationPanel::updateModelInfo()
{
  vector<graspdb::GraspModel> models;
  int max_id = 0;
  int insert_id = 0;
  if (!current_models_.empty())
  {
    max_id = current_models_[current_models_.size() - 1].getID();
    insert_id = current_models_.size();
  }
  int row = models_list_->count();
  graspdb_->loadGraspModelsByObjectName(object_list_->currentText().toStdString(), models);
  for (int i = models.size() - 1; i >= 0; i --)
  {
    if (models[i].getID() <= max_id)
    {
      break;
    }
    stringstream ss;
    ss << "model" << models[i].getID();
    QListWidgetItem *item = new QListWidgetItem(ss.str().c_str());
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(Qt::Unchecked);
    models_list_->insertItem(row, item);
    if (i == models.size() - 1)
      current_models_.push_back(models[i]);
    else
      current_models_.insert(current_models_.begin() + insert_id, models[i]);
  }
}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void ModelGenerationPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
  config.mapSetValue("MaxModelSize", model_size_spinbox_->value());
}

// Load all configuration data for this panel from the given Config object.
void ModelGenerationPanel::load(const rviz::Config &config)
{
  rviz::Panel::load(config);
  int max_model_size;
  if (config.mapGetInt("MaxModelSize", &max_model_size))
    model_size_spinbox_->setValue(max_model_size);
}

}
}

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rail::pick_and_place::ModelGenerationPanel,rviz::Panel )
