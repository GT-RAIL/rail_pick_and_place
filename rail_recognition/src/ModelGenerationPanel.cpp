
#include <rail_recognition/ModelGenerationPanel.h>

using namespace std;

namespace rail_recognition
{

ModelGenerationPanel::ModelGenerationPanel(QWidget* parent)
  : rviz::Panel(parent)
{
  display_model_client_ = nh_.serviceClient<rail_recognition::DisplayModel>("pc_registration/display_model");
  get_model_numbers_client_ = nh_.serviceClient<rail_recognition::GetModelNumbers>("pc_registration/get_model_numbers");
  generate_models_client_ = nh_.serviceClient<rail_recognition::GenerateModels>("pc_registration/generate_models");

  models_list_ = new QListWidget;
  models_dropdown_ = new QComboBox;

  updateModelInfo();

  //grasp/model list
  QVBoxLayout *models_layout = new QVBoxLayout;
  QLabel *name_label = new QLabel("Model List:");
  QPushButton *deselect_all_button = new QPushButton("Deselect All");
  models_layout->addWidget(name_label);
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
  QLabel *model_size_label = new QLabel("Max Model Size:");
  QHBoxLayout *model_size_layout = new QHBoxLayout;
  model_size_layout->addWidget(model_size_label);
  model_size_layout->addWidget(model_size_spinbox_);
  generate_layout->addLayout(model_size_layout);

  //generate and display buttons
  QPushButton *generate_button = new QPushButton("Generate Models");
  QPushButton *display_button = new QPushButton("Display Model");
  generate_layout->addWidget(generate_button);
  generate_layout->addWidget(display_button);

  //feedback
  model_generation_status_ = new QLabel("Ready to generate.");
  busy_feedback_ = new QLabel("");

  //final layout
  QGridLayout *layout = new QGridLayout;
  layout->addLayout(models_layout, 0, 0);
  layout->addLayout(generate_layout, 0, 1);
  layout->addWidget(model_generation_status_, 1, 0);
  layout->addWidget(busy_feedback_, 1, 1);
  //layout->addWidget(models_dropdown_, 2, 0);
  //layout->addWidget(display_button, 2, 1);
  layout->setColumnMinimumWidth(0, 150);
  layout->setRowMinimumHeight(0, 185);

  //connect things
  QObject::connect(deselect_all_button, SIGNAL(clicked()), this, SLOT(deselectAll()));
  QObject::connect(generate_button, SIGNAL(clicked()), this, SLOT(executeRegistration()));
  QObject::connect(display_button, SIGNAL(clicked()), this, SLOT(displayModel()));

  setLayout(layout);
}

void ModelGenerationPanel::deselectAll()
{
  for (unsigned int i = 0; i < models_list_->count(); i ++)
  {
    if (models_list_->item(i)->checkState() == Qt::Checked)
    {
      models_list_->item(i)->setCheckState(Qt::Unchecked);
    }
  }
}

void ModelGenerationPanel::executeRegistration()
{
  //Can this display?
  model_generation_status_->setText("Generating models...");
  busy_feedback_->setText("Please wait.");
  ros::spinOnce();
  rail_recognition::GenerateModels srv;
  for (unsigned int i = 0; i < models_list_->count(); i ++)
  {
    if (models_list_->item(i)->checkState() == Qt::Checked)
    {
      string selected_item = models_list_->item(i)->text().toStdString();
      int id = atoi(selected_item.substr(selected_item.find('_') + 1).c_str()) - 1;
      if (selected_item.at(0) == 'g')
        srv.request.individualGraspModelIds.push_back(id);
      else
        srv.request.mergedModelIds.push_back(id);
    }
  }
  srv.request.maxModelSize = model_size_spinbox_->value();
  if (!generate_models_client_.call(srv))
  {
    ROS_INFO("Could not call model generation service.");
    model_generation_status_->setText("Could not call model generation service!");
    busy_feedback_->setText("");
    return;
  }

  ROS_INFO("Model generation complete.");

  updateModelInfo();

  stringstream ss;
  int new_models = srv.response.newModelsTotal;
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
  model_generation_status_->setText("Generation complete.");
  busy_feedback_->setText(ss.str().c_str());
}

void ModelGenerationPanel::displayModel()
{
  rail_recognition::DisplayModel srv;

  //string selected_item = models_dropdown_->currentText().toStdString();
  if (models_list_->currentItem() == NULL)
  {
    ROS_INFO("No item selected.");
    return;
  }
  string selected_item = models_list_->currentItem()->text().toStdString();

  if (selected_item.size() < 3)
    return;
  if (selected_item.at(0) == 'g')
    srv.request.isMergedModel = false;
  else
    srv.request.isMergedModel = true;
  srv.request.modelId = atoi(selected_item.substr(selected_item.find('_') + 1).c_str()) - 1;
  srv.request.frame = "model";

  if (!display_model_client_.call(srv))
    ROS_INFO("Couldn't call display model client.");
}

void ModelGenerationPanel::updateModelInfo()
{
  GetModelNumbers srv;
  if (!get_model_numbers_client_.call(srv))
  {
    ROS_INFO("Could not call get model numbers service.");
    return;
  }

  int prev_model_count = models_list_->count();
  //models_list_->clear();
  //models_dropdown_->clear();
  if (prev_model_count == 0)
  {
    for (unsigned int i = 0; i < srv.response.total_individual_grasps; i++)
    {
      stringstream ss;
      ss << "grasp_" << i + 1;
      QListWidgetItem *item = new QListWidgetItem(ss.str().c_str(), models_list_);
      item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
      item->setCheckState(Qt::Unchecked);
      models_dropdown_->addItem(ss.str().c_str());
    }

    for (unsigned int i = 0; i < srv.response.total_merged_models; i ++)
    {
      stringstream ss;
      ss << "model_" << i + 1;
      QListWidgetItem* item = new QListWidgetItem(ss.str().c_str(), models_list_);
      item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
      item->setCheckState(Qt::Unchecked);
      models_dropdown_->addItem(ss.str().c_str());
    }
  }
  else
  {
    for (unsigned int i = prev_model_count; i < srv.response.total_individual_grasps + srv.response.total_merged_models; i++)
    {
      stringstream ss;
      ss << "model_" << i - srv.response.total_individual_grasps + 1;
      QListWidgetItem *item = new QListWidgetItem(ss.str().c_str(), models_list_);
      item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
      item->setCheckState(Qt::Unchecked);
      models_dropdown_->addItem(ss.str().c_str());
    }
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
void ModelGenerationPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
  int max_model_size;
  if(config.mapGetInt("MaxModelSize", &max_model_size))
    model_size_spinbox_->setValue(max_model_size);
}

} // end namespace rail_recognition

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rail_recognition::ModelGenerationPanel,rviz::Panel )
