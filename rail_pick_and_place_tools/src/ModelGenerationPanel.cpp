
#include <rail_pick_and_place_tools/ModelGenerationPanel.h>

using namespace std;

namespace rail
{
namespace pick_and_place
{

ModelGenerationPanel::ModelGenerationPanel(QWidget *parent) :
    rviz::Panel(parent),
    acGenerateModels("pc_registration/generate_models", true)
{
  display_model_client_ = nh_.serviceClient<rail_recognition::DisplayModel>("pc_registration/display_model");
  get_model_numbers_client_ = nh_.serviceClient<rail_recognition::GetModelNumbers>("pc_registration/get_model_numbers");

  models_list_ = new QListWidget;

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
  generate_button_ = new QPushButton("Generate Models");
  QPushButton *display_button = new QPushButton("Display Model");
  generate_layout->addWidget(generate_button_);
  generate_layout->addWidget(display_button);

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
  QObject::connect(deselect_all_button, SIGNAL(clicked()), this, SLOT(deselectAll()));
  QObject::connect(generate_button_, SIGNAL(clicked()), this, SLOT(executeRegistration()));
  QObject::connect(display_button, SIGNAL(clicked()), this, SLOT(displayModel()));

  setLayout(layout);
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
  rail_recognition::GenerateModelsGoal generateModelsGoal;
  for (unsigned int i = 0; i < models_list_->count(); i++)
  {
    if (models_list_->item(i)->checkState() == Qt::Checked)
    {
      string selected_item = models_list_->item(i)->text().toStdString();
      int id = atoi(selected_item.substr(selected_item.find('_') + 1).c_str()) - 1;
      if (selected_item.at(0) == 'g')
        generateModelsGoal.individualGraspModelIds.push_back(id);
      else
        generateModelsGoal.mergedModelIds.push_back(id);
    }
  }
  generateModelsGoal.maxModelSize = model_size_spinbox_->value();
  acGenerateModels.sendGoal(generateModelsGoal, boost::bind(&ModelGenerationPanel::doneCb, this, _1, _2),
      actionlib::SimpleActionClient<rail_recognition::GenerateModelsAction>::SimpleActiveCallback(),
      boost::bind(&ModelGenerationPanel::feedbackCb, this, _1));

  generate_button_->setEnabled(false);
}

void ModelGenerationPanel::doneCb(const actionlib::SimpleClientGoalState &state, const rail_recognition::GenerateModelsResultConstPtr &result)
{
  int new_models = result->newModelsTotal;
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

void ModelGenerationPanel::feedbackCb(const rail_recognition::GenerateModelsFeedbackConstPtr &feedback)
{
  model_generation_status_->setText(feedback->message.c_str());
}

void ModelGenerationPanel::displayModel()
{
  rail_recognition::DisplayModel srv;

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
  rail_recognition::GetModelNumbers srv;
  if (!get_model_numbers_client_.call(srv))
  {
    ROS_INFO("Could not call get model numbers service.");
    return;
  }

  int prev_model_count = models_list_->count();
  //models_list_->clear();
  if (prev_model_count == 0)
  {
    for (unsigned int i = 0; i < srv.response.total_individual_grasps; i++)
    {
      stringstream ss;
      ss << "grasp_" << i + 1;
      QListWidgetItem *item = new QListWidgetItem(ss.str().c_str(), models_list_);
      item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
      item->setCheckState(Qt::Unchecked);
    }

    for (unsigned int i = 0; i < srv.response.total_merged_models; i++)
    {
      stringstream ss;
      ss << "model_" << i + 1;
      QListWidgetItem *item = new QListWidgetItem(ss.str().c_str(), models_list_);
      item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
      item->setCheckState(Qt::Unchecked);
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
