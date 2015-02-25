
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
  QLabel *name_label = new QLabel("Grasps and Models:");
  models_layout->addWidget(name_label);
  models_layout->addWidget(models_list_);

  QVBoxLayout *generate_layout = new QVBoxLayout;
  QLabel *generate_instructions = new QLabel("Models will be generated from the selected grasps and models.");
  generate_instructions->setWordWrap(true);
  QPushButton *generate_button = new QPushButton("Generate Models");
  generate_layout->addWidget(generate_instructions, Qt::AlignBottom);
  generate_layout->addWidget(generate_button, Qt::AlignBottom);

  QPushButton *display_button = new QPushButton("Display Model");

  QGridLayout *layout = new QGridLayout;
  layout->addLayout(models_layout, 0, 0);
  layout->addLayout(generate_layout, 0, 1);
  layout->addWidget(models_dropdown_, 1, 0);
  layout->addWidget(display_button, 1, 1);

  //connect things
  QObject::connect(generate_button, SIGNAL(clicked()), this, SLOT(executeRegistration()));
  QObject::connect(display_button, SIGNAL(clicked()), this, SLOT(displayModel()));

  setLayout(layout);
}

void ModelGenerationPanel::executeRegistration()
{
  rail_recognition::GenerateModels srv;
  //TODO: get max model size from a text input field
  for (unsigned int i = 0; i < models_list_->count(); i ++)
  {
    if (models_list_->item(i)->checkState() == Qt::Checked)
    {
      //TODO: Determine if it's an individual grasp or a model that's selected, and update registration to use both grasps and models
      srv.request.models.push_back(i);
    }
  }
  srv.request.maxModelSize = 6;
  if (!generate_models_client_.call(srv))
  {
    ROS_INFO("Could not call model generation service.");
    return;
  }

  //TODO: add text box for status on the interface
  ROS_INFO("Model generation complete.");

  updateModelInfo();
}

void ModelGenerationPanel::displayModel()
{
  rail_recognition::DisplayModel srv;

  string selected_item = models_dropdown_->currentText().toStdString();

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

  models_list_->clear();
  models_dropdown_->clear();
  for (unsigned int i = 0; i < srv.response.total_individual_grasps; i ++)
  {
    stringstream ss;
    ss << "grasp_" << i + 1;
    QListWidgetItem* item = new QListWidgetItem(ss.str().c_str(), models_list_);
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

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void ModelGenerationPanel::save(rviz::Config config) const
{
  rviz::Panel::save(config);
}

// Load all configuration data for this panel from the given Config object.
void ModelGenerationPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
}

} // end namespace rail_recognition

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rail_recognition::ModelGenerationPanel,rviz::Panel )
