/*!
 * \file ModelGenerationPanel.cpp
 * \brief RViz plugin for model generation.
 *
 * The model generation panel allows for model generation requests to be made using a selection of grasps and models.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 8, 2015
 */

// RAIL Pick and Place Tools
#include "rail_pick_and_place_tools/ModelGenerationPanel.h"

// ROS
#include <pluginlib/class_list_macros.h>

// Qt
#include <QGridLayout>
#include <QMessageBox>

using namespace std;
using namespace rail::pick_and_place;

ModelGenerationPanel::ModelGenerationPanel(QWidget *parent)
    : rviz::Panel(parent), generate_models_ac_("/model_generator/generate_models", true),
      retrieve_grasp_ac_("/rail_grasp_retriever/retrieve_grasp", true),
      retrieve_grasp_model_ac_("/rail_grasp_model_retriever/retrieve_grasp_model", true)
{
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

  // list of current objects
  QHBoxLayout *objects_layout = new QHBoxLayout();
  QLabel *object_name_label = new QLabel("Object:");
  object_name_label->setAlignment(Qt::AlignRight);
  object_list_ = new QComboBox();
  objects_layout->addWidget(object_name_label);
  objects_layout->addWidget(object_list_);
  objects_layout->setAlignment(Qt::AlignCenter);

  // refresh and selection buttons
  QHBoxLayout *refresh_and_selections_layout = new QHBoxLayout();
  refresh_button_ = new QPushButton("Refresh");
  select_all_button_ = new QPushButton("Select All");
  deselect_all_button_ = new QPushButton("Deselect All");
  refresh_and_selections_layout->addWidget(refresh_button_);
  refresh_and_selections_layout->addWidget(select_all_button_);
  refresh_and_selections_layout->addWidget(deselect_all_button_);

  // delete button
  delete_button_ = new QPushButton("Delete");
  delete_button_->setEnabled(false);

  // grasp demonstration and model lists
  models_list_ = new QListWidget();

  // model generation options
  QHBoxLayout *generation_layout = new QHBoxLayout();
  // max model size options
  QLabel *model_size_label = new QLabel("Max Grasps Per Model:");
  model_size_label->setAlignment(Qt::AlignRight);
  model_size_spin_box_ = new QSpinBox();
  model_size_spin_box_->setRange(1, 99);
  model_size_spin_box_->setSingleStep(1);
  model_size_spin_box_->setValue(6);
  // model generation button
  generate_models_button_ = new QPushButton("Generate Models");
  generation_layout->addWidget(model_size_label);
  generation_layout->addWidget(model_size_spin_box_);
  generation_layout->addWidget(generate_models_button_);

  // action client feedback
  model_generation_status_ = new QLabel("Ready to generate models.");

  // build the final layout
  QVBoxLayout *layout = new QVBoxLayout();
  layout->addLayout(objects_layout);
  layout->addLayout(refresh_and_selections_layout);
  layout->addWidget(delete_button_);
  layout->addWidget(models_list_);
  layout->addLayout(generation_layout);
  layout->addWidget(model_generation_status_);

  // connect event callbacks
  QObject::connect(object_list_, SIGNAL(currentIndexChanged(
      const QString &)), this, SLOT(populateModelsList(
      const QString &)));
  QObject::connect(refresh_button_, SIGNAL(clicked()), this, SLOT(refresh()));
  QObject::connect(select_all_button_, SIGNAL(clicked()), this, SLOT(selectAll()));
  QObject::connect(deselect_all_button_, SIGNAL(clicked()), this, SLOT(deselectAll()));
  QObject::connect(delete_button_, SIGNAL(clicked()), this, SLOT(deleteModel()));
  QObject::connect(models_list_, SIGNAL(itemSelectionChanged()), this, SLOT(modelSelectionChanged()));
  QObject::connect(generate_models_button_, SIGNAL(clicked()), this, SLOT(executeGenerateModels()));

  // update with the initial state
  this->refresh();

  // set the final layout
  this->setLayout(layout);
}

ModelGenerationPanel::~ModelGenerationPanel()
{
  // cleanup
  graspdb_->disconnect();
  delete graspdb_;
}

void ModelGenerationPanel::refresh()
{
  // disable the button as we work
  refresh_button_->setEnabled(false);

  // clear the current objects
  object_list_->clear();
  // load from both the demonstration and model lists
  vector<string> object_names;
  vector<string> model_names;
  graspdb_->getUniqueGraspDemonstrationObjectNames(object_names);
  graspdb_->getUniqueGraspModelObjectNames(model_names);
  // combine the lists
  object_names.insert(object_names.end(), model_names.begin(), model_names.end());
  // sort the names
  sort(object_names.begin(), object_names.end());
  // make the lise unique
  object_names.erase(unique(object_names.begin(), object_names.end()), object_names.end());
  // add each item name
  for (size_t i = 0; i < object_names.size(); i++)
  {
    object_list_->addItem(object_names[i].c_str());
  }

  // re-enable the button
  refresh_button_->setEnabled(true);
}

void ModelGenerationPanel::selectAll()
{
  // disable the button as we work
  select_all_button_->setEnabled(false);

  // simply go through the current list
  for (size_t i = 0; i < models_list_->count(); i++)
  {
    if (models_list_->item(i)->flags() & Qt::ItemIsUserCheckable)
    {
      models_list_->item(i)->setCheckState(Qt::Checked);
    }
  }

  // re-enable the button
  select_all_button_->setEnabled(true);
}

void ModelGenerationPanel::deselectAll()
{
  // disable the button as we work
  deselect_all_button_->setEnabled(false);

  // simply go through the current list
  for (size_t i = 0; i < models_list_->count(); i++)
  {
    if (models_list_->item(i)->flags() & Qt::ItemIsUserCheckable)
    {
      models_list_->item(i)->setCheckState(Qt::Unchecked);
    }
  }

  // re-enable the button
  deselect_all_button_->setEnabled(true);
}

void ModelGenerationPanel::modelSelectionChanged()
{
  // check if there is an item selected
  if (models_list_->currentItem() != NULL && models_list_->currentItem()->flags() & Qt::ItemIsUserCheckable)
  {
    // grab the current item
    string selected_item = models_list_->currentItem()->text().toStdString();
    // extract the ID
    int id = atoi(selected_item.substr(selected_item.find(' ')).c_str());

    // enable the delete button
    string delete_text = "Delete " + selected_item;
    delete_button_->setText(delete_text.c_str());
    delete_button_->setEnabled(true);

    // make calls to visualize the model or grasp
    if (selected_item[0] == 'G' && retrieve_grasp_ac_.isServerConnected())
    {
      rail_pick_and_place_msgs::RetrieveGraspDemonstrationGoal goal;
      goal.id = id;
      retrieve_grasp_ac_.sendGoal(goal);
    } else if (retrieve_grasp_model_ac_.isServerConnected())
    {
      rail_pick_and_place_msgs::RetrieveGraspModelGoal goal;
      goal.id = id;
      retrieve_grasp_model_ac_.sendGoal(goal);
    }
  } else
  {
    // disable the delete button
    delete_button_->setText("Delete");
    delete_button_->setEnabled(false);
  }
}

void ModelGenerationPanel::deleteModel()
{
  if (models_list_->currentItem() != NULL)
  {
    // grab the current item
    string selected_item = models_list_->currentItem()->text().toStdString();
    // extract the ID
    int id = atoi(selected_item.substr(selected_item.find(' ')).c_str());

    // confirmation dialog
    string delete_text = "Are you sure you want to delete " + selected_item + "?";
    QMessageBox::StandardButton confirm = QMessageBox::question(this, "Delete?", delete_text.c_str(),
        QMessageBox::Yes | QMessageBox::No);
    if (confirm == QMessageBox::Yes)
    {
      // check for a grasp or a model
      if (selected_item[0] == 'G')
      {
        graspdb_->deleteGraspDemonstration(id);
      } else
      {
        graspdb_->deleteGraspModel(id);
      }
      delete models_list_->currentItem();
    }
  }
}

void ModelGenerationPanel::populateModelsList(const QString &text)
{
  // check if an object exists
  if (object_list_->count() > 0)
  {
    // clear the current list
    models_list_->clear();

    // load grasps/models
    vector<graspdb::GraspDemonstration> demonstrations;
    vector<graspdb::GraspModel> models;
    graspdb_->loadGraspDemonstrationsByObjectName(text.toStdString(), demonstrations);
    graspdb_->loadGraspModelsByObjectName(text.toStdString(), models);

    // first add grasp demonstrations
    if (demonstrations.size() > 0)
    {
      // header
      QListWidgetItem *grasps_label = new QListWidgetItem("--Grasp Demonstrations--", models_list_);
      grasps_label->setTextAlignment(Qt::AlignCenter);
      // makes it so the user can't select this label
      grasps_label->setFlags(Qt::ItemIsEnabled);
      // add each demonstration
      for (size_t i = 0; i < demonstrations.size(); i++)
      {
        stringstream ss;
        ss << "Grasp " << demonstrations[i].getID();
        QListWidgetItem *item = new QListWidgetItem(ss.str().c_str(), models_list_);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(Qt::Unchecked);
      }
    }

    if (models.size() > 0)
    {
      QListWidgetItem *models_label = new QListWidgetItem("--Object Models--", models_list_);
      models_label->setTextAlignment(Qt::AlignCenter);
      // makes it so the user can't select this label
      models_label->setFlags(Qt::ItemIsEnabled);
      // add each model
      for (size_t i = 0; i < models.size(); i++)
      {
        stringstream ss;
        ss << "Model " << models[i].getID();
        QListWidgetItem *item = new QListWidgetItem(ss.str().c_str(), models_list_);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(Qt::Unchecked);
      }
    }
  }
}

void ModelGenerationPanel::executeGenerateModels()
{
  // disable the button first
  generate_models_button_->setEnabled(false);

  if (!generate_models_ac_.isServerConnected())
  {
    model_generation_status_->setText("Model generation action server not found!");
    // make sure to re-enable the button
    generate_models_button_->setEnabled(true);
  } else
  {
    // grab the selected models
    rail_pick_and_place_msgs::GenerateModelsGoal goal;
    for (size_t i = 0; i < models_list_->count(); i++)
    {
      if (models_list_->item(i)->checkState() == Qt::Checked)
      {
        // grab the current item
        string selected_item = models_list_->item(i)->text().toStdString();
        // extract the ID
        int id = atoi(selected_item.substr(selected_item.find(' ')).c_str());

        if (selected_item[0] == 'G')
        {
          goal.grasp_demonstration_ids.push_back(id);
        } else
        {
          goal.grasp_model_ids.push_back(id);
        }
      }
    }

    // check the size
    if (goal.grasp_demonstration_ids.size() + goal.grasp_model_ids.size() == 0)
    {
      model_generation_status_->setText("No grasps or models selected.");
      // make sure to re-enable the button
      generate_models_button_->setEnabled(true);
    } else
    {
      // get the max model size
      goal.max_model_size = model_size_spin_box_->value();
      // send the goal asynchronously
      generate_models_ac_.sendGoal(goal, boost::bind(&ModelGenerationPanel::doneCallback, this, _1, _2),
          actionlib::SimpleActionClient<rail_pick_and_place_msgs::GenerateModelsAction>::SimpleActiveCallback(),
          boost::bind(&ModelGenerationPanel::feedbackCallback, this, _1));
    }
  }
}

void ModelGenerationPanel::doneCallback(const actionlib::SimpleClientGoalState &state,
    const rail_pick_and_place_msgs::GenerateModelsResultConstPtr &result)
{
  // check if the action was successful
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    // check how models were generated
    if (result->new_model_ids.size() > 0)
    {
      // built the final status string
      stringstream ss;
      ss << result->new_model_ids.size() << " model(s) successfully stored with ID(s) [";
      // update the model list while getting the IDs
      for (size_t i = 0; i < result->new_model_ids.size(); i++)
      {
        // add to the status message
        ss << result->new_model_ids[i];
        if ((int) i <= ((int) result->new_model_ids.size()) - 2)
        {
          ss << ", ";
        }

        // add to the models list
        stringstream ss2;
        ss2 << "Model " << result->new_model_ids[i];
        QListWidgetItem *item = new QListWidgetItem(ss2.str().c_str(), models_list_);
        item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
        item->setCheckState(Qt::Unchecked);
      }
      ss << "].";
      model_generation_status_->setText(ss.str().c_str());
    } else
    {
      model_generation_status_->setText("No valid models generated.");
    }
  } else
  {
    // state text should represent what went wrong
    model_generation_status_->setText(state.getText().c_str());
  }

  // re-enable the button
  generate_models_button_->setEnabled(true);
}

void ModelGenerationPanel::feedbackCallback(const rail_pick_and_place_msgs::GenerateModelsFeedbackConstPtr &feedback)
{
  // simply set the status to the current message
  model_generation_status_->setText(feedback->message.c_str());
}

void ModelGenerationPanel::save(rviz::Config config) const
{
  // first call the super class
  rviz::Panel::save(config);
  // save the model config
  config.mapSetValue("MaxModelSize", model_size_spin_box_->value());
}

void ModelGenerationPanel::load(const rviz::Config &config)
{
  // first call the super class
  rviz::Panel::load(config);
  // load the model config
  int max_model_size;
  if (config.mapGetInt("MaxModelSize", &max_model_size))
  {
    model_size_spin_box_->setValue(max_model_size);
  }
}

// tell pluginlib about this class (must outside of any namespace scope)
PLUGINLIB_EXPORT_CLASS(rail::pick_and_place::ModelGenerationPanel, rviz::Panel)
