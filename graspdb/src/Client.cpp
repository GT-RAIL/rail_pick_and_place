/*!
 * \file Client.cpp
 * \brief The main grasp database client.
 *
 * The graspdb client can communicate with a PostgreSQL database.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 11, 2015
 */

// graspdb
#include "graspdb/Client.h"

// ROS
#include <ros/ros.h>

using namespace std;
using namespace rail::pick_and_place::graspdb;

Client::Client(const Client &c)
    : host_(c.getHost()), user_(c.getUser()), password_(c.getPassword()), db_(c.getDatabase())
{
  port_ = c.getPort();
  connection_ = NULL;

  // check if a connection was made
  if (c.connected())
  {
    this->connect();
  }

  // check API versions
  this->checkAPIVersion();
}

Client::Client(const string &host, const uint16_t port, const string &user, const string &password, const string &db) :
    host_(host), user_(user), password_(password), db_(db)
{
  port_ = port;
  connection_ = NULL;

  // check API versions
  this->checkAPIVersion();
}

Client::~Client()
{
  // check for an existing connection
  this->disconnect();
}

void Client::checkAPIVersion() const
{
  // check API versions
#if PQXX_VERSION_MAJOR < 4
  ROS_WARN("libpqxx-%s is not fully supported. Please upgrade to libpqxx-4.0 or greater.", PQXX_VERSION);
#endif
}

uint16_t Client::getPort() const
{
  return port_;
}

const string &Client::getHost() const
{
  return host_;
}

const string &Client::getUser() const
{
  return user_;
}

const string &Client::getPassword() const
{
  return password_;
}

const string &Client::getDatabase() const
{
  return db_;
}

bool Client::connected() const
{
  return connection_ != NULL && connection_->is_open();
}

bool Client::connect()
{
  // check for an existing connection
  this->disconnect();

  try
  {
    // build the connection
    stringstream ss;
    ss << "dbname=" << db_ << " user=" << user_ << " password=" << password_;
    ss << " hostaddr=" << host_ << " port=" << port_;
    connection_ = new pqxx::connection(ss.str());

    if (this->connected())
    {
      // general statements
      connection_->prepare("pg_type.exists", "SELECT EXISTS (SELECT 1 FROM pg_type WHERE typname=$1)");

      // grasp_demonstrations statements
      connection_->prepare("grasp_demonstrations.delete", "DELETE FROM grasp_demonstrations WHERE id=$1");
      connection_->prepare("grasp_demonstrations.insert",
                           "INSERT INTO grasp_demonstrations " \
                           "(object_name, grasp_pose, eef_frame_id, point_cloud, image) " \
                           "VALUES (UPPER($1), $2, $3, $4, $5) RETURNING id, created");
      connection_->prepare("grasp_demonstrations.select",
                           "SELECT id, object_name, (grasp_pose).robot_fixed_frame_id, (grasp_pose).position, " \
          "(grasp_pose).orientation, eef_frame_id, point_cloud, image, created FROM grasp_demonstrations WHERE id=$1");
      connection_->prepare("grasp_demonstrations.select_all",
                           "SELECT id, object_name, (grasp_pose).robot_fixed_frame_id, (grasp_pose).position, " \
          "(grasp_pose).orientation, eef_frame_id, point_cloud, image, created FROM grasp_demonstrations");
      connection_->prepare("grasp_demonstrations.select_object_name",
                           "SELECT id, object_name, (grasp_pose).robot_fixed_frame_id, (grasp_pose).position, " \
          "(grasp_pose).orientation, eef_frame_id, point_cloud, image, created " \
          "FROM grasp_demonstrations WHERE UPPER(object_name)=UPPER($1)");
      connection_->prepare("grasp_demonstrations.unique", "SELECT DISTINCT object_name FROM grasp_demonstrations");

      // grasp_models statements
      connection_->prepare("grasp_models.delete", "DELETE FROM grasp_models WHERE id=$1");
      connection_->prepare("grasp_models.insert", "INSERT INTO grasp_models (object_name, point_cloud) " \
                           "VALUES (UPPER($1), $2, $3, $4, $5) RETURNING id, created");
      connection_->prepare("grasp_models.select",
                           "SELECT id, object_name, point_cloud, created FROM grasp_models WHERE id=$1");
      connection_->prepare("grasp_models.select_all", "SELECT id, object_name, point_cloud, created FROM grasp_models");
      connection_->prepare("grasp_models.select_object_name", "SELECT id, object_name, point_cloud, created " \
                           "FROM grasp_models WHERE UPPER(object_name)=UPPER($1)");
      connection_->prepare("grasp_models.unique", "SELECT DISTINCT object_name FROM grasp_models");

      // grasps statements
      connection_->prepare("grasps.delete", "DELETE FROM grasps WHERE id=$1");
      connection_->prepare("grasps.insert",
                           "INSERT INTO grasps (grasp_model_id, grasp_pose, eef_frame_id, successes, attempts) " \
          "VALUES ($1, $2, $3, $4, $5) RETURNING id, created");
      connection_->prepare("grasps.select",
                           "SELECT id, grasp_model_id, (grasp_pose).robot_fixed_frame_id, (grasp_pose).position, " \
          "(grasp_pose).orientation, eef_frame_id, successes, attempts, created FROM grasps WHERE id=$1");
      connection_->prepare("grasps.select_grasp_model_id",
                           "SELECT id, grasp_model_id, (grasp_pose).robot_fixed_frame_id, (grasp_pose).position, " \
          "(grasp_pose).orientation, eef_frame_id, successes, attempts, created FROM grasps  WHERE grasp_model_id=$1");

      // create the tables in the DB if they do not exist
      this->createTables();
    }
  } catch (const exception &e)
  {
    ROS_ERROR("%s", e.what());
  }

  return this->connected();
}

void Client::disconnect()
{
  // check for an existing connection
  if (connection_ != NULL)
  {
    if (this->connected())
    {
      connection_->disconnect();
    }
    delete connection_;
    connection_ = NULL;
  }
}

void Client::createTables() const
{
  // check for and create the pose type
  if (!this->doesTypeExist("pose"))
  {
    pqxx::work w(*connection_);
    string sql = "CREATE TYPE pose AS (" \
                   "robot_fixed_frame_id VARCHAR," \
                   "position NUMERIC[3]," \
                   "orientation NUMERIC[4]" \
                 ");";
    w.exec(sql);
    w.commit();
  }

  // shared worker
  pqxx::work w(*connection_);
  // create the grasp_demonstrations table if it doesn't exist
  string grasp_demonstrations_sql = "CREATE TABLE IF NOT EXISTS grasp_demonstrations (" \
                                   "id SERIAL PRIMARY KEY," \
                                   "object_name VARCHAR NOT NULL," \
                                   "grasp_pose pose NOT NULL," \
                                   "eef_frame_id VARCHAR NOT NULL," \
                                   "point_cloud BYTEA NOT NULL," \
                                   "image BYTEA NOT NULL," \
                                   "created TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()" \
                                 ");";
  w.exec(grasp_demonstrations_sql);

  // create the grasp models table if it doesn't exist
  string grasp_models_sql = "CREATE TABLE IF NOT EXISTS grasp_models (" \
                              "id SERIAL PRIMARY KEY," \
                              "object_name VARCHAR NOT NULL," \
                              "point_cloud BYTEA NOT NULL," \
                              "created TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()" \
                            ");";
  w.exec(grasp_models_sql);

  // create the grasps table if it doesn't exist
  string grasps_sql = "CREATE TABLE IF NOT EXISTS grasps (" \
                        "id SERIAL PRIMARY KEY," \
                        "grasp_model_id INTEGER NOT NULL REFERENCES grasp_models(id) ON DELETE CASCADE," \
                        "grasp_pose pose NOT NULL," \
                        "eef_frame_id VARCHAR NOT NULL," \
                        "successes INTEGER NOT NULL," \
                        "attempts INTEGER NOT NULL," \
                        "created TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()" \
                      ");";
  w.exec(grasps_sql);

  // commit the changes
  w.commit();
}

bool Client::doesTypeExist(const string &type) const
{
  pqxx::work w(*connection_);
  // create and execute the query
  pqxx::result result = w.prepared("pg_type.exists")(type).exec();
  w.commit();
  // return the result
  return result[0][0].as<bool>();
}

bool Client::loadGraspDemonstration(uint32_t id, GraspDemonstration &gd) const
{
  // create and execute the query
  pqxx::work w(*connection_);
  pqxx::result result = w.prepared("grasp_demonstrations.select")(id).exec();
  w.commit();

  // check the result
  if (result.empty())
  {
    return false;
  } else
  {
    // extract the information
    gd = this->extractGraspDemonstrationFromTuple(result[0]);
    return true;
  }
}

bool Client::loadGraspDemonstrations(vector<GraspDemonstration> &gds) const
{
  // create and execute the query
  pqxx::work w(*connection_);
  pqxx::result result = w.prepared("grasp_demonstrations.select_all").exec();
  w.commit();

  // check the result
  if (result.empty())
  {
    return false;
  } else
  {
    // extract each result
    for (size_t i = 0; i < result.size(); i++)
    {
      gds.push_back(this->extractGraspDemonstrationFromTuple(result[i]));
    }
    return true;
  }
}

bool Client::loadGraspDemonstrationsByObjectName(const string &object_name, vector<GraspDemonstration> &gds) const
{
  // create and execute the query
  pqxx::work w(*connection_);
  pqxx::result result = w.prepared("grasp_demonstrations.select_object_name")(object_name).exec();
  w.commit();

  // check the result
  if (result.empty())
  {
    return false;
  } else
  {
    // extract each result
    for (size_t i = 0; i < result.size(); i++)
    {
      gds.push_back(this->extractGraspDemonstrationFromTuple(result[i]));
    }
    return true;
  }
}

bool Client::loadGrasp(uint32_t id, Grasp &grasp) const
{
  // create and execute the query
  pqxx::work w(*connection_);
  pqxx::result result = w.prepared("grasps.select")(id).exec();
  w.commit();

  // check the result
  if (result.empty())
  {
    return false;
  } else
  {
    // extract the information
    grasp = this->extractGraspFromTuple(result[0]);
    return true;
  }
}

bool Client::loadGraspByGraspModelID(const uint32_t grasp_model_id, vector<Grasp> &grasps) const
{
  // create and execute the query
  pqxx::work w(*connection_);
  pqxx::result result = w.prepared("grasps.select_grasp_model_id")(grasp_model_id).exec();
  w.commit();

  // check the result
  if (result.empty())
  {
    return false;
  } else
  {
    // extract each result
    for (size_t i = 0; i < result.size(); i++)
    {
      grasps.push_back(this->extractGraspFromTuple(result[i]));
    }
    return true;
  }
}

bool Client::loadGraspModel(uint32_t id, GraspModel &gm) const
{
  // create and execute the query
  pqxx::work w(*connection_);
  pqxx::result result = w.prepared("grasp_models.select")(id).exec();
  w.commit();

  // check the result
  if (result.empty())
  {
    return false;
  } else
  {
    // extract the information
    gm = this->extractGraspModelFromTuple(result[0]);
    // now load the grasps
    vector<Grasp> grasps;
    this->loadGraspByGraspModelID(id, grasps);
    // add each grasp
    for (size_t i = 0; i < grasps.size(); i++)
    {
      gm.addGrasp(grasps[i]);
    }
    return true;
  }
}

bool Client::loadGraspModels(vector<GraspModel> &gms) const
{
  // create and execute the query
  pqxx::work w(*connection_);
  pqxx::result result = w.prepared("grasp_models.select_all").exec();
  w.commit();

  // check the result
  if (result.empty())
  {
    return false;
  } else
  {
    // extract each result
    for (size_t i = 0; i < result.size(); i++)
    {
      GraspModel gm = this->extractGraspModelFromTuple(result[i]);
      // now load the grasps
      vector<Grasp> grasps;
      this->loadGraspByGraspModelID(gm.getID(), grasps);
      // add each grasp
      for (size_t i = 0; i < grasps.size(); i++)
      {
        gm.addGrasp(grasps[i]);
      }
      gms.push_back(gm);
    }
    return true;
  }
}

bool Client::loadGraspModelsByObjectName(const string &object_name, vector<GraspModel> &gms) const
{
  // create and execute the query
  pqxx::work w(*connection_);
  pqxx::result result = w.prepared("grasp_models.select_object_name")(object_name).exec();
  w.commit();

  // check the result
  if (result.empty())
  {
    return false;
  } else
  {
    // extract each result
    for (size_t i = 0; i < result.size(); i++)
    {
      GraspModel gm = this->extractGraspModelFromTuple(result[i]);
      // now load the grasps
      vector<Grasp> grasps;
      this->loadGraspByGraspModelID(gm.getID(), grasps);
      // add each grasp
      for (size_t i = 0; i < grasps.size(); i++)
      {
        gm.addGrasp(grasps[i]);
      }
      gms.push_back(gm);
    }
    return true;
  }
}

bool Client::getUniqueGraspDemonstrationObjectNames(vector<string> &names) const
{
  return this->getStringArrayFromPrepared("grasp_demonstrations.unique", "object_name", names);
}

bool Client::getUniqueGraspModelObjectNames(vector<string> &names) const
{
  return this->getStringArrayFromPrepared("grasp_models.unique", "object_name", names);
}

bool Client::addGrasp(Grasp &grasp) const
{
  // build the SQL bits we need
  uint32_t grasp_model_id = grasp.getGraspModelID();
  const string &grasp_pose = this->toSQL(grasp.getGraspPose());
  const string &eef_frame_id = grasp.getEefFrameID();
  uint32_t succeses = grasp.getSuccesses();
  uint32_t attempts = grasp.getAttempts();

  // create and execute the query
  pqxx::work w(*connection_);
  pqxx::result result = w.prepared("grasps.insert")(grasp_model_id)(grasp_pose)(eef_frame_id)(succeses)(attempts)
      .exec();
  w.commit();

  // check the result
  if (!result.empty())
  {
    grasp.setID(result[0]["id"].as<uint32_t>());
    grasp.setCreated(this->extractTimeFromString(result[0]["created"].as<string>()));
    return true;
  } else
  {
    return false;
  }
}

// check API versions
#if PQXX_VERSION_MAJOR >= 4

/* Only pqxx 4.0.0 or greater support insert with binary strings */

bool Client::addGraspDemonstration(GraspDemonstration &gd) const
{
  // build the SQL bits we need
  const string &object_name = gd.getObjectName();
  string grasp_pose = this->toSQL(gd.getGraspPose());
  const string &eef_frame_id = gd.getEefFrameID();
  pqxx::binarystring pc = this->toBinaryString(gd.getPointCloud());
  pqxx::binarystring image = this->toBinaryString(gd.getImage());

  // create and execute the query
  pqxx::work w(*connection_);
  pqxx::result result = w.prepared("grasp_demonstrations.insert")(object_name)(grasp_pose)(eef_frame_id)(pc)(image)
      .exec();
  w.commit();

  // check the result
  if (!result.empty())
  {
    gd.setID(result[0]["id"].as<uint32_t>());
    gd.setCreated(this->extractTimeFromString(result[0]["created"].as<string>()));
    return true;
  } else
  {
    return false;
  }
}

bool Client::addGraspModel(GraspModel &gm) const
{
  // build the SQL bits we need
  const string &object_name = gm.getObjectName();
  pqxx::binarystring pc = this->toBinaryString(gm.getPointCloud());

  // create and execute the query
  pqxx::work w(*connection_);
  pqxx::result result = w.prepared("grasp_models.insert")(object_name)(pc).exec();
  w.commit();

  // check the result
  if (!result.empty())
  {
    gm.setID(result[0]["id"].as<uint32_t>());
    gm.setCreated(this->extractTimeFromString(result[0]["created"].as<string>()));

    // used to add back the grasps with the correct ID and created fields
    vector<Grasp> addedGrasps;

    // now add each grasp to the database

    for (size_t i = 0; i < gm.getNumGrasps(); i++)
    {
      // set the model ID and attempt to add it
      Grasp grasp = gm.getGrasp(i);
      grasp.setGraspModelID(gm.getID());
      if (this->addGrasp(grasp))
      {
        addedGrasps.push_back(grasp);
      }
    }

    // clear out the old grasp values
    while (gm.getNumGrasps() > 0)
    {
      gm.removeGrasp(0);
    }

    // add each new grasp
    for (size_t i = 0; i < addedGrasps.size(); i++)
    {
      gm.addGrasp(addedGrasps[i]);
    }

    return true;
  } else
  {
    return false;
  }
}

#else

bool Client::addGraspModel(GraspModel &gm) const
{
  ROS_WARN("libpqxx-%s does not support binary string insertion. Add grasp model ignored.", PQXX_VERSION);
  return false;
}

bool Client::addGraspDemonstration(GraspDemonstration &gd) const
{
  ROS_WARN("libpqxx-%s does not support binary string insertion. Add grasp demonstration ignored.", PQXX_VERSION);
  return false;
}

#endif

void Client::deleteGrasp(uint32_t id) const
{
  // create and execute the query
  pqxx::work w(*connection_);
  pqxx::result result = w.prepared("grasps.delete")(id).exec();
  w.commit();
}

void Client::deleteGraspDemonstration(uint32_t id) const
{
  // create and execute the query
  pqxx::work w(*connection_);
  pqxx::result result = w.prepared("grasp_demonstrations.delete")(id).exec();
  w.commit();
}

void Client::deleteGraspModel(uint32_t id) const
{
  // create and execute the query
  pqxx::work w(*connection_);
  pqxx::result result = w.prepared("grasp_models.delete")(id).exec();
  w.commit();
}

GraspDemonstration Client::extractGraspDemonstrationFromTuple(const pqxx::result::tuple &tuple) const
{
  // to return
  GraspDemonstration gd;

  // create the Position element
  string position_string = tuple["position"].as<string>();
  vector<double> position_values = this->extractArrayFromString(position_string);
  Position pos(position_values[0], position_values[1], position_values[2]);

  // create the Orientation element
  string orientation_string = tuple["orientation"].as<string>();
  vector<double> orientation_values = this->extractArrayFromString(orientation_string);
  Orientation ori(orientation_values[0], orientation_values[1], orientation_values[2], orientation_values[3]);

  // create the Pose element
  Pose pose(tuple["robot_fixed_frame_id"].as<string>(), pos, ori);

  // set our fields
  gd.setID(tuple["id"].as<uint32_t>());
  gd.setObjectName(tuple["object_name"].as<string>());
  gd.setGraspPose(pose);
  gd.setEefFrameID(tuple["eef_frame_id"].as<string>());
  gd.setCreated(this->extractTimeFromString(tuple["created"].as<string>()));

  // extract the point cloud if there is one
  if (tuple["point_cloud"].size() > 0)
  {
    pqxx::binarystring blob(tuple["point_cloud"]);
    gd.setPointCloud(this->extractPointCloud2FromBinaryString(blob));
  }

  // extract the image if there is one
  if (tuple["image"].size() > 0)
  {
    pqxx::binarystring blob(tuple["image"]);
    gd.setImage(this->extractImageFromBinaryString(blob));
  }

  return gd;
}

Grasp Client::extractGraspFromTuple(const pqxx::result::tuple &tuple) const
{
  // to return
  Grasp grasp;

  // create the Position element
  string position_string = tuple["position"].as<string>();
  vector<double> position_values = this->extractArrayFromString(position_string);
  Position pos(position_values[0], position_values[1], position_values[2]);

  // create the Orientation element
  string orientation_string = tuple["orientation"].as<string>();
  vector<double> orientation_values = this->extractArrayFromString(orientation_string);
  Orientation ori(orientation_values[0], orientation_values[1], orientation_values[2], orientation_values[3]);

  // create the Pose element
  Pose pose(tuple["robot_fixed_frame_id"].as<string>(), pos, ori);

  // set our fields
  grasp.setID(tuple["id"].as<uint32_t>());
  grasp.setGraspModelID(tuple["grasp_model_id"].as<uint32_t>());
  grasp.setGraspPose(pose);
  grasp.setEefFrameID(tuple["eef_frame_id"].as<string>());
  grasp.setSuccesses(tuple["successes"].as<uint32_t>());
  grasp.setAttempts(tuple["attempts"].as<uint32_t>());
  grasp.setCreated(this->extractTimeFromString(tuple["created"].as<string>()));

  return grasp;
}

GraspModel Client::extractGraspModelFromTuple(const pqxx::result::tuple &tuple) const
{
  // to return
  GraspModel gm;

  // set our fields
  gm.setID(tuple["id"].as<uint32_t>());
  gm.setObjectName(tuple["object_name"].as<string>());
  gm.setCreated(this->extractTimeFromString(tuple["created"].as<string>()));

  // extract the point cloud if there is one
  if (tuple["point_cloud"].size() > 0)
  {
    pqxx::binarystring blob(tuple["point_cloud"]);
    gm.setPointCloud(this->extractPointCloud2FromBinaryString(blob));
  }

  return gm;
}

sensor_msgs::PointCloud2 Client::extractPointCloud2FromBinaryString(const pqxx::binarystring &bs) const
{
  sensor_msgs::PointCloud2 pc;
  // deserialize from memory
  ros::serialization::IStream stream((uint8_t *) bs.data(), bs.size());
  ros::serialization::Serializer<sensor_msgs::PointCloud2>::read(stream, pc);
  return pc;
}

sensor_msgs::Image Client::extractImageFromBinaryString(const pqxx::binarystring &bs) const
{
  sensor_msgs::Image image;
  // deserialize from memory
  ros::serialization::IStream stream((uint8_t *) bs.data(), bs.size());
  ros::serialization::Serializer<sensor_msgs::Image>::read(stream, image);
  return image;
}

vector<double> Client::extractArrayFromString(string &array) const
{
  // to return
  vector<double> values;

  // remove the brackets and spaces
  array.erase(remove(array.begin(), array.end(), '{'), array.end());
  array.erase(remove(array.begin(), array.end(), '}'), array.end());
  array.erase(remove(array.begin(), array.end(), ' '), array.end());

  // split on the ','
  stringstream ss(array);
  string str;
  double dbl;
  while (getline(ss, str, ','))
  {
    // store as the double value
    istringstream i(str);
    i >> dbl;
    values.push_back(dbl);
  }

  return values;
}

time_t Client::extractTimeFromString(const string &str) const
{
  // set values we don't need to be 0
  struct tm t;
  bzero(&t, sizeof(t));
  // extract values in a datetime object and the timezone offset into an int (ignore nsec)
  int nsec, tz;
  sscanf(str.c_str(), "%d-%d-%d %d:%d:%d.%d%d", &t.tm_year, &t.tm_mon, &t.tm_mday, &t.tm_hour, &t.tm_min, &t.tm_sec,
         &nsec, &tz);
  // correct the information for C time
  t.tm_year -= 1900;
  t.tm_mon -= 1;
  // fix the timezone offset
  t.tm_hour += tz;
  // convert to a time_t object
  return mktime(&t);
}

bool Client::getStringArrayFromPrepared(const string &prepared_name, const string &column_name,
    vector<string> &strings) const
{
  // create and execute the query
  pqxx::work w(*connection_);
  pqxx::result result = w.prepared(prepared_name).exec();
  w.commit();

  // check the result
  if (result.empty())
  {
    return false;
  } else
  {
    // extract each result
    for (size_t i = 0; i < result.size(); i++)
    {
      strings.push_back(result[i][column_name].as<string>());
    }
    return true;
  }
}

string Client::toSQL(const Pose &p) const
{
  // build the SQL
  string sql = "(\"" + p.getRobotFixedFrameID() + "\",\"" + this->toSQL(p.getPosition()) + "\",\""
               + this->toSQL(p.getOrientation()) + "\")";
  return sql;
}

string Client::toSQL(const Position &p) const
{
  // build the SQL
  stringstream ss;
  ss << "{" << p.getX() << "," << p.getY() << "," << p.getZ() << "}";
  return ss.str();
}

string Client::toSQL(const Orientation &o) const
{
  // build the SQL
  stringstream ss;
  ss << "{" << o.getX() << "," << o.getY() << ", " << o.getZ() << "," << o.getW() << "}";
  return ss.str();
}

// check API versions
#if PQXX_VERSION_MAJOR >= 4

/* Only pqxx 4.0.0 or greater support insert with binary strings */

pqxx::binarystring Client::toBinaryString(const sensor_msgs::PointCloud2 &pc) const
{
  // determine the size for the buffer
  uint32_t size = ros::serialization::serializationLength(pc);
  uint8_t buffer[size];

  // serilize the message
  ros::serialization::OStream stream(buffer, size);
  ros::serialization::serialize(stream, pc);

  // construct a binary string
  pqxx::binarystring binary(buffer, size);
  return binary;
}

pqxx::binarystring Client::toBinaryString(const sensor_msgs::Image &image) const
{
  // determine the size for the buffer
  uint32_t size = ros::serialization::serializationLength(image);
  uint8_t buffer[size];

  // serilize the message
  ros::serialization::OStream stream(buffer, size);
  ros::serialization::serialize(stream, image);

  // construct a binary string
  pqxx::binarystring binary(buffer, size);
  return binary;
}

#endif
