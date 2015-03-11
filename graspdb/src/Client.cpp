/*!
 * \file Client.cpp
 * \brief The main grasp database client.
 *
 * The graspdb client can communicate with a PostgreSQL database.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 11, 2015
 */

#include <graspdb/Client.h>
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
      connection_->prepare("grasp_demonstrations.insert",
          "INSERT INTO grasp_demonstrations (object_name, grasp_pose, eef_frame_id, point_cloud) " \
          "VALUES ($1, $2, $3, $4) RETURNING id, created");
      connection_->prepare("grasp_demonstrations.select",
          "SELECT id, object_name, (grasp_pose).robot_fixed_frame_id, (grasp_pose).position, " \
          "(grasp_pose).orientation, eef_frame_id, point_cloud, created FROM grasp_demonstrations WHERE id=$1");
      connection_->prepare("grasp_demonstrations.select_object_name",
          "SELECT id, object_name, (grasp_pose).robot_fixed_frame_id, (grasp_pose).position, " \
          "(grasp_pose).orientation, eef_frame_id, point_cloud, created " \
          "FROM grasp_demonstrations WHERE object_name=$1");
      connection_->prepare("grasp_demonstrations.unique", "SELECT DISTINCT object_name FROM grasp_demonstrations");

      // grasp_models statements
      connection_->prepare("grasp_models.unique", "SELECT DISTINCT object_name FROM grasp_models");

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
  // create the grasp_collections table if it doesn't exist
  string grasp_collections_sql = "CREATE TABLE IF NOT EXISTS grasp_demonstrations (" \
                                   "id SERIAL PRIMARY KEY," \
                                   "object_name VARCHAR NOT NULL," \
                                   "grasp_pose pose NOT NULL," \
                                   "eef_frame_id VARCHAR NOT NULL," \
                                   "point_cloud BYTEA NOT NULL," \
                                   "created TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()" \
                                 ");";
  w.exec(grasp_collections_sql);

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
                        "grasp_model_id INTEGER NOT NULL REFERENCES grasp_models(id)," \
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

bool Client::getUniqueGraspDemonstrationObjectNames(vector<string> &names) const
{
  return this->getStringArrayFromPrepared("grasp_demonstrations.unique", "object_name", names);
}

bool Client::getUniqueGraspModelObjectNames(vector<string> &names) const
{
  return this->getStringArrayFromPrepared("grasp_models.unique", "object_name", names);
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
  pqxx::binarystring point_cloud = this->toBinaryString(gd.getPointCloud());

  // create and execute the query
  pqxx::work w(*connection_);
  pqxx::result result = w.prepared("grasp_demonstrations.insert")(object_name)(grasp_pose)(eef_frame_id)(point_cloud)
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

//TODO void Client::addModel(const Model &m)
//{
//  // check API versions
//#if PQXX_VERSION_MAJOR < 4
//  ROS_ERROR("libpqxx-%s does not support binarystring insertion. Cannot add model to database.", PQXX_VERSION);
//#else
//  // build the SQL bits we need
//  const string &objectName = m.getObjectName();
//  string graspPose = this->toSQL(gd.getGraspPose());
//  pqxx::binarystring pointCloud = this->toBinaryString(gd.getPointCloud());
//
//  // create and execute the query
//  pqxx::work w(*connection_);
//  w.prepared("grasp_demonstrations.insert")(objectName)(graspPose)(pointCloud).exec();
//  w.commit();
//#endif
//}

#endif

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

  // extract the point cloud
  pqxx::binarystring blob(tuple["point_cloud"]);
  gd.setPointCloud(this->extractPointCloud2FromBinaryString(blob));

  return gd;
}

sensor_msgs::PointCloud2 Client::extractPointCloud2FromBinaryString(const pqxx::binarystring &bs) const
{
  sensor_msgs::PointCloud2 pc;
  // deserialize from memory
  ros::serialization::IStream stream((uint8_t *) bs.data(), bs.size());
  ros::serialization::Serializer<sensor_msgs::PointCloud2>::read(stream, pc);
  return pc;
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
