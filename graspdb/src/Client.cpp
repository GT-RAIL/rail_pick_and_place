/*!
 * \file Client.cpp
 * \brief The main grasp database client.
 *
 * The graspdb client can communicate with a PostgreSQL database.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 3, 2015
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

Client::Client(const string host, const uint16_t port, const string user, const string password, const string db) :
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
      // set up the prepared statements
      connection_->prepare("pg_type.exists",
          "SELECT EXISTS (SELECT 1 FROM pg_type WHERE typname=$1)");
      connection_->prepare("grasp_demonstrations.insert",
          "INSERT INTO grasp_demonstrations (object_name, grasp_pose, point_cloud) VALUES ($1, $2, $3)");
      connection_->prepare("grasp_demonstrations.select",
          "SELECT id, object_name, (grasp_pose).fixed_frame_id, (grasp_pose).grasp_frame_id, (grasp_pose).position, " \
          "(grasp_pose).orientation, point_cloud, created FROM grasp_demonstrations WHERE id=$1");
      connection_->prepare("grasp_demonstrations.select_object_name",
          "SELECT id, object_name, (grasp_pose).fixed_frame_id, (grasp_pose).grasp_frame_id, (grasp_pose).position, " \
          "(grasp_pose).orientation, point_cloud, created FROM grasp_demonstrations WHERE object_name=$1");
      connection_->prepare("grasp_demonstrations.unique",
          "SELECT DISTINCT object_name FROM grasp_demonstrations");
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
                   "fixed_frame_id VARCHAR," \
                   "grasp_frame_id VARCHAR," \
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
                                   "point_cloud BYTEA NOT NULL," \
                                   "created TIMESTAMP WITH TIME ZONE NOT NULL DEFAULT NOW()" \
                                 ");";
  w.exec(grasp_collections_sql);

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

void Client::addGraspDemonstration(const GraspDemonstration &gd)
{
  // check API versions
#if PQXX_VERSION_MAJOR < 4
  ROS_ERROR("libpqxx-%s does not support binarystring insertion. Cannot add grasp to database.", PQXX_VERSION);
#else
  // build the SQL bits we need
  const string &objectName = gd.getObjectName();
  string graspPose = this->toSQL(gd.getGraspPose());
  pqxx::binarystring pointCloud(gd.getPointCloud(), gd.getPointCloudSize());

  // create and execute the query
  pqxx::work w(*connection_);
  w.prepared("grasp_demonstrations.insert")(objectName)(graspPose)(pointCloud).exec();
  w.commit();
#endif
}

bool Client::loadGraspDemonstration(uint32_t id, GraspDemonstration &gd)
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
    this->extractGraspDemonstrationFromTuple(result[0], gd);
    return true;
  }
}

bool Client::loadGraspDemonstrationsByObjectName(const std::string &object_name, std::vector<GraspDemonstration> &gds)
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

bool Client::getUniqueGraspDemonstrationObjectNames(std::vector<std::string> &names)
{
  // create and execute the query
  pqxx::work w(*connection_);
  pqxx::result result = w.prepared("grasp_demonstrations.unique").exec();
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
      names.push_back(result[i]["object_name"].as<string>());
    }
    return true;
  }
}

void Client::extractGraspDemonstrationFromTuple(const pqxx::result::tuple &tuple, GraspDemonstration &gd) const
{
  // create the Position element
  vector<double> position_values = this->extractArrayFromString(tuple["position"].as<string>());
  Position p(position_values[0], position_values[1], position_values[2]);

  // create the Orientation element
  vector<double> orientation_values = this->extractArrayFromString(tuple["orientation"].as<string>());
  Orientation o(orientation_values[0], orientation_values[1], orientation_values[2], orientation_values[3]);

  // create the Pose element
  Pose pose(tuple["fixed_frame_id"].as<string>(), tuple["grasp_frame_id"].as<string>(), p, o);

  // set our easy fields
  gd.setID(tuple["id"].as<uint32_t>());
  gd.setObjectName(tuple["object_name"].as<string>());
  gd.setGraspPose(pose);
  gd.setCreated(this->extractTimeFromString(tuple["created"].as<string>()));

  // extract the point cloud
  pqxx::binarystring blob(tuple["point_cloud"]);
  gd.setPointCloud(blob.data(), blob.size());
}

GraspDemonstration Client::extractGraspDemonstrationFromTuple(const pqxx::result::tuple &tuple) const
{
  GraspDemonstration gd;
  this->extractGraspDemonstrationFromTuple(tuple, gd);
  return gd;
}

void Client::extractArrayFromString(string array, vector<double> &values) const
{
  // remove the brackets and spaces
  array.erase(std::remove(array.begin(), array.end(), '{'), array.end());
  array.erase(std::remove(array.begin(), array.end(), '}'), array.end());
  array.erase(std::remove(array.begin(), array.end(), ' '), array.end());

  // split on the ','
  stringstream ss(array);
  string str;
  double dbl;
  while (std::getline(ss, str, ','))
  {
    // store as the double value
    istringstream i(str);
    i >> dbl;
    values.push_back(dbl);
  }
}

vector<double> Client::extractArrayFromString(string array) const
{
  vector<double> values;
  this->extractArrayFromString(array, values);
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


std::string Client::toSQL(const Pose &p) const
{
  // build the SQL
  string sql = "(\"" + p.getFixedFrameID() + "\",\"" + p.getGraspFrameID() + "\",\"" + this->toSQL(p.getPosition())
      + "\",\"" + this->toSQL(p.getOrientation()) + "\")";
  return sql;
}

std::string Client::toSQL(const Position &p) const
{
  // build the SQL
  stringstream ss;
  ss << "{" << p.getX() << "," << p.getY() << "," << p.getZ() << "}";
  return ss.str();
}

std::string Client::toSQL(const Orientation &o) const
{
  // build the SQL
  stringstream ss;
  ss << "{" << o.getX() << "," << o.getY() << ", " << o.getZ() << "," << o.getW() << "}";
  return ss.str();
}
