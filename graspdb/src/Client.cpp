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

Client::Client(const string host, const uint16_t port, const string user, const string password, const string db) :
    host_(host), user_(user), password_(password), db_(db)
{
  port_ = port;
  connection_ = NULL;

  // check API versions
#if PQXX_VERSION_MAJOR < 4
  ROS_WARN("libpqxx-%s is not fully supported. Please upgrade to libpqxx-4.0 or greater.", PQXX_VERSION);
#endif
}

Client::~Client()
{
  // check for an existing connection
  this->disconnect();
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
                                   "grasp_pose pose," \
                                   "point_cloud BYTEA," \
                                   "created DATE NOT NULL DEFAULT NOW()" \
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

std::string Client::toSQL(const Pose &p) const
{
  // build the SQL
  string sql = "(\"" + p.getFixedFrameID() + "\", \"" + p.getGraspFrameID() + "\", " + this->toSQL(p.getPosition())
      + "\", \"" + this->toSQL(p.getOrientation()) + "\")";
  return sql;
}

std::string Client::toSQL(const Position &p) const
{
  // build the SQL
  stringstream ss;
  ss << "{" << p.getX() << ", " << p.getY() << ", " << p.getZ() << "}";
  return ss.str();
}

std::string Client::toSQL(const Orientation &o) const
{
  // build the SQL
  stringstream ss;
  ss << "{" << o.getX() << ", " << o.getY() << ", " << o.getZ() << ", " << o.getW() << "}";
  return ss.str();
}
