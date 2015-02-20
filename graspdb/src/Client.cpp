#include <graspdb/Client.h>
#include <ros/ros.h>

using namespace std;
using namespace rail::pick_and_place::graspdb;

Client::Client(string host, unsigned int port, string user, string password, string db) :
    host_(host), user_(user), password_(password), db_(db)
{
  port_ = port;

  connection_ = NULL;
}

Client::~Client()
{
  // check for an existing connection
  this->disconnect();
}

unsigned int Client::getPort() const
{
  return port_;
}

string Client::getHost() const
{
  return host_;
}

string Client::getUser() const
{
  return user_;
}

string Client::getPassword() const
{
  return password_;
}

string Client::getDatabase() const
{
  return db_;
}

bool Client::connected()
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
    ss << "dbname=" << db_ << " user=" << user_ << " password=" << password_ << " hostaddr=" << host_ << " port=" << port_;
    connection_ = new pqxx::connection(ss.str());

    if (this->connected())
    {
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

void Client::createTables()
{
  // check for and create the pose type
  if (!this->doesTypeExist("pose"))
  {
    pqxx::work w(*connection_);
    string sql = "CREATE TYPE pose AS (" \
                   "frame_id CHAR(255)," \
                   "position NUMERIC[3]," \
                   "orientation NUMERIC[4]" \
                 ");";
    w.exec(sql);
    w.commit();
  }

  // check for and create the finger joint type
  if (!this->doesTypeExist("finger_joint"))
  {
    pqxx::work w(*connection_);
    string sql = "CREATE TYPE finger_joint AS (" \
                   "name CHAR(255)," \
                   "pose pose" \
                 ");";
    w.exec(sql);
    w.commit();
  }

  // shared worker
  pqxx::work w(*connection_);
  // create the grasp_collections table if it doesn't exist
  string grasp_collections_sql = "CREATE TABLE IF NOT EXISTS grasp_collections (" \
                                   "id INT PRIMARY KEY NOT NULL," \
                                   "object_name CHAR(255) NOT NULL," \
                                   "grasp_pose pose NOT NULL," \
                                   "finger_joints finger_joint[]," \
                                   "point_cloud BYTEA NOT NULL," \
                                   "created DATE NOT NULL DEFAULT NOW()" \
                                 ");";
  w.exec(grasp_collections_sql);

  // commit the changes
  w.commit();
}

bool Client::doesTypeExist(const std::string &type)
{
  pqxx::work w(*connection_);
  // create and execute the query
  pqxx::result result = w.exec("SELECT EXISTS (SELECT 1 FROM pg_type WHERE typname='" + type + "');");
  w.commit();
  // return the result
  return result[0][0].as<bool>();
}