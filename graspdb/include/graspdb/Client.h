#ifndef RAIL_GRASPDB_CLIENT_H_
#define RAIL_GRASPDB_CLIENT_H_

#include <string>
#include <pqxx/pqxx>
#include <graspdb/GraspDemonstration.h>

namespace rail
{
namespace pick_and_place
{
namespace graspdb
{

class Client
{
public:
  Client(std::string host, unsigned int port, std::string user, std::string password, std::string db);

  ~Client();

  unsigned int getPort() const;

  std::string getHost() const;

  std::string getUser() const;

  std::string getPassword() const;

  std::string getDatabase() const;

  bool connected() const;

  bool connect();

  void disconnect();

  void addGraspDemonstration(GraspDemonstration &gd);

private:
  std::string toSQL(Pose &p);
  std::string toSQL(Position &p);
  std::string toSQL(Orientation &o);

  std::string host_, user_, password_, db_;
  unsigned int port_;
  pqxx::connection *connection_;

  void createTables();

  bool doesTypeExist(const std::string &type);
};

}
}
}

#endif
