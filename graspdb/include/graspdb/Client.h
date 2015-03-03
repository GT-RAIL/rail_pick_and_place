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
  Client(const std::string host, const unsigned int port, const std::string user, const std::string password, const std::string db);

  ~Client();

  unsigned int getPort() const;

  std::string getHost() const;

  std::string getUser() const;

  std::string getPassword() const;

  std::string getDatabase() const;

  bool connected() const;

  bool connect();

  void disconnect();

  void addGraspDemonstration(const GraspDemonstration &gd);

private:
  void createTables() const;

  bool doesTypeExist(const std::string &type) const;

  std::string toSQL(const Pose &p) const;

  std::string toSQL(const Position &p) const;

  std::string toSQL(const Orientation &o) const;

  std::string host_, user_, password_, db_;
  unsigned int port_;
  pqxx::connection *connection_;
};

}
}
}

#endif
