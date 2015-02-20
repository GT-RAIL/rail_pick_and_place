#ifndef RAIL_GRASPDB_CLIENT_H_
#define RAIL_GRASPDB_CLIENT_H_

#include <string>
#include <pqxx/pqxx>

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

  bool connected();

  bool connect();

  void disconnect();

private:
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
