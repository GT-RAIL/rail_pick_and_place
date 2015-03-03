/*!
 * \file Client.h
 * \brief The main grasp database client.
 *
 * The graspdb client can communicate with a PostgreSQL database.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 3, 2015
 */

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

/*!
 * \class Client
 * \brief The main grasp database client.
 *
 * The graspdb client can communicate with a PostgreSQL database.
 */
class Client
{
public:
  /*!
   * \brief Create a new Client.
   *
   * Creates a new Client with the given connection information. A connection is not made by default.
   *
   * \param host The host IP of the database.
   * \param port The host port of the database.
   * \param user The user of the database.
   * \param password The password for the user of the database.
   * \param db The database name.
   */
  Client(const std::string host, const uint16_t port, const std::string user, const std::string password, const std::string db);

  /*!
   * \brief Cleans up a Client.
   *
   * Cleans up any connections used by the Client.
   */
  ~Client();

  /*!
   * \brief Port value accessor.
   *
   * Get the port value of this Client.
   *
   * \return The port value.
   */
  uint16_t getPort() const;

  /*!
   * \brief Host value accessor.
   *
   * Get the host value of this Client.
   *
   * \return The host value.
   */
  const std::string &getHost() const;

  /*!
   * \brief User value accessor.
   *
   * Get the user value of this Client.
   *
   * \return The user value.
   */
  const std::string &getUser() const;

  /*!
   * \brief Password value accessor.
   *
   * Get the password value of this Client.
   *
   * \return The password value.
   */
  const std::string &getPassword() const;

  /*!
   * \brief Database value accessor.
   *
   * Get the database value of this Client.
   *
   * \return The database value.
   */
  const std::string &getDatabase() const;

  /*!
   * \brief Check if there is a connection to the database.
   *
   * A boolean check to see if a connection exists to the grasp database.
   * \return True if a connection has been made.
   */
  bool connected() const;

  /*!
   * \brief Create a connection to the database.
   *
   * Attempts to create a connection to the grasp database. A flag is returned to indicate the success.
   * \return True if a connection has been sucessfully made.
   */
  bool connect();

  /*!
   * \brief Closes a connection to the database.
   *
   * Attempts to close a connection to the grasp database. No effect is seen if there is no current connection.
   */
  void disconnect();

  void addGraspDemonstration(const GraspDemonstration &gd);

private:
  /*!
   * \brief Creates tables and types.
   *
   * Creates the initial table and composite type schemas needed for the database. If these already exist, no action is taken.
   */
  void createTables() const;

  /*!
   * \brief Check if a composite type exists in the database.
   *
   * Makes an SQL call to the database to check if a composite type of the given name exists.
   *
   * \param type The name of the type to check for.
   * \return True if the type composite exists in the database.
   */
  bool doesTypeExist(const std::string &type) const;

  /*!
   * \brief Convert a Pose to a PostgreSQL object string.
   *
   * Converts the given Pose to a PostgreSQL object string for use in SQL queries.
   *
   * \param p The Pose object to convert to a PostgreSQL object string.
   * \return The converted PostgreSQL object string.
   */
  std::string toSQL(const Pose &p) const;

  /*!
   * \brief Convert a Position to a PostgreSQL object string.
   *
   * Converts the given Position to a PostgreSQL object string for use in SQL queries.
   *
   * \param p The Position object to convert to a PostgreSQL object string.
   * \return The converted PostgreSQL object string.
   */
  std::string toSQL(const Position &p) const;

  /*!
   * \brief Convert an Orientation to a PostgreSQL object string.
   *
   * Converts the given Orientation to a PostgreSQL object string for use in SQL queries.
   *
   * \param o The Orientation object to convert to a PostgreSQL object string.
   * \return The converted PostgreSQL object string.
   */
  std::string toSQL(const Orientation &o) const;

  /*! Database connection information. */
  std::string host_, user_, password_, db_;
  /*! Database port information. */
  uint16_t port_;
  /*! The main database connection client. */
  pqxx::connection *connection_;
};

}
}
}

#endif
