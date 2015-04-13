/*!
 * \file Client.h
 * \brief The main grasp database client.
 *
 * The graspdb client can communicate with a PostgreSQL database.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 11, 2015
 */

#ifndef RAIL_PICK_AND_PLACE_GRASPDB_CLIENT_H_
#define RAIL_PICK_AND_PLACE_GRASPDB_CLIENT_H_

// graspdb
#include "GraspDemonstration.h"
#include "GraspModel.h"

// ROS
#include <sensor_msgs/PointCloud2.h>

// PostgreSQL
#include <pqxx/pqxx>

// C++ Standard Library
#include <string>

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
  /*! The default PostgreSQL port. */
  static const unsigned int DEFAULT_PORT = 5432;

  /*!
   * \brief Create a new Client.
   *
   * Creates a new Client by copying the values from the given Client. A new connection is made if one exists.
   *
   * \param gd The Client to copy.
   */
  Client(const Client &c);

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
  Client(const std::string &host, const uint16_t port, const std::string &user, const std::string &password,
         const std::string &db);

  /*!
   * \brief Cleans up a Client.
   *
   * Cleans up any connections used by the Client.
   */
  virtual ~Client();

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

  /*!
   * \brief Load a grasp demonstration from the database.
   *
   * Load the grasp demonstration data from the database with the given ID and store it in the given GraspDemonstration.
   *
   * \param id The ID of the grasp demonstration to load.
   * \param gd The GraspDemonstration object to fill with the loaded data.
   * \return bool Returns true if a successful load was completed and the data was set correctly.
   */
  bool loadGraspDemonstration(uint32_t id, GraspDemonstration &gd) const;

  /*!
   * \brief Load all grasp demonstrations from the database.
   *
   * Load all grasp demonstrations data from the database with and store them in the given  vector.
   *
   * \param gds The vector to fill with GraspDemonstration objects with the loaded data.
   * \return bool Returns true if a successful load was completed and the data was set correctly.
   */
  bool loadGraspDemonstrations(std::vector<GraspDemonstration> &gds) const;

  /*!
   * \brief Load grasp demonstrations from the database from an object name.
   *
   * Load the grasp demonstrations data from the database with the given object names and store them in the given
   * vector.
   *
   * \param object_name The object name of the grasp demonstrations to load.
   * \param gds The vector to fill with GraspDemonstration objects with the loaded data.
   * \return bool Returns true if a successful load was completed and the data was set correctly.
   */
  bool loadGraspDemonstrationsByObjectName(const std::string &object_name, std::vector<GraspDemonstration> &gds) const;

  /*!
   * \brief Load a grasp from the database.
   *
   * Load the grasp data from the database with the given ID and store it in the given Grasp.
   *
   * \param id The ID of the grasp to load.
   * \param grasp The Grasp object to fill with the loaded data.
   * \return bool Returns true if a successful load was completed and the data was set correctly.
   */
  bool loadGrasp(uint32_t id, Grasp &grasp) const;

  /*!
   * \brief Load grasps from the database from an associated model ID.
   *
   * Load the grasp data from the database with the given grasp model ID and store them in the given vector.
   *
   * \param grasp_model_id The object name of the grasp demonstrations to load.
   * \param grasps The vector to fill with Grasp objects with the loaded data.
   * \return bool Returns true if a successful load was completed and the data was set correctly.
   */
  bool loadGraspByGraspModelID(const uint32_t grasp_model_id, std::vector<Grasp> &grasps) const;

  /*!
   * \brief Load a grasp model from the database.
   *
   * Load the grasp model data from the database with the given ID and store it in the given GraspModel. Associated
   * grasps are also loaded.
   *
   * \param id The ID of the grasp model to load.
   * \param gm The Grasp model object to fill with the loaded data.
   * \return bool Returns true if a successful load was completed and the data was set correctly.
   */
  bool loadGraspModel(uint32_t id, GraspModel &gm) const;

  /*!
   * \brief Load all grasp models from the database.
   *
   * Load all grasp models data from the database and store them in the given vector.
   *
   * \param gms The vector to fill with GraspModel objects with the loaded data.
   * \return bool Returns true if a successful load was completed and the data was set correctly.
   */
  bool loadGraspModels(std::vector<GraspModel> &gms) const;

  /*!
   * \brief Load grasp models from the database from an object name.
   *
   * Load the grasp models data from the database with the given object names and store them in the given vector.
   *
   * \param object_name The object name of the grasp model to load.
   * \param gms The vector to fill with GraspModel objects with the loaded data.
   * \return bool Returns true if a successful load was completed and the data was set correctly.
   */
  bool loadGraspModelsByObjectName(const std::string &object_name, std::vector<GraspModel> &gms) const;

  /*!
   * \brief Load the unique demonstration object names from the database.
   *
   * Load a list of the unique object names from the grasp demonstrations.
   *
   * \param names The vector to fill with the unique names.
   * \return bool Returns true if a successful load was completed and the data was set correctly.
   */
  bool getUniqueGraspDemonstrationObjectNames(std::vector<std::string> &names) const;

  /*!
   * \brief Load the unique grasp model object names from the database.
   *
   * Load a list of the unique object names from the grasp models.
   *
   * \param names The vector to fill with the unique names.
   * \return bool Returns true if a successful load was completed and the data was set correctly.
   */
  bool getUniqueGraspModelObjectNames(std::vector<std::string> &names) const;

  /*!
   * \brief Add a grasp to the database.
   *
   * Stores the given grasp data to the database. If the entity was successfully  added, the ID and created fields of
    * the Grasp are set accordingly.
   *
   * \param g The Grasp with the data to store.
   * \return Returns true if the entity was added to the database.
   */
  bool addGrasp(Grasp &graps) const;

  /*!
   * \brief Add a grasp demonstration to the database.
   *
   * Stores the given grasp demonstration data to the database. If the grasp was successfully added, the ID and created
   * fields of the GraspDemonstration are set accordingly.
   *
   * \param gd The GraspDemonstration with the data to store.
   * \return True if the grasp was successfully added.
   */
  bool addGraspDemonstration(GraspDemonstration &gd) const;

  /*!
   * \brief Add a grasp model and its associated grasps to the database.
   *
   * Stores the given grasp model data and its associated grasps to the database. If the entity was successfully
   * added, the ID and created fields of the GraspModel are set accordingly.
   *
   * \param gm The GraspModel with the data to store.
   * \return Returns true if the entity was added to the database.
   */
  bool addGraspModel(GraspModel &gm) const;

  /*!
   * \brief Delete a grasp from the database.
   *
   * Deletes the grasp from the database with the given ID.
   *
   * \param id The ID of the grasp to delete.
   */
  void deleteGrasp(uint32_t id) const;

  /*!
   * \brief Delete a grasp demonstration from the database.
   *
   * Deletes the grasp demonstration from the database with the given ID.
   *
   * \param id The ID of the grasp demonstration to delete.
   */
  void deleteGraspDemonstration(uint32_t id) const;

  /*!
   * \brief Delete a grasp model from the database.
   *
   * Deletes the grasp model from the database with the given ID. All associated grasps are also deleted.
   *
   * \param id The ID of the grasp model to delete.
   */
  void deleteGraspModel(uint32_t id) const;

private:
  /*!
   * \brief Check for a supported version of the libpqxx API.
   *
   * Checks for a valid version of the libpqxx API (4.0.0 or greater). If one is not found an error message is
   * printed to ROS_WARN. If a valid version is found during compile time, this function has no effect.
   */
  void checkAPIVersion() const;

  /*!
   * \brief Creates tables and types.
   *
   * Creates the initial table and composite type schemas needed for the database. If these already exist, no action
    * is taken.
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
   * \brief Extract a string column from the database.
   *
   * Attempt to get an array of strings from a prepared statement based on a given column name in the result. This
   * will call the named prepared statement and extract the values form the given column and place them in the given
   * vector.
   *
   * \param prepared_name The name of the prepared statement to execute.
   * \param column_name The column name to extract strings from.
   * \param strings The vector to fill with the values.
   * \return bool Returns true if a successful load was completed and the data was set correctly.
   */
  bool getStringArrayFromPrepared(const std::string &prepared_name, const std::string &column_name,
                                  std::vector<std::string> &strings) const;

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

  /*!
   * \brief Extract PointCloud2 values from a binary string.
   *
   * Extracts PointCloud2 values from the given PostgreSQL binary string and places them in a new ROS PointCloud2
   * message.
   *
   * \param bs The binary string representation of the serialized PointCloud2.
   * \return The PointCloud2 with values from the binary string.
   */
  sensor_msgs::PointCloud2 extractPointCloud2FromBinaryString(const pqxx::binarystring &bs) const;

  /*!
   * \brief Extract Image values from a binary string.
   *
   * Extracts Image values from the given PostgreSQL binary string and places them in a new ROS Image message.
   *
   * \param bs The binary string representation of the serialized Image.
   * \return The Image with values from the binary string.
   */
  sensor_msgs::Image extractImageFromBinaryString(const pqxx::binarystring &bs) const;

  /*!
   * \brief Extract array values from a string array with vector creation.
   *
   * Extracts double values from the given PostgreSQL array string (e.g., "{1,2,3}") and places them in a new vector.
   *
   * \param array The array string representation of the array.
   * \return The vector to populated with values from the string.
   */
  std::vector<double> extractArrayFromString(std::string &array) const;

  /*!
   * \brief Extract grasp demonstration information from the SQL result tuple.
   *
   * Extracts values from the given SQL result tuple and places them in a new GraspDemonstration object.
   *
   * \param result The SQL result tuple containing the correct values.
   * \return The GraspDemonstration populated with values from the SQL result tuple.
   */
  GraspDemonstration extractGraspDemonstrationFromTuple(const pqxx::result::tuple &tuple) const;

  /*!
   * \brief Extract grasp information from the SQL result tuple.
   *
   * Extracts values from the given SQL result tuple and places them in a new Grasp object.
   *
   * \param result The SQL result tuple containing the correct values.
   * \return The Grasp populated with values from the SQL result tuple.
   */
  Grasp extractGraspFromTuple(const pqxx::result::tuple &tuple) const;

  /*!
   * \brief Extract grasp model information from the SQL result tuple.
   *
   * Extracts values from the given SQL result tuple and places them in a new GraspModel object.
   *
   * \param result The SQL result tuple containing the correct values.
   * \return The GraspModel populated with values from the SQL result tuple.
   */
  GraspModel extractGraspModelFromTuple(const pqxx::result::tuple &tuple) const;

  /*!
   * \brief Extract a time from a timestamp.
   *
   * Extracts a time value from the given PostgreSQL.
   *
   * \param str The timestamp to parse.
   * \return The time value from the string.
   */
  time_t extractTimeFromString(const std::string &str) const;

// check API versions
#if PQXX_VERSION_MAJOR >= 4
/* Only pqxx 4.0.0 or greater support insert with binary strings */

  /*!
   * \brief Convert a ROS PointCloud2 to a PostgreSQL binary string.
   *
   * Converts the given ROS PointCloud2 message to a PostgreSQL binary string for use in SQL queries.
   *
   * \param pc The ROS PointCloud2 message to convert to a PostgreSQL binary string.
   */
  pqxx::binarystring toBinaryString(const sensor_msgs::PointCloud2 &pc) const;

  /*!
   * \brief Convert a ROS Image to a PostgreSQL binary string.
   *
   * Converts the given ROS Image message to a PostgreSQL binary string for use in SQL queries.
   *
   * \param pc The ROS Image message to convert to a PostgreSQL binary string.
   */
  pqxx::binarystring toBinaryString(const sensor_msgs::Image &image) const;

#endif

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
