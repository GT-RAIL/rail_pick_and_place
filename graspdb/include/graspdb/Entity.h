/*!
 * \file Entity.h
 * \brief An abstract database entity.
 *
 * An entity contains basic database entity information such as an ID and created timestamp.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 11, 2015
 */

#ifndef RAIL_PICK_AND_PLACE_GRASPDB_ENTITY_H_
#define RAIL_PICK_AND_PLACE_GRASPDB_ENTITY_H_

// C++ Standard Library
#include <ctime>
#include <stdint.h>

namespace rail
{
namespace pick_and_place
{
namespace graspdb
{

/*!
 * \class Entity
 * \brief An abstract database entity.
 *
 * An entity contains basic database entity information such as an ID and created timestamp.
 */
class Entity
{
public:
  /*! The default value for an unset identifier (i.e., a demonstration not yet in the database). */
  static const uint32_t UNSET_ID = 0;
  /*! The default value for an unset timestamp (i.e., a demonstration not yet in the database). */
  static const time_t UNSET_TIME = 0;

  /*!
   * \brief Create a new Entity.
   *
   * Creates a new GraspDemonstration with the given values.
   *
   * \param id The unique ID of the database entity (defaults to 0).
   * \param created The created timestamp of the database entity (defaults to 0).
   */
  Entity(const uint32_t id = UNSET_ID, const time_t created = UNSET_TIME);

  /*!
   * \brief ID value accessor.
   *
   * Get the ID value of this Entity.
   *
   * \return The ID value.
   */
  uint32_t getID() const;

  /*!
   * \brief ID value mutator.
   *
   * Set the ID value of this Entity.
   *
   * \param id The new ID value.
   */
  void setID(const uint32_t id);

  /*!
   * \brief Created timestamp value accessor.
   *
   * Get the created timestamp value of this Entity.
   *
   * \return The created timestamp value.
   */
  time_t getCreated() const;

  /*!
   * \brief Created timestamp value mutator.
   *
   * Set the created timestamp value of this Entity.
   *
   * \param created The new created timestamp value.
   */
  void setCreated(const time_t created);

private:
  /*! The ID. */
  uint32_t id_;
  /*! The created timestamp. */
  time_t created_;
};

}
}
}

#endif
