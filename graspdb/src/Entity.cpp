/*!
 * \file Entity.cpp
 * \brief An abstract database entity.
 *
 * An entity contains basic database entity information such as an ID and created timestamp.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date March 11, 2015
 */

// graspdb
#include "graspdb/Entity.h"

using namespace rail::pick_and_place::graspdb;

Entity::Entity(const uint32_t id, const time_t created)
{
  id_ = id;
  created_ = created;
}

uint32_t Entity::getID() const
{
  return id_;
}

void Entity::setID(const uint32_t id)
{
  id_ = id;
}

time_t Entity::getCreated() const
{
  return created_;
}

void Entity::setCreated(const time_t created)
{
  created_ = created;
}
