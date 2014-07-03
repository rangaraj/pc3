/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2007 INRIA
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */
#ifndef PC3_VECTOR_H
#define PC3_VECTOR_H
#include <math.h>
#include <iostream>
#include <string>
#include <sstream>

/**
 * \brief a 3d vector
 */
class Vector3D
{
public:
  /**
   * Create vector (0.0, 0.0, 0.0)
   */
  Vector3D ()
  {
	  x=0; y=0; z=0;
  }
  /**
   * x coordinate of vector
   */
  double x;
  /**
   * y coordinate of vector
   */
  double y;
  /**
   * z coordinate of vector
   */
  double z;
  /**
   * \param _x x coordinate of vector
   * \param _y y coordinate of vector
   * \param _z z coordinate of vector
   *
   * Create vector (_x, _y, _z)
   */
  Vector3D (double _x, double _y, double _z)
  {
	  x = _x;
	  y = _y;
	  z = _z;
  }

  std::string toString()
  {
	  std::stringstream ss;
	  ss << x << " : " << y << " : " << z ;
	  return(ss.str());
  }

};

/**
 * \brief a 2d vector
 */
class Vector2D
{
public:
  /**
   * Create vector vector (0.0, 0.0)
   */
  Vector2D ()
  {
	  x=0; y=0;
  }
  /**
   * x coordinate of vector
   */
  double x;
  /**
   * y coordinate of vector
   */
  double y;
  /**
   * \param _x x coordinate of vector
   * \param _y y coordinate of vector
   *
   * Create vector (_x, _y)
   */
  Vector2D (double _x, double _y)
  {
	  x = _x;
	  y = _y;
  }

  std::string toString()
  {
	  std::stringstream ss;
	  ss << x << " : " << y;
	  return(ss.str());
  }

};


#endif /* PC3_VECTOR_H */

