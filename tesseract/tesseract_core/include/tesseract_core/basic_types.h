/**
 * @file basic_types.h
 * @brief The tesseract_core package types.
 *
 * @author Levi Armstrong
 * @date April 15, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_CORE_BASIC_TYPES_H
#define TESSERACT_CORE_BASIC_TYPES_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <boost/shared_ptr.hpp>
#include <unordered_map>
#include <vector>

namespace tesseract
{

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> TrajArray;
typedef std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > vector_Affine3d;

struct AllowedCollisionMatrix
{
  /**
   * @brief This checks if two links are allowed to be in collision
   * @param link_name1 First link name
   * @param link_name2 Second link anme
   * @return True if allowed to be in collision, otherwise false
   */
  virtual bool isCollisionAllowed(const std::string& link_name1, const std::string& link_name2) const = 0;
};
typedef boost::shared_ptr<AllowedCollisionMatrix> AllowedCollisionMatrixPtr;
typedef boost::shared_ptr<const AllowedCollisionMatrix> AllowedCollisionMatrixConstPtr;


namespace BodyTypes
{
enum BodyType
{
  ROBOT_LINK,     /**< @brief These are links at the creation of the environment */
  ROBOT_ATTACHED  /**< @brief These are links that are added after initial creation */
};
}
typedef BodyTypes::BodyType BodyType;

namespace ContinouseCollisionTypes
{
enum ContinouseCollisionType
{
  CCType_None,
  CCType_Time0,
  CCType_Time1,
  CCType_Between
};
}
typedef ContinouseCollisionTypes::ContinouseCollisionType ContinouseCollisionType;

namespace ContactRequestTypes
{
enum DistanceRequestType
{
  SINGLE, /**< Return the global minimum for a pair of objects */
  ALL  ,  /**< Return all contacts for a pair of objects */
  LIMITED /**< Return limited set of contacts for a pair of objects */
};
}
typedef ContactRequestTypes::DistanceRequestType ContactRequestType;

struct ContactRequestBase
{
  ContactRequestType type;              /**< The type of request */
  double contact_distance;              /**< The maximum distance between two objects for which distance data should be calculated */
  std::vector<std::string> link_names;  /**< Name of the links to calculate distance data for. */
  AllowedCollisionMatrixConstPtr acm;   /**< The allowed collision matrix */

  ContactRequestBase() : type(ContactRequestType::SINGLE), contact_distance(0.0) {}
};

struct ContactRequest : public ContactRequestBase
{
  std::vector<std::string> joint_names; /**< Vector of joint names (size must match number of joints in robot chain) */
  Eigen::VectorXd joint_angles1;        /**< Vector of joint angles (size must match number of joints in robot chain/tree) */
  Eigen::VectorXd joint_angles2;        /**< Vector of joint angles used for continuous checking (size must match number of joints in robot chain/tree) */

  ContactRequest() : ContactRequestBase() {}
};

struct ContactResult
{
  double distance;
  BodyType body_types[2];
  std::string link_names[2];
  std::string attached_link_names[2];
  Eigen::Vector3d nearest_points[2];
  Eigen::Vector3d normal;
  Eigen::Vector3d cc_nearest_points[2];
  double cc_time;
  ContinouseCollisionType cc_type;
  bool valid;

  ContactResult() { clear(); }

  /// Clear structure data
  void clear()
  {
    distance = std::numeric_limits<double>::max();
    nearest_points[0].setZero();
    nearest_points[1].setZero();
    link_names[0] = "";
    link_names[1] = "";
    attached_link_names[0] = "";
    attached_link_names[1] = "";
    body_types[0] = BodyType::ROBOT_LINK;
    body_types[1] = BodyType::ROBOT_LINK;
    normal.setZero();
    cc_nearest_points[0].setZero();
    cc_nearest_points[1].setZero();
    cc_time = -1;
    cc_type = ContinouseCollisionType::CCType_None;
  }
};
typedef std::vector<ContactResult> ContactResultVector;

}

#endif // TESSERACT_CORE_BASIC_TYPES_H
