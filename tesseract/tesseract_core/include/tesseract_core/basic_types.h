#ifndef BASIC_TYPES_H
#define BASIC_TYPES_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>
#include <unordered_map>
#include <vector>

namespace tesseract
{

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> TrajArray;

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

struct ContactRequest
{
  ContactRequestType type;             /**< The type of distance request */
  double contact_distance;              /**< The maximum distance between two objects for which distance data should be calculated */
  std::vector<std::string> joint_names; /**< Vector of joint names (size must match number of joints in robot chain) */
  std::vector<std::string> link_names;  /**< Name of the links to calculate distance data for. */
  Eigen::VectorXd joint_angles1;        /**< Vector of joint angles (size must match number of joints in robot chain/tree) */
  Eigen::VectorXd joint_angles2;        /**< Vector of joint angles (size must match number of joints in robot chain/tree) */
  AllowedCollisionMatrixConstPtr acm;   /**< The allowed collision matrix */

  ContactRequest() : type(ContactRequestType::SINGLE), contact_distance(0.0) {}
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

#endif // BASIC_TYPES_H
