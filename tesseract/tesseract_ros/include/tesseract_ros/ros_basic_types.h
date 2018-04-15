#ifndef ROS_BASIC_TYPES_H
#define ROS_BASIC_TYPES_H

#include <unordered_map>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>
#include <geometric_shapes/shape_operations.h>

namespace tesseract
{

namespace tesseract_ros
{

/** @brief This holds a state of the environment */
struct EnvState
{
  std::unordered_map<std::string, double> joints;
  std::unordered_map<std::string, Eigen::Affine3d> transforms;
};
typedef boost::shared_ptr<EnvState> EnvStatePtr;
typedef boost::shared_ptr<const EnvState> EnvStateConstPtr;

/**< @brief Information on how the object is attached to the environment */
struct AttachedBodyInfo
{
  std::string name;                     /**< @brief The name of the attached body (must be unique) */
  std::string parent_link_name;         /**< @brief The name of the link to attach the body */
  std::string object_name;              /**< @brief The name of the AttachableObject being used */
  std::vector<std::string> touch_links; /**< @brief The names of links which the attached body is allowed to be in contact with */
};

/** @brief Contains geometry data for an attachable object */
struct AttachableObjectGeometry
{
  std::vector<shapes::ShapeConstPtr> shapes;  /**< @brief The shape */
  EigenSTL::vector_Affine3d shape_poses;      /**< @brief The pose of the shape */
  EigenSTL::vector_Vector4d shape_colors;     /**< @brief (Optional) The shape color (R, G, B, A) */
};

/** @brief Contains data about an attachable object */
struct AttachableObject
{
  std::string name;                   /**< @brief The name of the attachable object */
  AttachableObjectGeometry visual;    /**< @brief The objects visual geometry */
  AttachableObjectGeometry collision; /**< @brief The objects collision geometry */
};
typedef boost::shared_ptr<AttachableObject> AttachableObjectPtr;
typedef boost::shared_ptr<const AttachableObject> AttachableObjectConstPtr;

/** @brief Contains data representing an attached body */
struct AttachedBody
{
   AttachedBodyInfo info;        /**< @brief Information on how the object is attached to the environment */
   AttachableObjectConstPtr obj; /**< @brief The attached bodies object data */
};
typedef boost::shared_ptr<AttachedBody> AttachedBodyPtr;
typedef boost::shared_ptr<const AttachedBody> AttachedBodyConstPtr;

/** @brief ObjectColorMap Stores Object color in a 4d vector as RGBA*/
struct ObjectColor
{
  EigenSTL::vector_Vector4d visual;
  EigenSTL::vector_Vector4d collision;
};
typedef std::unordered_map<std::string, ObjectColor> ObjectColorMap;
typedef boost::shared_ptr<ObjectColorMap> ObjectColorMapPtr;
typedef boost::shared_ptr<const ObjectColorMap> ObjectColorMapConstPtr;
typedef std::unordered_map<std::string, AttachedBodyConstPtr> AttachedBodyConstPtrMap;
typedef std::unordered_map<std::string, AttachableObjectConstPtr> AttachableObjectConstPtrMap;
}
}
#endif // ROS_BASIC_TYPES_H
