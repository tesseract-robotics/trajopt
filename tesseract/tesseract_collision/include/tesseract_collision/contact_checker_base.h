#ifndef TESSERACT_COLLISION_CONTACT_CHECKER_BASE_H
#define TESSERACT_COLLISION_CONTACT_CHECKER_BASE_H

#include <tesseract_core/basic_types.h>
#include <geometric_shapes/shapes.h>
#include <eigen_stl_containers/eigen_stl_containers.h>

namespace tesseract
{

class ContactCheckerBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW


  /**
   * @brief Perform a discrete check using the current state and request
   * @param contacts A list of contact results.
   */
  virtual void calcDistancesDiscrete(ContactResultMap &contacts) = 0;

  /**
   * @brief Calculate distance information for all links in transforms (Discrete Check)
   *
   * If a links transform is provided but the object is disabled it will be ignored.
   *
   * @param req        The distance request information.
   * @param transforms The transforms for objects to check collision.
   * @param contacts   A list of contact results.
   */
  virtual void calcDistancesDiscrete(const ContactRequest &req, const TransformMap& transforms, ContactResultMap &contacts) const = 0;

  /**
   * @brief Calculate distance information for all links in transforms1/transforms2 (Continuous Check)
   *
   * If a links transform is provided but the object is disabled it will be ignored.
   *
   * @param req         The distance request information.
   * @param transforms1 The transforms for every object at start
   * @param transforms2 The transforms for every object at end
   * @param contacts    A list of contact results.
   */
  virtual void calcDistancesContinuous(const ContactRequest &req, const TransformMap& transforms1, const TransformMap& transforms2, ContactResultMap &contacts) const = 0;

  /**
   * @brief Perform a discrete collision check using the current state and request
   * @param contacts A list of contact results.
   */
  virtual void calcCollisionsDiscrete(ContactResultMap &contacts) = 0;

  /**
   * @brief Calculate collision information for all links in transforms (Discrete Check)
   *
   * If a links transform is provided but the object is disabled it will be ignored.
   *
   * @param req        The distance request information.
   * @param transforms The transforms for every object
   * @param contacts   A list of contact results.
   */
  virtual void calcCollisionsDiscrete(const ContactRequest &req, const TransformMap& transforms, ContactResultMap &contacts) const = 0;

  /**
   * @brief Calculate collision information for all links in transforms1/transfroms2 (Continuous Check)
   *
   * If a links transform is provided but the object is disabled it will be ignored.
   *
   * @param req         The distance request information.
   * @param transforms1 The transforms for every object at start
   * @param transforms2 The transforms for every object at end
   * @param contacts    A list of contact results.
   */
  virtual void calcCollisionsContinuous(const ContactRequest &req, const TransformMap& transforms1, const TransformMap& transforms2, ContactResultMap &contacts) const = 0;

  /**
   * @brief Add a object to the checker
   * @param name        The name of the object, must be unique.
   * @param type_id     User defined type id which gets stored in the results structure.
   * @param shapes      A vector of shapes that make up the collision object.
   * @param shape_poses A vector of poses for each shape, must be same length as shapes
   * @return true if successfully added, otherwise false.
   */
  virtual bool addObject(const std::string& name, const int& type_id, const std::vector<shapes::ShapeConstPtr>& shapes, const EigenSTL::vector_Affine3d& shape_poses, bool enabled = true) = 0;

  /**
   * @brief Remove an object from the checker
   * @param name The name of the object
   * @return true if successfully removed, otherwise false.
   */
  virtual bool removeObject(const std::string& name) = 0;

  /**
   * @brief Enable an object
   * @param name The name of the object
   */
  virtual void enableObject(const std::string& name) = 0;

  /**
   * @brief Disable an object
   * @param name The name of the object
   */
  virtual void disableObject(const std::string& name) = 0;

  /**
   * @brief Set a collision objects tansforms
   *
   * This is to be used by a collision monitor.
   *
   */
  virtual void setObjectsTransform(const std::string& name, const Eigen::Affine3d& pose) = 0;
  virtual void setObjectsTransform(const std::vector<std::string>& names, const EigenSTL::vector_Affine3d& poses) = 0;
  virtual void setObjectsTransform(const TransformMap& transforms) = 0;

  /**
   * @brief Set the active contact request information
   * @param req ContactRequest information
   */
  virtual void setContactRequest(const ContactRequest &req) = 0;

  /**
   * @brief Get the active contact request information
   * @return Active contact request information
   */
  virtual const ContactRequest& getContactRequest() const = 0;

};
typedef std::shared_ptr<ContactCheckerBase> ContactCheckerBasePtr;
typedef std::shared_ptr<const ContactCheckerBase> ContactCheckerBaseConstPtr;

}
#endif // TESSERACT_COLLISION_CONTACT_CHECKER_BASE_H
