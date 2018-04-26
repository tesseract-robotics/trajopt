/**
 * @file bullet_contact_checker.h
 * @brief Tesseract ROS Bullet Contact Checker implementation.
 *
 * @author John Schulman
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 * @copyright Copyright (c) 2013, John Schulman
 *
 * @par License
 * Software License Agreement (BSD-2-Clause)
 * @par
 * All rights reserved.
 * @par
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * @par
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * @par
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef TESSERACT_COLLISION_BULLET_CONTACT_CHECKER_H
#define TESSERACT_COLLISION_BULLET_CONTACT_CHECKER_H

#include <tesseract_collision/contact_checker_base.h>
#include <tesseract_collision/bullet/bullet_utils.h>
#include <memory>
#include <Eigen/Core>

namespace tesseract
{

class BulletContactChecker : public ContactCheckerBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BulletContactChecker() { name_ = "BULLET"; }

  void calcDistancesDiscrete(ContactResultMap &contacts) override;

  void calcDistancesDiscrete(const ContactRequest &req, const TransformMap& transforms, ContactResultMap &contacts) const override;

  void calcDistancesContinuous(const ContactRequest &req, const TransformMap& transforms1, const TransformMap& transforms2, ContactResultMap &contacts) const override;

  void calcCollisionsDiscrete(ContactResultMap &contacts) override;

  void calcCollisionsDiscrete(const ContactRequest &req, const TransformMap& transforms, ContactResultMap &contacts) const override;

  void calcCollisionsContinuous(const ContactRequest &req, const TransformMap& transforms1, const TransformMap& transforms2, ContactResultMap &contacts) const override;

  bool addObject(const std::string& name, const int& type_id, const std::vector<shapes::ShapeConstPtr>& shapes, const EigenSTL::vector_Affine3d& shape_poses, bool enabled = true) override;

  bool removeObject(const std::string& name) override;

  void enableObject(const std::string& name) override;

  void disableObject(const std::string& name) override;

  void setObjectsTransform(const std::string& name, const Eigen::Affine3d& pose) override;

  void setObjectsTransform(const std::vector<std::string>& names, const EigenSTL::vector_Affine3d& poses) override;

  void setObjectsTransform(const TransformMap& transforms) override;

  void setContactRequest(const ContactRequest &req) override;

  const ContactRequest& getContactRequest() const override;

private:
  std::string name_;                        /**< Name of the environment (may be empty) */
  BulletManager manager_;                   /**< Contains the collision objects */
  ContactRequest request_;                  /**< Active request to be used for methods that don't require a request */
  std::vector<std::string> active_objects_; /**< A list of active objects ot check for contact */

  void constructBulletObject(Link2Cow &collision_objects, std::vector<std::string> &active_objects, double contact_distance, const TransformMap& transforms, const std::vector<std::string> &active_links, bool continuous = false) const;

  void constructBulletObject(Link2Cow &collision_objects, std::vector<std::string> &active_objects, double contact_distance, const TransformMap& transforms1, const TransformMap& transforms2, const std::vector<std::string> &active_links) const;

};
typedef std::shared_ptr<BulletContactChecker> BulletContactCheckerPtr;
typedef std::shared_ptr<const BulletContactChecker> BulletContactCheckerConstPtr;
}

#endif // TESSERACT_COLLISION_BULLET_CONTACT_CHECKER_H
