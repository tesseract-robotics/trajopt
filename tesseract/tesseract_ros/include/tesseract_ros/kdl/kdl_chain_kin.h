#ifndef TESSERACT_ROS_KDL_CHAIN_KIN_H
#define TESSERACT_ROS_KDL_CHAIN_KIN_H

#include "tesseract_ros/ros_basic_kin.h"
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <boost/scoped_ptr.hpp>
#include <urdf/model.h>

namespace tesseract
{

/**
 * @brief ROS kinematics functions.
 *
 * Typically, just wrappers around the equivalent KDL calls.
 *
 */
class KDLChainKin : public ROSBasicKin
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  KDLChainKin() : ROSBasicKin(), initialized_(false) {}

  bool calcFwdKin(Eigen::Affine3d &pose, const Eigen::Affine3d change_base, const Eigen::VectorXd &joint_angles) const;

  bool calcFwdKin(Eigen::Affine3d &pose, const Eigen::Affine3d change_base, const Eigen::VectorXd &joint_angles, const std::string &link_name) const;

  bool calcJacobian(Eigen::MatrixXd &jacobian, const Eigen::Affine3d change_base, const Eigen::VectorXd &joint_angles) const;

  bool calcJacobian(Eigen::MatrixXd &jacobian, const Eigen::Affine3d change_base, const Eigen::VectorXd &joint_angles, const std::string &link_name) const;

  bool calcJacobian(Eigen::MatrixXd &jacobian, const Eigen::Affine3d change_base, const Eigen::VectorXd &joint_angles, const std::string &link_name, const Eigen::Vector3d link_point) const;

  bool checkJoints(const Eigen::VectorXd &vec) const;

  const std::vector<std::string>& getJointNames() const;

  const std::vector<std::string>& getLinkNames() const;

  const Eigen::MatrixX2d& getLimits() const;

  /**
   * @brief Initializes ROSKin
   * Creates KDL::Chain from urdf::Model, populates joint_list_, joint_limits_, and link_list_
   * @param model The urdf model
   * @param base_link The name of the base link for the kinematic chain
   * @param tip_link The name of the tip link for the kinematic chain
   * @param name The name of the kinematic chain
   * @return True if init() completes successfully
   */
  bool init(const urdf::ModelInterfaceConstSharedPtr model, const std::string &base_link, const std::string &tip_link, const std::string name);


  /**
   * @brief Checks if BasicKin is initialized (init() has been run: urdf model loaded, etc.)
   * @return True if init() has completed successfully
   */
  bool checkInitialized() const
  {
    if (!initialized_)
    {
      ROS_ERROR("Kinematics has not been initialized!");
    }

    return initialized_;
  }

  /** @brief Get the URDF model */
  const urdf::ModelInterfaceConstSharedPtr getURDF() const {return model_;}

  /**
   * @brief Number of joints in robot
   * @return Number of joints in robot
   */
  unsigned int numJoints() const { return robot_chain_.getNrOfJoints(); }

  /** @brief Get the base link name */
  const std::string& getBaseLinkName() const { return base_name_; }

  /** @brief Get the tip link name */
  const std::string& getTipLinkName() const { return tip_name_; }

  /** @brief Get the name of the kinematic chain */
  const std::string& getName() const { return name_; }

  /**
   * @brief Assigns values from another ROSKin to this
   * @param rhs Input ROSKin object to copy from
   * @return reference to this ROSKin object
   */
  KDLChainKin& operator=(const KDLChainKin& rhs);

  /**
   * @brief Convert KDL::Frame to Eigen::Affine3d
   * @param frame Input KDL Frame
   * @param transform Output Eigen transform (Affine3d)
   */
  static void KDLToEigen(const KDL::Frame &frame, Eigen::Affine3d &transform);

  /**
   * @brief Convert Eigen::Affine3d to KDL::Frame
   * @param transform Input Eigen transform (Affine3d)
   * @param frame Output KDL Frame
   */
  static void EigenToKDL(const Eigen::Affine3d &transform, KDL::Frame &frame);

  /**
   * @brief Convert KDL::Jacobian to Eigen::Matrix
   * @param jacobian Input KDL Jacobian
   * @param matrix Output Eigen MatrixXd
   */
  static void KDLToEigen(const KDL::Jacobian &jacobian, Eigen::MatrixXd &matrix);

  /**
   * @brief Convert Eigen::Vector to KDL::JntArray
   * @param vec Input Eigen vector
   * @param joints Output KDL joint array
   */
  static void EigenToKDL(const Eigen::VectorXd &vec, KDL::JntArray &joints) {joints.data = vec;}

private:
  bool initialized_;                                             /**< Identifies if the object has been initialized */
  urdf::ModelInterfaceConstSharedPtr model_;                     /**< URDF MODEL */
  KDL::Chain  robot_chain_;                                      /**< KDL Chain object */
  KDL::Tree   kdl_tree_;                                         /**< KDL tree object */
  std::string base_name_;                                        /**< Link name of first link in the kinematic chain */
  std::string tip_name_;                                         /**< Link name of last kink in the kinematic chain */
  std::string name_;                                             /**< Name of the kinematic chain */
  std::vector<std::string> joint_list_;                          /**< List of joint names */
  std::vector<std::string> link_list_;                           /**< List of link names */
  Eigen::MatrixX2d joint_limits_;                                /**< Joint limits */
  boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_; /**< KDL Forward Kinematic Solver */
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jac_solver_;       /**< KDL Jacobian Solver */
  std::map<std::string, int> link_name_too_segment_index_;       /**< A map from link name to kdl chain segment numer */

  /** @brief calcFwdKin helper function */
  bool calcFwdKinHelper(Eigen::Affine3d &pose, const Eigen::Affine3d change_base, const Eigen::VectorXd &joint_angles, int segment_num=-1) const;

  /** @brief calcJacobian helper function */
  bool calcJacobianHelper(KDL::Jacobian &jacobian, const Eigen::Affine3d change_base, const Eigen::VectorXd &joint_angles, int segment_num=-1) const;

  void addChildrenRecursive(const urdf::LinkConstSharedPtr urdf_link, const std::string &next_chain_segment);

}; // class KDLChainKin

typedef boost::shared_ptr<KDLChainKin> KDLChainKinPtr;
typedef boost::shared_ptr<const KDLChainKin> KDLChainKinConstPtr;
}
#endif // TESSERACT_ROS_KDL_CHAIN_KIN_H
