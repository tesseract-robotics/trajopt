#ifndef ROS_KIN_H
#define ROS_KIN_H
#include <trajopt/basic_kin.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <boost/scoped_ptr.hpp>
#include <moveit/robot_model/joint_model_group.h>

namespace trajopt
{

/**
 * @brief ROS kinematics functions.
 *
 * Typically, just wrappers around the equivalent KDL calls.
 *
 */
class ROSKin : public BasicKin
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ROSKin() : group_(NULL) {}

  bool calcFwdKin(const Eigen::VectorXd &joint_angles, Eigen::Affine3d &pose) const;

  bool calcFwdKin(const Eigen::VectorXd &joint_angles, Eigen::Affine3d &pose, const std::string &link_name) const;

  bool calcFwdKin(const Eigen::VectorXd &joint_angles,
                  const std::string &base,
                  const std::string &tip,
                  Eigen::Affine3d &pose) const;

  bool calcJacobian(const Eigen::VectorXd &joint_angles, Eigen::MatrixXd &jacobian) const;

  bool calcJacobian(const Eigen::VectorXd &joint_angles, Eigen::MatrixXd &jacobian, const std::string &link_name) const;

  bool checkJoints(const Eigen::VectorXd &vec) const;

  bool getJointNames(std::vector<std::string> &names) const;

  Eigen::MatrixXd getLimits() const { return joint_limits_; }

  bool getLinkNames(std::vector<std::string> &names) const;

  /**
   * @brief Initializes ROSKin
   * Creates KDL::Chain from urdf::Model, populates joint_list_, joint_limits_, and link_list_
   * @param group Input kinematic joint model group
   * @return True if init() completes successfully
   */
  bool init(const moveit::core::JointModelGroup* group);

  /**
   * @brief Get the name of the kinematic group
   * @return string with the group name
   */
  const moveit::core::JointModelGroup* getJointModelGroup() const {return group_;}

  /**
   * @brief Number of joints in robot
   * @return Number of joints in robot
   */
  unsigned int numJoints() const { return robot_chain_.getNrOfJoints(); }

  /**
   * @brief Get a subchain of the kinematic group
   * @param link_name Name of final link in chain
   * @param chain Output kinematic chain
   * @return True if the subchain was successfully created
   */
  bool getSubChain(const std::string link_name, KDL::Chain &chain) const;

  std::string getRobotBaseLinkName() const { return base_name_; }

  std::string getRobotTipLinkName() const { return tip_name_; }

  /**
   * @brief Assigns values from another ROSKin to this
   * @param rhs Input ROSKin object to copy from
   * @return reference to this ROSKin object
   */
  ROSKin& operator=(const ROSKin& rhs);

  /**
   * @brief Convert KDL::Frame to Eigen::Affine3d
   * @param frame Input KDL Frame
   * @param transform Output Eigen transform (Affine3d)
   */
  static void KDLToEigen(const KDL::Frame &frame, Eigen::Affine3d &transform);

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
  const moveit::core::JointModelGroup* group_;                   /**< Move group */
  KDL::Chain  robot_chain_;                                      /**< KDL Chain object */
  KDL::Tree   kdl_tree_;                                         /**< KDL tree object */
  std::string base_name_;                                        /**< Link name of first link in the kinematic chain */
  std::string tip_name_;                                         /**< Link name of last kink in the kinematic chain */
  std::vector<std::string> joint_list_;                          /**< List of joint names */
  std::vector<std::string> link_list_;                           /**< List of link names */
  Eigen::Matrix<double, Eigen::Dynamic, 2> joint_limits_;        /**< Joint limits */
  boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_; /**< KDL Forward Kinematic Solver */
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jac_solver_;       /**< KDL Jacobian Solver */

  /** @brief calcFwdKin helper function */
  bool calcFwdKinHelper(const Eigen::VectorXd &joint_angles, Eigen::Affine3d &pose, int segment_num=-1) const;

  /** @brief calcJacobian helper function */
  bool calcJacobianHelper(const Eigen::VectorXd &joint_angles, Eigen::MatrixXd &jacobian, int segment_num=-1) const;

  /**
   * @brief Get joint number of given joint in initialized robot
   * @param joint_name Input name of joint
   * @return joint index if joint_name part of joint_list_, n+1 otherwise
   */
  int getJointNum(const std::string &joint_name) const;

  /**
   * @brief Get link number of given joint in initialized robot
   * @param link_name Input name of link
   * @return link index if link_name part of link_list_, l+1 otherwise
   */
  int getLinkNum(const std::string &link_name) const;

}; // class BasicKin

typedef boost::shared_ptr<ROSKin> ROSKinPtr;
}
#endif // ROS_KIN_H
