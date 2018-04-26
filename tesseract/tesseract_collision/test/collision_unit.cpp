
#include <gtest/gtest.h>
#include <ros/ros.h>
#include "tesseract_collision/bullet/bullet_contact_checker.h"

TEST(TesseractCollisionUnit, CollisionUnit)
{
  tesseract::BulletContactChecker checker;

  // Add box to checker
  shapes::ShapePtr box(new shapes::Box(1, 1, 1));
  Eigen::Affine3d box_pose;
  box_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj1_shapes;
  EigenSTL::vector_Affine3d obj1_poses;
  obj1_shapes.push_back(box);
  obj1_poses.push_back(box_pose);

  checker.addObject("box_link", 0, obj1_shapes, obj1_poses);

  // Add box to checker
  shapes::ShapePtr thin_box(new shapes::Box(0.1, 1, 1));
  Eigen::Affine3d thin_box_pose;
  thin_box_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj2_shapes;
  EigenSTL::vector_Affine3d obj2_poses;
  obj2_shapes.push_back(thin_box);
  obj2_poses.push_back(thin_box_pose);

  checker.addObject("thin_box_link", 0, obj2_shapes, obj2_poses);

  // Add sphere to checker
  shapes::ShapePtr sphere(new shapes::Sphere(0.25));
  Eigen::Affine3d sphere_pose;
  sphere_pose.setIdentity();

  std::vector<shapes::ShapeConstPtr> obj3_shapes;
  EigenSTL::vector_Affine3d obj3_poses;
  obj3_shapes.push_back(sphere);
  obj3_poses.push_back(sphere_pose);

  checker.addObject("sphere_link", 0, obj3_shapes, obj3_poses);

  // Check if they are in collision
  tesseract::ContactRequest req;
  req.link_names.push_back("box_link");
  req.link_names.push_back("sphere_link");
  req.contact_distance = 0.1;
  req.type = tesseract::ContactRequestType::SINGLE;

  // Test when object is inside another
  tesseract::TransformMap location;
  location["box_link"] = Eigen::Affine3d::Identity();
  location["sphere_link"] = Eigen::Affine3d::Identity();

  tesseract::ContactResultMap result;
  checker.calcCollisionsDiscrete(req, location, result);

  tesseract::ContactResultVector result_vector;
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

  EXPECT_LT(std::abs(result_vector[0].distance + 0.75), 0.0001);
  EXPECT_TRUE(!result_vector.empty());

  // Test object is out side the contact distance
  location["sphere_link"].translation() = Eigen::Vector3d(1, 0, 0);
  result.clear();
  result_vector.clear();

  checker.calcCollisionsDiscrete(req, location, result);
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

  EXPECT_TRUE(result_vector.empty());

  // Test object right at the contact distance
  result.clear();
  result_vector.clear();
  req.contact_distance = 0.25;

  checker.calcCollisionsDiscrete(req, location, result);
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);

  EXPECT_LT(std::abs(0.25 - result_vector[0].distance), 0.0001);
  EXPECT_TRUE(!result_vector.empty());

  // Test Cast object
  result.clear();
  result_vector.clear();
  tesseract::TransformMap location2;
  location.clear();
  location["thin_box_link"] = Eigen::Affine3d::Identity();
  location["sphere_link"] = Eigen::Affine3d::Identity();
  location2["thin_box_link"] = Eigen::Affine3d::Identity();
  location2["sphere_link"] = Eigen::Affine3d::Identity();

  location["sphere_link"].translation() = Eigen::Vector3d(1, 0, 0);
  location2["sphere_link"].translation() = Eigen::Vector3d(-1, 0, 0);

  req.link_names.clear();
  req.link_names.push_back("sphere_link");
  req.contact_distance = 0.1;
  req.type = tesseract::ContactRequestType::SINGLE;

  checker.calcCollisionsContinuous(req, location, location2, result);
  tesseract::moveContactResultsMapToContactResultsVector(result, result_vector);
  EXPECT_TRUE(!result_vector.empty());

}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
