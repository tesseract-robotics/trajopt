#include <trajopt_utils/macros.h>
#include <gtest/gtest.h>
#include <iostream>

int main(int argc, char** argv)
{
  std::cout << "testing" << std::endl;
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
