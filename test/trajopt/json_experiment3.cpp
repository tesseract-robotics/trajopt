#include <json/json.h>
#include <iostream>
#include <boost/assign.hpp>
using namespace std;
using namespace boost::assign;
using namespace Json;
#include <boost/shared_ptr.hpp>
#include "trajopt/problem_description.hpp"
#include <gtest/gtest.h>
using namespace trajopt;
#ifdef __CDT_PARSER__
#define NULL 0
#endif


#ifdef __CDT_PARSER__
#define FAIL_IF_FALSE(expr)
#else
#define FAIL_IF_FALSE(expr) if (!expr) {\
    printf("Failure: %s\n", #expr);\
    return false;\
  }
#endif

class CostInfoA : public CostInfo {
  int a;
  double b;
  string c;
  CostPtr hatch() {return CostPtr();}
  virtual bool fromJson(const Value& v) {
    bool ok = true;
    ok &= CostInfo::fromJson(v);
    FAIL_IF_FALSE(v.isMember("params"));
    const Value& params = v["params"];
    ok &= childFromJson(params, a, "a");
    ok &= childFromJson(params, b, "b");
    ok &= childFromJson(params, c, "c");
    return ok;
  }
};
CostInfoPtr createCostInfoA() {
  return CostInfoPtr(new CostInfoA());
}

TEST(json,  prob) {
  ProblemConstructionInfo pci;

  {
    cout << "test1" << endl;
    Json::Value v;
    EXPECT_FALSE(pci.fromJson(v));
  }

  {
    cout << "test2" << endl;
    Json::Value v;
    v["basic_info"]["n_steps"] = 2;
    v["basic_info"]["manip"] = "leftarm";
    EXPECT_TRUE(pci.fromJson(v));

    cout << "test3" << endl;
    Json::Value vcost;
    vcost["type"] = "cost_a";
    v["costs"].append(vcost);
    CostInfo::RegisterMaker("cost_a", &createCostInfoA);
    EXPECT_FALSE(pci.fromJson(v));

    cout << "test4" << endl;
    v["costs"][0]["params"];
    EXPECT_FALSE(pci.fromJson(v));

    cout << "test5" << endl;
    v["costs"][0]["params"]["a"] = 10;
    v["costs"][0]["params"]["b"] = 10;
    v["costs"][0]["params"]["c"] = "yoyoyo";
    EXPECT_TRUE(pci.fromJson(v));
  }
}


int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
