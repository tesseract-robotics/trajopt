#pragma once
#include <trajopt_ros/trajopt/typedefs.hpp>
#include <openrave/openrave.h>

namespace trajopt {

/**
Extract trajectory array from solution vector x using indices in array vars
*/
TrajArray TRAJOPT_API getTraj(const DblVec& x, const VarArray& vars);
TrajArray TRAJOPT_API getTraj(const DblVec& x, const AffArray& arr);



inline Vector3d toVector3d(const OR::Vector& v) {
  return Vector3d(v.x, v.y, v.z);
}
inline Vector4d toVector4d(const OR::Vector& v) {
  return Vector4d(v.x, v.y, v.z, v.w);
}
Eigen::Matrix3d toRot(const OR::Vector& rq);

inline OR::Transform toRaveTransform(const Vector4d& q, const Vector3d& p) {
  return OR::Transform(OR::Vector(q[0], q[1], q[2], q[3]),
                       OR::Vector(p[0], p[1], p[2]));
}

inline DblVec trajToDblVec(const TrajArray& x) {
  return DblVec(x.data(), x.data()+x.rows()*x.cols());
}

inline VectorXd concat(const VectorXd& a, const VectorXd& b) {
  VectorXd out(a.size()+b.size());
  out.topRows(a.size()) = a;
  out.middleRows(a.size(), b.size()) = b;
  return out;
}

template <typename T>
vector<T> concat(const vector<T>& a, const vector<T>& b) {
  vector<T> out;
  vector<int> x;
  out.insert(out.end(), a.begin(), a.end());
  out.insert(out.end(), b.begin(), b.end());
  return out;
}

template <typename T> 
vector<T> singleton(const T& x) {
  return vector<T>(1,x);
} 


void TRAJOPT_API AddVarArrays(OptProb& prob, int rows, const vector<int>& cols, const vector<string>& name_prefix, const vector<VarArray*>& newvars);

void TRAJOPT_API AddVarArray(OptProb& prob, int rows, int cols, const string& name_prefix, VarArray& newvars);



}

