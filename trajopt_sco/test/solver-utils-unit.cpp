#include <cstdio>
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <iostream>
#include <sstream>
#include <vector>
#include <trajopt_sco/expr_ops.hpp>
#include <trajopt_sco/solver_interface.hpp>
#include <trajopt_sco/solver_utils.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>

using namespace sco;

TEST(solver_utils, var_to_triplets)
{
  std::vector<VarRepPtr> x_info;
  VarVector x;
  for (unsigned int i = 0; i < 2; ++i)
  {
    std::stringstream var_name;
    var_name << "x_" << i;
    VarRepPtr x_el(new VarRep(i, var_name.str(), nullptr));
    x_info.push_back(x_el);
    x.push_back(Var(x_el.get()));
  }

  // x_affine = [3, 2]*x + 1
  AffExpr x_affine;
  x_affine.vars = x;
  x_affine.coeffs = DblVec{3, 2};
  x_affine.constant = 1;

  std::cout << "x_affine=  " << x_affine << std::endl;
  std::cout << "expecting A = [3, 2];" << std::endl
            << "          u = [1]" << std::endl;
  Eigen::MatrixXd m_A_expected(1,2);
  m_A_expected << 3, 2;
  DblVec v_u_expected{-1};

  IntVec m_A_i, m_A_j;
  DblVec m_A_ij;
  DblVec v_u;
  AffExprVector x_affine_vector(1, x_affine);
  exprToTriplets(x_affine_vector, m_A_i, m_A_j, m_A_ij, v_u);
  
  EXPECT_TRUE(v_u_expected == v_u) << "v_u_expected != v_u" << std::endl
                                   << "v_u:" << std::endl
                                   << CSTR(v_u) << std::endl;
  EXPECT_EQ(m_A_ij.size(), 2) << "m_A_ij should be of size 2" << std::endl;
  Eigen::MatrixXd m_A(1, 2);
  triplets_to_full(m_A_i, m_A_j, m_A_ij, m_A);
  EXPECT_TRUE(m_A_expected.isApprox(m_A)) << "m_A_expected != m_A" << std::endl
                                          << "m_A:" << std::endl
                                          << m_A << std::endl;

  QuadExpr x_squared = exprSquare(x_affine);
  std::cout << "x_squared= " << x_squared << std::endl;
  std::cout << "expecting Q = [9, 6;" << std::endl
            << "               6, 4]" << std::endl
            << "          q = [6, 4]" << std::endl;
  Eigen::MatrixXd m_Q_expected(2, 2);
  m_Q_expected << 9, 6,
                  6, 4;
  DblVec v_q_expected{6, 4};

  IntVec m_Q_i, m_Q_j;
  DblVec m_Q_ij;
  DblVec v_q;
  exprToTriplets(x_squared, m_Q_i, m_Q_j, m_Q_ij, v_q);

  EXPECT_TRUE(v_q_expected == v_q) << "v_q_expected != v_q" << std::endl
                                  << "v_q:" << std::endl
                                  << CSTR(v_q) << std::endl;
  EXPECT_EQ(m_Q_ij.size(), 4) << "m_Q_ij should be of size 4" << std::endl;
  Eigen::MatrixXd m_Q(2, 2);
  triplets_to_full(m_Q_i, m_Q_j, m_Q_ij, m_Q);
  EXPECT_TRUE(m_Q_expected.isApprox(m_Q)) << "m_Q_expected != m_Q" << std::endl
                                          << "m_Q:" << std::endl
                                          << m_Q << std::endl;

}

TEST(solver_utils, triplet_to_CSC_sparse)
{

    /*
     * M = [ 1, 2, 3,
     *       1, 0, 9,
     *       1, 8, 0]
     */
    // triplet representation of M
    DblVec data_vij = {1, 2, 3, 1, 9, 1, 8};
    IntVec data_i = {0, 0, 0, 1, 1, 2, 2};
    IntVec data_j = {0, 1, 2, 0, 2, 0, 1};

    DblVec P;
    IntVec rows_i;
    IntVec cols_p;

    triplets_to_CSC(rows_i, cols_p, P, 3, 3, 7,
                    data_i, data_j, data_vij);

    EXPECT_TRUE(rows_i.size() == P.size()) << "rows_i.size() != P.size()";
    EXPECT_TRUE((P == DblVec{1, 1, 1, 2, 8, 3, 9})) << "bad P:\n"
                                                            << CSTR(P);
    EXPECT_TRUE(rows_i == data_j) << "rows_i != data_j:\n" << CSTR(rows_i) 
                                                           << " vs\n"
                                                           << CSTR(data_j);
    EXPECT_TRUE((cols_p == IntVec{0, 3, 5, 7})) << "cols_p not in "
                                                << "CRC form:\n"
                                                << CSTR(cols_p);

    /*
     * M = [ 0, 2, 0,
     *       7, 0, 0,
     *       0, 0, 0]
     */
    // triplet representation of M
    data_vij = {2, 7, 0};
    data_i = {0, 1, 1};
    data_j = {1, 0, 1};

    triplets_to_CSC(rows_i, cols_p, P, 3, 3, 3,
                    data_i, data_j, data_vij);

    EXPECT_TRUE(rows_i.size() == P.size()) << "rows_i.size() != P.size()";
    EXPECT_TRUE((P == DblVec{7, 2})) << "bad P:\n"
                                     << CSTR(P);
    EXPECT_TRUE((rows_i == IntVec{1, 0})) << "rows_i != data_j:\n"
                                          << CSTR(rows_i)
                                          << " vs\n"
                                          << CSTR((IntVec{1, 0}));
    EXPECT_TRUE((cols_p == IntVec{0, 1, 2, 2})) << "cols_p not in "
                                                << "CRC form:\n"
                                                << CSTR(cols_p);

    /*
     * M = [ 0, 0, 0,
     *       0, 0, 0,
     *       0, 6, 0]
     */
    // triplet representation of M
    data_vij = {6};
    data_i = {2};
    data_j = {1};

    triplets_to_CSC(rows_i, cols_p, P, 3, 3, 3,
                    data_i, data_j, data_vij);

    EXPECT_TRUE(rows_i.size() == P.size()) << "rows_i.size() != P.size()";
    EXPECT_TRUE((P == DblVec{6})) << "bad P:\n"
                                  << CSTR(P);
    EXPECT_TRUE((rows_i == IntVec{2})) << "rows_i != data_j:\n"
                                       << CSTR(rows_i)
                                       << " vs\n"
                                       << CSTR((IntVec{1, 0}));
    EXPECT_TRUE((cols_p == IntVec{0, 0, 1, 1})) << "cols_p not in "
                                                << "CRC form:\n"
                                                << CSTR(cols_p);
}

TEST(solver_utils, triplet_to_CSC_sparse_upper_triangular)
{

    /*
     * M = [ 1, 2, 0,
     *       2, 4, 0,
     *       0, 0, 9]
     */
    // triplet representation of M
    DblVec data_vij = {1, 2, 2, 4, 9};
    IntVec data_i = {0, 0, 1, 1, 2};
    IntVec data_j = {0, 1, 0, 1, 2};

    DblVec P;
    IntVec rows_i;
    IntVec cols_p;

    triplets_to_CSC(rows_i, cols_p, P, 3, 3, 5,
                    data_i, data_j, data_vij, true);

    EXPECT_TRUE(rows_i.size() == P.size()) << "rows_i.size() != P.size()";
    EXPECT_TRUE((P == DblVec{1, 2, 4, 9})) << "bad P:\n"
                                           << CSTR(P);
    EXPECT_TRUE((rows_i == IntVec{0, 0, 1, 2})) << "rows_i != expected"
        << ":\n" << CSTR(rows_i) << " vs\n" << CSTR((IntVec{0, 0, 1, 2}));
    EXPECT_TRUE((cols_p == IntVec{0, 1, 3, 4})) << "cols_p not in "
                                                << "CRC form:\n"
                                                << CSTR(cols_p);
}