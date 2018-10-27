#include <gtest/gtest.h>
#include <trajopt_sco/qpoases_interface.hpp>
#include <trajopt_utils/stl_to_string.hpp>

using namespace std;
using namespace sco;

TEST(qpoases_interface, triplet_to_CSC_sparse)
{

    /*
     * M = [ 1, 2, 3,
     *       1, 0, 9,
     *       1, 8, 0]
     */
    // triplet representation of M
    vector<double> data_vij = {1, 2, 3, 1, 9, 1, 8};
    vector<int> data_i = {0, 0, 0, 1, 1, 2, 2};
    vector<int> data_j = {0, 1, 2, 0, 2, 0, 1};

    vector<double> P;
    vector<int> rows_i;
    vector<int> cols_p;

    triplets_to_CSC(rows_i, cols_p, P, 3, 3, 7,
                    data_i, data_j, data_vij);

    EXPECT_TRUE(rows_i.size() == P.size()) << "rows_i.size() != P.size()";
    EXPECT_TRUE((P == vector<double>{1, 1, 1, 2, 0, 8, 3, 9, 0})) << "bad P:\n"
                                                                  << CSTR(P);
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
    EXPECT_TRUE((P == vector<double>{0, 7, 2, 0, 0})) << "bad P:\n"
                                                      << CSTR(P);

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
    EXPECT_TRUE((P == vector<double>{0, 0, 6, 0})) << "bad P:\n"
                                                   << CSTR(P);
}

TEST(qpoases_interface, triplet_to_CSC_sparse_upper_triangular)
{

    /*
     * M = [ 1, 2, 0,
     *       2, 4, 0,
     *       0, 0, 9]
     */
    // triplet representation of M
    vector<double> data_vij = {1, 2, 2, 4, 9};
    vector<int> data_i = {0, 0, 1, 1, 2};
    vector<int> data_j = {0, 1, 0, 1, 2};

    vector<double> P;
    vector<int> rows_i;
    vector<int> cols_p;

    triplets_to_CSC(rows_i, cols_p, P, 3, 3, 5,
                    data_i, data_j, data_vij, true);

    EXPECT_TRUE(rows_i.size() == P.size()) << "rows_i.size() != P.size()";
    EXPECT_TRUE((P == vector<double>{1, 2, 4, 9})) << "bad P:\n"
                                                   << CSTR(P);
    EXPECT_TRUE((rows_i == vector<int>{0, 0, 1, 2})) << "rows_i != expected"
        << ":\n" << CSTR(rows_i) << " vs\n" << CSTR((vector<int>{0, 0, 1, 2}));
    EXPECT_TRUE((cols_p == vector<int>{0, 1, 3, 4})) << "cols_p not in "
                                                     << "CRC form:\n"
                                                     << CSTR(cols_p);
}