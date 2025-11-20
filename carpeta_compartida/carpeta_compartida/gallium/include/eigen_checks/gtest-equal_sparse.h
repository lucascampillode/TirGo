/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef EIGEN_CHECKS_INTERNAL_GTEST_SPARSE_H_
#define EIGEN_CHECKS_INTERNAL_GTEST_SPARSE_H_

#include <Eigen/Sparse>
#include <gtest/gtest.h>
#include <eigen_checks/gtest-equal.h>

namespace eigen_checks
{
namespace internal
{
template <typename LHSMatrix, typename RHSMatrix>
::testing::AssertionResult MatricesNear(const Eigen::SparseMatrix<LHSMatrix>& lhs,
                                        const std::string& name_lhs,
                                        const Eigen::SparseMatrix<RHSMatrix>& rhs,
                                        const std::string& name_rhs,
                                        typename Eigen::SparseMatrix<LHSMatrix>::Scalar tolerance,
                                        const std::string& name_tolerance)
{
  if (lhs.rows() != rhs.rows())
  {
    ::testing::AssertionResult failure_reason(false);
    failure_reason << "The matrices have a different number of rows: " << name_lhs
                   << " has " << lhs.rows() << " rows while " << name_rhs << " has "
                   << rhs.rows() << " rows." << std::endl;
    return failure_reason;
  }
  if (lhs.cols() != rhs.cols())
  {
    ::testing::AssertionResult failure_reason(false);
    failure_reason << "The matrices have a different number of cols: " << name_lhs
                   << " has " << lhs.cols() << " cols while " << name_rhs << " cols "
                   << rhs.cols() << " cols." << std::endl;
    return failure_reason;
  }

  // Early exit for dynamic-sized matrices where one dimension is zero. No need to check
  // values...
  if (rhs.rows() == 0 || rhs.cols() == 0 || lhs.rows() == 0 || lhs.cols() == 0)
  {
    return ::testing::AssertionSuccess();
  }

  typedef typename Eigen::SparseMatrix<LHSMatrix>::Scalar Scalar;
  typedef typename Eigen::SparseMatrix<LHSMatrix>::StorageIndex Index;


  if (!(lhs.nonZeros() == rhs.nonZeros()))
  {
    ::testing::AssertionResult failure_reason(false);
    failure_reason << "The matrices have a different number of zeros: " << name_lhs
                   << " has " << lhs.nonZeros() << " zeros while " << name_rhs
                   << " zeros " << rhs.nonZeros() << " zeros." << std::endl;
    return failure_reason;
  }

  // Early exit if sparse matrix is empty
  if (rhs.nonZeros() == 0)
  {
    return ::testing::AssertionSuccess();
  }

  // If we want't to ignore the values and only check the structure of the matrix we can
  // pass a negative tolerance
  if (tolerance > 0)
  {
    {
      const Eigen::Map<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>> lhs_values(
          lhs.valuePtr(), lhs.nonZeros());
      const Eigen::Map<const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>> rhs_values(
          rhs.valuePtr(), rhs.nonZeros());

      ::testing::AssertionResult failure_reason_values =
          MatricesNear(lhs_values, name_lhs + "_values", rhs_values, name_rhs + "_values",
                       tolerance, name_tolerance);
      if (false == static_cast<bool>(failure_reason_values))
      {
        return failure_reason_values;
      }
    }
  }

  // Inner index won't be the size of the cols, it depends if the sparse matrix
  // is row or col major
  /*
  {
    const Eigen::Map<const Eigen::Matrix<Index, Eigen::Dynamic, 1>> lhs_inner_index(
        lhs.innerIndexPtr(), lhs.innerSize());

    const Eigen::Map<const Eigen::Matrix<Index, Eigen::Dynamic, 1>> rhs_inner_index(
        rhs.innerIndexPtr(), rhs.innerSize());

    ::testing::AssertionResult failure_reason_inner_index =
        MatricesNear(lhs_inner_index, name_lhs + "_inner_index", rhs_inner_index,
                     name_rhs + "_inner_index", 0, name_tolerance);
    if (false == static_cast<bool>(failure_reason_inner_index))
    {
      return failure_reason_inner_index;
    }
  }
  */

  {
    const Eigen::Map<const Eigen::Matrix<Index, Eigen::Dynamic, 1>> lhs_outer_index(
        lhs.outerIndexPtr(), lhs.outerSize());

    const Eigen::Map<const Eigen::Matrix<Index, Eigen::Dynamic, 1>> rhs_outer_index(
        rhs.outerIndexPtr(), rhs.outerSize());

    ::testing::AssertionResult failure_reason_inner_index =
        MatricesNear(lhs_outer_index, name_lhs + "_outer_index", rhs_outer_index,
                     name_rhs + "_outer_index", 0, name_tolerance);
    if (false == static_cast<bool>(failure_reason_inner_index))
    {
      return failure_reason_inner_index;
    }
  }

  return ::testing::AssertionSuccess();
}
}
}

#endif
