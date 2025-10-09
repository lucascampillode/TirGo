/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */
#ifndef _EIGEN_CHECKS_SIZE_CHECK_H_
#define _EIGEN_CHECKS_SIZE_CHECK_H_

#include <Eigen/Dense>

namespace Eigen {

//////////////////////////////////////////////////////////
/** Check Eigen Matrix sizes, both statically and dynamically
 *
 * Help:
 *
 * Eigen provides some macros for STATIC assert of matrix sizes, the most common of them are (see Eigen's StaticAssert.h):
 *
 *      EIGEN_STATIC_ASSERT_VECTOR_SPECIFIC_SIZE
 *      EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE
 *      EIGEN_STATIC_ASSERT_VECTOR_ONLY
 *
 * but they do not work if the evaluated types are of dynamic size.
 *
 * The function :  MatrixSizeCheck <Rows, Cols>::check(M)   checks that the Matrix M is of size ( Rows x Cols ).
 * This check is performed statically or dynamically, depending on the type of argument provided.
 */
 
template<int Size, int DesiredSize>
struct StaticDimCheck
{
  template<typename T>
  StaticDimCheck(const T& t)
  {
    static_assert(Size == DesiredSize, "Size of static Vector or Matrix does not match");
  }
};

template<int DesiredSize>
struct StaticDimCheck<Eigen::Dynamic, DesiredSize>
{
  template<typename T>
  StaticDimCheck(const T& t)
  {
    assert(t == DesiredSize && "Size of dynamic Vector or Matrix does not match");
  }
};

template<int DesiredR, int DesiredC>
struct MatrixSizeCheck
{
  /** \brief Assert matrix size dynamically or statically
   *
   * @param t the variable for which we wish to assert the size.
   *
   * Usage: to assert that t is size (Rows x Cols)
   *
   *  MatrixSizeCheck<Rows, Cols>::check(t);
   */
  template<typename T>
  static void check(const Eigen::MatrixBase<T>& t)
  {
    StaticDimCheck<Eigen::MatrixBase<T>::RowsAtCompileTime, DesiredR>(t.rows());
    StaticDimCheck<Eigen::MatrixBase<T>::ColsAtCompileTime, DesiredC>(t.cols());
  }
};

template <int Dim>
using VectorSizeCheck = MatrixSizeCheck<Dim, 1>;

template <int Dim>
using RowVectorSizeCheck = MatrixSizeCheck<1, Dim>;

} /* namespace Eigen */

#endif /* _EIGEN_CHECKS_SIZE_CHECK_H_ */
