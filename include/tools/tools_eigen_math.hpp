// Author: Jiarong Lin          ziv.lin.ljr@gmail.com

#ifndef __EIGEN_MATH_HPP__
#define __EIGEN_MATH_HPP__

#include <Eigen/Eigen>
#include <math.h>
#include <stdio.h>

namespace Eigen_math
{
template <typename T>
static T vector_project_on_vector( const T &vec_a, const T &vec_b )
{
    return vec_a.dot( vec_b ) * vec_b / ( vec_b.norm() * vec_b.norm() );
}

template <typename T>
static T vector_project_on_unit_vector( const T &vec_a, const T &vec_b )
{
    return vec_a.dot( vec_b ) * vec_b;
}

template <typename T>
T vector_angle( const Eigen::Matrix<T, 3, 1> &vec_a, const Eigen::Matrix<T, 3, 1> &vec_b, int if_force_sharp_angle = 0 )
{
    T vec_a_norm = vec_a.norm();
    T vec_b_norm = vec_b.norm();
    if ( vec_a_norm == 0 || vec_b_norm == 0 ) // zero vector is pararrel to any vector.
    {
        return 0.0;
    }
    else
    {
        if ( if_force_sharp_angle )
        {
            // return acos( abs( vec_a.dot( vec_b ) )*0.9999 / ( vec_a_norm * vec_b_norm ) );
            return acos( abs( vec_a.dot( vec_b ) ) / ( vec_a_norm * vec_b_norm ) );
        }
        else
        {
            // return acos( (vec_a.dot(vec_b))*0.9999 / (vec_a_norm*vec_b_norm));
            return acos( ( vec_a.dot( vec_b ) ) / ( vec_a_norm * vec_b_norm ) );
        }
    }
}
} // namespace Eigen_math

#endif
