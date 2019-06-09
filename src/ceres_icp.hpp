// Author: Lin Jiarong          ziv.lin.ljr@gmail.com

#ifndef __ceres_icp_hpp__
#define __ceres_icp_hpp__
#define MAX_LOG_LEVEL -100
#include "eigen_math.hpp"
#include <Eigen/Eigen>
#include <ceres/ceres.h>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <math.h>
#include <stdio.h>
#include <vector>
// Point to Point ICP
// Contour to contour ICP
// Plane to Plane ICP

//p2p with motion deblur
template <typename _T>
struct ceres_icp_point2point
{
    Eigen::Matrix<_T, 3, 1> m_current_pt;
    Eigen::Matrix<_T, 3, 1> m_closest_pt;
    _T m_motion_blur_s;
    ceres_icp_point2point( const Eigen::Matrix<_T, 3, 1> &current_pt,
                           const Eigen::Matrix<_T, 3, 1> &closest_pt,
                           const _T &motion_blur_s = 1.0 ) : m_current_pt( current_pt ), m_closest_pt( closest_pt ), m_motion_blur_s( motion_blur_s ){};

    template <typename T>
    bool operator()( const T *_q, const T *_t, T *residual ) const
    {

        Eigen::Quaternion<T> q_start{ T( 1.0 ), T( 0 ), T( 0 ), T( 0 ) };
        Eigen::Matrix<T, 3, 1> t_start{ T( 0 ), T( 0 ), T( 0 ) };

        Eigen::Quaternion<T> q_end{ _q[ 3 ], _q[ 0 ], _q[ 1 ], _q[ 2 ] };
        Eigen::Matrix<T, 3, 1> t_end{ _t[ 0 ], _t[ 1 ], _t[ 2 ] };

        Eigen::Quaternion<T> q_interpolate = q_start.slerp( ( T ) m_motion_blur_s, q_end );
        Eigen::Matrix<T, 3, 1> t_interpolate = t_start * T( 1.0 - m_motion_blur_s ) + t_end * T( m_motion_blur_s );

        Eigen::Matrix<T, 3, 1> pt{ T( m_current_pt( 0 ) ), T( m_current_pt( 1 ) ), T( m_current_pt( 2 ) ) };
        Eigen::Matrix<T, 3, 1> pt_tranfromed;

        pt_tranfromed = q_interpolate * pt + t_interpolate;

        residual[ 0 ] = pt_tranfromed( 0 ) - T( m_closest_pt( 0 ) );
        residual[ 1 ] = pt_tranfromed( 1 ) - T( m_closest_pt( 1 ) );
        residual[ 2 ] = pt_tranfromed( 2 ) - T( m_closest_pt( 2 ) );
        return true;
    };

    static ceres::CostFunction *Create( const Eigen::Matrix<_T, 3, 1> current_pt,
                                        const Eigen::Matrix<_T, 3, 1> closest_pt,
                                        const _T motion_blur_s = 1.0 )
    {
        return ( new ceres::AutoDiffCostFunction<
                 ceres_icp_point2point, 3, 4, 3>(
            new ceres_icp_point2point( current_pt, closest_pt, motion_blur_s ) ) );
    }
};

//point-to-line with motion deblur
template <typename _T>
struct ceres_icp_point2line
{
    Eigen::Matrix<_T, 3, 1> m_current_pt;
    Eigen::Matrix<_T, 3, 1> m_target_line_a, m_target_line_b;
    Eigen::Matrix<_T, 3, 1> m_unit_vec_ab;
    _T m_motion_blur_s;
    Eigen::Matrix<_T, 4, 1> m_q_start;
    Eigen::Matrix<_T, 3, 1> m_t_start;
    _T m_weigth;
    ceres_icp_point2line( const Eigen::Matrix<_T, 3, 1> &current_pt,
                          const Eigen::Matrix<_T, 3, 1> &target_line_a,
                          const Eigen::Matrix<_T, 3, 1> &target_line_b,
                          const _T motion_blur_s = 1.0,
                          Eigen::Matrix<_T, 4, 1> q_s = Eigen::Matrix<_T, 4, 1>( 1, 0, 0, 0 ),
                          Eigen::Matrix<_T, 3, 1> t_s = Eigen::Matrix<_T, 3, 1>( 0, 0, 0 ) ) : m_current_pt( current_pt ), m_target_line_a( target_line_a ),
                                                                                               m_target_line_b( target_line_b ),
                                                                                               m_motion_blur_s( motion_blur_s ),
                                                                                               m_q_start( q_s ),
                                                                                               m_t_start( t_s )
    {
        m_unit_vec_ab = target_line_b - target_line_a;
        m_unit_vec_ab = m_unit_vec_ab / m_unit_vec_ab.norm();
        // m_weigth = 1/m_current_pt.norm();
        m_weigth = 1.0;
        //cout << m_unit_vec_ab.transpose() <<endl;
    };

    template <typename T>
    bool operator()( const T *_q, const T *_t, T *residual ) const
    {

        Eigen::Quaternion<T> q_I( ( T ) 1.0, ( T ) 0.0, ( T ) 0.0, ( T ) 0.0 );

        Eigen::Quaternion<T> q_start{ ( T ) m_q_start( 0 ), ( T ) m_q_start( 1 ), ( T ) m_q_start( 2 ), ( T ) m_q_start( 3 ) };
        Eigen::Matrix<T, 3, 1> t_start{ ( T ) m_t_start( 0 ), ( T ) m_t_start( 1 ), ( T ) m_t_start( 2 ) };

        Eigen::Quaternion<T> q_end{ _q[ 3 ], _q[ 0 ], _q[ 1 ], _q[ 2 ] };
        Eigen::Matrix<T, 3, 1> t_end{ _t[ 0 ], _t[ 1 ], _t[ 2 ] };

        Eigen::Quaternion<T> q_interpolate = q_I.slerp( ( T ) m_motion_blur_s, q_end );
        // Eigen::Matrix< T, 3, 1 > t_interpolate = t_start *  T ( 1.0 - m_motion_blur_s )  +  t_end * T ( m_motion_blur_s );
        Eigen::Matrix<T, 3, 1> t_interpolate = t_end * T( m_motion_blur_s );

        Eigen::Matrix<T, 3, 1> pt{ T( m_current_pt( 0 ) ), T( m_current_pt( 1 ) ), T( m_current_pt( 2 ) ) };
        Eigen::Matrix<T, 3, 1> pt_transfromed;
        pt_transfromed = q_start * ( q_interpolate * pt + t_interpolate ) + t_start;

        Eigen::Matrix<T, 3, 1> tar_line_pt_a{ T( m_target_line_a( 0 ) ), T( m_target_line_a( 1 ) ), T( m_target_line_a( 2 ) ) };
        Eigen::Matrix<T, 3, 1> vec_line_ab_unit{ T( m_unit_vec_ab( 0 ) ), T( m_unit_vec_ab( 1 ) ), T( m_unit_vec_ab( 2 ) ) };

        Eigen::Matrix<T, 3, 1> vec_ac = pt_transfromed - tar_line_pt_a;
        Eigen::Matrix<T, 3, 1> residual_vec = vec_ac - Eigen_math::vector_project_on_unit_vector( vec_ac, vec_line_ab_unit );

        residual[ 0 ] = residual_vec( 0 ) * T( m_weigth );
        residual[ 1 ] = residual_vec( 1 ) * T( m_weigth );
        residual[ 2 ] = residual_vec( 2 ) * T( m_weigth );

        //cout << " *** " << residual[0] << "  " << residual[1] << "  " << residual[2] << " *** " << endl;

        //cout << residual[0]<< "  " << residual[1] << "  " <<endl;
        return true;
    };

    static ceres::CostFunction *Create( const Eigen::Matrix<_T, 3, 1> &current_pt,
                                        const Eigen::Matrix<_T, 3, 1> &target_line_a,
                                        const Eigen::Matrix<_T, 3, 1> &target_line_b,
                                        const _T motion_blur_s = 1.0,
                                        Eigen::Matrix<_T, 4, 1> q_s = Eigen::Matrix<_T, 4, 1>( 1, 0, 0, 0 ),
                                        Eigen::Matrix<_T, 3, 1> t_s = Eigen::Matrix<_T, 3, 1>( 0, 0, 0 ) )
    {
        // TODO: can be vector or distance
        return ( new ceres::AutoDiffCostFunction<
                 ceres_icp_point2line, 3, 4, 3>(
            new ceres_icp_point2line( current_pt, target_line_a, target_line_b, motion_blur_s ) ) );
    }
};

// point to plane with motion deblur
template <typename _T>
struct ceres_icp_point2plane
{
    Eigen::Matrix<_T, 3, 1> m_current_pt;
    Eigen::Matrix<_T, 3, 1> m_target_line_a, m_target_line_b, m_target_line_c;
    Eigen::Matrix<_T, 3, 1> m_unit_vec_ab, m_unit_vec_ac, m_unit_vec_n;
    _T m_motion_blur_s;
    _T m_weigth;
    Eigen::Matrix<_T, 4, 1> m_q_start;
    Eigen::Matrix<_T, 3, 1> m_t_start;
    ceres_icp_point2plane( const Eigen::Matrix<_T, 3, 1> &current_pt,
                           const Eigen::Matrix<_T, 3, 1> &target_line_a,
                           const Eigen::Matrix<_T, 3, 1> &target_line_b,
                           const Eigen::Matrix<_T, 3, 1> &target_line_c,
                           const _T motion_blur_s = 1.0,
                           Eigen::Matrix<_T, 4, 1> q_s = Eigen::Matrix<_T, 4, 1>( 1, 0, 0, 0 ),
                           Eigen::Matrix<_T, 3, 1> t_s = Eigen::Matrix<_T, 3, 1>( 0, 0, 0 ) ) : m_current_pt( current_pt ), m_target_line_a( target_line_a ),
                                                                                                m_target_line_b( target_line_b ),
                                                                                                m_target_line_c( target_line_c ),
                                                                                                m_motion_blur_s( motion_blur_s ),
                                                                                                m_q_start( q_s ),
                                                                                                m_t_start( t_s )

    {
        //assert( motion_blur_s <= 1.5 && motion_blur_s >= 0.0 );
        //assert( motion_blur_s <= 1.01 && motion_blur_s >= 0.0 );
        m_unit_vec_ab = target_line_b - target_line_a;
        m_unit_vec_ab = m_unit_vec_ab / m_unit_vec_ab.norm();

        m_unit_vec_ac = target_line_c - target_line_a;
        m_unit_vec_ac = m_unit_vec_ac / m_unit_vec_ac.norm();

        m_unit_vec_n = m_unit_vec_ab.cross( m_unit_vec_ac );
        //m_unit_vec_n = m_unit_vec_n / m_unit_vec_n.norm();
        // m_weigth = 1/m_current_pt.norm();
        m_weigth = 1.0;
        //m_weigth = motion_blur_s;
        //cout << " --- " << m_unit_vec_n.transpose() << endl;
    };

    template <typename T>
    bool operator()( const T *_q, const T *_t, T *residual ) const
    {

        Eigen::Quaternion<T> q_I( ( T ) 1.0, ( T ) 0.0, ( T ) 0.0, ( T ) 0.0 );

        Eigen::Quaternion<T> q_start{ ( T ) m_q_start( 0 ), ( T ) m_q_start( 1 ), ( T ) m_q_start( 2 ), ( T ) m_q_start( 3 ) };
        Eigen::Matrix<T, 3, 1> t_start{ ( T ) m_t_start( 0 ), ( T ) m_t_start( 1 ), ( T ) m_t_start( 2 ) };

        Eigen::Quaternion<T> q_end{ _q[ 3 ], _q[ 0 ], _q[ 1 ], _q[ 2 ] };
        Eigen::Matrix<T, 3, 1> t_end{ _t[ 0 ], _t[ 1 ], _t[ 2 ] };

        Eigen::Quaternion<T> q_interpolate = q_I.slerp( ( T ) m_motion_blur_s, q_end );
        // Eigen::Matrix< T, 3, 1 > t_interpolate = t_start *  T ( 1.0 - m_motion_blur_s )  +  t_end * T ( m_motion_blur_s );
        Eigen::Matrix<T, 3, 1> t_interpolate = t_end * T( m_motion_blur_s );

        Eigen::Matrix<T, 3, 1> pt{ T( m_current_pt( 0 ) ), T( m_current_pt( 1 ) ), T( m_current_pt( 2 ) ) };
        Eigen::Matrix<T, 3, 1> pt_transfromed;
        pt_transfromed = q_start * ( q_interpolate * pt + t_interpolate ) + t_start;

        Eigen::Matrix<T, 3, 1> tar_line_pt_a{ T( m_target_line_a( 0 ) ), T( m_target_line_a( 1 ) ), T( m_target_line_a( 2 ) ) };
        Eigen::Matrix<T, 3, 1> vec_line_plane_norm{ T( m_unit_vec_n( 0 ) ), T( m_unit_vec_n( 1 ) ), T( m_unit_vec_n( 2 ) ) };

        Eigen::Matrix<T, 3, 1> vec_ad = pt_transfromed - tar_line_pt_a;
        Eigen::Matrix<T, 3, 1> residual_vec = Eigen_math::vector_project_on_unit_vector( vec_ad, vec_line_plane_norm ) * T( m_weigth );

        residual[ 0 ] = residual_vec( 0 ) * T( m_weigth );
        residual[ 1 ] = residual_vec( 1 ) * T( m_weigth );
        residual[ 2 ] = residual_vec( 2 ) * T( m_weigth );
        //cout << residual_vec.rows() << "  " <<residual_vec.cols()  <<endl;

        //cout << " *** " << residual_vec[0] << "  " << residual_vec[1] << "  " << residual_vec[2] ;
        //cout << " *** " << residual[0] << "  " << residual[1] << "  " << residual[2] << " *** " << endl;
        return true;
    };

    static ceres::CostFunction *Create( const Eigen::Matrix<_T, 3, 1> &current_pt,
                                        const Eigen::Matrix<_T, 3, 1> &target_line_a,
                                        const Eigen::Matrix<_T, 3, 1> &target_line_b,
                                        const Eigen::Matrix<_T, 3, 1> &target_line_c,
                                        const _T motion_blur_s = 1.0,
                                        Eigen::Matrix<_T, 4, 1> q_s = Eigen::Matrix<_T, 4, 1>( 1, 0, 0, 0 ),
                                        Eigen::Matrix<_T, 3, 1> t_s = Eigen::Matrix<_T, 3, 1>( 0, 0, 0 ) )
    {
        // TODO: can be vector or distance
        return ( new ceres::AutoDiffCostFunction<
                 ceres_icp_point2plane, 3, 4, 3>(
            new ceres_icp_point2plane( current_pt, target_line_a, target_line_b, target_line_c, motion_blur_s, q_s, t_s ) ) );
    }
};

#endif
