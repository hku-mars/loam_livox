#ifndef __PCL_TOOLS_HPP__
#define __PCL_TOOLS_HPP__
#include <iostream>
#include <stdio.h>
#include <math.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Eigen>
#include "os_compatible.hpp"


#define IF_SAVE_ACSII 0

#define min_f( a, b, c ) ( fminf( a, fminf( b, c ) ) )
#define max_f( a, b, c ) ( fmaxf( a, fmaxf( b, c ) ) )

namespace PCL_TOOLS
{
using namespace std;
using namespace Common_tools;
struct Pt_compare
{
    //inline bool operator()( const pcl::PointXYZ& a,  const pcl::PointXYZ & b)
    template < typename _T >
    inline bool operator()( const _T &a, const _T &b )
    {
        return ( ( a.x < b.x ) || ( a.x == b.x && a.y < b.y ) || ( ( a.x == b.x ) && ( a.y == b.y ) && ( a.z < b.z ) ) );
    }

    template < typename _T >
    bool operator()( const _T &a, const _T &b ) const
    {
        return ( a.x == b.x ) && ( a.y == b.y ) && ( a.z == b.z );
    }
};

struct Pt_hasher
{
    template < typename _T >
    std::size_t operator()( const _T &k ) const
    {
        return ( ( std::hash< float >()( k.x ) ^ ( std::hash< float >()( k.y ) << 1 ) ) >> 1 ) ^ ( std::hash< float >()( k.z ) << 1 );
    }
};


struct Eigen_pt_compare
{
    //inline bool operator()( const pcl::PointXYZ& a,  const pcl::PointXYZ & b)
    template < typename _T >
    inline bool operator()( const _T &a, const _T &b )
    {
        return ( ( a(0) < b(0) ) || ( a(0) == b(0) && a(1) < b(1) ) || ( ( a(0) == b(0) ) && ( a(1) == b(1) ) && ( a(2) < b(2) ) ) );
    }

    template < typename _T >
    bool operator()( const _T &a, const _T &b ) const
    {
        return ( a(0) == b(0) ) && ( a(1) == b(1) ) && ( a(2) == b(2) );
    }
};

struct Eigen_pt_hasher
{
    template < typename _T >
    std::size_t operator()( const _T &k ) const
    {
        return ( ( std::hash< float >()( k(0) ) ^ ( std::hash< float >()( k(1) ) << 1 ) ) >> 1 ) ^ ( std::hash< float >()( k(2) ) << 1 );
    }
};

template < typename TT, typename PointType >
Eigen::Matrix< TT, 3, 1 > pcl_pt_to_eigen( const PointType &pt )
{
    return Eigen::Matrix< TT, 3, 1 >( ( TT ) pt.x, ( TT ) pt.y, ( TT ) pt.z );
}

template < typename T >
Eigen::Matrix< float, 3, 1 > pcl_pt_to_eigenf( const T &pt )
{
    return pcl_pt_to_eigen<float>(pt);
}

template < typename T >
Eigen::Matrix< double, 3, 1 > pcl_pt_to_eigend( const T &pt )
{
    return pcl_pt_to_eigen<double>(pt);
}


template < typename TT, typename T >
TT eigen_to_pcl_pt( const T &pt )
{
    TT res_pt;
    res_pt.x = pt( 0 );
    res_pt.y = pt( 1 );
    res_pt.z = pt( 2 );
    return res_pt;
}

template < typename PointType, typename T >
pcl::PointCloud<PointType> eigen_pt_to_pcl_pointcloud( const vector<T> & eigen_pt_vec )
{
    pcl::PointCloud<PointType> pcl_pc_vec;
    pcl_pc_vec.resize(eigen_pt_vec.size());
    for (size_t i = 0; i< eigen_pt_vec.size() ; i++)
    {
        pcl_pc_vec[ i ] = eigen_to_pcl_pt< PointType >( eigen_pt_vec[ i ] );
    }
    return pcl_pc_vec;
}


template < typename T, typename PointType >
pcl::PointCloud< PointType > pointcloud_transfrom( pcl::PointCloud< PointType > pcl_pt_in, Eigen::Matrix< T, 3, 3 > tranfrom_R, Eigen::Matrix< T, 3, 1 > tranfrom_T )
{
    pcl::PointCloud< PointType > pcl_pc_res;
    pcl_pc_res.resize( pcl_pt_in.size() );
    //cout << "Pointcloud_transfrom_T: \r\n " << tranfrom_T.transpose() << endl;
    //cout << "Pointcloud_transfrom_R: \r\n " << tranfrom_R << endl;
    for ( size_t i = 0; i < pcl_pt_in.size(); i++ )
    {
        auto eigen_pt = PCL_TOOLS::pcl_pt_to_eigen< T, PointType >( pcl_pt_in.points[ i ] );
        pcl_pc_res.points.push_back( PCL_TOOLS::eigen_to_pcl_pt< PointType>( tranfrom_R * eigen_pt + tranfrom_T ) );
    }
    return pcl_pc_res;
}

int save_PCLXYZI_to_txt( const std::string &filename, const pcl::PointCloud<pcl::PointXYZI>::ConstPtr pcl_pt_vec, int if_append = 0 )
{
    std::fstream fos;
    if ( if_append > 0 )
    {
        fos.open( filename.c_str(), std::ios_base::app | std::ios_base::out );
    }
    else
    {
        fos.open( filename.c_str(), std::ios_base::out );
    }

    if ( fos.is_open() )
    {
        for(size_t idx = 0;idx < pcl_pt_vec->size(); idx++)
        {
            fos << pcl_pt_vec->points[idx].x << ","
                << pcl_pt_vec->points[idx].y << ","
                << pcl_pt_vec->points[idx].z << ","
                << pcl_pt_vec->points[idx].intensity 
                << std::endl;
        }
        fos.flush();
        fos.close();
        std::cout << "Save points to " << filename << " finish, points number =  " <<  pcl_pt_vec->size() << endl;
        return 0;
    }
    else
    {
        return -1;
    }
}

class PCL_point_cloud_to_pcd
{
  public:
    string m_save_dir_name;
    string m_save_file_name;
    int    m_save_files_index;

    ~PCL_point_cloud_to_pcd(){};
    PCL_point_cloud_to_pcd()
    {
        m_save_dir_name = "";
        m_save_files_index = 0;
    };

    void set_save_dir_name( string & dir_name )
    {
        cout << "PCD save dir set as :" << dir_name << endl;
        create_dir( dir_name );
        m_save_dir_name = dir_name;
    };

    template <typename T>
    void save_to_pcd_files( string file_name, T &cloud, int index = 0 )
    {
        char tempchar[ 5000 ];

        if ( index == 0 )
        {

            sprintf( tempchar, "%s/%s_%d.pcd", m_save_dir_name.c_str(), file_name.c_str(), m_save_files_index );
            m_save_files_index++;
        }
        else
        {

            sprintf( tempchar, "%s/%s_%d.pcd", m_save_dir_name.c_str(), file_name.c_str(), index );
            cout << "----- Save cloud to " << tempchar << " -----" << endl;
        }

#if IF_SAVE_ACSII
        pcl::io::savePCDFileASCII( tempchar, cloud );
#else
        pcl::io::savePCDFile( tempchar, cloud );
#endif
    };
};


template < typename T >
int load_from_pcd_file( const char *file_name, pcl::PointCloud< T > &target_cloud )
{
    if ( pcl::io::loadPCDFile< T >( file_name, target_cloud ) == -1 )
    {
        PCL_ERROR( "Couldn't read file %s \n", file_name );
        return ( 0 );
    }
    else
    {
        cout << "Get points number:" << target_cloud.size() << endl;
        return 1;
    }
};

void rgb2hsv( const unsigned char &src_r, const unsigned char &src_g, const unsigned char &src_b, unsigned char &dst_h, unsigned char &dst_s, unsigned char &dst_v )
{
    float r = src_r / 255.0f;
    float g = src_g / 255.0f;
    float b = src_b / 255.0f;

    float h, s, v; // h:0-360.0, s:0.0-1.0, v:0.0-1.0

    float max = max_f( r, g, b );
    float min = min_f( r, g, b );

    v = max;

    if ( max == 0.0f )
    {
        s = 0;
        h = 0;
    }
    else if ( max - min == 0.0f )
    {
        s = 0;
        h = 0;
    }
    else
    {
        s = ( max - min ) / max;

        if ( max == r )
        {
            h = 60 * ( ( g - b ) / ( max - min ) ) + 0;
        }
        else if ( max == g )
        {
            h = 60 * ( ( b - r ) / ( max - min ) ) + 120;
        }
        else
        {
            h = 60 * ( ( r - g ) / ( max - min ) ) + 240;
        }
    }

    if ( h < 0 )
        h += 360.0f;

    dst_h = ( unsigned char ) ( h / 2 );   // dst_h : 0-180
    dst_s = ( unsigned char ) ( s * 255 ); // dst_s : 0-255
    dst_v = ( unsigned char ) ( v * 255 ); // dst_v : 0-255
}

void hsv2rgb( const unsigned char &src_h, const unsigned char &src_s, const unsigned char &src_v, unsigned char &dst_r, unsigned char &dst_g, unsigned char &dst_b )
{
    // Online color picker: https://alloyui.com/examples/color-picker/hsv
    float h = src_h * 2.0f;   // 0-360
    float s = src_s / 255.0f; // 0.0-1.0
    float v = src_v / 255.0f; // 0.0-1.0

    float r = 0, g = 0, b = 0; // 0.0-1.0

    int   hi = ( int ) ( h / 60.0f ) % 6;
    float f = ( h / 60.0f ) - hi;
    float p = v * ( 1.0f - s );
    float q = v * ( 1.0f - s * f );
    float t = v * ( 1.0f - s * ( 1.0f - f ) );

    switch ( hi )
    {
    case 0:
        r = v, g = t, b = p;
        break;
    case 1:
        r = q, g = v, b = p;
        break;
    case 2:
        r = p, g = v, b = t;
        break;
    case 3:
        r = p, g = q, b = v;
        break;
    case 4:
        r = t, g = p, b = v;
        break;
    case 5:
        r = v, g = p, b = q;
        break;
    }
    //cout << "[" << r << " ," << g << " ," << b << " ]" ;
    dst_r = ( unsigned char ) ( r * 255 ); // dst_r : 0-255
    dst_g = ( unsigned char ) ( g * 255 ); // dst_r : 0-255
    dst_b = ( unsigned char ) ( b * 255 ); // dst_r : 0-255
    //cout << "[" << ( int ) dst_r << " ," << ( int ) dst_g << " ," << ( int ) dst_b << " ]" << endl;
}

pcl::PointCloud< pcl::PointXYZRGBA > PCL_XYZI_to_RGBA( pcl::PointCloud< pcl::PointXYZI > &pt_in, float alpha = 0.1 )
{
    int                                  size = pt_in.size();
    float                                min_val = 3e8;
    float                                max_val = -3e8;
    pcl::PointCloud< pcl::PointXYZRGBA > pt_out;
    pt_out.resize( size );
    for ( int i = 0; i < size; i++ )
    {
        min_val = fminf( min_val, pt_in[ i ].intensity );
        max_val = fmaxf( max_val, pt_in[ i ].intensity );
        pt_out[ i ].x = pt_in[ i ].x;
        pt_out[ i ].y = pt_in[ i ].y;
        pt_out[ i ].z = pt_in[ i ].z;
        pt_out[ i ].a = alpha;
    }
    std::cout << "Input point size = " << size << std::endl;
    printf( "Intensity min_max value =  [%.2f, %.2f] \r\n", min_val, max_val );
    unsigned char r =0, g =0 , b =0 ;
    float         h =0, s =0 , v =0 ;
    float         max_min_val_diff = max_val - min_val;
    s = 255.0;
    v = 255.0;
    for ( int i = 0; i < size; i++ )
    {
        r = g = b = 0;
        h = 240.0 * ( pt_in[ i ].intensity - min_val ) / max_min_val_diff;
        hsv2rgb( h, s, v, r, g, b );
        //cout << "[" << h << " ," << s << " ," << v << " ] ";
        //cout << "[" << ( int ) r << " ," << ( int ) g << " ," << ( int ) b << " ]" << endl;
        pt_out[ i ].r = r;
        pt_out[ i ].g = g;
        pt_out[ i ].b = b;
    }
    return pt_out;
}

template < typename T, typename PointType >
vector< Eigen::Matrix< T, 3, 1 > > pcl_pts_to_eigen_pts( const typename pcl::PointCloud< PointType >::Ptr input_cloud )
{
    vector< Eigen::Matrix< T, 3, 1 > > out_res;
    size_t                             pc_size = input_cloud->size();
    out_res.resize( pc_size );
    for ( size_t i = 0; i < pc_size; i++ )
    {
        //out_res[ i ] << input_cloud->points[ i ]x, input_cloud->points[ i ].y, input_cloud->points[ i ].z;
        out_res[i] << input_cloud->points[i].x, input_cloud->points[i].y, input_cloud->points[i].z;
    }
    return out_res;
}

} // namespace PCL_TOOLS
#endif
