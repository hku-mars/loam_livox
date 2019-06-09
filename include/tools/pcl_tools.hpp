// Author: Lin Jiarong          ziv.lin.ljr@gmail.com

#ifndef __PCL_TOOLS_HPP__
#define __PCL_TOOLS_HPP__
#include <iostream>
#include <stdio.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define IF_SAVE_ACSII 0

namespace PCL_TOOLS
{
using namespace std;

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

class PCL_tools
{
  public:
    string m_save_dir_name;
    string m_save_file_name;
    int    m_save_files_index;

    ~PCL_tools(){};
    PCL_tools()
    {
        m_save_dir_name = "";
        m_save_files_index = 0;
    };

    void set_save_dir_name( string dir_name )
    {
        cout << "PCD save dir set as :" << dir_name << endl;
        mkdir( dir_name.c_str(), 0775 );
        m_save_dir_name = dir_name;
    };

    template < typename T >
    void save_to_pcd_files( string file_name, T &cloud, int with_index = 0 )
    {
        char tempchar[ 5000 ];

        if ( with_index == 0 )
        {
            sprintf( tempchar, "%s/%s.pcd", m_save_dir_name.c_str(), file_name.c_str() );
            cout << "----- Save cloud to " << tempchar << " -----" << endl;
        }
        else
        {
            sprintf( tempchar, "%s/%s_%d.pcd", m_save_dir_name.c_str(), file_name.c_str(), m_save_files_index );
            m_save_files_index++;
        }

#if IF_SAVE_ACSII
        pcl::io::savePCDFileASCII( tempchar, cloud );
#else
        pcl::io::savePCDFile( tempchar, cloud );
#endif
    };
};
}
#endif
