// Author: Lin Jiarong          ziv.lin.ljr@gmail.com

#ifndef __LOGGER_HPP__
#define __LOGGER_HPP__
#include <ostream>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <map>
#include <string>
#include <sstream>
#include <sys/stat.h>  // mkdir dir
#include <stdarg.h> //need for such like printf(...)
using namespace std;

// #define FILE_LOGGER_VERSION      "V1.0"
// #define FILE_LOGGER_VERSION_INFO "First version"

#define FILE_LOGGER_VERSION      "V1.1"
#define FILE_LOGGER_VERSION_INFO "Add macro, make logger more easy to call"

#define LOG_FILE_LINE( x )                                             \
*( ( x ).get_ostream() ) << __FILE__ << "   " << __LINE__ << endl; \
( x ).get_ostream()->flush();
#define LOG_FILE_LINE_AB( a, b )                                          \
*( ( a ).get_ostream( b ) ) << __FILE__ << "   " << __LINE__ << endl; \
( a ).get_ostream()->flush();
#define LOG_FUNCTION_LINE( x )                                             \
*( ( x ).get_ostream() ) << __FUNCTION__ << "   " << __LINE__ << endl; \
( x ).get_ostream()->flush();
#define LOG_FUNCTION_LINE_AB( a, b )                                          \
*( ( a ).get_ostream( b ) ) << __FUNCTION__ << "   " << __LINE__ << endl; \
( x ).get_ostream()->flush();

namespace Common_tools // Commond tools
{
    class File_logger
    {
    public:
        std::map< string, std::ostream * > m_map_file_os;
        char m_temp_char[10000];
        string m_temp_string;
        string m_save_dir_name = string ( "/home/ziv/data/" );
        
        void release()
        {
            for ( auto it = m_map_file_os.begin(); it != m_map_file_os.end(); it++ )
            {
                //cout << "File logger relese ostream, name = [" << it->first << "]";
                it->second->flush();
                ( it->second ) = NULL;
                delete it->second ;
                //cout << " Done!" <<endl;
            }
        };
        
        ~File_logger()
        {
            release();
        };
        
        void set_log_dir ( string _dir_name )
        {
            release();
            m_save_dir_name = _dir_name;
            mkdir ( m_save_dir_name.c_str(), 0775 );
        }
        
        File_logger ( string _dir_name = string ( "/home/ziv/data/" ) )
        {
            set_log_dir ( _dir_name );
            m_map_file_os.insert ( std::pair<string, std::ostream*> ( "screen", &std::cout ) );
        }
        
        string version()
        {
            std::stringstream ss;
            ss << "===== This is version of File_logger =====" << endl;
            ss << "Version      : " << FILE_LOGGER_VERSION << endl;
            ss << "Version info : " << FILE_LOGGER_VERSION_INFO << endl;
            ss << "Complie date : " << __DATE__ << "  " << __TIME__ << endl;
            ss << "=====           End                  =====" << endl;
            return string ( ss.str() );
        }
        
        void init ( std::string _file_name, std::string prefix_name = string ( "log" ), int mode = std::ios::out )
        {
            std::ofstream* ofs = new std::ofstream();
            sprintf ( m_temp_char, "%s/%s_%s", m_save_dir_name.c_str(), prefix_name.c_str(), _file_name.c_str() );
            ofs->open ( m_temp_char, ios::out );
            
            if ( ofs->is_open() )
            {
                cout << "Open " << _file_name << " successful." << endl;
                m_map_file_os.insert ( std::pair<string, std::ostream*> ( prefix_name, ofs ) );
            }
            else
            {
                cout << "Fail to open " << _file_name  << endl;
                m_map_file_os.insert ( std::pair<string, std::ostream*> ( prefix_name, &std::cout ) );
            }
        };
        
        std::ostream* get_ostream ( std::string prefix_name = string ( "log" ) )
        {
            auto it = m_map_file_os.find ( prefix_name );
            
            if ( it != m_map_file_os.end() )
            {
                return ( it->second );
            }
            else // if no exit, create a new one.
            {
                init ( "tempadd.txt", prefix_name );
                return get_ostream ( prefix_name );
            }
        }
        
        int printf ( const char * fmt, ... )
        {
            va_list ap;
            char* result = 0;
            va_start ( ap, fmt );
            if(vasprintf ( &result, fmt, ap )==0)
            {
                return 0;
            }
            //cout << "Fmt = " << fmt <<endl;
            //cout << "Args" = << m_args <<endl;m_temp_char
            m_temp_string = string(result);
            //cout << m_temp_string;
            * ( get_ostream() ) << m_temp_string ;
            ( get_ostream ( "log" ) )->flush();
            return m_temp_string.length();
        }
        
    };
};
#endif
