#ifndef __LOGGER_HPP__
#define __LOGGER_HPP__
#include "os_compatible.hpp"
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>
#include <ostream>
#include <sstream>
#include <stdarg.h> //need for such like printf(...)
#include <stdio.h>
#include <string>
#include <thread>

// #define FILE_LOGGER_VERSION      "V1.0"
// #define FILE_LOGGER_VERSION_INFO "First version"

//#define FILE_LOGGER_VERSION "V1.1"
//#define FILE_LOGGER_VERSION_INFO "Add macro, make logger more easy to call"

//#define FILE_LOGGER_VERSION "V1.2"
//#define FILE_LOGGER_VERSION_INFO "Compatible with windows."

//#define FILE_LOGGER_VERSION "V1.3"
//#define FILE_LOGGER_VERSION_INFO "Support verbose"

// #define FILE_LOGGER_VERSION "V1.4"
// #define FILE_LOGGER_VERSION_INFO "Add macro code block, enable turn off the printf quickly"

#define FILE_LOGGER_VERSION "V1.5"
#define FILE_LOGGER_VERSION_INFO "Add variable restore block, enable turn off/on screen prinf in local block."

#define printf_line printf( " %s %d \r\n", __FILE__, __LINE__ );

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

#define ADD_SCREEN_PRINTF_OUT_METHOD                                                            \
    int               m_if_verbose_screen_printf = 1;                                           \
    char *            m_sceen_output_cstr = new char[ 10000 ]();                                \
    std::string       m_sceen_output_string = std::string( m_sceen_output_cstr );               \
    std::stringstream m_sceen_output_stringstream = std::stringstream( m_sceen_output_string ); \
    inline void       screen_printf( const char *fmt, ... )                                     \
    {                                                                                           \
        if ( m_if_verbose_screen_printf )                                                       \
            return;                                                                             \
        va_list ap;                                                                             \
        va_start( ap, fmt );                                                                    \
        if ( Common_tools::vasprintf( &m_sceen_output_cstr, fmt, ap ) == 0 )                    \
        {                                                                                       \
            return;                                                                             \
        }                                                                                       \
        std::printf( "%s", m_sceen_output_cstr );                                               \
    }                                                                                           \
    std::ostream *sreen_outstream()                                                             \
    {                                                                                           \
        if ( m_if_verbose_screen_printf )                                                       \
        {                                                                                       \
            return &m_sceen_output_stringstream;                                                \
        }                                                                                       \
        else                                                                                    \
        {                                                                                       \
            return &std::cout;                                                                  \
        }                                                                                       \
    }

#define screen_out ( *sreen_outstream() )
#define ENABLE_SCREEN_PRINTF Common_tools::Variable_restore_point< int > val_restore( &m_if_verbose_screen_printf, 0 );
#define DISABLE_SCREEN_PRINTF Common_tools::Variable_restore_point< int > val_restore( &m_if_verbose_screen_printf, 1 );

namespace Common_tools // Commond tools
{
using namespace std;

template < typename T >
struct Variable_restore_point
{
    T *m_targer_variable;
    T  m_initial_value;

    Variable_restore_point( T *variable_ptr )
    {
        m_targer_variable = variable_ptr;
        m_initial_value = *m_targer_variable;
    }

    Variable_restore_point( T *variable_ptr, const T temp_val )
    {
        m_targer_variable = variable_ptr;
        m_initial_value = *m_targer_variable;
        *m_targer_variable = temp_val;
    }

    ~Variable_restore_point()
    {
        *m_targer_variable = m_initial_value;
    }
};

class File_logger
{
  public:
    std::map< string, std::ostream * > m_map_file_os;
    char                               m_temp_char[ 10000 ];
    string                             m_temp_string;
    std::stringstream                  m_temp_stringstream;
    string                             m_save_dir_name = string( "./" );
    std::mutex                         m_mutex_log;
    int                                m_if_verbose = 0;
    void                               release()
    {
        for ( auto it = m_map_file_os.begin(); it != m_map_file_os.end(); it++ )
        {
            //cout << "File logger relese ostream, name = [" << it->first << "]";
            it->second->flush();
            ( it->second ) = NULL;
            delete it->second;
            //cout << " Done!" <<endl;
        }
    };

    ~File_logger()
    {
        release();
    };

    void set_log_dir( string _dir_name )
    {
        release();
        m_save_dir_name = _dir_name;
        create_dir( m_save_dir_name );
        //mkdir(m_save_dir_name.c_str(), 0775);
    }

    File_logger( string _dir_name = string( "/home/ziv/data/" ) )
    {
        set_log_dir( _dir_name );
        m_map_file_os.insert( std::pair< string, std::ostream * >( "screen", &std::cout ) );
        m_temp_string.reserve( 1e4 );
        m_temp_stringstream = std::stringstream( m_temp_string );
    }

    string version()
    {
        std::stringstream ss;
        ss << "===== This is version of File_logger =====" << endl;
        ss << "Version      : " << FILE_LOGGER_VERSION << endl;
        ss << "Version info : " << FILE_LOGGER_VERSION_INFO << endl;
        ss << "Complie date : " << __DATE__ << "  " << __TIME__ << endl;
        ss << "=====           End                  =====" << endl;
        return string( ss.str() );
    }

    void init( std::string _file_name, std::string prefix_name = string( "log" ), int mode = std::ios::out )
    {
        std::ofstream *ofs = new std::ofstream();
        sprintf( m_temp_char, "%s/%s_%s", m_save_dir_name.c_str(), prefix_name.c_str(), _file_name.c_str() );
        ofs->open( m_temp_char, ios::out );

        if ( ofs->is_open() )
        {
            cout << "Open " << _file_name << " successful." << endl;
            m_map_file_os.insert( std::pair< string, std::ostream * >( prefix_name, ofs ) );
        }
        else
        {
            cout << "Fail to open " << _file_name << endl;
            m_map_file_os.insert( std::pair< string, std::ostream * >( prefix_name, &std::cout ) );
        }
    };

    std::ostream *get_ostream( std::string prefix_name = string( "log" ) )
    {
        if ( m_if_verbose )
            return &m_temp_stringstream;
        auto it = m_map_file_os.find( prefix_name );

        if ( it != m_map_file_os.end() )
        {
            return ( it->second );
        }
        else // if no exit, create a new one.
        {
            init( "tempadd.txt", prefix_name );
            return get_ostream( prefix_name );
        }
    }

    int printf( const char *fmt, ... )
    {
        if ( m_if_verbose )
            return 0;
        std::unique_lock< std::mutex > lock( m_mutex_log );
#ifdef _WIN32
        va_list ap;
        char *  result = 0;
        va_start( ap, fmt );
        if ( vasprintf( &result, fmt, ap ) == 0 )
        {
            return 0;
        }
        //cout << "Fmt = " << fmt <<endl;
        //cout << "Args" = << m_args <<endl;m_temp_char
        m_temp_string = string( result );
        //cout << m_temp_string;
        *( get_ostream() ) << m_temp_string;
        ( get_ostream( "log" ) )->flush();
        return m_temp_string.length();
        //return 0;
#else
        va_list ap;
        char *  result = 0;
        va_start( ap, fmt );
        if ( vasprintf( &result, fmt, ap ) == 0 )
        {
            return 0;
        }
        //cout << "Fmt = " << fmt <<endl;
        //cout << "Args" = << m_args <<endl;m_temp_char
        m_temp_string = string( result );
        //cout << m_temp_string;
        *( get_ostream() ) << m_temp_string;
        ( get_ostream( "log" ) )->flush();
        return m_temp_string.length();
#endif
    }
};
}; // namespace Common_tools
#endif
