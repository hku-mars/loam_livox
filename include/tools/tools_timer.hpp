#ifndef __TIME_TICKER_HPP__
#define __TIME_TICKER_HPP__

#include <chrono>
#include <iostream>
#include <map>
#include <string>
#include <stdlib.h>
#include <thread>
namespace Common_tools // Commond tools
{

static std::chrono::time_point< std::chrono::system_clock > timer_now()
{
    return std::chrono::system_clock::now();
}

static double timer_tic()
{
   //std::time_t t = std::chrono::system_clock::to_time_t( timer_now() );
   auto t = std::chrono::system_clock::to_time_t( timer_now() );
   return double( t );
}

class Timer
{
  public:
    typedef std::map< std::string, std::chrono::time_point< std::chrono::system_clock > >           Map_string_timepoint;
    typedef std::map< std::string, std::chrono::time_point< std::chrono::system_clock > >::iterator Map_string_timepoint_it;
    // typedef std::map<std::string, double> Map_string_timepoint;
  private:
    Map_string_timepoint m_map_str_timepoint;
    char m_temp_char[4096];
    int  m_if_with_threadid = 1;
  public:

    Map_string_timepoint_it find_timepoint( const std::string &str )
    {
        Map_string_timepoint_it it = m_map_str_timepoint.find( str );
        if ( it == m_map_str_timepoint.end() )
        {
            m_map_str_timepoint.insert( std::make_pair( str, timer_now() ) );
            return m_map_str_timepoint.find( str );
        }
        else
        {
            return it;
        }
    }

    Timer()
    {
        ( find_timepoint( std::string( " " ) ) )->second = timer_now();
    }

    std::string get_thread_id_str()
    {
      if(m_if_with_threadid)
      {
        std::stringstream ss;
        ss << std::this_thread::get_id();
        //cout << "Id =" << std::this_thread::get_id() << endl;
        return  ss.str();
      }
      else
      {
        return std::to_string(0);
      }
    }


    uint64_t get_thread_id()
    {
        return std::stoull(get_thread_id_str());
     }


    void tic( std::string str = std::string( " " ) )
    {
      find_timepoint( str.append(  get_thread_id_str()  ) )->second = timer_now();
    }

    double toc( std::string str = std::string( " " ), int if_retick = 1 )
    {

        Map_string_timepoint_it it = find_timepoint( str.append(  get_thread_id_str() ) ) ;

        std::chrono::duration<double> time_diff = timer_now() - it->second;
        if ( if_retick )
        {
            it->second = timer_now();
        }
        return time_diff.count() * 1000;
        ;
    }

    std::string toc_string( std::string str = std::string( " " ), int if_retick = 1 )
    {

        sprintf( m_temp_char, "[Timer](%s): %s cost time = %.2f ms ",  get_thread_id_str().c_str(), str.c_str(), toc( str, if_retick ) );
        return std::string( m_temp_char );
    }
};

} // namespace Common_tools

#endif
