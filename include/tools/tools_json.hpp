#ifndef __JSON_TOOLS_H__
#define __JSON_TOOLS_H__

#include "rapidjson/rapidjson.h"

#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include "rapidjson/prettywriter.h"

namespace Common_tools
{
template < typename T >
T *get_json_array( const rapidjson::Document::Array &json_array )
{
    T *res_mat = new T[ json_array.Size() ];
    for ( size_t i = 0; i < json_array.Size(); i++ )
    {
        res_mat[ i ] = ( T ) json_array[ i ].GetDouble();
    }
    return res_mat;
};

template < typename T, typename TT >
void save_mat_to_jason_writter( T &writer, const std::string &name, const TT &eigen_mat )
{
    writer.Key( name.c_str() ); // output a key,
    writer.StartArray();        // Between StartArray()/EndArray(),
    for ( size_t i = 0; i < ( size_t )( eigen_mat.cols() * eigen_mat.rows() ); i++ )
        writer.Double( eigen_mat( i ) );
    writer.EndArray();
}
} // namespace Common_tools

#endif