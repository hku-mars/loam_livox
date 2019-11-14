#ifndef __CERES_POSE_GRAPH_3D_HPP__
#define __CERES_POSE_GRAPH_3D_HPP__
#include <Eigen/Eigen>
#include <ceres/ceres.h>

/***************************************
 *
 * Copy form ceres:
 *
*/
namespace Ceres_pose_graph_3d
{
struct Pose3d
{
    Eigen::Matrix<double, 3, 1> p;
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();

    // The name of the data type in the g2o file format.
    static std::string name()
    {
        return "VERTEX_SE3:QUAT";
    }

    Pose3d() = default;
    Pose3d( Eigen::Quaterniond &in_q, Eigen::Vector3d &in_t ) :  p( in_t ), q( in_q ){};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

struct Constraint3d
{
    int id_begin;
    int id_end;

    // The transformation that represents the pose of the end frame E w.r.t. the
    // begin frame B. In other words, it transforms a vector in the E frame to
    // the B frame.
    Pose3d t_be;

    // The inverse of the covariance matrix for the measurement. The order of the
    // entries are x, y, z, delta orientation.
    Eigen::Matrix< double, 6, 6 > information;
    Constraint3d()
    {
        information.setIdentity();
    };
    Constraint3d( int id_begin, int id_end, Eigen::Quaterniond &q, Eigen::Vector3d &p ) : id_begin( id_begin ), id_end( id_end ), t_be( q, p )
    {
        information.setIdentity();
    };
    // The name of the data type in the g2o file format.
    static std::string name()
    {
        return "EDGE_SE3:QUAT";
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

inline std::istream &operator>>( std::istream &input, Pose3d &pose )
{
    input >> pose.p.x() >> pose.p.y() >> pose.p.z() >> pose.q.x() >>
        pose.q.y() >> pose.q.z() >> pose.q.w();
    // Normalize the quaternion to account for precision loss due to
    // serialization.
    pose.q.normalize();
    return input;
}

inline std::istream &operator>>( std::istream &input, Constraint3d &constraint )
{
    Pose3d &t_be = constraint.t_be;
    input >> constraint.id_begin >> constraint.id_end >> t_be;

    for ( int i = 0; i < 6 && input.good(); ++i )
    {
        for ( int j = i; j < 6 && input.good(); ++j )
        {
            input >> constraint.information( i, j );
            if ( i != j )
            {
                constraint.information( j, i ) = constraint.information( i, j );
            }
        }
    }
    return input;
}

typedef std::map< int, Pose3d, std::less< int >, Eigen::aligned_allocator< std::pair< const int, Pose3d > > > MapOfPoses;
typedef std::vector< Constraint3d, Eigen::aligned_allocator< Constraint3d > >                                 VectorOfConstraints;
typedef std::vector< Pose3d, Eigen::aligned_allocator< Pose3d > >                                             VectorOfPose;

class G2O_reader
{
  public:
    template < typename Pose, typename Allocator >
    static bool ReadVertex( std::ifstream *                                     infile,
                            std::map< int, Pose, std::less< int >, Allocator > *poses )
    {
        int  id;
        Pose pose;
        *infile >> id >> pose;

        // Ensure we don't have duplicate poses.
        if ( poses->find( id ) != poses->end() )
        {
            LOG( ERROR ) << "Duplicate vertex with ID: " << id;
            return false;
        }
        ( *poses )[ id ] = pose;

        return true;
    }

    template < typename Constraint, typename Allocator >
    static void ReadConstraint( std::ifstream *                       infile,
                                std::vector< Constraint, Allocator > *constraints )
    {
        Constraint constraint;
        *infile >> constraint;
        constraints->push_back( constraint );
    }

    template < typename Pose, typename Constraint, typename MapAllocator, typename VectorAllocator >
    static bool ReadG2oFile( const std::string &                                    filename,
                             std::map< int, Pose, std::less< int >, MapAllocator > *poses,
                             std::vector< Constraint, VectorAllocator > *           constraints )
    {

        poses->clear();
        constraints->clear();

        std::ifstream infile( filename.c_str() );
        if ( !infile )
        {
            return false;
        }

        std::string data_type;
        while ( infile.good() )
        {
            // Read whether the type is a node or a constraint.
            infile >> data_type;
            if ( data_type == Pose::name() )
            {
                if ( !ReadVertex( &infile, poses ) )
                {
                    return false;
                }
            }
            else if ( data_type == Constraint::name() )
            {
                ReadConstraint( &infile, constraints );
            }
            else
            {
                LOG( ERROR ) << "Unknown data type: " << data_type;
                return false;
            }

            // Clear any trailing whitespace from the line.
            infile >> std::ws;
        }

        return true;
    };
};

// Computes the error term for two poses that have a relative pose measurement
// between them. Let the hat variables be the measurement. We have two poses x_a
// and x_b. Through sensor measurements we can measure the transformation of
// frame B w.r.t frame A denoted as t_ab_hat. We can compute an error metric
// between the current estimate of the poses and the measurement.
//
// In this formulation, we have chosen to represent the rigid transformation as
// a Hamiltonian quaternion, q, and position, p. The quaternion ordering is
// [x, y, z, w].

// The estimated measurement is:
//      t_ab = [ p_ab ]  = [ R(q_a)^T * (p_b - p_a) ]
//             [ q_ab ]    [ q_a^{-1] * q_b         ]
//
// where ^{-1} denotes the inverse and R(q) is the rotation matrix for the
// quaternion. Now we can compute an error metric between the estimated and
// measurement transformation. For the orientation error, we will use the
// standard multiplicative error resulting in:
//
//   error = [ p_ab - \hat{p}_ab                 ]
//           [ 2.0 * Vec(q_ab * \hat{q}_ab^{-1}) ]
//
// where Vec(*) returns the vector (imaginary) part of the quaternion. Since
// the measurement has an uncertainty associated with how accurate it is, we
// will weight the errors by the square root of the measurement information
// matrix:
//
//   residuals = I^{1/2) * error
// where I is the information matrix which is the inverse of the covariance.
class PoseGraph3dErrorTerm
{
  public:
    PoseGraph3dErrorTerm( const Pose3d &                       t_ab_measured,
                          const Eigen::Matrix< double, 6, 6 > &sqrt_information )
        : t_ab_measured_( t_ab_measured ), sqrt_information_( sqrt_information ) {}

    template < typename T >
    bool operator()( const T *const p_a_ptr, const T *const q_a_ptr,
                     const T *const p_b_ptr, const T *const q_b_ptr,
                     T *residuals_ptr ) const
    {
        Eigen::Map< const Eigen::Matrix< T, 3, 1 > > p_a( p_a_ptr );
        Eigen::Map< const Eigen::Quaternion< T > >   q_a( q_a_ptr );

        Eigen::Map< const Eigen::Matrix< T, 3, 1 > > p_b( p_b_ptr );
        Eigen::Map< const Eigen::Quaternion< T > >   q_b( q_b_ptr );

        // Compute the relative transformation between the two frames.
        Eigen::Quaternion< T > q_a_inverse = q_a.conjugate();
        Eigen::Quaternion< T > q_ab_estimated = q_a_inverse * q_b;

        // Represent the displacement between the two frames in the A frame.
        Eigen::Matrix< T, 3, 1 > p_ab_estimated = q_a_inverse * ( p_b - p_a );

        // Compute the error between the two orientation estimates.
        Eigen::Quaternion< T > delta_q =
            t_ab_measured_.q.template cast< T >() * q_ab_estimated.conjugate();

        // Compute the residuals.
        // [ position         ]   [ delta_p          ]
        // [ orientation (3x1)] = [ 2 * delta_q(0:2) ]
        Eigen::Map< Eigen::Matrix< T, 6, 1 > > residuals( residuals_ptr );
        residuals.template block< 3, 1 >( 0, 0 ) =
            p_ab_estimated - t_ab_measured_.p.template cast< T >();
        residuals.template block< 3, 1 >( 3, 0 ) = T( 2.0 ) * delta_q.vec();

        // Scale the residuals by the measurement uncertainty.
        residuals.applyOnTheLeft( sqrt_information_.template cast< T >() );

        return true;
    }

    static ceres::CostFunction *Create(
        const Pose3d &                       t_ab_measured,
        const Eigen::Matrix< double, 6, 6 > &sqrt_information )
    {
        return new ceres::AutoDiffCostFunction< PoseGraph3dErrorTerm, 6, 3, 4, 3, 4 >(
            new PoseGraph3dErrorTerm( t_ab_measured, sqrt_information ) );
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:
    // The measurement for the position of B relative to A in the A frame.
    const Pose3d t_ab_measured_;
    // The square root of the measurement information matrix.
    const Eigen::Matrix< double, 6, 6 > sqrt_information_;
};

// Output the poses to the file with format: id x y z q_x q_y q_z q_w.
bool OutputPoses(const std::string& filename, const MapOfPoses& poses) {
  std::fstream outfile;
  outfile.open(filename.c_str(), std::istream::out);
  if (!outfile) {
    LOG(ERROR) << "Error opening the file: " << filename;
    return false;
  }
  for (std::map<int, Pose3d, std::less<int>,
                Eigen::aligned_allocator<std::pair<const int, Pose3d> > >::
           const_iterator poses_iter = poses.begin();
       poses_iter != poses.end(); ++poses_iter) {
    const std::map<int, Pose3d, std::less<int>,
                   Eigen::aligned_allocator<std::pair<const int, Pose3d> > >::
        value_type& pair = *poses_iter;
    outfile << pair.first << " " << pair.second.p.transpose() << " "
            << pair.second.q.x() << " " << pair.second.q.y() << " "
            << pair.second.q.z() << " " << pair.second.q.w() << '\n';
  }
  return true;
}

// Constructs the nonlinear least squares optimization problem from the pose
// graph constraints.
void BuildOptimizationProblem( const VectorOfConstraints &constraints,
                               MapOfPoses *poses, ceres::Problem *problem )
{

    if ( constraints.empty() )
    {
        std::cout << "No constraints, no problem to optimize." << std::endl;
        return;
    }

    ceres::LossFunction *         loss_function = NULL;
    ceres::LocalParameterization *quaternion_local_parameterization =
        new ceres::EigenQuaternionParameterization;

    for ( VectorOfConstraints::const_iterator constraints_iter =
              constraints.begin();
          constraints_iter != constraints.end(); ++constraints_iter )
    {
        const Constraint3d &constraint = *constraints_iter;

        MapOfPoses::iterator pose_begin_iter = poses->find( constraint.id_begin );
        MapOfPoses::iterator pose_end_iter = poses->find( constraint.id_end );

        const Eigen::Matrix< double, 6, 6 > sqrt_information =
            constraint.information.llt().matrixL();
        // Ceres will take ownership of the pointer.
        ceres::CostFunction *cost_function =
            PoseGraph3dErrorTerm::Create( constraint.t_be, sqrt_information );

        problem->AddResidualBlock( cost_function, loss_function,
                                   pose_begin_iter->second.p.data(),
                                   pose_begin_iter->second.q.coeffs().data(),
                                   pose_end_iter->second.p.data(),
                                   pose_end_iter->second.q.coeffs().data() );

        problem->SetParameterization( pose_begin_iter->second.q.coeffs().data(),
                                      quaternion_local_parameterization );
        problem->SetParameterization( pose_end_iter->second.q.coeffs().data(),
                                      quaternion_local_parameterization );
    }

    // The pose graph optimization problem has six DOFs that are not fully
    // constrained. This is typically referred to as gauge freedom. You can apply
    // a rigid body transformation to all the nodes and the optimization problem
    // will still have the exact same cost. The Levenberg-Marquardt algorithm has
    // internal damping which mitigates this issue, but it is better to properly
    // constrain the gauge freedom. This can be done by setting one of the poses
    // as constant so the optimizer cannot change it.
    MapOfPoses::iterator pose_start_iter = poses->begin();
    CHECK( pose_start_iter != poses->end() ) << "There are no poses.";
    problem->SetParameterBlockConstant( pose_start_iter->second.p.data() );
    problem->SetParameterBlockConstant( pose_start_iter->second.q.coeffs().data() );
}

void pose_graph_optimization( MapOfPoses &         poses,
                              VectorOfConstraints &constraints )
{
    ceres::Problem problem;
    BuildOptimizationProblem( constraints, &poses, &problem );

    ceres::Solver::Options options;
    options.max_num_iterations = 200;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

    ceres::Solver::Summary summary;
    ceres::Solve( options, &problem, &summary );

    //std::cout << summary.FullReport() << '\n';
    std::cout << summary.BriefReport() << '\n';
    std::cout << summary.IsSolutionUsable() << std::endl;
}

void pose_graph_optimization_from_g2o_file( std::string &in_file_name )
{
    std::cout << "*************************************" << std::endl;
    std::cout << "Post graph optimization form g2o file: " << in_file_name << std::endl;
    std::cout << "*************************************" << std::endl;
    MapOfPoses          poses;
    VectorOfConstraints constraints;
    G2O_reader::ReadG2oFile( in_file_name, &poses, &constraints );
    pose_graph_optimization( poses, constraints );
}

} // namespace Ceres_pose_graph_3d

template<typename PointType>
class Mapping_refine
{
  public:
    pcl::VoxelGrid< PointType > m_down_sample_filter;

    Ceres_pose_graph_3d::MapOfPoses pose3d_map_oir, pose3d_map_opm;
    pcl::PointCloud< PointType > m_pts_aft_refind;
    float                            m_down_sample_res = 0.2;
    int                              m_step_skip = 2;
    std::string                           m_save_path;
    Mapping_refine()
    {
        set_down_sample_resolution(0.2);
    };

    void set_down_sample_resolution(float res)
    {
        m_down_sample_res = res;
        m_down_sample_filter.setLeafSize( res, res, res );
        //m_pts_aft_refind.reserve(1e8);
    }

    void set_save_dir(std::string & path_name)
    {
        m_save_path =  path_name;
        Common_tools::create_dir(m_save_path);
    }

    static bool load_poses_from_txt_files( std::string file_name, Ceres_pose_graph_3d::MapOfPoses &pose3d_map )
    {
        std::ifstream infile( file_name.c_str() );
        if ( !infile )
        {
            std::cout << "Open file: " << file_name << " fail, please check" << std::endl;
            return false;
        }

        std::string data_type;
        int         id;
        while ( infile.good() )
        {
            Ceres_pose_graph_3d::G2O_reader::ReadVertex( &infile, &pose3d_map );
            infile >> std::ws;
        }
        infile.close();
        return true;
    }

    static bool load_filename_from_txt_files( std::string file_name, std::map< int, std::string > &map_file_name )
    {
        std::ifstream infile( file_name.c_str() );
        if ( !infile )
        {
            std::cout << "Open file: " << file_name << " fail, please check" << std::endl;
            return false;
        }

        std::string data_type;
        int         id = 0;
        std::string name;
        while ( infile.good() )
        {
            infile >> id >> name;
            map_file_name.insert( std::make_pair( id, name ) );
        }
        infile.close();
        return true;
    }

    template < typename T, typename TT >
    static std::vector< Eigen::Matrix< T, 3, 1 > > refine_pts( std::vector< Eigen::Matrix< T, 3, 1 > > &raw_pt_vec,
                                                        const Eigen::Matrix< TT, 3, 3 > &R_mat_ori, Eigen::Matrix< TT, 3, 1 > &t_vec_ori,
                                                        const Eigen::Matrix< TT, 3, 3 > &R_mat_opm, Eigen::Matrix< TT, 3, 1 > &t_vec_opm )
    {
        std::vector< Eigen::Matrix< float, 3, 1 > > opm_pt_vec;
        opm_pt_vec.resize( raw_pt_vec.size() );
        Eigen::Matrix< T, 3, 3 > R_aff = ( R_mat_opm * R_mat_ori.transpose() ).template cast< T >();
        Eigen::Matrix< T, 3, 1 > T_aff = ( R_aff.template cast< TT >() * ( -t_vec_ori ) + t_vec_opm ).template cast< T >();

        for ( size_t i = 0; i < raw_pt_vec.size(); i++ )
        {
            opm_pt_vec[ i ] = R_aff * raw_pt_vec[ i ] + T_aff;
        }
        return opm_pt_vec;
    }

    template<typename _PointType>
    pcl::PointCloud< _PointType >  refine_pointcloud(  std::map<int, pcl::PointCloud< _PointType >> & map_idx_pc,
                          Ceres_pose_graph_3d::MapOfPoses & pose3d_map_oir,
                          Ceres_pose_graph_3d::MapOfPoses & pose3d_map_opm,
                          int idx = 0,
                          int if_save = 1)
    {
        assert( map_idx_pc.size() == pose3d_map_oir.size() );
        assert( map_idx_pc.size() == pose3d_map_opm.size() );
        pcl::PointCloud< _PointType > pcl_pts;
        auto it = std::next(map_idx_pc.begin(), idx);
        if(it== map_idx_pc.end())
        {
          return pcl_pts;
        }
        else
        {
            int                         id = it->first;
            std::vector< Eigen::Matrix< float, 3, 1 > > pts_vec_ori, pts_vec_opm;
            Ceres_pose_graph_3d::Pose3d pose_ori = pose3d_map_oir.find( id )->second;
            Ceres_pose_graph_3d::Pose3d pose_opm = pose3d_map_opm.find( id )->second;
            auto pc_in =map_idx_pc.find(id)->second.makeShared();
            if(if_save == 0)
            {
              m_down_sample_filter.setInputCloud( pc_in );
              m_down_sample_filter.filter( *pc_in );
            }
            pts_vec_ori = PCL_TOOLS::pcl_pts_to_eigen_pts< float, _PointType >( pc_in ) ;

            pts_vec_opm = refine_pts( pts_vec_ori, pose_ori.q.toRotationMatrix(), pose_ori.p,
                                      pose_opm.q.toRotationMatrix(), pose_opm.p );
            pcl_pts = PCL_TOOLS::eigen_pt_to_pcl_pointcloud< _PointType >( pts_vec_opm );
            if ( if_save )
            {
                pts_vec_opm = PCL_TOOLS::pcl_pts_to_eigen_pts< float, _PointType >( pcl_pts.makeShared() );
                Points_cloud_map< float > pt_cell_temp;
                pt_cell_temp.set_resolution( 1.0 );
                pt_cell_temp.set_point_cloud( pts_vec_opm );
                pt_cell_temp.save_to_file( m_save_path, std::string( "/refined_" ).append( std::to_string( it->first ) ).append( ".json" ) );
                pt_cell_temp.clear_data();
            }

            return pcl_pts;
            //m_down_sample_filter.setInputCloud( m_pts_aft_refind.makeShared() );
            //m_down_sample_filter.filter( m_pts_aft_refind );
        }
    }

    template<typename _PointType>
    void refine_mapping(  std::map<int, pcl::PointCloud< _PointType >> & map_idx_pc,
                          Ceres_pose_graph_3d::MapOfPoses & pose3d_map_oir,
                          Ceres_pose_graph_3d::MapOfPoses & pose3d_map_opm,
                          int if_save = 1)
    {
        assert( map_idx_pc.size() == pose3d_map_oir.size() );
        assert( map_idx_pc.size() == pose3d_map_opm.size() );

        std::vector< Eigen::Matrix< float, 3, 1 > > pts_vec_ori, pts_vec_opm;
        for ( auto it = pose3d_map_oir.begin(); it != pose3d_map_oir.end(); it++ )
        {
            int                         id = it->first;
            Ceres_pose_graph_3d::Pose3d pose_ori = pose3d_map_oir.find( id )->second;
            Ceres_pose_graph_3d::Pose3d pose_opm = pose3d_map_opm.find( id )->second;

            pts_vec_ori = PCL_TOOLS::pcl_pts_to_eigen_pts< float, _PointType >( map_idx_pc.find(id)->second.makeShared() ) ;
            pts_vec_opm = refine_pts( pts_vec_ori, pose_ori.q.toRotationMatrix(), pose_ori.p,
                                      pose_opm.q.toRotationMatrix(), pose_opm.p );
            pcl::PointCloud< _PointType > pcl_pts = PCL_TOOLS::eigen_pt_to_pcl_pointcloud< _PointType >( pts_vec_opm );
            if ( if_save )
            {
                pts_vec_opm = PCL_TOOLS::pcl_pts_to_eigen_pts< float, _PointType >( pcl_pts.makeShared() );
                Points_cloud_map< float > pt_cell_temp;
                pt_cell_temp.set_resolution( 1.0 );
                pt_cell_temp.set_point_cloud( pts_vec_opm );
                pt_cell_temp.save_to_file( m_save_path, std::string( "/refined_" ).append( std::to_string( it->first ) ).append( ".json" ) );
                pt_cell_temp.clear_data();
            }
            m_down_sample_filter.setInputCloud( pcl_pts.makeShared() );
            m_down_sample_filter.filter( pcl_pts );
            m_pts_aft_refind += pcl_pts;
            //m_down_sample_filter.setInputCloud( m_pts_aft_refind.makeShared() );
            //m_down_sample_filter.filter( m_pts_aft_refind );
            std::next(it, m_step_skip);
        }
    }

    void refine_mapping( std::string &path_name, int if_save = 1 )
    {
        m_save_path = path_name;
        m_pts_aft_refind.clear();
        std::string file_name_vec, file_pose_ori, file_poses_opm;
        file_name_vec = std::string( path_name ).append( "/file_name.txt" );
        file_pose_ori = std::string( path_name ).append( "/poses_ori.txt" );
        file_poses_opm = std::string( path_name ).append( "/poses_opm.txt" );
        std::map< int, std::string >    filename_map;
        load_filename_from_txt_files( file_name_vec, filename_map );
        load_poses_from_txt_files( file_pose_ori, pose3d_map_oir );
        load_poses_from_txt_files( file_poses_opm, pose3d_map_opm );
        std::cout << "Size of files:" << filename_map.size() << std::endl;
        std::cout << "Size of poses_ori:" << pose3d_map_oir.size() << std::endl;
        std::cout << "Size of poses_opm:" << pose3d_map_opm.size() << std::endl;
        std::string save_path_name = std::string( path_name ).append( "/refined" );
        Common_tools::create_dir( save_path_name );
        assert( filename_map.size() == pose3d_map_oir.size() );
        assert( filename_map.size() == pose3d_map_opm.size() );
        Points_cloud_map< float >                                  pt_cell_map;
        Points_cloud_map< float >                                  pt_cell_full;
        std::vector< std::vector< Eigen::Matrix< float, 3, 1 > > > pts_vec_vec;
        pcl::PointCloud< pcl::PointXYZ > pcl_pts;
        std::map<int, pcl::PointCloud< pcl::PointXYZ >> map_idx_pc;
        for ( auto it = filename_map.begin(); it != filename_map.end(); it++ )
        {
            int         id = it->first;
            std::string file_name = it->second;
            std::cout << "In file name = " << file_name << std::endl;
            //std::cout << pose_ori.p.transpose() << std::endl;
            //std::cout << pose_opm.p.transpose() << std::endl;
            std::vector< Eigen::Matrix< float, 3, 1 > > pts_vec_ori, pts_vec_opm;
            pts_vec_ori = pt_cell_map.load_pts_from_file( file_name );
            map_idx_pc.insert(std::make_pair( id, PCL_TOOLS::eigen_pt_to_pcl_pointcloud< pcl::PointXYZ >( pts_vec_ori ) ));
        }
        refine_mapping<pcl::PointXYZ>(map_idx_pc,pose3d_map_oir, pose3d_map_opm, if_save );

        m_down_sample_filter.setInputCloud( m_pts_aft_refind.makeShared() );
        m_down_sample_filter.filter( m_pts_aft_refind );
        pt_cell_full.set_resolution( 10.0 );
        pt_cell_full.set_point_cloud( PCL_TOOLS::pcl_pts_to_eigen_pts< float, pcl::PointXYZ >( m_pts_aft_refind.makeShared() ) );
        pt_cell_full.save_to_file( save_path_name, "mapping_refined.json" );
    }
};

#endif
