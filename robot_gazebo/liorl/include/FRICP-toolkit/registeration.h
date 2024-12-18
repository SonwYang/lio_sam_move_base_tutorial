#include <iostream>
#include "ICP.h"
#include "io_pc.h"
#include "FRICP.h"


typedef double Scalar;
typedef Eigen::Matrix<Scalar, 3, Eigen::Dynamic> Vertices;
typedef Eigen::Matrix<Scalar, 3, 1> VectorN;
 
class Registeration{
    public:
    Eigen::MatrixXd res_trans;
    enum Method{ICP, AA_ICP, FICP, RICP, PPL, RPPL, SparseICP, SICPPPL} method=RICP;
    int dim = 3;

    Registeration(int mode_){
        method = Method(mode_);
    }
    ~Registeration(){}

    Eigen::MatrixXd compute_transform(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& source,
             const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& target)
    {
        //--- Model that will be rigidly transformed
        Vertices vertices_source, normal_source, src_vert_colors;
        read_pcd_online(vertices_source, normal_source, src_vert_colors, source, dim);

        //--- Model that source will be aligned to
        Vertices vertices_target, normal_target, tar_vert_colors;
        read_pcd_online(vertices_target, normal_target, tar_vert_colors, target, dim);

        // scaling
        Eigen::Vector3d source_scale, target_scale;
        source_scale = vertices_source.rowwise().maxCoeff() - vertices_source.rowwise().minCoeff();
        target_scale = vertices_target.rowwise().maxCoeff() - vertices_target.rowwise().minCoeff();
        double scale = std::max(source_scale.norm(), target_scale.norm());
        std::cout << "scale = " << scale << std::endl;
        vertices_source /= scale;
        vertices_target /= scale;

        /// De-mean
        VectorN source_mean, target_mean;
        source_mean = vertices_source.rowwise().sum() / double(vertices_source.cols());
        target_mean = vertices_target.rowwise().sum() / double(vertices_target.cols());
        vertices_source.colwise() -= source_mean;
        vertices_target.colwise() -= target_mean;

        double time;
        // set ICP parameters
        ICP::Parameters pars;

        // set Sparse-ICP parameters
        SICP::Parameters spars;
        spars.p = 0.4;
        spars.print_icpn = false;


        ///--- Execute registration
        std::cout << "begin registration..." << std::endl;
        FRICP<3> fricp;
        double begin_reg = omp_get_wtime();
        double converge_rmse = 0;
        switch(method)
        {
            case ICP:
            {
                pars.f = ICP::NONE;
                pars.use_AA = false;
                fricp.point_to_point(vertices_source, vertices_target, source_mean, target_mean, pars);
                res_trans = pars.res_trans;
                break;
            }
            case AA_ICP:
            {
                AAICP::point_to_point_aaicp(vertices_source, vertices_target, source_mean, target_mean, pars);
                res_trans = pars.res_trans;
                break;
            }
            case FICP:
            {
                pars.f = ICP::NONE;
                pars.use_AA = true;
                fricp.point_to_point(vertices_source, vertices_target, source_mean, target_mean, pars);
                res_trans = pars.res_trans;
                break;
            }
            case RICP:
            {
                pars.f = ICP::WELSCH;
                pars.use_AA = true;
                fricp.point_to_point(vertices_source, vertices_target, source_mean, target_mean, pars);
                res_trans = pars.res_trans;
                break;
            }
            case PPL:
            {
                pars.f = ICP::NONE;
                pars.use_AA = false;
                if(normal_target.size()==0)
                {
                    std::cout << "Warning! The target model without normals can't run Point-to-plane method!" << std::endl;
                    exit(0);
                }
                fricp.point_to_plane(vertices_source, vertices_target, normal_source, normal_target, source_mean, target_mean, pars);
                res_trans = pars.res_trans;
                break;
            }
            case RPPL:
            {
                pars.nu_end_k = 1.0/6;
                pars.f = ICP::WELSCH;
                pars.use_AA = true;
                if(normal_target.size()==0)
                {
                    std::cout << "Warning! The target model without normals can't run Point-to-plane method!" << std::endl;
                    exit(0);
                }
                fricp.point_to_plane_GN(vertices_source, vertices_target, normal_source, normal_target, source_mean, target_mean, pars);
                res_trans = pars.res_trans;
                break;
            }
            case SparseICP:
            {
                SICP::point_to_point(vertices_source, vertices_target, source_mean, target_mean, spars);
                res_trans = spars.res_trans;
                break;
            }
            case SICPPPL:
            {
                if(normal_target.size()==0)
                {
                    std::cout << "Warning! The target model without normals can't run Point-to-plane method!" << std::endl;
                    exit(0);
                }
                SICP::point_to_plane(vertices_source, vertices_target, normal_target, source_mean, target_mean, spars);
                res_trans = spars.res_trans;
                break;
            }
        }
    
        return res_trans;
    }
};
