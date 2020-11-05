#ifndef GRIDSLAM_H
#define GRIDSLAM_H

#include <pcl/point_types.h>
#include <octomap/octomap.h>
#include <Eigen/Dense>

namespace myslam
{

    static inline double mapAccess(const octomap::OcTree &tree,pcl::PointXYZ endpoint)
    {
        octomap::point3d pt(endpoint.x, endpoint.y, endpoint.z);
        double prob = 0;
        octomap::OcTreeNode* node = tree.search(pt);

        if(node)
        prob = node->getOccupancy();

        return prob;

    }

    template<int row,int col>
    static inline Eigen::Matrix<double,row,col> mapGradient(const octomap::OcTree &tree, pcl::PointXYZ endpoint)
    {
       //Repenstar

        Eigen::Matrix<double,row,col> retval;

        retval(0,0) = dmx;
        retval(0,1) = dmy;

        return retval;
    }

    template <int row, int col>
    static inline Eigen::Matrix<double,row,col> modelGradient(pcl::PointXYZ endpoint, Eigen::Vector3d pose)
    {
        // Eigen::MatrixXd deriv(row, col);
        Eigen::Matrix<double,row,col> deriv;

        deriv(0, 0) = 1;
        deriv(0, 1) = 0;
        deriv(0, 2) = -sin(pose[2]) * endpoint.x - cos(pose[2]) * endpoint.y;

        deriv(1, 0) = 0;
        deriv(1, 1) = 1;
        deriv(1, 2) = cos(pose[2]) * endpoint.x - sin(pose[2]) * endpoint.y;

        return deriv;
    }

} // namespace myslam

#endif
