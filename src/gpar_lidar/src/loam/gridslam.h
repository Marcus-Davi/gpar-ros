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
        octomap::point3d pt(endpoint.x, endpoint.y, endpoint.z);
        octomap::OcTreeKey key_center = tree.coordToKey(pt);
        octomap::OcTreeKey key_right(key_center); // x+1
        key_right.k[0]++;
        octomap::OcTreeKey key_corner(key_center); //x+1, y+1
        key_corner.k[1]++;
        key_corner.k[0]++;
        octomap::OcTreeKey key_top(key_center); //y+1
        key_top.k[1]++;
        // Node Probs
        octomap::OcTreeNode *node_center = tree.search(key_center);
        octomap::OcTreeNode *node_right = tree.search(key_right);
        octomap::OcTreeNode *node_corner = tree.search(key_corner);
        octomap::OcTreeNode *node_top = tree.search(key_top);

        octomap::point3d pt_center = tree.keyToCoord(key_center);
        octomap::point3d pt_right = tree.keyToCoord(key_right);
        octomap::point3d pt_corner = tree.keyToCoord(key_corner);
        octomap::point3d pt_top = tree.keyToCoord(key_top);

        double dx0 = pt.x() - pt_center.x();
        double dy0 = pt.y() - pt_center.y();

        double dx1 = pt_corner.x() - pt.x();
        double dy1 = pt_corner.y() - pt.y();

        double p00 = 0, p01 = 0, p11 = 0, p10 = 0;
        if (node_center != NULL)
            p00 = node_center->getOccupancy();
        if (node_right != NULL)
            p10 = node_right->getOccupancy();

        if (node_corner != NULL)
            p11 = node_corner->getOccupancy();

        if (node_top != NULL)
            p01 = node_top->getOccupancy();

        double dmx = dy0 * (p11 - p01) + dy1 * (p10 - p00);
        double dmy = dx0 * (p11 - p10) + dx1 * (p01 - p00);

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
