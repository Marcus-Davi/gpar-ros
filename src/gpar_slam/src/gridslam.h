#ifndef GRIDSLAM_H
#define GRIDSLAM_H

#include <pcl/point_types.h>
#include <octomap/octomap.h>
#include <Eigen/Dense>

namespace myslam
{

    class MyMap{
        public:

        private:

    };



    static inline void TransformEndpoint(pcl::PointXYZ& pt, const Eigen::Vector3d& transform){
        double rx = cos(transform[2])*pt.x - sin(transform[2])*pt.y;
        double ry = sin(transform[2])*pt.x + cos(transform[2])*pt.y;

        pt.x = rx + transform[0];
        pt.y = ry + transform[1];


    }

    static inline double mapAccess(const octomap::OcTree &tree, pcl::PointXYZ endpoint)
    {
        octomap::point3d pt(endpoint.x, endpoint.y, endpoint.z);
        octomap::OcTreeKey key_p00 = tree.coordToKey(pt);
        octomap::OcTreeKey key_p01(key_p00);
        key_p01.k[0]--;
        octomap::OcTreeKey key_p11(key_p00);
        key_p11.k[0]--;
        key_p11.k[1]++;
        octomap::OcTreeKey key_p10(key_p00);
        key_p10.k[1]++;

        octomap::OcTreeNode *node_00 = tree.search(key_p00);
        octomap::OcTreeNode *node_10 = tree.search(key_p10);
        octomap::OcTreeNode *node_11 = tree.search(key_p11);
        octomap::OcTreeNode *node_01 = tree.search(key_p01);

        double p00 = 0.4, p01 = 0.4, p10 = 0.4, p11 = 0.4;
        if (node_00)
            p00 = node_00->getOccupancy();

        if (node_10)
            p10 = node_10->getOccupancy();

        if (node_11)
            p11 = node_11->getOccupancy();

        if (node_01)
            p01 = node_01->getOccupancy();

        octomap::point3d pt_00 = tree.keyToCoord(key_p00);
        octomap::point3d pt_01 = tree.keyToCoord(key_p01);
        octomap::point3d pt_11 = tree.keyToCoord(key_p11);
        octomap::point3d pt_10 = tree.keyToCoord(key_p10);

        double dx0 = (pt_00.x() - pt.x());
        double dy0 = (pt.y() - pt_00.y());

        double dx1 = (pt.x() - pt_11.x());
        double dy1 = (pt_11.y() - pt.y());

        double scale = 1.0 / (tree.getResolution() * tree.getResolution());

        double prob = scale * (dy0 * (dx0 * p11 + dx1 * p10) + dy1 * (dx0 * p01 + dx1 * p00));

        return prob;
    }

    template <int row, int col>
    static inline Eigen::Matrix<double, row, col> mapGradient(const octomap::OcTree &tree, pcl::PointXYZ endpoint)
    {

        octomap::point3d pt(endpoint.x, endpoint.y, endpoint.z);
        octomap::OcTreeKey key_p00 = tree.coordToKey(pt);
        octomap::OcTreeKey key_p01(key_p00);
        key_p01.k[0]--;
        octomap::OcTreeKey key_p11(key_p00);
        key_p11.k[0]--;
        key_p11.k[1]++;
        octomap::OcTreeKey key_p10(key_p00);
        key_p10.k[1]++;

        octomap::OcTreeNode *node_00 = tree.search(key_p00);
        octomap::OcTreeNode *node_10 = tree.search(key_p10);
        octomap::OcTreeNode *node_11 = tree.search(key_p11);
        octomap::OcTreeNode *node_01 = tree.search(key_p01);

        double p00 = 0.4, p01 = 0.4, p10 = 0.4, p11 = 0.4;
        if (node_00)
            p00 = node_00->getOccupancy();

        if (node_10)
            p10 = node_10->getOccupancy();

        if (node_11)
            p11 = node_11->getOccupancy();

        if (node_01)
            p01 = node_01->getOccupancy();

        octomap::point3d pt_00 = tree.keyToCoord(key_p00);
        octomap::point3d pt_01 = tree.keyToCoord(key_p01);
        octomap::point3d pt_11 = tree.keyToCoord(key_p11);
        octomap::point3d pt_10 = tree.keyToCoord(key_p10);

        double dx0 = (pt_00.x() - pt.x());
        double dy0 = (pt.y() - pt_00.y());

        double dx1 = (pt.x() - pt_11.x());
        double dy1 = (pt_11.y() - pt.y());

        double scale = 1.0 / (tree.getResolution() * tree.getResolution());

        double dmx = scale * (dy0*(p10 - p11) + dy1*(p00 - p01));
        double dmy = scale * (dx0*(p11 - p01) + dx1*(p10 - p00));

        Eigen::Matrix<double, row, col> retval;

        retval(0) = dmx;
        retval(1) = dmy;

        return retval;
    }

    template <int row, int col>
    static inline Eigen::Matrix<double, row, col> modelGradient(const pcl::PointXYZ& endpoint,const Eigen::Vector3d& pose)
    {
        // Eigen::MatrixXd deriv(row, col);
        Eigen::Matrix<double, row, col> deriv;

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
