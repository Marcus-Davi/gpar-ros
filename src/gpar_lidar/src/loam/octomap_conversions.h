#include <octomap/octomap.h>
#include <pcl/point_cloud.h>

static inline void octomap2pcl(const octomap::OcTree &tree, pcl::PointCloud<pcl::PointXYZ> &pc)
{
    octomap::point3d pt;
    pc.clear();
    for (octomap::OcTree::iterator it = tree.begin(); it != tree.end(); ++it)
    {
        if (tree.isNodeOccupied(*it))
        {
            pt = it.getCoordinate();
            pc.push_back(pcl::PointXYZ(pt.x(), pt.y(), pt.z()));
        }
    }
    return;
}

static inline void pcl2octopc(const pcl::PointCloud<pcl::PointXYZ> &pc, octomap::Pointcloud& octo_pc){
    octo_pc.clear();
		for(int i=0;i<pc.size();++i){
			octomap::point3d pt(pc.points[i].x,pc.points[i].y,pc.points[i].z);
			octo_pc.push_back(pt);
		}
}