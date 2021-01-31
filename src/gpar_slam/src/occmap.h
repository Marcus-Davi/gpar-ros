#ifndef OCCMAP_H
#define OCCMAP_H

#include <math.h>

namespace myslam
{

    class OccMap
    {
    public:
        OccMap(double res_, int size_) : size(size_), resolution(res_)
        {
            grid = new float[size * size];
            grid_index_update = new int[size * size];

            memset(grid, 0, size * size * sizeof(float));
            memset(grid_index_update, -1, size * size * sizeof(int));

            scale = 1 / resolution;
        }

        OccMap(double res_, int size_, float probOcc_, float probFree_) : size(size_), resolution(res_), probOcc(probOcc_), probFree(probFree_)
        {
            grid = new float[size * size];
            grid_index_update = new int[size * size];

            memset(grid, 0, size * size * sizeof(float));
            memset(grid_index_update, -1, size * size * sizeof(int));

            logOddOcc = log(probOcc / 1 - probOcc);
            logOddFree = log(probFree / 1 - probFree);

            scale = 1 / resolution;
        }

        void setProbHit(float update)
        {
            probOcc = update;
            logOddOcc = log(update / 1 - update);
        }

        void setProbMiss(float update)
        {
            probFree = update;
            logOddFree = log(update / 1 - update);
        }

        void setOccupancyThres(float thres)
        {
            occThres = thres;
        }

        // 0 < start < 1
        void setStartOrigin(float x, float y)
        {
            start_x = x;
            start_y = y;

            tfX = start_x * resolution * size;
            tfY = start_y * resolution * size;
        }

        // NAO USAR NUVEM JA TRANSFORMADA, POIS A POSE JA TEM ESSA INFO
        void insertPointCloudRays(octomap::Pointcloud &pointcloud, octomap::point3d &pose)
        {
            // Usar bresenham
            markFreeIndex = currUpdateIndex + 1;
            markOccIndex = currUpdateIndex + 2;

            int n = pointcloud.size();

            // Transforma p/ coordenada GRID
            octomap::Pointcloud pointcloud_tf(pointcloud); // copy construct ?

            for (int i = 0; i < n; ++i)
            {
                pointcloud_tf[i].x() += tfX;
                pointcloud_tf[i].y() += tfY;
            }

            // Sensor origin
            int ix0 = floor((pose.x() + tfX) * scale);
            int iy0 = floor((pose.y() + tfY) * scale);

            int ix, iy;
            for (int i = 0; i < n; ++i)
            {

                ix = round(pointcloud_tf[i].x() * scale);
                iy = round(pointcloud_tf[i].y() * scale);

                if (ix > this->size - 1 || iy > this->size - 1 || ix <= 0 || iy <= 0)
                {
                }
                else
                {
                    // send Rays
                    breseham(ix0, iy0, ix, iy);
                }
            }
        }

        ~OccMap()
        {
            delete grid;
        }

    private:
        void breseham(int x0, int y0, int x1, int y1)
        {
            if (abs(y1 - y0) < abs(x1 - x0))
            {
                if (x0 > x1)
                {
                    plotLineLow(x1, y1, x0, y0);
                }
                else
                {
                    plotLineLow(x0, y0, x1, y1);
                }
            }
            else
            {
                if (y0 > y1)
                {
                    plotLineHigh(x1, y1, x0, y0);
                }
                else
                {
                    plotLineHigh(x0, y0, x1, y1);
                }
            }
        }

        void plotLineLow(int x0, int y0, int x1, int y1)
        {
            int dx = x1 - x0;
            int dy = y1 - y0;
            int yi = 1;
            if (dy < 0)
            {
                yi = -1;
                dy = -dy;
            }
            int D = 2 * dy - dx;
            int y = y0;

            mapValue(grid, 2, 3) = 20;

            for (int x = x0; x < x1; ++x)
            {
                if (mapValue(grid_index_update, x, y) < currUpdateIndex)
                {
                }
            }
        }

        void plotLineHigh(int x0, int y0, int x1, int y1)
        {
        }

        //access map as a matrix
        template <class T>
        T &mapValue(T *array, int x, int y) //lvalue
        {
            return array[x + y * size]; // return access
        }

        // template <class T>
        // void setMapValue(T *array, int x, int y, float val)
        // {
        //     array[x + y * size] = val;
        // }

        void updateSetFree(int x, int y)
        {
            if (mapValue(grid, x, y) < 50.0){
                mapValue(grid, x, y) += this->logOddFree;
            }
        }

        void updateSetOcc(int x, int y)
        {

            mapValue(grid, x, y) += this->logOddFree;
        }

        void updateUnsetFree(int x, int y)
        {

            mapValue(grid,x,y) -= this->logOddFree;
        }

        float *grid;
        int *grid_index_update;

        double resolution;
        double scale;
        int size;
        float start_x;
        float start_y;

        float tfX;
        float tfY;
        float probOcc;
        float probFree;
        float logOddOcc;
        float logOddFree;
        float occThres; // Occupancy values greated than threshold are considered points in point clouds

        // Update Index
        int currUpdateIndex = 0;
        int markFreeIndex = -1;
        int markOccIndex = -1;
    };

} // namespace myslam

#endif