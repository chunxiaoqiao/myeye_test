//
// Created by qiao on 20-6-20.
//
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
boost::mutex updateModelMutex;
boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    // --------------------------------------------

    // -----Open 3D viewer and add point cloud-----

    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setCameraPosition(0, 0, 0, 0, 0, 0, 0, 0, -1);
    viewer->setBackgroundColor(0,0,0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->initCameraParameters ();
    return (viewer);
}
void viewerRunner(boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer)
{
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100));
    }
}
int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> &pcloud1 = *cloud_ptr;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    pcloud1.points.push_back( pcl::PointXYZ(10, 10, 80) );
    pcloud1.width = cloud_ptr->size();
    pcloud1.height = 1;
    pcloud1.is_dense = true;
    viewer = simpleVis(cloud_ptr);
    boost::thread vthread(&viewerRunner,viewer);
    while(1)//循环抓取深度数据
    {
        pcloud1.clear();
        for ( int _row = 0; _row<100; _row++ )
        {
            for ( int _col = 0; _col < 100; _col++ )

            {
                float x, y, z;
                pcl::PointXYZ ptemp(x, y, z);
                pcloud1.points.push_back( ptemp );
            }
        }
        pcloud1.width = cloud_ptr->size();
        pcloud1.height = 1;
        pcloud1.is_dense = true;
        boost::mutex::scoped_lock updateLock(updateModelMutex);
        viewer->updatePointCloud<pcl::PointXYZ>(cloud_ptr,"sample cloud");
        updateLock.unlock();
        boost::this_thread::sleep (boost::posix_time::microseconds (100));
    }
//    vthread.joint();
    return 0;
}