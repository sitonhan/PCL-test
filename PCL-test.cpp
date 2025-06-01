#define _CRT_SECURE_NO_WARNINGS

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/pfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/common/distances.h>
#include <pcl/surface/concave_hull.h>
#include <boost/thread/thread.hpp>

// 定义一个结构体来存储包围盒的AABB的信息
struct AABB {
    float min_x, max_x;
    float min_y, max_y;
    float min_z, max_z;
};

// 计算点云的AABB
AABB computeAABB(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    AABB aabb;
    if (cloud->points.empty()) {
        aabb.min_x = aabb.max_x = 0;
        aabb.min_y = aabb.max_y = 0;
        aabb.min_z = aabb.max_z = 0;
        return aabb;
    }

    aabb.min_x = aabb.max_x = cloud->points[0].x;
    aabb.min_y = aabb.max_y = cloud->points[0].y;
    aabb.min_z = aabb.max_z = cloud->points[0].z;

    for (const auto& point : cloud->points) {
        if (point.x < aabb.min_x) aabb.min_x = point.x;
        if (point.x > aabb.max_x) aabb.max_x = point.x;
        if (point.y < aabb.min_y) aabb.min_y = point.y;
        if (point.y > aabb.max_y) aabb.max_y = point.y;
        if (point.z < aabb.min_z) aabb.min_z = point.z;
        if (point.z > aabb.max_z) aabb.max_z = point.z;
    }

    return aabb;
}

// 计算点到平面的距离
float computeDistanceToPlane(const pcl::PointXYZ point, const std::vector<float>& planeModel) {
    float numerator = std::abs(planeModel[0] * point.x + planeModel[1] * point.y + planeModel[2] * point.z + planeModel[3]);
    float denominator = std::sqrt(planeModel[0] * planeModel[0] + planeModel[1] * planeModel[1] + planeModel[2] * planeModel[2]);
    float distance = numerator / denominator;
    return distance;
}

// 对点云进行切块分割
int cropPointCloud(const pcl::PointXYZ point1, const pcl::PointXYZ point2, 
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudCropped)
{
    // 创建 CropBox 滤波器对象
    pcl::CropBox<pcl::PointXYZ> cropBoxFilter;
    cropBoxFilter.setInputCloud(cloud/*halfCloud*/);
    // 定义立方体区域的边界
    Eigen::Vector4f minPoint;
    minPoint[0] = point1.x; // X 最小值
    minPoint[1] = point1.y; // Y 最小值
    minPoint[2] = point1.z; // Z 最小值
    minPoint[3] = 1.0;

    Eigen::Vector4f maxPoint;
    maxPoint[0] = point2.x; // X 最大值
    maxPoint[1] = point2.y; // Y 最大值
    maxPoint[2] = point2.z; // Z 最大值
    maxPoint[3] = 1.0;

    cropBoxFilter.setMin(minPoint);
    cropBoxFilter.setMax(maxPoint);

    // 存储筛选结果的点云
    cropBoxFilter.filter(*cloudCropped);
    pcl::io::savePCDFile("./data/dianji_filtered.pcd", *cloudCropped);
    // 输出筛选结果
    std::cout << "筛选出的点云大小: " << cloudCropped->points.size() << std::endl;

    return 0;
}

// 计算切片
int slicePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
    const std::vector<float> planeModel,
	float gap,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudSliced)
{
    //计算点到平面距离，小于gap的判定为平面截取的点轮廓
    //创建轮廓点云，压缩到一个平面上
    for (const auto& point : cloud->points)
    {
        float distance = computeDistanceToPlane(point, planeModel);
        if (distance < gap)
        {
            cloudSliced->push_back(point);
        }
    }
    // 输出筛选结果
    std::cout << "轮廓的点云大小: " << cloudSliced->points.size() << std::endl;
    pcl::io::savePCDFile("./data/dianji_contour.pcd", *cloudSliced);

    return 0;
}

// 搜索点云中与指定点距离最近点的索引
int searchNearestPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointXYZ& point) {
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    std::vector<int> indices(1);
    std::vector<float> sqr_distances(1);

    if (kdtree.nearestKSearch(point, 1, indices, sqr_distances) > 0) {
        return indices[0];
    }
    else {
        return 0; // 未找到
    }
}


// 输入：点云cloud，待计算法向量的点索引indices，搜索半径radius
// 输出：每个指定点的法向量（pcl::Normal）
std::vector<pcl::Normal> computeNormalsAtIndices(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::vector<int>& indices,
    float radius)
{
    // 创建法线估计对象
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    // 设置需要计算法线的点的索引
    pcl::PointIndices::Ptr point_indices(new pcl::PointIndices());
    point_indices->indices = indices;
    ne.setIndices(point_indices);

    // 创建kd树用于近邻搜索
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    // 设置搜索半径
    ne.setRadiusSearch(radius);

    // 输出法线
    pcl::PointCloud<pcl::Normal> normals;
    ne.compute(normals);

    // 将结果转换为vector返回
    std::vector<pcl::Normal> result(normals.points.begin(), normals.points.end());
    return result;
}

// 扩展点云曲线上的点
pcl::PointCloud<pcl::PointXYZ>::Ptr expandCurveAlongNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::vector<int> indices, float radius, float distance) {
    // 创建法线估计对象
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    // 设置输入点云和搜索树
    ne.setSearchSurface(cloud);
    ne.setSearchMethod(tree);
    // 设置搜索半径
    ne.setRadiusSearch(radius);
    ne.setInputCloud(cloud);
    // 设置待求点索引
    pcl::PointIndices::Ptr point_indices(new pcl::PointIndices());
    point_indices->indices = indices;
    ne.setIndices(point_indices);
    // 计算法线
    ne.compute(*cloud_normals);


    // 创建扩展后的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr expanded_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 遍历每个点并沿法线方向扩展
    for (size_t i = 0; i < point_indices->indices.size(); ++i) {
        pcl::PointXYZ original_point = cloud->points[i];
        pcl::Normal normal = cloud_normals->points[i];

        // 计算扩展后的点
        pcl::PointXYZ expanded_point;
        expanded_point.x = original_point.x + distance * normal.normal_x;
        expanded_point.y = original_point.y + distance * normal.normal_y;
        expanded_point.z = original_point.z + distance * normal.normal_z;

        // 添加扩展后的点到新的点云中
        expanded_cloud->points.push_back(expanded_point);
    }

    expanded_cloud->width = static_cast<uint32_t>(expanded_cloud->points.size());
    expanded_cloud->height = 1;

    return expanded_cloud;
}


int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliers(new pcl::PointCloud<pcl::PointXYZ>);

    // 读取点云文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("./data/dianji.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read the PCD file\n");
        return (-1);
    }

    std::cout << "Loaded "
        << cloud->width * cloud->height
        << " data points from the PCD file." << std::endl;

    //计算包围盒
    AABB aabb = computeAABB(cloud);

    // 创建 CropBox 滤波器对象
    pcl::CropBox<pcl::PointXYZ> cropBoxHalfFilter;
    cropBoxHalfFilter.setInputCloud(cloud);

    { ////点云对半分
    //// 定义立方体区域的边界
    //Eigen::Vector4f Point1;
    //Point1[0] = aabb.min_x; // X 最小值
    //Point1[1] = aabb.min_y; // Y 最小值
    //Point1[2] = aabb.min_z; // Z 最小值
    //Point1[3] = 1.0;

    //Eigen::Vector4f Point2;
    //Point2[0] = aabb.max_x; // X 最大值
    //Point2[1] = aabb.max_y; // Y 最大值
    //Point2[2] = 0; // Z 最大值
    //Point2[3] = 1.0;

    //cropBoxHalfFilter.setMin(Point1);
    //cropBoxHalfFilter.setMax(Point2);

    //// 存储筛选结果的点云
    //pcl::PointCloud<pcl::PointXYZ>::Ptr halfCloud(new pcl::PointCloud<pcl::PointXYZ>);
    //cropBoxHalfFilter.filter(*halfCloud);
    //pcl::io::savePCDFile("./data/dianji_half.pcd", *halfCloud);
    //// 输出筛选结果
    //std::cout << "筛选出的点云大小: " << halfCloud->points.size() << std::endl;
    }


        // 可视化
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("surface_2d"));
        viewer->setBackgroundColor(0, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

        if (1)
        {
            // 计算凹包(2d)
            pcl::ConcaveHull<pcl::PointXYZ> surface_3d;						// 2d凹包
            surface_3d.setInputCloud(cloud);
            surface_3d.setDimension(3);										// 凹包设置凹包是2d的还是3d的
            surface_3d.setAlpha(30);										// 值越小，生成的凹包越细分
            std::vector<pcl::Vertices> polygons_3d;							// 保存凹包顶点
            surface_3d.reconstruct(*cloud, polygons_3d);				    // 计算凹包
            pcl::Vertices vertices = polygons_3d[0];
            std::cout << "凹包围顶点数量: " << vertices.vertices.size() << std::endl;

            // 凹包可视化
            viewer->addPolygonMesh<pcl::PointXYZ>(cloud, polygons_3d, "polygons_3d_");
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "polygons_3d_");
            viewer->setRepresentationToWireframeForAllActors();
        }
        


    /*
    {   // 计算凹包(3d)
        pcl::ConcaveHull<pcl::PointXYZ> cloud_3d;						// 3d凹包
        cloud_3d.setInputCloud(cloud);
        cloud_3d.setDimension(3);										// 凹包设置凹包是2d的还是3d的
        cloud_3d.setAlpha(100);										// 值越小，生成的凹包越细分
        std::vector<pcl::Vertices> polygons_3d;							// 保存凹包顶点
        cloud_3d.reconstruct(*cloud, polygons_3d);				    // 计算凹包

        // 凹包可视化
        viewer->addPolygonMesh<pcl::PointXYZ>(cloud, polygons_3d, "polygons_3d");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "polygons_2d");
        viewer->setRepresentationToWireframeForAllActors();

        while (!viewer->wasStopped())
        {

            viewer->spinOnce(1000);
            boost::this_thread::sleep(boost::posix_time::microseconds(1000));

        }
    }
    */


#if 0
        float gap = 100;
        float longth = aabb.max_x - aabb.min_x;
        for (float i = 0;gap * i < longth; i++)
        {

            // 切分点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr cropppedPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
            cropPointCloud(pcl::PointXYZ(aabb.min_x + i * gap, aabb.min_y, aabb.min_z), pcl::PointXYZ(aabb.min_x + (i + 1) * gap, aabb.max_y, aabb.max_z), cloud, cropppedPointCloud);

            // 切片轮廓
            pcl::PointCloud<pcl::PointXYZ>::Ptr contour(new pcl::PointCloud<pcl::PointXYZ>);
            //平面参数
            float x_plane = aabb.min_x + i * gap + 0.5 * gap;
            std::vector<float> planeModel = { 1, 0, 0, -x_plane };
            slicePointCloud(cropppedPointCloud, planeModel, 1, contour);



            // 计算凹包(2d)
            pcl::ConcaveHull<pcl::PointXYZ> surface_2d;						// 2d凹包
            surface_2d.setInputCloud(contour);
            surface_2d.setDimension(2);										// 凹包设置凹包是2d的还是3d的
            surface_2d.setAlpha(200);										// 值越小，生成的凹包越细分
            std::vector<pcl::Vertices> polygons_2d;							// 保存凹包顶点
            surface_2d.reconstruct(*contour, polygons_2d);				    // 计算凹包
            pcl::Vertices vertices = polygons_2d[0];
            std::cout << "凹包围顶点数量: " << vertices.vertices.size() << std::endl;

            // 凹包可视化
            viewer->addPolygonMesh<pcl::PointXYZ>(contour, polygons_2d, "polygons_2d_" + std::to_string(i));
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "polygons_2d_" + std::to_string(i));
            viewer->setRepresentationToWireframeForAllActors();


            /*
            //计算凹包围顶点索引
            std::vector<int> indices;
            for (size_t i = 0; i < vertices.vertices.size(); i++)
            {
                int index = searchNearestPoint(cloud, contour->points[vertices.vertices[i]]);
                if (index)
                {
                    indices.push_back(index);
                }
            }

            // 扩展点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr expanded_cloud = expandCurveAlongNormals(cloud, indices, 10, 20);
            viewer->addPointCloud<pcl::PointXYZ>(expanded_cloud, "expanded_cloud_" + std::to_string(i));
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "expanded_cloud_" + std::to_string(i));
            */


        }

#endif // 0



    while (!viewer->wasStopped())
    {

        viewer->spinOnce(1000);
        boost::this_thread::sleep(boost::posix_time::microseconds(1000));

    }

    return 0;

}