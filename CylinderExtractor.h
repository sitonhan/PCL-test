#ifndef CYLINDER_EXTRACTOR_H
#define CYLINDER_EXTRACTOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <memory>

class CylinderExtractor
{
public:
    // 构造函数
    CylinderExtractor();

    // 设置参数
    void setNormalEstimationKSearch(int k);
    void setMaxIterations(int max_iter);
    void setDistanceThreshold(float threshold);
    void setRadiusLimits(float min_radius, float max_radius);
    void setNormalDistanceWeight(float weight);
    void setMinSupportNum(int min_num);

    // 主处理函数
    bool extract(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& cylinder_cloud,
        pcl::ModelCoefficients::Ptr& coefficients);

    // 可视化函数
    void visualize(const pcl::PointCloud<pcl::PointXYZ>::Ptr& original_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cylinder_cloud,
        const pcl::ModelCoefficients::Ptr& coefficients);

    // 获取内点索引
    pcl::PointIndices::Ptr getInliers() const { return inliers_cylinder_; }

    // 获取处理状态信息
    std::string getStatusMessage() const { return status_message_; }

private:
    // 参数
    int normal_k_;
    int max_iterations_;
    float distance_threshold_;
    float min_radius_;
    float max_radius_;
    float normal_distance_weight_;
    int min_support_num_;

    // 结果
    pcl::PointIndices::Ptr inliers_cylinder_;
    std::string status_message_;

    // 可视化器
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;

    // 内部方法
    bool computeNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        pcl::PointCloud<pcl::Normal>::Ptr& normals);

    // 可视化辅助方法
    void addCoordinateSystem();
    void addCylinderModel(const pcl::ModelCoefficients::Ptr& coefficients);
};

#endif // CYLINDER_EXTRACTOR_H