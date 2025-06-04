#include "CylinderExtractor.h"
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/colors.h>

CylinderExtractor::CylinderExtractor() :
    normal_k_(50),
    max_iterations_(10000),
    distance_threshold_(0.05f),
    min_radius_(0.0f),
    max_radius_(0.1f),
    normal_distance_weight_(0.1f)
{
    inliers_cylinder_.reset(new pcl::PointIndices);
}

void CylinderExtractor::setNormalEstimationKSearch(int k)
{
    if (k > 0) {
        normal_k_ = k;
    }
}

void CylinderExtractor::setMaxIterations(int max_iter)
{
    if (max_iter > 0) {
        max_iterations_ = max_iter;
    }
}

void CylinderExtractor::setDistanceThreshold(float threshold)
{
    if (threshold > 0) {
        distance_threshold_ = threshold;
    }
}

void CylinderExtractor::setRadiusLimits(float min_radius, float max_radius)
{
    if (min_radius >= 0 && max_radius > min_radius) {
        min_radius_ = min_radius;
        max_radius_ = max_radius;
    }
}

void CylinderExtractor::setNormalDistanceWeight(float weight)
{
    if (weight >= 0 && weight <= 1) {
        normal_distance_weight_ = weight;
    }
}

void CylinderExtractor::setMinSupportNum(int min_num)
{
    min_support_num_ = min_num;
}

bool CylinderExtractor::computeNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    pcl::PointCloud<pcl::Normal>::Ptr& normals)
{
    if (cloud->empty()) {
        status_message_ = "�������Ϊ��!";
        return false;
    }

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setKSearch(normal_k_);
    ne.compute(*normals);

    if (normals->empty()) {
        status_message_ = "���߼���ʧ��!";
        return false;
    }

    return true;
}

bool CylinderExtractor::extract(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cylinder_cloud,
    pcl::ModelCoefficients::Ptr& coefficients)
{
    // ����״̬
    status_message_.clear();
    inliers_cylinder_->indices.clear();
    coefficients.reset(new pcl::ModelCoefficients);

    // �������
    if (!input_cloud || input_cloud->empty()) {
        status_message_ = "���������Ч��Ϊ��!";
        return false;
    }

    // ���㷨��
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    if (!computeNormals(input_cloud, normals)) {
        return false;
    }

    // �����ָ����
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(normal_distance_weight_);
    seg.setMaxIterations(max_iterations_);
    seg.setDistanceThreshold(distance_threshold_);
    seg.setRadiusLimits(min_radius_, max_radius_);
    seg.setInputCloud(input_cloud);
    seg.setInputNormals(normals);

    // ִ�зָ�
    seg.segment(*inliers_cylinder_, *coefficients);

    if (inliers_cylinder_->indices.empty()) {
        status_message_ = "δ�ܼ�⵽Բ������!";
        return false;
    }

    // ��ȡԲ������
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(input_cloud);
    extract.setIndices(inliers_cylinder_);
    extract.setNegative(false);
    extract.filter(*cylinder_cloud);

    if (cylinder_cloud->empty()) {
        status_message_ = "��ȡ��Բ������Ϊ��!";
        return false;
    }

    status_message_ = "Բ��������ȡ�ɹ�!";
    return true;
}

void CylinderExtractor::visualize(const pcl::PointCloud<pcl::PointXYZ>::Ptr& original_cloud,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cylinder_cloud,
    const pcl::ModelCoefficients::Ptr& coefficients)
{
    // �������ӻ���
    viewer_.reset(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer_->setBackgroundColor(0.1, 0.1, 0.1);

    // �������ϵ
    addCoordinateSystem();

    // ��ʾԭʼ����(��ɫ)
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> original_color(original_cloud, 150, 150, 150);
    viewer_->addPointCloud<pcl::PointXYZ>(original_cloud, original_color, "original_cloud");
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original_cloud");

    // ��ʾԲ������(��ɫ)
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cylinder_color(cylinder_cloud, 255, 0, 0);
    viewer_->addPointCloud<pcl::PointXYZ>(cylinder_cloud, cylinder_color, "cylinder_cloud");
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cylinder_cloud");

    // �����ϵ�Բ��ģ��(��ɫ)
    addCylinderModel(coefficients);

    // ���ͼ��
    viewer_->addText("Red: Extracted cylinder points", 10, 70, 20, 1.0, 1.0, 1.0, "legend1");
    viewer_->addText("Green: Fitted cylinder model", 10, 40, 20, 1.0, 1.0, 1.0, "legend2");
    viewer_->addText("Gray: Original point cloud", 10, 10, 20, 1.0, 1.0, 1.0, "legend3");

    // �������λ��
    viewer_->initCameraParameters();
    viewer_->resetCamera();

    // �������ӻ�ѭ��
    while (!viewer_->wasStopped()) {
        viewer_->spinOnce(100);
    }
}

void CylinderExtractor::addCoordinateSystem()
{
    // �������ϵ(1.0�׳���)
    viewer_->addCoordinateSystem(1.0, "global");
}

void CylinderExtractor::addCylinderModel(const pcl::ModelCoefficients::Ptr& coefficients)
{
    if (coefficients->values.size() != 7) return;

    // ��ȡԲ������
    const float& a = coefficients->values[0];
    const float& b = coefficients->values[1];
    const float& c = coefficients->values[2];
    const float& x0 = coefficients->values[3];
    const float& y0 = coefficients->values[4];
    const float& z0 = coefficients->values[5];
    const float& radius = coefficients->values[6];

    // ����Բ��ģ��
    pcl::ModelCoefficients cylinder_coeff;
    cylinder_coeff.values.resize(7);
    cylinder_coeff.values[0] = a;
    cylinder_coeff.values[1] = b;
    cylinder_coeff.values[2] = c;
    cylinder_coeff.values[3] = x0 * 100;
    cylinder_coeff.values[4] = y0 * 100;
    cylinder_coeff.values[5] = z0 * 100;
    cylinder_coeff.values[6] = radius;

    // ���Բ�������ӻ���(��ɫ)
    viewer_->addCylinder(cylinder_coeff, "fitted_cylinder");
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
        0.0, 1.0, 0.0, "fitted_cylinder");
    viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,
        0.5, "fitted_cylinder");
}