#pragma once


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
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <open3d/Open3D.h>
#include <Eigen/Dense>
#include <vector>
#include <queue>
#include <set>
#include <cmath>
#include <stdexcept>

// ����һ���ṹ�����洢��Χ�е�AABB����Ϣ
struct AABB {
    float min_x, max_x;
    float min_y, max_y;
    float min_z, max_z;
};

// ������Ƶ�AABB
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

// ����㵽ƽ��ľ���
float computeDistanceToPlane(const pcl::PointXYZ point, const std::vector<float>& planeModel) {
    float numerator = std::abs(planeModel[0] * point.x + planeModel[1] * point.y + planeModel[2] * point.z + planeModel[3]);
    float denominator = std::sqrt(planeModel[0] * planeModel[0] + planeModel[1] * planeModel[1] + planeModel[2] * planeModel[2]);
    float distance = numerator / denominator;
    return distance;
}

// �Ե��ƽ����п�ָ�
int cropPointCloud(const pcl::PointXYZ point1, const pcl::PointXYZ point2,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudCropped)
{
    // ���� CropBox �˲�������
    pcl::CropBox<pcl::PointXYZ> cropBoxFilter;
    cropBoxFilter.setInputCloud(cloud/*halfCloud*/);
    // ��������������ı߽�
    Eigen::Vector4f minPoint;
    minPoint[0] = point1.x; // X ��Сֵ
    minPoint[1] = point1.y; // Y ��Сֵ
    minPoint[2] = point1.z; // Z ��Сֵ
    minPoint[3] = 1.0;

    Eigen::Vector4f maxPoint;
    maxPoint[0] = point2.x; // X ���ֵ
    maxPoint[1] = point2.y; // Y ���ֵ
    maxPoint[2] = point2.z; // Z ���ֵ
    maxPoint[3] = 1.0;

    cropBoxFilter.setMin(minPoint);
    cropBoxFilter.setMax(maxPoint);

    // �洢ɸѡ����ĵ���
    cropBoxFilter.filter(*cloudCropped);
    pcl::io::savePCDFile("./data/dianji_filtered.pcd", *cloudCropped);
    // ���ɸѡ���
    std::cout << "ɸѡ���ĵ��ƴ�С: " << cloudCropped->points.size() << std::endl;

    return 0;
}

// ������Ƭ
int slicePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::vector<float> planeModel,
    float gap,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudSliced)
{
    //����㵽ƽ����룬С��gap���ж�Ϊƽ���ȡ�ĵ�����
    //�����������ƣ�ѹ����һ��ƽ����
    for (const auto& point : cloud->points)
    {
        float distance = computeDistanceToPlane(point, planeModel);
        if (distance < gap)
        {
            cloudSliced->push_back(point);
        }
    }
    // ���ɸѡ���
    std::cout << "�����ĵ��ƴ�С: " << cloudSliced->points.size() << std::endl;
    pcl::io::savePCDFile("./data/dianji_contour.pcd", *cloudSliced);

    return 0;
}

// ������������ָ�����������������
int searchNearestPoint(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const pcl::PointXYZ& point) {
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);

    std::vector<int> indices(1);
    std::vector<float> sqr_distances(1);

    if (kdtree.nearestKSearch(point, 1, indices, sqr_distances) > 0) {
        return indices[0];
    }
    else {
        return 0; // δ�ҵ�
    }
}


// ���룺����cloud�������㷨�����ĵ�����indices�������뾶radius
// �����ÿ��ָ����ķ�������pcl::Normal��
std::vector<pcl::Normal> computeNormalsAtIndices(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::vector<int>& indices,
    float radius)
{
    // �������߹��ƶ���
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    // ������Ҫ���㷨�ߵĵ������
    pcl::PointIndices::Ptr point_indices(new pcl::PointIndices());
    point_indices->indices = indices;
    ne.setIndices(point_indices);

    // ����kd�����ڽ�������
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    // ���������뾶
    ne.setRadiusSearch(radius);

    // �������
    pcl::PointCloud<pcl::Normal> normals;
    ne.compute(normals);

    // �����ת��Ϊvector����
    std::vector<pcl::Normal> result(normals.points.begin(), normals.points.end());
    return result;
}

// ��չ���������ϵĵ�
pcl::PointCloud<pcl::PointXYZ>::Ptr expandCurveAlongNormals(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const std::vector<int> indices, float radius, float distance) {
    // �������߹��ƶ���
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    // ����������ƺ�������
    ne.setSearchSurface(cloud);
    ne.setSearchMethod(tree);
    // ���������뾶
    ne.setRadiusSearch(radius);
    ne.setInputCloud(cloud);
    // ���ô��������
    pcl::PointIndices::Ptr point_indices(new pcl::PointIndices());
    point_indices->indices = indices;
    ne.setIndices(point_indices);
    // ���㷨��
    ne.compute(*cloud_normals);


    // ������չ��ĵ���
    pcl::PointCloud<pcl::PointXYZ>::Ptr expanded_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // ����ÿ���㲢�ط��߷�����չ
    for (size_t i = 0; i < point_indices->indices.size(); ++i) {
        pcl::PointXYZ original_point = cloud->points[i];
        pcl::Normal normal = cloud_normals->points[i];

        // ������չ��ĵ�
        pcl::PointXYZ expanded_point;
        expanded_point.x = original_point.x + distance * normal.normal_x;
        expanded_point.y = original_point.y + distance * normal.normal_y;
        expanded_point.z = original_point.z + distance * normal.normal_z;

        // �����չ��ĵ㵽�µĵ�����
        expanded_cloud->points.push_back(expanded_point);
    }

    expanded_cloud->width = static_cast<uint32_t>(expanded_cloud->points.size());
    expanded_cloud->height = 1;

    return expanded_cloud;
}


/**
 * ʹ��ͳ�Ʒ���ȥ����Ⱥ��
 * @param cloud �������
 * @param output �������
 * @param mean_k ���ǵ��ڽ�����(ͨ��50-100)
 * @param stddev_mul_thresh ��׼ƫ�������ֵ(ͨ��1.0-3.0)
 */
void removeStatisticalOutliers(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& output,
    int mean_k = 50,
    float stddev_mul_thresh = 1.0)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(mean_k);                     // ���ÿ����ڽ�������
    sor.setStddevMulThresh(stddev_mul_thresh);// ���ñ�׼ƫ����ֵ
    sor.filter(*output);
}

/**
 * ʹ���������񷽷�������
 * @param cloud �������
 * @param output �������
 * @param leaf_size ���ش�С(��)
 */
void voxelGridDownsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& output,
    float leaf_size = 0.01f)
{
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(leaf_size, leaf_size, leaf_size); // �������ش�С
    vg.filter(*output);
}

// ����ƽ���������εĽ��㣬�����ؽ��㼰�����������ε�����
std::vector<std::pair<Eigen::Vector3d, size_t>> planeTriangleIntersection(
    const Eigen::Vector3d& plane_point,
    const Eigen::Vector3d& plane_normal,
    const Eigen::Vector3d& v0,
    const Eigen::Vector3d& v1,
    const Eigen::Vector3d& v2,
    size_t triangle_index) {

    std::vector<std::pair<Eigen::Vector3d, size_t>> intersection_points;

    // ���ÿ������ƽ��Ľ���
    auto checkEdge = [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        Eigen::Vector3d ab = b - a;
        double denominator = plane_normal.dot(ab);

        // �������ƽ��ƽ�У�����
        if (std::abs(denominator) < 1e-6) return;

        double t = (plane_normal.dot(plane_point - a)) / denominator;

        // ��������ڱߵķ�Χ��
        if (t >= 0.0 && t <= 1.0) {
            intersection_points.emplace_back(a + t * ab, triangle_index);
        }
        };

    checkEdge(v0, v1);
    checkEdge(v1, v2);
    checkEdge(v2, v0);

    // ͨ������0��1��2������
    // �����2�����㣬��ʾƽ�����������ཻ
    if (intersection_points.size() == 2) {
        return intersection_points;
    }

    return {};
}

// ����������Ƭ�ķ�����
Eigen::Vector3d computeTriangleNormal(
    const Eigen::Vector3d& v0,
    const Eigen::Vector3d& v1,
    const Eigen::Vector3d& v2) {
    Eigen::Vector3d edge1 = v1 - v0;
    Eigen::Vector3d edge2 = v2 - v0;
    return edge1.cross(edge2).normalized();
}

// ��������Ķ��㷨��
void computeVertexNormals(open3d::geometry::TriangleMesh& mesh) {
    // ��ʼ�����ж��㷨��Ϊ������
    mesh.vertex_normals_.resize(mesh.vertices_.size(), Eigen::Vector3d::Zero());

    // ����ÿ�������Σ��ۼ�ÿ������ķ�����
    for (const auto& triangle : mesh.triangles_) {
        // ���������εķ�����
        const Eigen::Vector3d& v0 = mesh.vertices_[triangle[0]];
        const Eigen::Vector3d& v1 = mesh.vertices_[triangle[1]];
        const Eigen::Vector3d& v2 = mesh.vertices_[triangle[2]];
        Eigen::Vector3d normal = computeTriangleNormal(v0, v1, v2);

        // �������η������ۼӵ�ÿ������ķ�������
        mesh.vertex_normals_[triangle[0]] += normal;
        mesh.vertex_normals_[triangle[1]] += normal;
        mesh.vertex_normals_[triangle[2]] += normal;
    }

    // ��һ�����ж��㷨��
    for (auto& normal : mesh.vertex_normals_) {
        if (normal.norm() > 1e-6) {
            normal.normalize();
        }
        else {
            // ������ܵ��㷨�ߣ����磬�˻��������Σ�
            normal = Eigen::Vector3d(0.0, 0.0, 1.0);
        }
    }
}

Eigen::Vector3d computeNormalVector(
    const Eigen::Vector3d& n1,
    const Eigen::Vector3d& n2,
    double alpha) {

    // ȷ�� n1 �� n2 �ǵ�λ����
    Eigen::Vector3d normalized_n1 = n1.normalized();
    Eigen::Vector3d normalized_n2 = n2.normalized();

    // ��� n1 �� n2 �Ƿ�ֱ
    if (std::abs(normalized_n1.dot(normalized_n2)) > 1e-6) {
        throw std::invalid_argument("n1 and n2 must be perpendicular.");
    }

    // ѡ��һ���� n1 �����ߵ�����
    Eigen::Vector3d v(1.0, 0.0, 0.0);

    // ��� v �� n1 ̫�ӽ�����ѡ����һ������
    if (std::abs(v.dot(normalized_n1)) > 0.9) {
        v = Eigen::Vector3d(0.0, 1.0, 0.0);
    }

    // ���㴹ֱ�� n1 ������ u1
    Eigen::Vector3d proj = normalized_n1.dot(v) * normalized_n1;
    Eigen::Vector3d v_perp = v - proj;
    Eigen::Vector3d u1 = v_perp.normalized();

    // ���� u2 = n1 �� u1
    Eigen::Vector3d u2 = normalized_n1.cross(u1);

    // ���� n2 �� u1 �� u2 �����ϵ�ͶӰ
    double a = normalized_n2.dot(u1);
    double b = normalized_n2.dot(u2);

    // ��ⷽ�� a*cos(��) + b*sin(��) = cos(alpha)
    double cos_alpha = std::cos(alpha);
    double theta;

    if (std::abs(a) < 1e-6) {
        // ���������a = 0
        theta = std::asin(cos_alpha / b);
    }
    else if (std::abs(b) < 1e-6) {
        // ���������b = 0
        theta = std::acos(cos_alpha / a);
    }
    else {
        // һ�������ʹ�����ܹ�ʽ
        double k = (cos_alpha - a) / b;
        theta = 2.0 * std::atan(k);
    }

    // ���� n3
    Eigen::Vector3d n3 = u1 * std::cos(theta) + u2 * std::sin(theta);

    return n3.normalized();
}


// ֻ��������������������Ƭ
std::shared_ptr<open3d::geometry::TriangleMesh> keepOuterShell(
    const open3d::geometry::TriangleMesh& mesh) {

    if (mesh.triangles_.empty()) {
        return std::make_shared<open3d::geometry::TriangleMesh>();
    }

    // �����ڽӹ�ϵ���� -> ������
    std::vector<std::vector<size_t>> face_adjacency(mesh.triangles_.size());

    // �����ߵ����ӳ��
    std::map<std::pair<size_t, size_t>, std::vector<size_t>> edge_to_faces;

    for (size_t i = 0; i < mesh.triangles_.size(); ++i) {
        const auto& triangle = mesh.triangles_[i];

        // �����ε�������
        std::vector<std::pair<size_t, size_t>> edges = {
            {triangle[0], triangle[1]},
            {triangle[1], triangle[2]},
            {triangle[2], triangle[0]}
        };

        // �淶���ߣ�С����ID��ǰ��
        for (auto& edge : edges) {
            if (edge.first > edge.second) {
                std::swap(edge.first, edge.second);
            }
            edge_to_faces[edge].push_back(i);
        }
    }

    // �����ڽӱ�
    for (const auto& edge_faces : edge_to_faces) {
        const auto& faces = edge_faces.second;
        if (faces.size() == 2) { // �ڲ�������������
            face_adjacency[faces[0]].push_back(faces[1]);
            face_adjacency[faces[1]].push_back(faces[0]);
        }
    }

    // �ҳ����б߽��
    std::set<std::pair<size_t, size_t>> boundary_edges;
    for (const auto& edge_faces : edge_to_faces) {
        if (edge_faces.second.size() == 1) { // �߽��ֻ����һ����
            boundary_edges.insert(edge_faces.first);
        }
    }

    // �ҳ����а����߽�ߵ��棨���ܵ������Ƭ��
    std::vector<size_t> boundary_faces;
    for (size_t i = 0; i < mesh.triangles_.size(); ++i) {
        const auto& triangle = mesh.triangles_[i];

        // �����ε�������
        std::vector<std::pair<size_t, size_t>> edges = {
            {triangle[0], triangle[1]},
            {triangle[1], triangle[2]},
            {triangle[2], triangle[0]}
        };

        // �淶����
        for (auto& edge : edges) {
            if (edge.first > edge.second) {
                std::swap(edge.first, edge.second);
            }
        }

        // ��������ΰ����κα߽�ߣ������Ǳ߽���
        for (const auto& edge : edges) {
            if (boundary_edges.count(edge) > 0) {
                boundary_faces.push_back(i);
                break;
            }
        }
    }

    // ���û���ҵ��߽��棬����ԭʼ����
    if (boundary_faces.empty()) {
        return std::make_shared<open3d::geometry::TriangleMesh>(mesh);
    }

    // ʹ��BFS�ҳ���߽��������������ͨ����
    std::vector<bool> visited(mesh.triangles_.size(), false);
    std::vector<size_t> largest_component;
    size_t max_size = 0;

    // ��ÿ���߽������BFS
    for (size_t seed : boundary_faces) {
        if (visited[seed]) continue;

        std::queue<size_t> q;
        std::vector<size_t> component;

        q.push(seed);
        visited[seed] = true;
        component.push_back(seed);

        while (!q.empty()) {
            size_t current = q.front();
            q.pop();

            for (size_t neighbor : face_adjacency[current]) {
                if (!visited[neighbor]) {
                    visited[neighbor] = true;
                    component.push_back(neighbor);
                    q.push(neighbor);
                }
            }
        }

        // ���������ͨ����
        if (component.size() > max_size) {
            max_size = component.size();
            largest_component = component;
        }
    }

    // ����ֻ�����������Ƭ��������
    auto result_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    result_mesh->vertices_ = mesh.vertices_;

    // ����ѡ�е�������Ƭ
    result_mesh->triangles_.reserve(largest_component.size());
    for (size_t idx : largest_component) {
        result_mesh->triangles_.push_back(mesh.triangles_[idx]);
    }

    // �����µķ���
    result_mesh->ComputeVertexNormals();

    return result_mesh;
}
