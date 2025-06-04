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


/**
 * 使用统计方法去除离群点
 * @param cloud 输入点云
 * @param output 输出点云
 * @param mean_k 考虑的邻近点数(通常50-100)
 * @param stddev_mul_thresh 标准偏差乘数阈值(通常1.0-3.0)
 */
void removeStatisticalOutliers(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& output,
    int mean_k = 50,
    float stddev_mul_thresh = 1.0)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(mean_k);                     // 设置考虑邻近点数量
    sor.setStddevMulThresh(stddev_mul_thresh);// 设置标准偏差阈值
    sor.filter(*output);
}

/**
 * 使用体素网格方法降采样
 * @param cloud 输入点云
 * @param output 输出点云
 * @param leaf_size 体素大小(米)
 */
void voxelGridDownsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& output,
    float leaf_size = 0.01f)
{
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(leaf_size, leaf_size, leaf_size); // 设置体素大小
    vg.filter(*output);
}

// 计算平面与三角形的交点，并返回交点及其所在三角形的索引
std::vector<std::pair<Eigen::Vector3d, size_t>> planeTriangleIntersection(
    const Eigen::Vector3d& plane_point,
    const Eigen::Vector3d& plane_normal,
    const Eigen::Vector3d& v0,
    const Eigen::Vector3d& v1,
    const Eigen::Vector3d& v2,
    size_t triangle_index) {

    std::vector<std::pair<Eigen::Vector3d, size_t>> intersection_points;

    // 检查每个边与平面的交点
    auto checkEdge = [&](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
        Eigen::Vector3d ab = b - a;
        double denominator = plane_normal.dot(ab);

        // 如果边与平面平行，跳过
        if (std::abs(denominator) < 1e-6) return;

        double t = (plane_normal.dot(plane_point - a)) / denominator;

        // 如果交点在边的范围内
        if (t >= 0.0 && t <= 1.0) {
            intersection_points.emplace_back(a + t * ab, triangle_index);
        }
        };

    checkEdge(v0, v1);
    checkEdge(v1, v2);
    checkEdge(v2, v0);

    // 通常会有0、1或2个交点
    // 如果有2个交点，表示平面与三角形相交
    if (intersection_points.size() == 2) {
        return intersection_points;
    }

    return {};
}

// 计算三角面片的法向量
Eigen::Vector3d computeTriangleNormal(
    const Eigen::Vector3d& v0,
    const Eigen::Vector3d& v1,
    const Eigen::Vector3d& v2) {
    Eigen::Vector3d edge1 = v1 - v0;
    Eigen::Vector3d edge2 = v2 - v0;
    return edge1.cross(edge2).normalized();
}

// 计算网格的顶点法线
void computeVertexNormals(open3d::geometry::TriangleMesh& mesh) {
    // 初始化所有顶点法线为零向量
    mesh.vertex_normals_.resize(mesh.vertices_.size(), Eigen::Vector3d::Zero());

    // 遍历每个三角形，累加每个顶点的法向量
    for (const auto& triangle : mesh.triangles_) {
        // 计算三角形的法向量
        const Eigen::Vector3d& v0 = mesh.vertices_[triangle[0]];
        const Eigen::Vector3d& v1 = mesh.vertices_[triangle[1]];
        const Eigen::Vector3d& v2 = mesh.vertices_[triangle[2]];
        Eigen::Vector3d normal = computeTriangleNormal(v0, v1, v2);

        // 将三角形法向量累加到每个顶点的法向量上
        mesh.vertex_normals_[triangle[0]] += normal;
        mesh.vertex_normals_[triangle[1]] += normal;
        mesh.vertex_normals_[triangle[2]] += normal;
    }

    // 归一化所有顶点法线
    for (auto& normal : mesh.vertex_normals_) {
        if (normal.norm() > 1e-6) {
            normal.normalize();
        }
        else {
            // 处理可能的零法线（例如，退化的三角形）
            normal = Eigen::Vector3d(0.0, 0.0, 1.0);
        }
    }
}

Eigen::Vector3d computeNormalVector(
    const Eigen::Vector3d& n1,
    const Eigen::Vector3d& n2,
    double alpha) {

    // 确保 n1 和 n2 是单位向量
    Eigen::Vector3d normalized_n1 = n1.normalized();
    Eigen::Vector3d normalized_n2 = n2.normalized();

    // 检查 n1 和 n2 是否垂直
    if (std::abs(normalized_n1.dot(normalized_n2)) > 1e-6) {
        throw std::invalid_argument("n1 and n2 must be perpendicular.");
    }

    // 选择一个与 n1 不共线的向量
    Eigen::Vector3d v(1.0, 0.0, 0.0);

    // 如果 v 与 n1 太接近，则选择另一个向量
    if (std::abs(v.dot(normalized_n1)) > 0.9) {
        v = Eigen::Vector3d(0.0, 1.0, 0.0);
    }

    // 计算垂直于 n1 的向量 u1
    Eigen::Vector3d proj = normalized_n1.dot(v) * normalized_n1;
    Eigen::Vector3d v_perp = v - proj;
    Eigen::Vector3d u1 = v_perp.normalized();

    // 计算 u2 = n1 × u1
    Eigen::Vector3d u2 = normalized_n1.cross(u1);

    // 计算 n2 在 u1 和 u2 方向上的投影
    double a = normalized_n2.dot(u1);
    double b = normalized_n2.dot(u2);

    // 求解方程 a*cos(θ) + b*sin(θ) = cos(alpha)
    double cos_alpha = std::cos(alpha);
    double theta;

    if (std::abs(a) < 1e-6) {
        // 特殊情况：a = 0
        theta = std::asin(cos_alpha / b);
    }
    else if (std::abs(b) < 1e-6) {
        // 特殊情况：b = 0
        theta = std::acos(cos_alpha / a);
    }
    else {
        // 一般情况：使用万能公式
        double k = (cos_alpha - a) / b;
        theta = 2.0 * std::atan(k);
    }

    // 构造 n3
    Eigen::Vector3d n3 = u1 * std::cos(theta) + u2 * std::sin(theta);

    return n3.normalized();
}


// 只保留网格的最外层三角面片
std::shared_ptr<open3d::geometry::TriangleMesh> keepOuterShell(
    const open3d::geometry::TriangleMesh& mesh) {

    if (mesh.triangles_.empty()) {
        return std::make_shared<open3d::geometry::TriangleMesh>();
    }

    // 构建邻接关系：面 -> 相邻面
    std::vector<std::vector<size_t>> face_adjacency(mesh.triangles_.size());

    // 构建边到面的映射
    std::map<std::pair<size_t, size_t>, std::vector<size_t>> edge_to_faces;

    for (size_t i = 0; i < mesh.triangles_.size(); ++i) {
        const auto& triangle = mesh.triangles_[i];

        // 三角形的三条边
        std::vector<std::pair<size_t, size_t>> edges = {
            {triangle[0], triangle[1]},
            {triangle[1], triangle[2]},
            {triangle[2], triangle[0]}
        };

        // 规范化边（小顶点ID在前）
        for (auto& edge : edges) {
            if (edge.first > edge.second) {
                std::swap(edge.first, edge.second);
            }
            edge_to_faces[edge].push_back(i);
        }
    }

    // 构建邻接表
    for (const auto& edge_faces : edge_to_faces) {
        const auto& faces = edge_faces.second;
        if (faces.size() == 2) { // 内部边连接两个面
            face_adjacency[faces[0]].push_back(faces[1]);
            face_adjacency[faces[1]].push_back(faces[0]);
        }
    }

    // 找出所有边界边
    std::set<std::pair<size_t, size_t>> boundary_edges;
    for (const auto& edge_faces : edge_to_faces) {
        if (edge_faces.second.size() == 1) { // 边界边只属于一个面
            boundary_edges.insert(edge_faces.first);
        }
    }

    // 找出所有包含边界边的面（可能的外层面片）
    std::vector<size_t> boundary_faces;
    for (size_t i = 0; i < mesh.triangles_.size(); ++i) {
        const auto& triangle = mesh.triangles_[i];

        // 三角形的三条边
        std::vector<std::pair<size_t, size_t>> edges = {
            {triangle[0], triangle[1]},
            {triangle[1], triangle[2]},
            {triangle[2], triangle[0]}
        };

        // 规范化边
        for (auto& edge : edges) {
            if (edge.first > edge.second) {
                std::swap(edge.first, edge.second);
            }
        }

        // 如果三角形包含任何边界边，则它是边界面
        for (const auto& edge : edges) {
            if (boundary_edges.count(edge) > 0) {
                boundary_faces.push_back(i);
                break;
            }
        }
    }

    // 如果没有找到边界面，返回原始网格
    if (boundary_faces.empty()) {
        return std::make_shared<open3d::geometry::TriangleMesh>(mesh);
    }

    // 使用BFS找出与边界面相连的最大连通分量
    std::vector<bool> visited(mesh.triangles_.size(), false);
    std::vector<size_t> largest_component;
    size_t max_size = 0;

    // 对每个边界面进行BFS
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

        // 更新最大连通分量
        if (component.size() > max_size) {
            max_size = component.size();
            largest_component = component;
        }
    }

    // 创建只包含最外层面片的新网格
    auto result_mesh = std::make_shared<open3d::geometry::TriangleMesh>();
    result_mesh->vertices_ = mesh.vertices_;

    // 复制选中的三角面片
    result_mesh->triangles_.reserve(largest_component.size());
    for (size_t idx : largest_component) {
        result_mesh->triangles_.push_back(mesh.triangles_[idx]);
    }

    // 计算新的法线
    result_mesh->ComputeVertexNormals();

    return result_mesh;
}
