#include "function.hpp"
#include "CylinderExtractor.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>  // 用于保存为VTK格式
#include <pcl/io/obj_io.h>  // 用于保存为OBJ格式
#include <iostream>
#include <Eigen/Dense>
#include <open3d/Open3D.h>

#define PI 3.14159265358979323846


int main(int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inliers(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outliers(new pcl::PointCloud<pcl::PointXYZ>);


#if 0
    // 读取点云文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("./complete.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read the PCD file\n");
        return (-1);
    }

    std::cout << "Loaded "
        << cloud->width * cloud->height
        << " data points from the PCD file." << std::endl;
#endif // 0



#if 0
    //计算包围盒
    AABB aabb = computeAABB(cloud);

    // 创建 CropBox 滤波器对象
    pcl::CropBox<pcl::PointXYZ> cropBoxHalfFilter;
    cropBoxHalfFilter.setInputCloud(cloud);

    // 可视化
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("surface_2d"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
#endif // 0


#if 0
    //点云对半分
    // 定义立方体区域的边界
    Eigen::Vector4f Point1;
    Point1[0] = aabb.min_x; // X 最小值
    Point1[1] = aabb.min_y; // Y 最小值
    Point1[2] = aabb.min_z; // Z 最小值
    Point1[3] = 1.0;

    Eigen::Vector4f Point2;
    Point2[0] = aabb.max_x; // X 最大值
    Point2[1] = aabb.max_y; // Y 最大值
    Point2[2] = 0; // Z 最大值
    Point2[3] = 1.0;

    cropBoxHalfFilter.setMin(Point1);
    cropBoxHalfFilter.setMax(Point2);

    // 存储筛选结果的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr halfCloud(new pcl::PointCloud<pcl::PointXYZ>);
    cropBoxHalfFilter.filter(*halfCloud);
    pcl::io::savePCDFile("./data/dianji_half.pcd", *halfCloud);
    // 输出筛选结果
    std::cout << "筛选出的点云大小: " << halfCloud->points.size() << std::endl;
#endif // 0


#if 0
    // 去噪处理
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    removeStatisticalOutliers(cloud, cloud_filtered, 150, 1.0);
    std::cout << "去噪后点云点数: " << cloud_filtered->size() << std::endl;

    // 降采样处理
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    voxelGridDownsample(cloud_filtered, cloud_downsampled, 3.2f);
    std::cout << "降采样后点云点数: " << cloud_downsampled->size() << std::endl;

    // 保存法向量到PCD文件

    std::string output_cloud_downsampled = "cloud_downsampled.pcd";
    if (pcl::io::savePLYFile(output_cloud_downsampled, *cloud_downsampled) == -1) {
        PCL_ERROR("Failed to save the output PCD file!\n");
        return -1;
    }
    std::cout << "Saved point cloud to " << output_cloud_downsampled << std::endl;
#endif // 0




#if 0
    // 创建法线估计对象
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    // 创建搜索树
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    // 设置搜索半径
    ne.setRadiusSearch(30);  // 3cm的搜索半径，可根据点云密度调整

    // 计算法线
    std::cout << "Computing normals..." << std::endl;
    ne.compute(*normals);

    // 检查法向量是否成功计算
    if (normals->size() == 0) {
        std::cerr << "Failed to compute normals!" << std::endl;
        return -1;
    }

    std::cout << "Normals computed for " << normals->size() << " points." << std::endl;

    // 保存法向量到PLY文件
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    std::string output_file = "cloud_with_normals.ply";
    if (pcl::io::savePLYFile(output_file, *cloud_with_normals) == -1) {
        PCL_ERROR("Failed to save the output PLY file!\n");
        return -1;
    }

    std::cout << "Saved point cloud with normals to " << output_file << std::endl;

#endif // 0

   

#if 0
    // 可视化
    pcl::visualization::PCLVisualizer viewer("Point Cloud Processing");

    // 原始点云(灰色)
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> original_color(cloud, 150, 150, 150);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, original_color, "original");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original");

    // 处理后点云(红色)
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> processed_color(cloud_downsampled, 255, 0, 0);
    viewer.addPointCloud<pcl::PointXYZ>(cloud_downsampled, processed_color, "processed");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "processed");



    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }

#endif // 1


#if 0
        // ransac提取圆柱特征
        // 创建圆柱提取器并设置参数
        CylinderExtractor extractor;
		extractor.setNormalEstimationKSearch(150);   // 设置法线估计的k近邻数
		extractor.setNormalDistanceWeight(0.1);     // 设置法线距离权重
        extractor.setMaxIterations(20000);          // 增加迭代次数
        extractor.setDistanceThreshold(10);       // 更小的距离阈值
        extractor.setRadiusLimits(100, 200);       // 设置半径范围

        // 执行提取
        pcl::PointCloud<pcl::PointXYZ>::Ptr cylinder_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        if (extractor.extract(cloud, cylinder_cloud, coefficients)) {
            // 输出结果
            std::cout << extractor.getStatusMessage() << std::endl;
            std::cout << "圆柱参数:" << std::endl;
            std::cout << "轴线方向: [" << coefficients->values[3] << ", "
                << coefficients->values[4] << ", "
                << coefficients->values[5] << "]" << std::endl;
            std::cout << "轴线上点: [" << coefficients->values[0] << ", "
                << coefficients->values[1] << ", "
                << coefficients->values[2] << "]" << std::endl;
            std::cout << "圆柱半径: " << coefficients->values[6] << std::endl;
            std::cout << "圆柱点数: " << cylinder_cloud->size() << std::endl;

            // 保存结果
            pcl::io::savePCDFile("extracted_cylinder.pcd", *cylinder_cloud);
            std::cout << "圆柱点云已保存到 extracted_cylinder.pcd" << std::endl;
            
            // 可视化结果
            extractor.visualize(cloud, cylinder_cloud, coefficients);
        }
        else {
            std::cerr << "圆柱提取失败: " << extractor.getStatusMessage() << std::endl;
            return -1;
        }

#endif // 1



#if 0
        // 计算凹包(2d)
        pcl::ConcaveHull<pcl::PointXYZ> surface_3d;						// 2d凹包
        surface_3d.setInputCloud(cloud);
        surface_3d.setDimension(3);										// 凹包设置凹包是2d的还是3d的
        surface_3d.setAlpha(20);										// 值越小，生成的凹包越细分
        std::vector<pcl::Vertices> polygons_3d;							// 保存凹包顶点
        surface_3d.reconstruct(*cloud, polygons_3d);				    // 计算凹包
        pcl::Vertices vertices = polygons_3d[0];
        std::cout << "凹包围顶点数量: " << vertices.vertices.size() << std::endl;

        // 凹包可视化
        viewer->addPolygonMesh<pcl::PointXYZ>(cloud, polygons_3d, "polygons_3d_");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "polygons_3d_");
        viewer->setRepresentationToWireframeForAllActors();

#endif
        

        
#if 0
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
        // 读取点云文件
        if (pcl::io::loadPCDFile<pcl::PointXYZ>("./cloud_downsampled - Cloud.pcd", *cloud_downsampled) == -1) {
            PCL_ERROR("Couldn't read the PCD file\n");
            return (-1);
        }

        std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from the PCD file." << std::endl;

        // 计算凹包(3d)
        pcl::ConcaveHull<pcl::PointXYZ> cloud_3d_hull;						// 3d凹包
        cloud_3d_hull.setInputCloud(cloud_downsampled);
        cloud_3d_hull.setDimension(3);										// 凹包设置凹包是2d的还是3d的
        cloud_3d_hull.setAlpha(70);										// 值越小，生成的凹包越细分
        std::vector<pcl::Vertices> polygons_3d;							// 保存凹包顶点
        cloud_3d_hull.reconstruct(*cloud, polygons_3d);				    // 计算凹包

        // 获取三角化结果
        pcl::PolygonMesh mesh;
        cloud_3d_hull.reconstruct(mesh);

        std::cout << "Concave hull has " << mesh.polygons.size() << " polygons." << std::endl;

        // 保存结果
        std::string output_file = "hull_mesh.ply";
        std::string extension = output_file.substr(output_file.find_last_of(".") + 1);

        if (extension == "ply") {
            pcl::io::savePLYFile(output_file, mesh);
        }
        else if (extension == "obj") {
            pcl::io::saveOBJFile(output_file, mesh);
        }
        else if (extension == "vtk") {
            pcl::io::saveVTKFile(output_file, mesh);
        }
        else {
            std::cerr << "Unsupported file format. Saving as PLY instead." << std::endl;
            pcl::io::savePLYFile(output_file + ".ply", mesh);
        }

        std::cout << "Concave hull saved to " << output_file << std::endl;

        // mesh可视化
        pcl::visualization::PCLVisualizer viewer ("3D Viewer");
        viewer.addPolygonMesh(mesh, "polygons_3d");
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, "polygons_3d");
        viewer.setRepresentationToWireframeForAllActors();

        while (!viewer.wasStopped())
        {
            viewer.spinOnce(1000);
            boost::this_thread::sleep(boost::posix_time::microseconds(1000));
        }
#endif // 0

        // 提取到的圆柱特征参数
        Eigen::Vector3d axis_direction(0.924475, 0.22846, 0.305208);
        Eigen::Vector3d axis_point(100.871, 107.386, 2183.77);
        float radius = 130.975f;

    // Open3D 部分
    //  // 1. 读取PLY格式的mesh文件
        auto mesh_ptr = std::make_shared<open3d::geometry::TriangleMesh>();
        if (!open3d::io::ReadTriangleMesh("hull_mesh-smooth.ply", *mesh_ptr)) {
            std::cerr << "Failed to read mesh from " << argv[1] << std::endl;
            return 1;
        }

        std::cout << "Loaded mesh with " << mesh_ptr->vertices_.size() << " vertices and "
            << mesh_ptr->triangles_.size() << " triangles." << std::endl;

        // 2. 计算顶点法线
        if (!mesh_ptr->HasVertexNormals()) {
            std::cout << "Computing vertex normals..." << std::endl;
            computeVertexNormals(*mesh_ptr);
        }
        else {
            std::cout << "Mesh already has vertex normals." << std::endl;
        }

        // 2. 通过一个点和两个方向向量构造空间平面
        // 根据表面距离和半径计算角度
        float huchang = 70.0f;
        float angle = huchang / radius;
                
		// 计算初始方向向量构造基础平面
        // 搜索z方向最大的顶点
        // 初始化最大值和对应顶点
        double max_z = -std::numeric_limits<double>::infinity();
        Eigen::Vector3d max_vertex = mesh_ptr->vertices_[0];
        // 遍历所有顶点，找出Z坐标最大的顶点
        for (const auto& vertex : mesh_ptr->vertices_) {
            if (vertex.z() > max_z) {
                max_z = vertex.z();
                max_vertex = vertex;
            }
        }
		//计算通过此顶点且与axis_direction垂直的方向向量
        // 确保输入向量不为零向量
        if (axis_direction.norm() < 1e-6) {
            throw std::invalid_argument("Input vector is close to zero.");
        }
        // 规范化输入向量
        Eigen::Vector3d normalized_vector = axis_direction.normalized();

        // 选择一个与输入向量不平行的提示向量
        Eigen::Vector3d safe_hint = Eigen::Vector3d::UnitX();
        if (std::abs(safe_hint.dot(normalized_vector)) > 0.99) {
            // 如果提示向量与输入向量几乎平行，则选择另一个方向
            safe_hint = Eigen::Vector3d::UnitY();
            if (std::abs(safe_hint.dot(normalized_vector)) > 0.99) {
                safe_hint = Eigen::Vector3d::UnitZ();
            }
        }
        // 计算垂直于输入向量的方向向量
        Eigen::Vector3d perpendicular_direction = normalized_vector.cross(safe_hint);
        // 再次规范化结果
        perpendicular_direction.normalize();

        // 计算平面的法向量（两个方向向量的叉乘）
        Eigen::Vector3d plane_normal0 = axis_direction.cross(perpendicular_direction);
        plane_normal0.normalize();  // 确保法向量是单位向量


        std::vector<std::shared_ptr< open3d::geometry::Geometry>> visualizations_ptr = {mesh_ptr};

		// 计算与基准平面夹角为theta的平面法向量
        for (size_t i = 0; angle * (i + 0.5) < PI; i++)
        {
            float theta = angle * (i + 0.5); // 计算当前角度
            Eigen::Vector3d normal = computeNormalVector(axis_direction, plane_normal0, theta);


            // 3. 计算平面与mesh的交点
            std::vector<Eigen::Vector3d> intersection_points;
            std::vector<Eigen::Vector3d> intersection_normals;
            std::vector<std::pair<size_t, size_t>> intersection_segments;

            // 遍历所有三角形
            for (size_t i = 0; i < mesh_ptr->triangles_.size(); ++i) {
                const auto& triangle = mesh_ptr->triangles_[i];
                const Eigen::Vector3d& v0 = mesh_ptr->vertices_[triangle[0]];
                const Eigen::Vector3d& v1 = mesh_ptr->vertices_[triangle[1]];
                const Eigen::Vector3d& v2 = mesh_ptr->vertices_[triangle[2]];

                // 计算平面与当前三角形的交点，并获取三角形索引
                auto points_with_indices = planeTriangleIntersection(axis_point, normal, v0, v1, v2, i);

                
                if (points_with_indices.size() == 2) {
                    // 添加两个交点和它们的法向量
                    for (const auto& point_with_index : points_with_indices) {
                        const Eigen::Vector3d& point = point_with_index.first;
                        size_t triangle_idx = point_with_index.second;

                        // 计算交点的法向量（使用三角形顶点法线的加权平均）
                        const auto& triangle_vertices = mesh_ptr->triangles_[triangle_idx];
                        const Eigen::Vector3d& normal0 = mesh_ptr->vertex_normals_[triangle_vertices[0]];
                        const Eigen::Vector3d& normal1 = mesh_ptr->vertex_normals_[triangle_vertices[1]];
                        const Eigen::Vector3d& normal2 = mesh_ptr->vertex_normals_[triangle_vertices[2]];

                        // 简单平均三个顶点的法线作为交点的法线
                        Eigen::Vector3d normal = (normal0 + normal1 + normal2).normalized();

                        intersection_points.push_back(point);
                        intersection_normals.push_back(normal);
                    }
					
                    // 添加线段（每对交点形成一条线段）
                    size_t idx1 = intersection_points.size() - 2;
                    size_t idx2 = intersection_points.size() - 1;
                    intersection_segments.emplace_back(idx1, idx2);


                }
            }

            std::cout << "Found " << intersection_points.size() << " intersection points and "
                << intersection_segments.size() << " segments." << std::endl;

            // 4. 创建线段集用于可视化
            auto line_set_ptr = std::make_shared<open3d::geometry::LineSet>();

            // 设置点
            line_set_ptr->points_.resize(intersection_points.size());
            for (size_t i = 0; i < intersection_points.size(); ++i) {
                // 直接使用Eigen向量赋值，不需要构造函数
                line_set_ptr->points_[i] = intersection_points[i];
            }

            // 设置线段
            line_set_ptr->lines_.resize(intersection_segments.size());
            for (size_t i = 0; i < intersection_segments.size(); ++i) {
                // 直接使用Eigen向量赋值，不需要构造函数
                line_set_ptr->lines_[i] = Eigen::Vector2i(
                    intersection_segments[i].first,
                    intersection_segments[i].second
                );
            }

            // 设置线段颜色为红色
            line_set_ptr->colors_.resize(line_set_ptr->lines_.size());
            for (auto& color : line_set_ptr->colors_) {
                // 直接使用Eigen向量赋值，不需要构造函数
                color = Eigen::Vector3d(1.0, 0.0, 0.0);  // 红色
            }

            // 5. 创建法向量可视化
            auto normals_line_set_ptr = std::make_shared<open3d::geometry::LineSet>();

            // 法线的起点和终点
            std::vector<Eigen::Vector3d> normal_points;
            std::vector<Eigen::Vector2i> normal_lines;

            std::vector<Eigen::Vector3d> normals_neg;
            std::vector<Eigen::Vector3d> trace_points;

            // 法线长度比例
            double normal_length = 200;

            for (size_t i = 0; i < intersection_points.size(); ++i) {
                const Eigen::Vector3d& point = intersection_points[i];
                const Eigen::Vector3d& normal = intersection_normals[i];

                // 法线起点
                normal_points.push_back(point);
                // 法线终点
                normal_points.push_back(point + normal * normal_length);
                // 线段索引
                normal_lines.push_back(Eigen::Vector2i(i * 2, i * 2 + 1));

                // 法线反向
                normals_neg.push_back(-normal);
                // 路点=法线终点
                trace_points.push_back(point + normal * normal_length);
            }

            // 设置法向量线段集
            normals_line_set_ptr->points_ = normal_points;
            normals_line_set_ptr->lines_ = normal_lines;

            // 设置法向量颜色为蓝色
            normals_line_set_ptr->colors_.resize(normal_lines.size());
            for (auto& color : normals_line_set_ptr->colors_) {
                color = Eigen::Vector3d(0.0, 0.0, 1.0);  // 蓝色
            }
            // 6. 添加可视化
			visualizations_ptr.push_back(line_set_ptr);
			visualizations_ptr.push_back(normals_line_set_ptr);
            auto point_cloud_normal_ptr = std::make_shared<open3d::geometry::PointCloud>();
            point_cloud_normal_ptr->points_ = trace_points;
			point_cloud_normal_ptr->normals_ = normals_neg;
            
            // 7. 保存结果
            open3d::io::WriteLineSet("./result/intersection_result" + std::to_string(i) + ".ply", *line_set_ptr);
            open3d::io::WriteLineSet("./result/intersection_normals" + std::to_string(i) + ".ply", *normals_line_set_ptr);
            open3d::io::WritePointCloudToPCD("./result/intersection_cloud_normal" + std::to_string(i) + ".pcd", *point_cloud_normal_ptr, "auto");
            std::cout << "Intersection result saved to intersection_result.ply" << std::endl;
            std::cout << "Intersection normals saved to intersection_normals.ply" << std::endl;
        }

        std::vector<std::shared_ptr<const open3d::geometry::Geometry>> const_visualizations_ptr;
        for (const auto& geom : visualizations_ptr) {
            const_visualizations_ptr.push_back(std::const_pointer_cast<const open3d::geometry::Geometry>(geom));
        }
        open3d::visualization::DrawGeometries(const_visualizations_ptr, "Mesh-Plane Intersection with Normals");



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


#if 0
        while (!viewer->wasStopped())
        {

            viewer->spinOnce(1000);
            boost::this_thread::sleep(boost::posix_time::microseconds(1000));

        }
#endif // 0



    return 0;

}