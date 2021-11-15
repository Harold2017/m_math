//
// Created by Harold on 2021/11/15.
//

// code from: https://github.com/PointCloudLibrary/pcl/blob/master/surface/include/pcl/surface/impl/texture_mapping.hpp
// revised by Harold to fit Open3D

#ifndef M_MATH_M_TEXTURE_MAPPING_HPP
#define M_MATH_M_TEXTURE_MAPPING_HPP

#include <open3d/Open3D.h>

namespace M_MATH
{
    namespace texture_mapping
    {
        struct Camera
        {
            Eigen::Matrix4d pose;
            double focal_length = 0;
            double focal_length_w = -1;
            double focal_length_h = -1;
            double center_w = -1;
            double center_h = -1;
            double height = 0;
            double width = 0;
            std::string texture_file;
        };

        struct UvIndex
        {
            int idx_cloud;
            int idx_face;
        };

        using CameraVector = std::vector<Camera, Eigen::aligned_allocator<Camera>>;

        inline bool getPointUVCoordinates(const Eigen::Vector3d &pt, const Camera &cam, Eigen::Vector2d &UV_coordinates)
        {
            if (pt(2) > 0)
            {
                // compute image center and dimension
                double sizeX = cam.width;
                double sizeY = cam.height;
                double cx, cy;
                if (cam.center_w > 0)
                    cx = cam.center_w;
                else
                    cx = sizeX / 2.0;
                if (cam.center_h > 0)
                    cy = cam.center_h;
                else
                    cy = sizeY / 2.0;

                double focal_x, focal_y;
                if (cam.focal_length_w > 0)
                    focal_x = cam.focal_length_w;
                else
                    focal_x = cam.focal_length;
                if (cam.focal_length_h > 0)
                    focal_y = cam.focal_length_h;
                else
                    focal_y = cam.focal_length;

                // project point on camera's image plane
                UV_coordinates(0) = static_cast<float>((focal_x * (pt(0) / pt(2)) + cx) / sizeX);       //horizontal
                UV_coordinates(1) = 1.0 - static_cast<float>((focal_y * (pt(1) / pt(2)) + cy) / sizeY); //vertical

                // point is visible!
                if (UV_coordinates(0) >= 0.0 && UV_coordinates(0) <= 1.0 && UV_coordinates(1) >= 0.0 && UV_coordinates(1) <= 1.0)
                    return (true); // point was visible by the camera
            }

            // point is NOT visible by the camera
            UV_coordinates(0) = -1.0;
            UV_coordinates(1) = -1.0;
            return (false); // point was not visible by the camera
        }

        inline bool isFaceProjected(const Camera &camera, const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3, Eigen::Vector2d &proj1, Eigen::Vector2d &proj2, Eigen::Vector2d &proj3)
        {
            return (getPointUVCoordinates(p1, camera, proj1) &&
                    getPointUVCoordinates(p2, camera, proj2) &&
                    getPointUVCoordinates(p3, camera, proj3));
        }

        inline void getTriangleCircumcscribedCircleCentroid(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, const Eigen::Vector2d &p3, Eigen::Vector2d &circumcenter, double &radius)
        {
            // compute centroid's coordinates (translate back to original coordinates)
            circumcenter(0) = (p1(0) + p2(0) + p3(0)) / 3.;
            circumcenter(1) = (p1(1) + p2(1) + p3(1)) / 3.;
            double r1 = (circumcenter(0) - p1(0)) * (circumcenter(0) - p1(0)) + (circumcenter(1) - p1(1)) * (circumcenter(1) - p1(1));
            double r2 = (circumcenter(0) - p2(0)) * (circumcenter(0) - p2(0)) + (circumcenter(1) - p2(1)) * (circumcenter(1) - p2(1));
            double r3 = (circumcenter(0) - p3(0)) * (circumcenter(0) - p3(0)) + (circumcenter(1) - p3(1)) * (circumcenter(1) - p3(1));

            // radius
            radius = std::sqrt(std::max(r1, std::max(r2, r3)));
        }

        inline bool checkPointInsideTriangle(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, const Eigen::Vector2d &p3, const Eigen::Vector2d &pt)
        {
            // Compute vectors
            Eigen::Vector2d v0, v1, v2;
            v0 = p3 - p1; // v0= C - A
            v1 = p2 - p1; // v1= B - A
            v2 = pt - p1; // v2= P - A

            // Compute dot products
            double dot00 = v0.dot(v0); // dot00 = dot(v0, v0)
            double dot01 = v0.dot(v1); // dot01 = dot(v0, v1)
            double dot02 = v0.dot(v2); // dot02 = dot(v0, v2)
            double dot11 = v1.dot(v1); // dot11 = dot(v1, v1)
            double dot12 = v1.dot(v2); // dot12 = dot(v1, v2)

            // Compute barycentric coordinates
            double invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
            double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
            double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

            // Check if point is in triangle
            return ((u >= 0) && (v >= 0) && (u + v < 1));
        }

        // FIXME: keep other textures
        void TextureMeshWithMultipleCameras(std::shared_ptr<open3d::geometry::TriangleMesh> mesh,
                                            const texture_mapping::CameraVector &cameras)
        {
            auto mesh_pcd = std::make_shared<open3d::geometry::PointCloud>(mesh->vertices_);
            for (int current_cam = 0; current_cam < static_cast<int>(cameras.size()); ++current_cam)
            {
                printf("Processing camera %d of %d.\n", current_cam + 1, static_cast<int>(cameras.size()));

                // transform mesh into camera's frame
                auto camera_cloud = mesh_pcd->Transform(cameras[current_cam].pose.inverse());

                // create uv map for current faces
                open3d::geometry::PointCloud projections;
                std::vector<bool> visibility(mesh->triangles_.size());
                std::vector<UvIndex> indexes_uv_to_points;

                // for each face
                Eigen::Vector2d nan_point;
                nan_point(0) = std::numeric_limits<double>::quiet_NaN();
                nan_point(1) = std::numeric_limits<double>::quiet_NaN();
                UvIndex u_null;
                u_null.idx_cloud = -1;
                u_null.idx_face = -1;

                int cpt_invisible = 0;
                for (int idx_face = 0; idx_face < static_cast<int>(mesh->triangles_.size()); ++idx_face)
                {
                    // project each vertice, if one is out of view, stop
                    Eigen::Vector2d uv_coord1, uv_coord2, uv_coord3;

                    if (isFaceProjected(cameras[current_cam],
                                        camera_cloud.points_[mesh->triangles_[idx_face](0)],
                                        camera_cloud.points_[mesh->triangles_[idx_face](1)],
                                        camera_cloud.points_[mesh->triangles_[idx_face](2)],
                                        uv_coord1,
                                        uv_coord2,
                                        uv_coord3))
                    {
                        // face is in the camera's FOV

                        // add UV coordinates
                        projections.points_.emplace_back(uv_coord1(0), uv_coord1(1), 0);
                        projections.points_.emplace_back(uv_coord2(0), uv_coord2(1), 0);
                        projections.points_.emplace_back(uv_coord3(0), uv_coord3(1), 0);

                        // remember corresponding face
                        UvIndex u1, u2, u3;
                        u1.idx_cloud = mesh->triangles_[idx_face](0);
                        u2.idx_cloud = mesh->triangles_[idx_face](1);
                        u3.idx_cloud = mesh->triangles_[idx_face](2);
                        u1.idx_face = idx_face;
                        u2.idx_face = idx_face;
                        u3.idx_face = idx_face;
                        indexes_uv_to_points.push_back(u1);
                        indexes_uv_to_points.push_back(u2);
                        indexes_uv_to_points.push_back(u3);

                        //keep track of visibility
                        visibility[idx_face] = true;
                    }
                    else
                    {
                        projections.points_.emplace_back(nan_point(0), nan_point(1), 0);
                        projections.points_.emplace_back(nan_point(0), nan_point(1), 0);
                        projections.points_.emplace_back(nan_point(0), nan_point(1), 0);
                        indexes_uv_to_points.push_back(u_null);
                        indexes_uv_to_points.push_back(u_null);
                        indexes_uv_to_points.push_back(u_null);
                        //keep track of visibility
                        visibility[idx_face] = false;
                        cpt_invisible++;
                    }
                }

                // projections contains all UV points of the current faces
                // indexes_uv_to_points links a uv point to its point in the camera cloud
                // visibility contains tells if a face was in the camera FOV (false = skip)

                // TODO: handle case were no face could be projected
                if (visibility.size() - cpt_invisible != 0)
                {
                    // create kdtree
                    open3d::geometry::KDTreeFlann kdtree(projections);
                    std::vector<int> idxNeighbors;
                    std::vector<double> neighborsSquaredDistance;
                    // af first (idx_pcam < current_cam), check if some of the faces attached to previous cameras occlude the current faces
                    // then (idx_pcam == current_cam), check for self occlusions. At this stage, we skip faces that were already marked as occluded
                    cpt_invisible = 0;
                    for (int idx_pcam = 0; idx_pcam <= current_cam; ++idx_pcam)
                    {
                        // project all faces
                        for (int idx_face = 0; idx_face < static_cast<int>(mesh->triangles_.size()); ++idx_face)
                        {
                            if (idx_pcam == current_cam && !visibility[idx_face])
                            {
                                // we are now checking for self occlusions within the current faces
                                // the current face was already declared as occluded.
                                // therefore, it cannot occlude another face anymore => we skip it
                                continue;
                            }

                            // project each vertice, if one is out of view, stop
                            Eigen::Vector2d uv_coord1, uv_coord2, uv_coord3;

                            if (isFaceProjected(cameras[current_cam],
                                                camera_cloud.points_[mesh->triangles_[idx_face](0)],
                                                camera_cloud.points_[mesh->triangles_[idx_face](1)],
                                                camera_cloud.points_[mesh->triangles_[idx_face](2)],
                                                uv_coord1,
                                                uv_coord2,
                                                uv_coord3))
                            {
                                // face is in the camera's FOV
                                // get its circumscribed circle
                                double radius;
                                Eigen::Vector2d center;
                                getTriangleCircumcscribedCircleCentroid(uv_coord1, uv_coord2, uv_coord3, center, radius);

                                // get points inside circ.circle
                                if (kdtree.SearchRadius<Eigen::Vector3d>(Eigen::Vector3d(center(0), center(1), 0), radius, idxNeighbors, neighborsSquaredDistance) > 0)
                                {
                                    // for each neighbor
                                    for (const auto &idxNeighbor : idxNeighbors)
                                    {
                                        if (std::max(camera_cloud.points_[mesh->triangles_[idx_face](0)](2),
                                                     std::max(camera_cloud.points_[mesh->triangles_[idx_face](1)](2),
                                                              camera_cloud.points_[mesh->triangles_[idx_face](2)](2))) < camera_cloud.points_[indexes_uv_to_points[idxNeighbor].idx_cloud](2))
                                        {
                                            if (checkPointInsideTriangle(uv_coord1, uv_coord2, uv_coord3, Eigen::Vector2d(projections.points_[idxNeighbor](0), projections.points_[idxNeighbor](1))))
                                            {
                                                // current neighbor is inside triangle and is closer => the corresponding face
                                                visibility[indexes_uv_to_points[idxNeighbor].idx_face] = false;
                                                cpt_invisible++;
                                                //TODO we could remove the projections of this face from the kd-tree cloud, but I fond it slower, and I need the point to keep ordered to querry UV coordinates later
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                // now, visibility is true for each face that belongs to the current camera
                // if a face is not visible, we push it into the next one.

                if (!mesh->HasTriangleUvs())
                {
                    std::vector<Eigen::Vector2d> uvs(3 * visibility.size());
                    mesh->triangle_uvs_ = uvs;
                }
                else
                    mesh->triangle_uvs_.resize(3 * visibility.size());
                std::vector<Eigen::Vector3i> occluded_faces(visibility.size());
                std::vector<Eigen::Vector3i> visible_faces(visibility.size());

                int cpt_occluded_faces = 0;
                int cpt_visible_faces = 0;

                for (auto idx_face = 0; idx_face < visibility.size(); ++idx_face)
                {
                    if (visibility[idx_face])
                    {
                        // face is visible by the current camera copy UV coordinates
                        mesh->triangle_uvs_[cpt_visible_faces * 3](0) = projections.points_[idx_face * 3](0);
                        mesh->triangle_uvs_[cpt_visible_faces * 3](1) = projections.points_[idx_face * 3](1);

                        mesh->triangle_uvs_[cpt_visible_faces * 3 + 1](0) = projections.points_[idx_face * 3 + 1](0);
                        mesh->triangle_uvs_[cpt_visible_faces * 3 + 1](1) = projections.points_[idx_face * 3 + 1](1);

                        mesh->triangle_uvs_[cpt_visible_faces * 3 + 2](0) = projections.points_[idx_face * 3 + 2](0);
                        mesh->triangle_uvs_[cpt_visible_faces * 3 + 2](1) = projections.points_[idx_face * 3 + 2](1);

                        visible_faces[cpt_visible_faces] = mesh->triangles_[idx_face];

                        cpt_visible_faces++;
                    }
                    else
                    {
                        // face is occluded copy face into temp vector
                        occluded_faces[cpt_visible_faces] = mesh->triangles_[idx_face];
                        cpt_visible_faces++;
                    }
                }
                mesh->triangle_uvs_.resize(cpt_visible_faces * 3);

                //mesh->triangles_ = visible_faces;
            }

            // we have been through all the cameras;
            // if any faces are left, they were not visible by any camera
            // we still need to produce uv corrdincates for them
        }
    }
}

#endif //M_MATH_M_TEXTURE_MAPPING_HPP