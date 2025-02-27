/**
 * FishEye.h
 *
 * This file is part of Mini-SLAM.
 *
 * Implementation of the Fish-Eye camera model using the Kannala–Brandt model.
 * The camera is parameterized by 8 parameters:
 *   [fx, fy, cx, cy, k1, k2, k3, k4]
 */

 #ifndef JJSLAM_FISHEYE_H
 #define JJSLAM_FISHEYE_H
 
 #include "CameraModel.h"
 #include <vector>
 #include <cassert>
 #include <opencv2/opencv.hpp>
 #include <Eigen/Dense>
 
 class FishEye : public CameraModel {
 public:
     // Default constructor: resize parameter vector to 8 elements.
     FishEye() {
         vParameters_.resize(8);
     }
 
     /**
      * Constructor with parameters:
      *   [fx, fy, cx, cy, k1, k2, k3, k4]
      */
     FishEye(const std::vector<float>& _vParameters) : CameraModel(_vParameters) {
         assert(vParameters_.size() == 8);
     }
 
     // Fish-eye projection using the Kannala–Brandt model.
     // Given a 3D point in camera coordinates, project() computes its image coordinates.
     void project(const Eigen::Vector3f& p3D, Eigen::Vector2f& p2D);
 
     // Fish-eye unprojection using the Kannala–Brandt model.
     // Given pixel coordinates, unproject() computes the corresponding unit direction vector.
     void unproject(const Eigen::Vector2f& p2D, Eigen::Vector3f& p3D);
 
     // Computes the Jacobian of the projection function with respect to the 3D point.
     void projectJac(const Eigen::Vector3f& p3D, Eigen::Matrix<float,2,3>& Jac);
 
     // Jacobian for unprojection is left unimplemented.
     void unprojectJac(const Eigen::Vector2f& p2D, Eigen::Matrix<float,3,2>& Jac) {}
 };
 
 #endif // JJSLAM_FISHEYE_H
 