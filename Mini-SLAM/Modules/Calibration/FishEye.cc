/**
 * FishEye.cc
 *
 * Implementation of the Fish-Eye camera model using the Kannala–Brandt model.
 */

 #include "FishEye.h"
 #include <cmath>
 
 // Define shorthand for accessing the camera parameters
 #define fx vParameters_[0]
 #define fy vParameters_[1]
 #define cx vParameters_[2]
 #define cy vParameters_[3]
 #define k1 vParameters_[4]
 #define k2 vParameters_[5]
 #define k3 vParameters_[6]
 #define k4 vParameters_[7]
 
 void FishEye::project(const Eigen::Vector3f& p3D, Eigen::Vector2f& p2D) {
     // Extract 3D coordinates
     float x = p3D(0), y = p3D(1), z = p3D(2);
     float r = std::sqrt(x*x + y*y);
 
     // Handle the degenerate case: if r is almost zero, return the principal point
     if (r < 1e-6f) {
         p2D(0) = cx;
         p2D(1) = cy;
         return;
     }
 
     // Compute the angle theta = arctan(r / z)
     float theta = std::atan(r / z);
 
     // Compute the distorted radius d(theta) using the polynomial:
     // d(theta) = theta + k1*theta^3 + k2*theta^5 + k3*theta^7 + k4*theta^9
     float theta2 = theta * theta;
     float theta3 = theta2 * theta;
     float theta4 = theta2 * theta2;
     float theta5 = theta4 * theta;
     float theta6 = theta4 * theta2;
     float theta7 = theta6 * theta;
     float theta8 = theta4 * theta4;
     float theta9 = theta8 * theta;
     float d = theta + k1 * theta3 + k2 * theta5 + k3 * theta7 + k4 * theta9;
 
     // Compute the final pixel coordinates.
     // Note: (x/r) and (y/r) give the direction in the image plane.
     p2D(0) = fx * d * (x / r) + cx;
     p2D(1) = fy * d * (y / r) + cy;
 }
 
 void FishEye::unproject(const Eigen::Vector2f& p2D, Eigen::Vector3f& p3D) {
     // Compute normalized image coordinates
     float mx = (p2D(0) - cx) / fx;
     float my = (p2D(1) - cy) / fy;
     float r_prime = std::sqrt(mx*mx + my*my);
 
     // If r_prime is nearly zero, the ray points along the optical axis.
     if (r_prime < 1e-6f) {
         p3D << 0, 0, 1;
         return;
     }
 
     // Solve for theta in d(theta) = r_prime using Newton's method.
     // Initialize theta with r_prime (small-angle approximation)
     float theta = r_prime;
     for (int i = 0; i < 10; i++) {
         float theta2 = theta * theta;
         float theta3 = theta2 * theta;
         float theta4 = theta2 * theta2;
         float theta5 = theta4 * theta;
         float theta6 = theta4 * theta2;
         float theta7 = theta6 * theta;
         float theta8 = theta4 * theta4;
         float theta9 = theta8 * theta;
         float d = theta + k1 * theta3 + k2 * theta5 + k3 * theta7 + k4 * theta9;
         // Derivative d'(theta) = 1 + 3*k1*theta^2 + 5*k2*theta^4 + 7*k3*theta^6 + 9*k4*theta^8
         float d_deriv = 1 + 3 * k1 * theta2 + 5 * k2 * theta4 + 7 * k3 * theta6 + 9 * k4 * theta8;
         float diff = d - r_prime;
         theta -= diff / d_deriv;
     }
 
     float sin_theta = std::sin(theta);
     float cos_theta = std::cos(theta);
     // Reconstruct the unit direction vector from the normalized image coordinates.
     p3D(0) = sin_theta * (mx / r_prime);
     p3D(1) = sin_theta * (my / r_prime);
     p3D(2) = cos_theta;
 }
 
 void FishEye::projectJac(const Eigen::Vector3f& p3D, Eigen::Matrix<float,2,3>& Jac) {
     float x = p3D(0), y = p3D(1), z = p3D(2);
     float r = std::sqrt(x*x + y*y);
 
     // If r is too small, fall back to the pinhole Jacobian.
     if (r < 1e-6f) {
         Jac(0,0) = fx / z;
         Jac(0,1) = 0.f;
         Jac(0,2) = -fx * x / (z*z);
         Jac(1,0) = 0.f;
         Jac(1,1) = fy / z;
         Jac(1,2) = -fy * y / (z*z);
         return;
     }
 
     // Compute theta = arctan(r/z)
     float theta = std::atan(r / z);
 
     // Compute distortion and its derivative:
     // d(theta) = theta + k1*theta^3 + k2*theta^5 + k3*theta^7 + k4*theta^9
     float theta2 = theta * theta;
     float theta3 = theta2 * theta;
     float theta4 = theta2 * theta2;
     float theta5 = theta4 * theta;
     float theta6 = theta4 * theta2;
     float theta7 = theta6 * theta;
     float theta8 = theta4 * theta4;
     float theta9 = theta8 * theta;
     float d = theta + k1 * theta3 + k2 * theta5 + k3 * theta7 + k4 * theta9;
     // d'(theta) = 1 + 3*k1*theta^2 + 5*k2*theta^4 + 7*k3*theta^6 + 9*k4*theta^8
     float d_deriv = 1 + 3 * k1 * theta2 + 5 * k2 * theta4 + 7 * k3 * theta6 + 9 * k4 * theta8;
 
     // Precompute common terms for the chain rule.
     float z2 = z * z;
     float r2 = r * r;
     float denom = r2 + z2; // = r^2 + z^2
 
     // Derivatives of theta = arctan(r/z)
     // dtheta/dx = (z * x) / (r * (r^2 + z^2))
     // dtheta/dy = (z * y) / (r * (r^2 + z^2))
     // dtheta/dz = -r / (r^2 + z^2)
     float dtheta_dx = (z * x) / (r * denom);
     float dtheta_dy = (z * y) / (r * denom);
     float dtheta_dz = -r / denom;
 
     // For the u-component, let F = d * (x/r).
     // Its derivative with respect to each coordinate is computed as:
     //   ∂F/∂x = d_deriv * (dtheta/dx) * (x/r) + d * (∂(x/r)/∂x)
     // where ∂(x/r)/∂x = (r^2 - x^2) / r^3 = y^2 / r^3.
     float dF_dx = d_deriv * dtheta_dx * (x / r) + d * ((r*r - x*x) / (r*r*r));
     //   ∂F/∂y = d_deriv * (dtheta/dy) * (x/r) + d * (∂(x/r)/∂y)
     // where ∂(x/r)/∂y = -x*y / r^3.
     float dF_dy = d_deriv * dtheta_dy * (x / r) - d * (x * y / (r*r*r));
     //   ∂F/∂z = d_deriv * (dtheta/dz) * (x/r)
     float dF_dz = d_deriv * dtheta_dz * (x / r);
 
     // For the v-component, let G = d * (y/r).
     // Similarly, we have:
     float dG_dx = d_deriv * dtheta_dx * (y / r) - d * (x * y / (r*r*r));
     float dG_dy = d_deriv * dtheta_dy * (y / r) + d * ((r*r - y*y) / (r*r*r));
     float dG_dz = d_deriv * dtheta_dz * (y / r);
 
     // Finally, the Jacobian of the projection is:
     // ∂u/∂(x,y,z) = fx * [dF_dx, dF_dy, dF_dz]
     // ∂v/∂(x,y,z) = fy * [dG_dx, dG_dy, dG_dz]
     Jac(0,0) = fx * dF_dx;
     Jac(0,1) = fx * dF_dy;
     Jac(0,2) = fx * dF_dz;
     Jac(1,0) = fy * dG_dx;
     Jac(1,1) = fy * dG_dy;
     Jac(1,2) = fy * dG_dz;
 }
 