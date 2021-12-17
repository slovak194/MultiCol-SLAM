//
// Created by slovak on 12/17/21.
//

#ifndef MULTICOL_SLAM_INCLUDE_CAM_MODEL_PERSPECTIVE_H_
#define MULTICOL_SLAM_INCLUDE_CAM_MODEL_PERSPECTIVE_H_

// extern includes
#include <opencv2/opencv.hpp>
#include <limits>
#include <Eigen/Dense>

namespace MultiColSLAM
{
class cCamModelPerspective_
{
 public:
  // construtors
  cCamModelPerspective_() {}

  void WorldToImg(const double& x, const double& y, const double& z,    // 3D scene point
                          double& u, double& v) const;

  void WorldToImg(const cv::Vec3d& X,			// 3D scene point
                          cv::Vec2d& m);

  void ImgToWorld(double& x, double& y, double& z,						// 3D scene point
                          const double& u, const double& v);

  void ImgToWorld(cv::Point3_<double>& X,						// 3D scene point
                          const cv::Point_<double>& m);

  void SetMirrorMasks(std::vector<cv::Mat> mirrorMasks_);
  cv::Mat GetMirrorMask(int pyrL);

  double GetWidth();
  double GetHeight();

  double Get_v0();
  double Get_u0();

  inline Eigen::Matrix<double, 12 + 5, 1> toVector() const;
  inline void fromVector(const Eigen::Matrix<double, 12 + 5, 1> tmp);

  bool isPointInMirrorMask(const double& u, const double& v, int pyr);

  void undistortPointsOcam(
      const double& ptx, const double& pty,
      const double& undistScaleFactor,
      double& out_ptx, double& out_pty);

  void distortPointsOcam(
      const double& ptx, const double& pty,
      double& dist_ptx, double& dist_pty);

  cv::Mat_<double> Get_P();


};

}


#endif //MULTICOL_SLAM_INCLUDE_CAM_MODEL_PERSPECTIVE_H_
