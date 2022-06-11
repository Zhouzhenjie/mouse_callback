//
// Created by ksals on 2022/6/10.
//

#include "mouse_callback/mouse_callback.h"

void MouseCB::onMouse(int event, int x, int y, int __attribute__((unused)) flags, void* param)
{
  MouseCB* p_this = ((MouseCB*)param);
  static int count = 1;
  if (event == cv::EVENT_LBUTTONDOWN)
  {
    std::cout << "Point" << std::to_string(count++) << ": " << cv::Point2d(x, y) << std::endl;
    p_this->points_2d_.emplace_back(cv::Point2d(x, y));
    if (p_this->points_2d_.size() < 4)
      return;

    std::cout << "===========================\n";
    cv::Mat_<double> r_vec(3, 1);
    cv::Mat_<double> t_vec(3, 1);

    ros::NodeHandle nh_3d(p_this->parent_nh_, "3D_points");
    std::vector<std::string> point_keys{ "p1", "p2", "p3", "p4" };
    XmlRpc::XmlRpcValue point_3d;
    for (auto& key : point_keys)
    {
      if(nh_3d.getParam(key, point_3d))
      {
        auto x_3 = point_3d[0];
        auto y_3 = point_3d[1];
        auto z_3 = point_3d[2];
        p_this->points_3d_.emplace_back(cv::Point3d(x_3, y_3, z_3));
      }
    }

    cv::solvePnP(p_this->points_3d_, p_this->points_2d_, p_this->cam_intrinsic_mat_k_, p_this->dist_coefficients_,
                 r_vec, t_vec, false, cv::SOLVEPNP_AP3P);

    cv::Rodrigues(r_vec, r_vec);

    cv::Mat trans_mat;
    cv::hconcat(r_vec, t_vec, trans_mat);

    std::cout << trans_mat << std::endl;
    cv::destroyAllWindows();
  }
}

MouseCB::MouseCB(ros::NodeHandle& nh)
{
  initialize(nh);
  execute();
}

void MouseCB::initialize(ros::NodeHandle& nh)
{
  parent_nh_ = nh;
  
  camera_info_manager::CameraInfoManager info_manager{nh};
  info_manager.loadCameraInfo("");
  camera_info_ = info_manager.getCameraInfo();

  cam_intrinsic_mat_k_.create(3, 3);
  memcpy(cam_intrinsic_mat_k_.data, camera_info_.K.data(), 9 * sizeof(double));
  ROS_ASSERT(cv::determinant(cam_intrinsic_mat_k_) != 0);
  dist_coefficients_ = camera_info_.D;
  ROS_INFO("Successfully read camera info.");

  std::string img = this->template getParam<std::string>(parent_nh_, "img_path", "");
  img_ = cv::imread(img);
  points_3d_.clear();
  points_2d_.clear();
  ROS_INFO("Successfully read input img.");
}

void MouseCB::execute()
{
  cv::namedWindow("input_img");
  cv::setMouseCallback("input_img", MouseCB::onMouse, (void*)this);
  cv::imshow("input_img", img_);
  while (ros::ok())
  {
    cv::waitKey(1);
  }
}
