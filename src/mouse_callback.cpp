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

    p_this->points_3d_.emplace_back(cv::Point3d(-50, -50, 0));
    p_this->points_3d_.emplace_back(cv::Point3d(-50, 50, 0));
    p_this->points_3d_.emplace_back(cv::Point3d(50, 50, 0));
    p_this->points_3d_.emplace_back(cv::Point3d(50, -50, 0));

    cv::solvePnP(p_this->points_3d_, p_this->points_2d_,
                 p_this->cam_intrinsic_mat_k_, p_this->dist_coefficients_,
                 r_vec, t_vec, false, cv::SOLVEPNP_AP3P);

    cv::Rodrigues(r_vec, r_vec);

    cv::Mat trans;
    cv::hconcat(r_vec, t_vec, trans);
    std::cout << trans << std::endl;
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
  // TODO
  cam_intrinsic_mat_k_ = (cv::Mat_<double>(3, 3) << 1265.55389, 0, 666.65908, 0, 1266.73808, 526.60665, 0, 0, 1);
  dist_coefficients_ = std::vector<double>{ -0.214808, 0.127733, 0.000432, -0.000236, 0.000000 };
  std::string img = this->template getParam<std::string>(nh, "img_path", "");
  img_ = cv::imread(img);
  points_3d_.clear();
  points_2d_.clear();
  ROS_INFO("Success.");
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
