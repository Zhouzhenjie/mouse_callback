//
// Created by ksals on 2022/6/10.
//

#include "mouse_callback/mouse_callback.h"

void MouseCB::onMouse(int event, int x, int y, int flags, void *param)
{
  if (event == cv::EVENT_LBUTTONDOWN)
  {
    if (((MouseCB*)param)->points_2d_.size() == 4)
    {
      using namespace std;
      cout << ((MouseCB*)param)->points_2d_[0] << endl;
      cout << ((MouseCB*)param)->points_2d_[1] << endl;
      cout << ((MouseCB*)param)->points_2d_[2] << endl;
      cout << ((MouseCB*)param)->points_2d_[3] << endl;

      cv::Mat_<double> r_vec(3, 1);
      cv::Mat_<double> t_vec(3, 1);

      ((MouseCB*)param)->points_3d_.emplace_back(cv::Point3d(-50, -50, 0));
      ((MouseCB*)param)->points_3d_.emplace_back(cv::Point3d(-50, 50, 0));
      ((MouseCB*)param)->points_3d_.emplace_back(cv::Point3d(50, 50, 0));
      ((MouseCB*)param)->points_3d_.emplace_back(cv::Point3d(50, -50, 0));

      cv::solvePnP(((MouseCB*)param)->points_3d_, ((MouseCB*)param)->points_2d_,
                   ((MouseCB*)param)->cam_intrinsic_mat_k_, ((MouseCB*)param)->dist_coefficients_,
                   r_vec, t_vec, false, cv::SOLVEPNP_AP3P);

      cv::Rodrigues(r_vec, r_vec);

      cv::Mat trans;
      cv::hconcat(r_vec, t_vec, trans);
      cout << trans << endl;
      cv::destroyAllWindows();
    }
    ((MouseCB*)param)->points_2d_.emplace_back(cv::Point2d(x, y));
  }
}

MouseCB::MouseCB(ros::NodeHandle &nh)
{
  initialize(nh);
  execute();
}

void MouseCB::initialize(ros::NodeHandle &nh)
{
  cam_intrinsic_mat_k_ = cv::Mat_<double>(3, 3) << 1265.55389, 0, 666.65908,
                                                              0, 1266.73808, 526.60665,
                                                              0, 0, 1;
  dist_coefficients_ = std::vector<double>{-0.214808, 0.127733, 0.000432, -0.000236, 0.000000};
  parent_nh_ = nh;
  ros::NodeHandle input_img_nh(parent_nh_, "input_img");
  std::string img = this->template getParam<std::string>(input_img_nh, "img_path", "/");
  img_ = cv::imread(img);
  ROS_INFO("Success.");
}

void MouseCB::execute()
{
  cv::namedWindow("input_img");
  cv::setMouseCallback("input_img", MouseCB::onMouse, (void*)this);
  cv::imshow("input_img", img_);
  cv::waitKey(0);
}
