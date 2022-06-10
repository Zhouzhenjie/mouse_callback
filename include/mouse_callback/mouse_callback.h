//
// Created by ksals on 2022/6/10.
//

#pragma once

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

class MouseCB
{
public:

    ros::NodeHandle parent_nh_;
    cv::Mat img_;
    std::vector<cv::Point2d> points_2d_;
    std::vector<cv::Point3d> points_3d_;
    cv::Mat_<double> cam_intrinsic_mat_k_;
    std::vector<double> dist_coefficients_;

public:

    MouseCB(ros::NodeHandle& nh);

    ~MouseCB() = default;

    void initialize(ros::NodeHandle& nh);

    void execute();

    static void onMouse(int event, int x, int y, int flags, void *param);

    template<typename T>
    inline T getParam(const ros::NodeHandle &nh, const std::string &param_name, const T &default_val)
    {
        T param_val;
        nh.param<T>(param_name, param_val, default_val);
        return param_val;
    }
};
