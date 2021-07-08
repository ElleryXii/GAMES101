//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v)
    {
        float u_img = u * (float)width;
        float v_img = (1 - v) * (float)height;

        int u0 = std::floor(u_img), u1 = std::ceil(u_img);
        int v0 = std::floor(v_img), v1 = std::ceil(v_img);
        float s = u_img - std::floor(u_img), t = v_img - std::floor(v_img);

        auto p00 = getColor2(v0, u0);
        auto p01 = getColor2(v1, u0);
        auto p10 = getColor2(v0, u1);
        auto p11 = getColor2(v1, u1);
        
        Eigen::Vector3f c1 = p00 + s * (p10 - p00);
        Eigen::Vector3f c2 = p01 + s * (p11 - p01);
        Eigen::Vector3f color = c1 + t * (c2 - c1);
        //int a = getColor(u, v) == Eigen::Vector3f(color[0], color[1], color[2]) ? 1 : 0;
        //std::cout << a << std::endl;
        return color;
    }

    Eigen::Vector3f getColor2(int v, int u)
    {
        int u_i = std::min(width - 1, u);
        u_i = std::max(0, u_i);
        int v_i = std::min(height - 1, v);
        v_i = std::max(0, v_i);
        auto color = image_data.at<cv::Vec3b>(v_i, u_i);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    //Eigen::Vector3f getColorBilinear(float u, float v)
    //{
    //    float u_img = u * (width - 1);
    //    float v_img = (1 - v) * (height - 1);

    //    Eigen::Vector2f u00{ std::floor(u_img), std::floor(v_img) };
    //    Eigen::Vector2f u01{ std::ceil(u_img), std::floor(v_img) };
    //    Eigen::Vector2f u10{ std::floor(u_img), std::ceil(v_img) };
    //    Eigen::Vector2f u11{ std::ceil(u_img), std::ceil(v_img) };

    //    float s = u_img - std::floor(u_img);
    //    float t = v_img - std::floor(v_img);

    //    Eigen::Vector3f u0 = getColor2(u00.x(), u00.y()) + s * (getColor2(u01.x(), u01.y()) - getColor2(u00.x(), u00.y()));
    //    Eigen::Vector3f u1 = getColor2(u10.x(), u10.y()) + s * (getColor2(u11.x(), u11.y()) - getColor2(u10.x(), u10.y()));

    //    return u0 + t * (u1 - u0);
    //}

};
#endif //RASTERIZER_TEXTURE_H
