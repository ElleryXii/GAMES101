#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926535;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    view << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1, -eye_pos[2], 0, 0, 0, 1;
    return view;
}


// Create the model matrix for rotating the triangle around any axis through origin.
Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {
    float r = angle * MY_PI / 180.0f;

    //Use Rodrigues' Rotation Formula
    Matrix3f N;
    N << 0, -axis[2], axis[1] , axis[2], 0, -axis[0], -axis[1], axis[0], 0;

    axis = axis.transpose();
    MatrixXf temp = std::cos(r)* Matrix3f::Identity() + (1.0 - std::cos(r)) * axis * axis.transpose() + sin(r) * N;
    Eigen::Matrix4f rotate_martix = Eigen::Matrix4f::Identity();
    rotate_martix.block(0, 0, 3, 3) = temp; 
    return rotate_martix;
}


// Create the model matrix for rotating the triangle around the Z axis. Then return it.
Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    float r = rotation_angle*MY_PI/180.0f;

    // z-axis
    model(0,0) = cos(r);
    model(0,1) = -sin(r);
    model(1,0) = sin(r);
    model(1,1) = cos(r);

    // //x-axis
    // model(1,1) = cos(r);
    // model(1,2) = -sin(r);
    // model(2,1) = sin(r);
    // model(2,2) = cos(r);

    // //y-axis
    // model(0,0) = cos(r);
    // model(0,2) = sin(r);
    // model(2,0) = - sin(r);
    // model(2,2) = cos(r);

    return model;
}


//Create the projection matrix for the given parameters. Then return it.
Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // top of the near plane
    float top = std::tan(eye_fov/2.0f*MY_PI/180.0f) * std::abs(zNear);
    // make top = -top here if you want the triangle upright.
    top = -top;
    float bottom = -top;
    float right = top * aspect_ratio;
    float left = -right;

    Eigen::Matrix4f perspToOrtho = Eigen::Matrix4f::Identity();
    perspToOrtho(0,0) = zNear;
    perspToOrtho(1,1) = zNear;
    perspToOrtho(2,2) = zNear+zFar;
    perspToOrtho(2,3) = (-1)* zNear *zFar;
    perspToOrtho(3,2) = 1;
    perspToOrtho(3,3) = 0;

    Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
    scale(0,0) = 2.0f/(right - left);
    scale(1,1) = 2.0f/(top - bottom);
    scale(2,2) = 2.0f/(zNear - zFar);

    Eigen::Matrix4f translate = Eigen::Matrix4f::Identity();
    translate(0,3) = (-1)*(right+left)/2.0f;
    translate(1,3) = (-1)*(top+bottom)/2.0f;
    translate(2,3) = (-1)*(zNear+zFar)/2.0f;

    projection = scale * translate * perspToOrtho;

    return projection;
}

int main(int argc, const char** argv)
{
    Vector3f angle(0,0,0);
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle(2) = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle.z()));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    auto z_axis = Vector3f(0.0, 0.0, 1.0);
    auto x_axis = Vector3f(1.0, 0.0, 0.0);
    auto y_axis = Vector3f(0.0, 1.0, 0.0);
    auto axis = z_axis;
    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        //r.set_model(get_model_matrix(angle));
        
        r.set_model(get_rotation(axis, axis.dot(angle)));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            axis = y_axis;
            angle(1) += 10;
        }
        else if (key == 'd') {
            axis = y_axis;
            angle(1) -= 10;
        }
        else if (key == 'w') {
            axis = x_axis;
            angle(0) += 10;
        }
        else if (key == 's') {
            axis = x_axis;
            angle(0) -= 10;
        }
        else if (key == 'q') {
            axis = z_axis;
            angle(2) += 10;
        }
        else if (key == 'e') {
            axis = z_axis;
            angle(2) -= 10;
        }
    }

    return 0;
}
