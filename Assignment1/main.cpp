#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <iostream>
#include <opencv2/opencv.hpp>
#include<conio.h>
#include<vector>

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    //Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f model;
    model << cos(rotation_angle), -sin(rotation_angle), 0, 0, sin(rotation_angle),
        cos(rotation_angle), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function
    Eigen::Matrix4f projection;
    float w = 2 * zNear * tan(eye_fov / 2);
    float h = w/aspect_ratio;
    projection << 2 * zNear / w, 0, 0, 0, 0, 2 * zNear / h, 0, 0, 0, 0, zFar / (zFar - zNear),
        -zNear * zFar / (zFar - zNear),0, 0, 1, 0;
    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    return projection;
}


int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";
    //std::cout << filename << std::endl;
    if (argc >= 3) {
        //std::cout << 1;
        command_line = true;
        angle = std::atof(argv[2]); // -r by default
        //std::cout << 1;
        //std::cout << argv[0];
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

    char key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));
        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        //std::cout << filename[0] << std::endl;
        cv::imwrite(filename, image);
        return 0;
    }

    while (key != 27) {
        key = 0;
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        cv::waitKey(10);
        
        if (_kbhit()) key = _getch();

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
        else if (key == 'w'){
            eye_pos[2] = eye_pos[2]>2?eye_pos[2]-1:eye_pos[2];
        }
        else if (key == 's') {
            eye_pos[2]++;
        }
    }

    return 0;
}