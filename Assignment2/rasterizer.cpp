// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


Vector3f cross(Vector3f a, Vector3f b)
{
    Vector3f r;
    r << a.y() * b.z() - a.z() * b.y(), a.z()* b.x() - a.x() * b.z(), a.x()* b.y() - a.y() * b.x();
    return r;
}

static bool insideTriangle(int x, int y,const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    Vector3f v1, v2, v3;
    Vector3f vp1, vp2, vp3;
    //Vector3f vpp1, vpp2, vpp3;
    v1 = _v[1] - _v[0];
    v2 = _v[2] - _v[1];
    v3 = _v[0] - _v[2];
    v1.z() = 0;
    v2.z() = 0;
    v3.z() = 0;
    //vp1 << v1.x(), v1.y(),0;
    //vp2 << v2.x(), v2.y(),0;
    //vp3 << v3.x(), v3.y(),0;
    float a, b, c;
    //vp1 << x - _v[0].x(), y - _v[0].y(),z-_v[0].z();
    //vp2 << x - _v[1].x(), y - _v[1].y(), z - _v[1].z();
    //vp3 << x - _v[2].x(), y - _v[2].y(), z - _v[2].z();
    vp1 << x - _v[0].x(), y - _v[0].y(),0;
    vp2 << x - _v[1].x(), y - _v[1].y(), 0;
    vp3 << x - _v[2].x(), y - _v[2].y(), 0;
    //v1 = v1.cross(vp1);
    //v2 = v2.cross(vp2);
    //v3 = v3.cross(vp3);
    //std::cout << vp1 << std::endl;
    v1 = cross(v1, vp1);
    v2 = cross(v2, vp2);
    v3 = cross(v3, vp3);
    //std::cout<<v1<<std::endl;
    a = v1.dot(v2);
    b = v2.dot(v3);
    c = v3.dot(v1);
    if (a * b > 0 && a * c > 0 && b * c > 0)return true;
    return false;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

int min_(int x, int y, int z)
{
    int a = x < y ? x : y;
    int b = a < z ? a : z;
    return b;
}

int max_(int x, int y, int z)
{
    int a = x > y ? x : y;
    return a > z ? a : z;
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    float alpha, beta, gamma;
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    // If so, use the following code to get the interpolated z value.
    int small_x, large_x, small_y, large_y;
    small_x = min_(t.v[0].x(), t.v[1].x(), t.v[2].x());
    large_x = max_(t.v[0].x(), t.v[1].x(), t.v[2].x());
    small_y = min_(t.v[0].y(), t.v[1].y(), t.v[2].y());
    large_y = max_(t.v[0].y(), t.v[1].y(), t.v[2].y());
    for (int x = small_x; x < large_x; x++)
    {
        for (int y = small_y; y < large_y; y++)
        {
            std::tuple<float, float, float> tp = computeBarycentric2D(x, y, t.v);
            std::tie(alpha, beta, gamma) = tp;
            float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
            float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
            z_interpolated *= w_reciprocal;
            if (insideTriangle(x, y, t.v))
            {
                //std::cout << 1<<std::endl;
                Vector3f point;
                point << x, y, z_interpolated;
                set_pixel(point,t.getColor());
            }
        }
    }

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;
    //std::cout << 1<<std::endl;

}

// clang-format on