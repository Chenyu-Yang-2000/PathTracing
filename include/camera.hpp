#ifndef CAMERA_H
#define CAMERA_H

#include "ray.hpp"
#include <vecmath.h>
#include <float.h>
#include <cmath>
#include <random>

class Camera {
public:
    Camera(const Vector3f &center, const Vector3f &direction, const Vector3f &up, int imgW, int imgH) {
        this->center = center;
        this->direction = direction.normalized();
        this->horizontal = Vector3f::cross(this->direction, up).normalized();
        this->up = Vector3f::cross(this->horizontal, this->direction);
        this->width = imgW;
        this->height = imgH;
    }

    // Generate rays for each screen-space coordinate
    virtual Ray generateRay(const Vector2f &point) = 0;
    virtual ~Camera() = default;

    int getWidth() const { return width; }
    int getHeight() const { return height; }

protected:
    // Extrinsic parameters
    Vector3f center;
    Vector3f direction;
    Vector3f up;
    Vector3f horizontal;
    // Intrinsic parameters
    int width;
    int height;
};


class PerspectiveCamera : public Camera {

public:
    PerspectiveCamera(const Vector3f &center, const Vector3f &direction,
            const Vector3f &up, int imgW, int imgH, double angle) : Camera(center, direction, up, imgW, imgH) {
        // angle is in radian.
        f_x = imgW / (2 * std::tan(angle / 2));
        f_y = imgH / (2 * std::tan(angle / 2));
    }

    Ray generateRay(const Vector2f &point) override {
        Vector3f d_c = Vector3f((point[0] - width / 2.0f) / f_x, (height / 2.0f - point[1]) / f_y, 1);
        //d_c.normalize();
        Vector3f d_w = (horizontal * d_c[0] - up * d_c[1] + direction * d_c[2]).normalized();
        Ray r(center, d_w);
        return r;
    }
    double f_x, f_y;
};


class LensCamera : public Camera {

public:
    LensCamera(const Vector3f &center, const Vector3f &direction, const Vector3f &up, 
               int imgW, int imgH, double angle, double _depth, double _aperture) : Camera(center, direction, up, imgW, imgH) {
        f_x = imgW / (2 * std::tan(angle / 2));
        f_y = imgH / (2 * std::tan(angle / 2));
        depth = _depth; aperture = _aperture;
    }

    Ray generateRay(const Vector2f &point) override {
        Vector3f d_c = Vector3f((point[0] - width / 2.0f) / f_x, (height / 2.0f - point[1]) / f_y, 1);
        Vector3f d_w = (horizontal * d_c[0] - up * d_c[1] + direction * d_c[2]).normalized();
        double t = depth / (Vector3f::dot(d_w, direction));
        // random permute
        Vector3f focal_point = center + d_w * t;
        double dx = n(e) * aperture, dy = n(e) * aperture;
        Vector3f random_center = center + dx * horizontal + dy * up;
        d_w = (focal_point - random_center).normalized();
        // generate ray
        Ray r(random_center, d_w);
        return r;
    }
    double f_x, f_y, depth, aperture;
    std::default_random_engine e {(unsigned)time(0)};
    std::normal_distribution<double> n {0, 1};
};   

#endif //CAMERA_H
