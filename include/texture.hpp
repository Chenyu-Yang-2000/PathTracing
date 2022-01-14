#pragma once
#include <vecmath.h>
#include <iostream>
#include <cstring>
#include "image.hpp"

class Texture {
    Image *image;
    Vector3f color;
    Vector3f xaxis, yaxis;
    double xbias, ybias, disturb_scale = 1.0;
    std::string name;

public:
    Texture(char *filename, Vector3f xaxis, double xbias, Vector3f yaxis, double ybias);

    Texture(char *filename);

    Texture(Vector3f color);

    Vector3f query(double x, double y) const;

    Vector3f query(const Vector3f &point) const;

    Vector3f getColor() const;
    Vector3f getColor(double u, double v) const;

    double getGray(double u, double v) const;

    double getDisturb(double u, double v, Vector2f &grad) const;

    void setDisturbScale(double scale);
};
