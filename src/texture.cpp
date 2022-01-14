#define STB_IMAGE_IMPLEMENTATION
#include "texture.hpp"
#include "stb_image.h"

Texture::Texture(char *filename, Vector3f xaxis, double xbias, Vector3f yaxis, double ybias) {
    name = filename;
    int len = strlen(filename);
    int c;
    int width, height;
    if (strcmp(".ppm", filename + len - 4) == 0) {
        FILE *file = fopen(filename, "r");
        fscanf(file, "%*s%d%d%*d", &width, &height);
        image = new Image(width, height);
        for (int i = 0; i < height; ++i) {
            for (int j = 0; j < width; ++j) {
                int r, g, b;
                fscanf(file, "%d%d%d", &r, &g, &b);
                image->SetPixel(j, i, Vector3f(r / 255., g / 255., b / 255.));
            }
        }
        fclose(file);
    }
    else {
        unsigned char *pic = stbi_load(filename, &width, &height, &c, 0);
        image = new Image(width, height);
        for (int i = 0; i < height; ++i) {
            for (int j = 0; j < width; ++j) {
                int index = i * width * c + j * c;
                int r = pic[index], g = pic[index + 1], b = pic[index + 2];
                image->SetPixel(j, i, Vector3f(r / 255., g / 255., b / 255.));
            }
        }
        stbi_image_free(pic);
    }
    this->xaxis = xaxis;
    this->xbias = xbias;
    this->yaxis = yaxis;
    this->ybias = ybias;
    std::cerr << filename << " width = " << width << " height = " << height << " color = " << c << std::endl;
}

Texture::Texture(char *filename) {
    name = filename;
    int c;
    int width, height;
    unsigned char *pic = stbi_load(filename, &width, &height, &c, 0);
    image = new Image(width, height);
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            int index = i * width * c + j * c;
            int r = pic[index], g = pic[index] + 1, b = pic[index + 2];
            image->SetPixel(j, i, Vector3f(r / 255., g / 255., b / 255.));
        }
    }
    stbi_image_free(pic);
    std::cerr << filename << " width = " << width << " height = " << height << " color = " << c << std::endl;
}

Texture::Texture(Vector3f color) {
    this->image = nullptr;
    this->color = color;
}

Vector3f Texture::query(double x, double y) const {
    int _x = int(x * (image->Width() - 1));
    int _y = int(y * (image->Height() - 1));
    if (_x >= 0 && _x < image->Width() && _y >= 0 && _y < image->Height()) 
        return image->GetPixel(_x, _y);
    else
        return Vector3f::ZERO;
}

Vector3f Texture::query(const Vector3f &point) const {
    if (!image) return color;
    double _x = Vector3f::dot(xaxis, point) + xbias;
    double _y = Vector3f::dot(yaxis, point) + ybias;
    return query(_x, _y);
}

Vector3f Texture::getColor() const {
    return color;
}

Vector3f Texture::getColor(double u, double v) const {
    if (!image) return color;
    u -= int(u); v -= int(v);
    if (u < 0) u += 1;
    if (v < 0) v += 1;
    u = u * (image->Width() - 1);
    v = v * (image->Height() - 1);
    
    // Nearest
    /*int x = int(u + 0.5), y = int(v + 0.5);
    Vector3f ret = image->GetPixel(x, y);*/

    // Biliear interpolation
    int x = int(u), y = int(v);
    double alpha = u - x, beta = v - y;
    Vector3f ret = Vector3f::ZERO;
    ret += (1 - alpha) * (1 - beta) * image->GetPixel(x, y);
    ret += alpha * (1 - beta) * image->GetPixel((x + 1) % image->Width(), y);
    ret += (1 - alpha) * beta * image->GetPixel(x, (y + 1) % image->Height());
    ret += alpha * beta * image->GetPixel((x + 1) % image->Width(), (y + 1) % image->Height());
    
    return ret;
}

double Texture::getDisturb(double u, double v, Vector2f &grad) const {
    if (!image) return 0;
    double disturb = getGray(u, v);
    double du = 1.0 / (image->Width() - 1), dv = 1.0 / (image->Height() - 1);
    grad[0] = (image->Width() - 1) * (getGray(u + du, v) - getGray(u - du, v)) / 2.0 * disturb_scale;
    grad[1] = (image->Height() - 1) * (getGray(u, v + dv) - getGray(u, v - dv)) / 2.0 * disturb_scale;
    return disturb * disturb_scale;
}

double Texture::getGray(double u, double v) const {
    return (Vector3f::dot(getColor(u, v), Vector3f(0.299, 0.587, 0.114)) - 0.5) * 2;
}

void Texture::setDisturbScale(double scale) {
    disturb_scale = scale;
}