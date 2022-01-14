#ifndef LIGHT_H
#define LIGHT_H

#include <Vector3f.h>
#include <random>
#include "object3d.hpp"

class Light {
public:
    Light() : hash(rand()) {};

    virtual ~Light() = default;

    unsigned getHash() const {
        return hash;
    }

    virtual void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const = 0;

    virtual void getIllumination(Object3D* group, const Vector3f &p, const Vector3f &n,
                                 Vector3f &dir, Vector3f &col, unsigned& hash) const = 0;
    
    virtual void getSampledIllumination(Object3D* group, const Vector3f &p, const Vector3f &n,
                                 Vector3f &dir, Vector3f &col) const = 0;

    virtual bool intersect(const Ray &r, Hit &h, double tmin) = 0;

protected:
    unsigned hash;
};


class DirectionalLight : public Light {
public:
    DirectionalLight() = delete;

    DirectionalLight(const Vector3f &d, const Vector3f &c) : Light() {
        direction = d.normalized();
        color = c;
    }

    ~DirectionalLight() override = default;

    void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const override {
        // the direction to the light is the opposite of the
        // direction of the directional light source
        dir = -direction;
        col = color;
    }

    // For shading and hashing
    void getIllumination(Object3D* group, const Vector3f &p, const Vector3f &n, 
                         Vector3f &dir, Vector3f &col, unsigned& hash) const override {
        dir = -direction;
        Ray light_ray(p + 2 * feps * n, dir);
		Hit light_hit;
        bool isIntersect = group->intersect(light_ray, light_hit, feps);
        col = isIntersect ? Vector3f::ZERO : color;
        hash = isIntersect ? hash * 13 + light_hit.getHash() : hash;
    }

    bool intersect(const Ray &r, Hit &h, double tmin) override {
        return false;
    }

    void getSampledIllumination(Object3D* group, const Vector3f &p, const Vector3f &n, 
                         Vector3f &dir, Vector3f &col) const override {
        unsigned hash;
        getIllumination(group, p, n, dir, col, hash);
    }

private:

    Vector3f direction;
    Vector3f color;

};

class PointLight : public Light {
public:
    PointLight() = delete;

    PointLight(const Vector3f &p, const Vector3f &c) : Light() {
        position = p;
        color = c;
    }

    ~PointLight() override = default;

    void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const override {
        dir = (position - p).normalized();
        col = color;
    }

    // For shading and hashing
    void getIllumination(Object3D* group, const Vector3f &p, const Vector3f &n, 
                         Vector3f &dir, Vector3f &col, unsigned& hash) const override {
        dir = (position - p).normalized();
        Ray light_ray(p + 2 * feps * n, dir);
		Hit light_hit;
        bool isIntersect = group->intersect(light_ray, light_hit, feps);
        if (!isIntersect || light_hit.getT() >= (position - p).length())
            col = color;
        else{
            col = Vector3f::ZERO;
            hash = isIntersect ? hash * 13 + light_hit.getHash() : hash;
        }
    }

    bool intersect(const Ray &r, Hit &h, double tmin) override {
        return false;
    }

    void getSampledIllumination(Object3D* group, const Vector3f &p, const Vector3f &n, 
                         Vector3f &dir, Vector3f &col) const override {
        unsigned hash;
        getIllumination(group, p, n, dir, col, hash);
    }

private:

    Vector3f position;
    Vector3f color;

};

class AreaLight : public Light {
public:
    AreaLight() = delete;

    AreaLight(const Vector3f &pos, const Vector3f& dx, const Vector3f& dy, const Vector3f &c, int res) : Light() {
        position = pos;
        deltax = dx; 
        //deltay = dy - Vector3f::dot(dy, dx) / dx.squaredLength() * dx;
        deltay = dy;
        color = c;
        resolution = res;
    }

    ~AreaLight() override = default;

    void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const override {
        dir = (position + deltax / 2 + deltay / 2 - p).normalized();
        col = color;
    }

    // For shading and hashing
    void getIllumination(Object3D* group, const Vector3f &p, const Vector3f &n, 
                         Vector3f &dir, Vector3f &col, unsigned& hash) const override {
        dir = (position + deltax / 2 + deltay / 2 - p).normalized();
        int light_count = 0;
        for (int i = 0; i < resolution; ++i){
            for(int j = 0; j < resolution; ++j){
                Vector3f light_point = position \
                                     + (deltax * i / (resolution - 1))\
                                     + (deltay * j / (resolution - 1));
                Ray light_ray(p + 2 * feps * n, (light_point - p).normalized());
                Hit light_hit;
                bool isIntersect = group->intersect(light_ray, light_hit, feps);
                if (!isIntersect || light_hit.getT() >= (light_point - p).length())
                    light_count += 1;
            }
        }
        col = color * light_count / (resolution * resolution);
    }

    bool intersect(const Ray &r, Hit &h, double tmin) override {
        Vector3f orig = r.getOrigin(), dir = r.getDirection();
        Vector3f normal = Vector3f::cross(deltax, deltay).normalized();
        double d = -Vector3f::dot(position, normal), n_dot_d = Vector3f::dot(normal, dir);
        if (n_dot_d <= feps && n_dot_d >= -feps)
            return false;
        double t = -(d + Vector3f::dot(normal, orig)) / n_dot_d;
        if (t >= h.getT() || t < tmin || t <= 0) 
            return false;
        double r1 = Vector3f::dot((r.pointAtParameter(t) - position), deltax) / deltax.squaredLength();
	    double r2 = Vector3f::dot((r.pointAtParameter(t) - position), deltay) / deltay.squaredLength();
        if (r1 > 1 - feps || r2 > 1 - feps || r1 < feps || r2 < feps)
		    return false;
        Vector3f n = n_dot_d > 0 ? -normal : normal;
        h.set(t, nullptr, n, color, true, getHash());
        return true;
    }

    void getSampledIllumination(Object3D* group, const Vector3f &p, const Vector3f &n, 
                                   Vector3f &dir, Vector3f &col) const {
        // random sample
        double x = rand() / double(RAND_MAX), y = rand() / double(RAND_MAX);
        Vector3f light_point = position + x * deltax + y * deltay;
        // compute normal
        Vector3f light_normal = Vector3f::cross(deltax, deltay);
        double light_area = light_normal.length();
        light_normal.normalize();
        // compute the vector pointing to light
        dir = light_point - p;
        double squared_length = dir.squaredLength();
        dir.normalize();
        // whether in shade
        Ray light_ray(p + 2 * feps * n, dir);
		Hit light_hit;
        bool isIntersect = group->intersect(light_ray, light_hit, feps);
        if (!isIntersect || light_hit.getT() >= (light_point - p).length())
            col = color  * std::max(Vector3f::dot(light_normal, -dir), 0.0);// * light_area / squared_length;
        else
            col = Vector3f::ZERO;
    }

private:

    Vector3f position;
    Vector3f deltax;
    Vector3f deltay;
    Vector3f color;
    int resolution;
};

class SpotLight : public Light {
public:
    SpotLight() = delete;

    SpotLight(const Vector3f &p, const Vector3f &c, const Vector3f& dir, double s) : Light() {
        position = p;
        color = c;
        direction = dir.normalized();
        shininess = s;
    }

    ~SpotLight() override = default;

    void getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const override {
        dir = (position - p).normalized();
        float s_factor = pow(std::max(Vector3f::dot(direction, -dir), 0.0), shininess);
        col = s_factor * color;
    }

    // For shading and hashing
    void getIllumination(Object3D* group, const Vector3f &p, const Vector3f &n, 
                         Vector3f &dir, Vector3f &col, unsigned& hash) const override {
        dir = (position - p).normalized();
        Ray light_ray(p + 2 * feps * n, dir);
		Hit light_hit;
        bool isIntersect = group->intersect(light_ray, light_hit, feps);
        if (!isIntersect || light_hit.getT() >= (position - p).length()){
            float s_factor = pow(std::max(Vector3f::dot(direction, -dir), 0.0), shininess);
            col = s_factor * color;
        }
        else{
            col = Vector3f::ZERO;
            hash = isIntersect ? hash * 13 + light_hit.getHash() : hash;
        }
    }

    bool intersect(const Ray &r, Hit &h, double tmin) override {
        return false;
    }

    void getSampledIllumination(Object3D* group, const Vector3f &p, const Vector3f &n, 
                         Vector3f &dir, Vector3f &col) const override {
        unsigned hash;
        getIllumination(group, p, n, dir, col, hash);
    }

private:

    Vector3f position;
    Vector3f color;
    Vector3f direction;
    float shininess;
};

#endif // LIGHT_H
