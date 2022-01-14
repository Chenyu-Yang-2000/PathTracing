#pragma once
#ifndef RAYTRACING_H
#define RAYTRACING_H

#include <cstring>
#include "scene_parser.hpp"
#include "render.hpp"
#include "hit.hpp"

class RayTracing : public Render {
public:
    RayTracing() {};
    ~RayTracing() override = default;

    RayTracing(SceneParser* _scene, Camera* _camera, Image* _image, std::string _save_path, int _max_depth=6) : 
        Render(_scene, _camera, _image, _save_path) {
            max_depth = _max_depth;
        }

    void render();

    Vector3f rayTrace(const Ray& r, int depth, unsigned& hash);
    Vector3f calDiffusion(const Ray& r, Hit& h, unsigned& hash);
    Vector3f calReflection(const Ray& r, Hit& h, int depth, unsigned& hash);
    Vector3f calRefraction(const Ray& r, Hit& h, int depth, unsigned& hash);
    bool lightIntersect(const Ray& r, Hit& h, double tmin);

private:
    int max_depth;
};

#endif //RAYTRACING_H