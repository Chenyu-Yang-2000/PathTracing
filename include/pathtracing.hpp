#pragma once
#ifndef PATHTRACING_H
#define PATHTRACING_H

#include <cstring>
#include "scene_parser.hpp"
#include "render.hpp"
#include "hit.hpp"

class PathTracing : public Render {
public:
    PathTracing() {};
    ~PathTracing() override = default;

    PathTracing(SceneParser* _scene, Camera* _camera, Image* _image, std::string _save_path, 
                int _max_depth=12, int _num_iters = 20000, int _save_freq=10, double _stop_p=0.5) : 
        Render(_scene, _camera, _image, _save_path) {
            max_depth = _max_depth; num_iters = _num_iters; save_freq = _save_freq; stop_p = _stop_p;
        }

    void render();

    Vector3f pathTrace(const Ray& r, int depth, bool cal_light=true);
    Vector3f calDiffusion(const Ray& r, Hit& h, int depth);
    Vector3f calReflection(const Ray& r, Hit& h, int depth);
    Vector3f calRefraction(const Ray& r, Hit& h, int depth);
    bool lightIntersect(const Ray& r, Hit& h, double tmin);
    Vector3f sampleRandomRay(const Vector3f& norm);

private:
    int max_depth, num_iters, save_freq;
    double stop_p;
};

#endif //RAYTRACING_H