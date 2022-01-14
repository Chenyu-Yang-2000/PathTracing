#pragma once
#ifndef RENDER_H
#define RENDER_H

#include <cstring>
#include "scene_parser.hpp"
#include "image.hpp"
#include "camera.hpp"

class Render {
protected:
    SceneParser* scene;
    Camera* camera;
    Image* image;
    std::string save_path;

public:
    Render() : scene(nullptr), camera(nullptr), image(nullptr) {}
    virtual ~Render() = default;

    explicit Render(SceneParser* _scene, Camera* _camera, Image* _image, const std::string _save_path) : 
        scene(_scene), camera(_camera), image(_image), save_path(_save_path) {}
    virtual void render() = 0;
};

#endif //RENDER_H