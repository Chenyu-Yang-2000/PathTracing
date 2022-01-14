#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <iostream>

#include "scene_parser.hpp"
#include "image.hpp"
#include "camera.hpp"
#include "group.hpp"
#include "light.hpp"
#include "raytracing.hpp"
#include "pathtracing.hpp"

#include <string>

using namespace std;

int main(int argc, char *argv[]) {
    for (int argNum = 1; argNum < argc; ++argNum) {
        std::cout << "Argument " << argNum << " is: " << argv[argNum] << std::endl;
    }

    if (argc != 4) {
        cout << "Usage: ./bin/PT <renderer> <input scene file> <output bmp file>" << endl;
        return 1;
    }
    string inputFile = argv[2];
    string outputFile = argv[3];  // only bmp is allowed.

    SceneParser sceneParser(inputFile.data());
    Camera *camera = sceneParser.getCamera();
    Image image(camera->getWidth(), camera->getHeight());

    if (strcmp(argv[1], "raytracing") == 0){
        RayTracing renderer(&sceneParser, camera, &image, outputFile);
        renderer.render();
    }
    else if (strcmp(argv[1], "pathtracing") == 0){
        PathTracing renderer(&sceneParser, camera, &image, outputFile);
        renderer.render();
    }
    else {
        cout << "NotImplementedError" << endl;
        return 1;
    }

    cout << "Rendering finished!" << endl;
    return 0;
}
