#pragma once

#include <vector>
#include <map>
#include <iostream>
#include "ray.hpp"
#include "hit.hpp"
#include "triangle.hpp"

#define MAX_FACES 8
#define MAX_DEPTH 24

using namespace std;

struct KDTreeNode {
    Vector3f minCoord, maxCoord;
    vector<Triangle*> faces;
    KDTreeNode *lc, *rc;
    bool inside(const Triangle *);
};

class KDTree {
    Vector3f *vertexes;
    KDTreeNode *build(int depth, int d, const vector<Triangle*> &faces, 
                      const Vector3f& minCoord, const Vector3f& maxCoord);
public:
    KDTreeNode *root;
    KDTree(const vector<Triangle*> &faces);
    KDTree(const vector<Triangle*> &faces, const Vector3f& minCoord, const Vector3f& maxCoord);
    double getCuboidIntersection(KDTreeNode *p, const Ray &ray);
    bool intersect(KDTreeNode *p, const Ray &ray, Hit &hit, double tmin);
};