#include "mesh.hpp"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cstdlib>
#include <utility>
#include <sstream>

bool Mesh::intersect(const Ray &r, Hit &h, double tmin) {
    double t;
        if (!bound.intersect(r, t) || t > h.getT()) return false;
    // Optional: Change this brute force method into a faster one.
    /*bool result = false;
    for (auto trig : faces)
        result |= trig->intersect(r, h, tmin);
    return result;*/
    return kdtree->intersect(kdtree->root, r, h, tmin);
}

Mesh::Mesh(const char *filename, Material *material) : Object3D(material) {

    // Optional: Use tiny obj loader to replace this simple one.
    std::ifstream f;
    f.open(filename);
    if (!f.is_open()) {
        std::cout << "Cannot open " << filename << "\n";
        return;
    }
    std::string line;
    std::string vTok("v");
    std::string fTok("f");
    std::string texTok("vt");
    std::string vnTok("vn");
    char bslash = '/', space = ' ';
    std::string tok;
    bool useVT = false, useVN = false;
    minCoord = Vector3f(1e38); maxCoord = Vector3f(-1e38);
    //int texID;
    while (true) {
        std::getline(f, line);
        if (f.eof()) {
            break;
        }
        if (line.size() < 3) {
            continue;
        }
        if (line.at(0) == '#') {
            continue;
        }
        std::stringstream ss(line);
        ss >> tok;
        if (tok == vTok) {
            Vector3f vec;
            ss >> vec[0] >> vec[1] >> vec[2];
            v.push_back(vec);
            minCoord = ::min(minCoord, vec);
            maxCoord = ::max(maxCoord, vec);
        }  else if (tok == texTok) {
            useVT = true;
            Vector2f texcoord;
            ss >> texcoord[0];
            ss >> texcoord[1];
            vt.push_back(texcoord);
        } else if (tok == vnTok) {
            useVN = true;
            Vector3f vec;
            ss >> vec[0] >> vec[1] >> vec[2];
            vn.push_back(vec);
        } else if (tok == fTok) {
            TriangleIndex triID, vtID, vnID;
            std::string index_first, index;
			std::vector<string> index_vec;
            ss >> index_first;
			while (ss >> index) index_vec.push_back(index);
            for (int j = 1; j < index_vec.size(); j++){
                for(int i = 0; i < 3; ++i){
                    if (i == 0) index = index_first;
                    else if (i == 1) index = index_vec[j-1];
                    else if (i == 2) index = index_vec[j];
                    if (index.find(bslash) != std::string::npos)
                        std::replace(index.begin(), index.end(), bslash, space);
                    std::stringstream indexss(index);
                    indexss >> triID[i]; triID[i]--;
                    if (useVT) {indexss >> vtID[i]; vtID[i]--;}
                    if (useVN) {indexss >> vnID[i]; vnID[i]--;}
                }
                t.push_back(triID);
                if(useVT) vtIndex.push_back(vtID);
                if(useVN) vnIndex.push_back(vnID);
            }
        }
    }

    f.close();
    
    // shift the center to (0, 0, 0)
    Vector3f center = 0.5 * (minCoord + maxCoord);
    for(int i = 0; i < v.size(); ++i)
        v[i] -= center;
    minCoord -= center; maxCoord -= center;
    //printf("min: x: %.4f, y: %.4f, z: %.4f\n", minCoord[0], minCoord[1], minCoord[2]);

    // set bounding box
    bound.set(minCoord - 10 * feps, maxCoord + 10 * feps);

    for(int i = 0; i < t.size(); ++i){
        Triangle* trig = new Triangle(v[t[i][0]], v[t[i][1]], v[t[i][2]], material);
        if (useVT)
            trig->setVT(vt[vtIndex[i][0]], vt[vtIndex[i][1]], vt[vtIndex[i][2]]);
        if (useVN)
            trig->setVNormal(vn[vnIndex[i][0]], vn[vnIndex[i][1]], vn[vnIndex[i][2]]);
        faces.push_back(trig);
    }

    // Build KD-Tree
    kdtree = new KDTree(faces, minCoord, maxCoord);
}

Mesh::Mesh(const std::vector<Vector3f>& VV, const std::vector<Vector3f>& VN, 
           const std::vector<TriangleIndex>& VF, Material *material) : Object3D(material) {
    v.assign(VV.begin(), VV.end());
    vn.assign(VN.begin(), VN.end());
    t.assign(VF.begin(), VF.end());
    
    // set bounding box
    minCoord = Vector3f(1e38); maxCoord = Vector3f(-1e38);
    for(int i = 0; i < v.size(); ++i){
        minCoord = ::min(v[i], minCoord);
        maxCoord = ::max(v[i], maxCoord);
    }
    bound.set(minCoord - 10 * feps, maxCoord + 10 * feps);

    // build faces
    for(int i = 0; i < t.size(); ++i){
        Triangle* trig = new Triangle(v[t[i][0]], v[t[i][1]], v[t[i][2]], material);
        trig->setVNormal(vn[t[i][0]], vn[t[i][1]], vn[t[i][2]]);
        faces.push_back(trig);
    }

    // Build KD-Tree
    kdtree = new KDTree(faces, minCoord, maxCoord);
}