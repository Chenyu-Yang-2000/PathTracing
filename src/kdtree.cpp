#include "kdtree.hpp"
#include "triangle.hpp"

bool KDTreeNode::inside(const Triangle *face) {
    Vector3f faceMin = face->min();
    Vector3f faceMax = face->max();
    return (faceMin.x() < maxCoord.x() || faceMin.x() == maxCoord.x() && faceMin.x() == faceMax.x())
        && (faceMax.x() > minCoord.x() || faceMax.x() == minCoord.x() && faceMin.x() == faceMax.x())
        && (faceMin.y() < maxCoord.y() || faceMin.y() == maxCoord.y() && faceMin.y() == faceMax.y())
        && (faceMax.y() > minCoord.y() || faceMax.y() == minCoord.y() && faceMin.y() == faceMax.y())
        && (faceMin.z() < maxCoord.z() || faceMin.z() == maxCoord.z() && faceMin.z() == faceMax.z())
        && (faceMax.z() > minCoord.z() || faceMax.z() == minCoord.z() && faceMin.z() == faceMax.z());
}

KDTreeNode *KDTree::build(int depth, int d, const vector<Triangle*> &faces, 
                          const Vector3f& minCoord, const Vector3f& maxCoord) {
    KDTreeNode *p = new KDTreeNode;
    p->minCoord = minCoord, p->maxCoord = maxCoord;
    Vector3f maxL, minR;
    if (d == 0) {
        maxL = Vector3f((p->minCoord.x() + p->maxCoord.x()) / 2, p->maxCoord.y(), p->maxCoord.z());
        minR = Vector3f((p->minCoord.x() + p->maxCoord.x()) / 2, p->minCoord.y(), p->minCoord.z());
    }
    else if (d == 1) {
        maxL = Vector3f(p->maxCoord.x(), (p->minCoord.y() + p->maxCoord.y()) / 2, p->maxCoord.z());
        minR = Vector3f(p->minCoord.x(), (p->minCoord.y() + p->maxCoord.y()) / 2, p->minCoord.z());
    }
    else {
        maxL = Vector3f(p->maxCoord.x(), p->maxCoord.y(), (p->minCoord.z() + p->maxCoord.z()) / 2);
        minR = Vector3f(p->minCoord.x(), p->minCoord.y(), (p->minCoord.z() + p->maxCoord.z()) / 2);
    }
    p->faces.clear();
    for (auto face : faces)
        if (p->inside(face))
            p->faces.push_back(face);
    if (p->faces.size() > MAX_FACES && depth < MAX_DEPTH) {
        p->lc = build(depth + 1, (d + 1) % 3, p->faces, minCoord, maxL);
        p->rc = build(depth + 1, (d + 1) % 3, p->faces, minR, maxCoord);
        
        vector<Triangle*> faceL = p->lc->faces, faceR = p->rc->faces;
        std::map<Triangle*, int> cnt;
        for (auto face : faceL) cnt[face]++;
        for (auto face : faceR) cnt[face]++;
        p->lc->faces.clear();
        p->rc->faces.clear();
        p->faces.clear();
        for (auto face : faceL)
            if (cnt[face] == 1)
                p->lc->faces.push_back(face);
            else
                p->faces.push_back(face);
        for (auto face : faceR)
            if (cnt[face] == 1)
                p->rc->faces.push_back(face);
    }
    else
        p->lc = p->rc = nullptr;
    return p;
}

KDTree::KDTree(const vector<Triangle*> &faces) {
    Vector3f minCoord = Vector3f(1e100);
    Vector3f maxCoord = Vector3f(-1e100);        
    for (auto face : faces) {
        minCoord = ::min(minCoord, face->min());
        maxCoord = ::max(maxCoord, face->max());
    }
    root = build(1, 0, faces, minCoord, maxCoord);
}

KDTree::KDTree(const vector<Triangle*> &faces, const Vector3f& minCoord, const Vector3f& maxCoord){
    root = build(1, 0, faces, minCoord, maxCoord);
}

double KDTree::getCuboidIntersection(KDTreeNode *p, const Ray &ray) {
    Vector3f orig = ray.getOrigin(), dir = ray.getDirection();
    if (!(orig >= p->minCoord && orig <= p->maxCoord)) { // outside
        double t = -1e100;
        if (fabs(dir.x()) > 0)
            t = std::max(t, std::min((p->minCoord.x() - orig.x()) / dir.x(), (p->maxCoord.x() - orig.x()) / dir.x()));
        if (fabs(dir.y()) > 0)
            t = std::max(t, std::min((p->minCoord.y() - orig.y()) / dir.y(), (p->maxCoord.y() - orig.y()) / dir.y()));
        if (fabs(dir.z()) > 0)
            t = std::max(t, std::min((p->minCoord.z() - orig.z()) / dir.z(), (p->maxCoord.z() - orig.z()) / dir.z()));
        if (t < -feps) return 1e100;
        Vector3f pp = orig + dir * t;
        if (!(pp >= p->minCoord && pp <= p->maxCoord)) return 1e100;
        return t;
    }
    else return -1e100;
}

bool KDTree::intersect(KDTreeNode *p, const Ray &ray, Hit &hit, double tmin) {
    bool isIntersect = false;
    for (auto face : p->faces) {
        isIntersect |= face->intersect(ray, hit, tmin);
    }
    
    double tl = p->lc ? getCuboidIntersection(p->lc, ray) : 1e100;
    double tr = p->rc ? getCuboidIntersection(p->rc, ray) : 1e100;
    if (tl < tr) {
        if (hit.getT() <= tl) return isIntersect;
        if (p->lc) isIntersect |= intersect(p->lc, ray, hit, tmin);
        if (hit.getT() <= tr) return isIntersect;
        if (p->rc) isIntersect |= intersect(p->rc, ray, hit, tmin);
    }
    else {
        if (hit.getT() <= tr) return isIntersect;
        if (p->rc) isIntersect |= intersect(p->rc, ray, hit, tmin);
        if (hit.getT() <= tl) return isIntersect;
        if (p->lc) isIntersect |= intersect(p->lc, ray, hit, tmin);            
    }
    return isIntersect;
}