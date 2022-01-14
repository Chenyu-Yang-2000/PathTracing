#ifndef REVSURFACE_HPP
#define REVSURFACE_HPP

#include "object3d.hpp"
#include "curve.hpp"
#include "bound.hpp"
#include <tuple>
#include <random>
#include "mesh.hpp"

class RevSurface : public Object3D {

    Curve *pCurve;
    Bound bound;
    Mesh* discrete_face;
    std::vector<double> mus;

public:
    RevSurface(Curve *pCurve, Material* material) : pCurve(pCurve), Object3D(material) {
        // Check flat.
        for (const auto &cp : pCurve->getControls()) {
            if (cp.z() != 0.0) {
                printf("Profile of revSurface must be flat on xy plane.\n");
                exit(0);
            }
        }
        bound.set(Vector3f(-pCurve->radius, pCurve->ymin - 10 * feps, -pCurve->radius - 10 * feps),
                  Vector3f(pCurve->radius, pCurve->ymax + 10 * feps, pCurve->radius + 10 * feps));

        discrete_face = discretize();
    }

    ~RevSurface() override {
        delete pCurve;
    }

    bool intersect(const Ray &r, Hit &h, double tmin) override {
        double t, theta, mu;
        if (!bound.intersect(r, t) || t > h.getT()) return false;
        // Discrete face
        // return discrete_face->intersect(r, h, tmin);
        getInitialPara(r, t, theta, mu);
        Vector3f normal, point;
        if (!solveNewton(r, tmin, t, theta, mu, normal, point)) return false;
        // Multiple iterations with random perturbation
        // if (!solve(r, tmin, t, theta, mu, normal, point, 5)) return false;
        if (!isnormal(mu) || !isnormal(theta) || !isnormal(t)) return false;
        if (t < tmin || t > h.getT() || mu < pCurve->range[0] || mu > pCurve->range[1]) return false;
        theta = fmod(fmod(theta, 2 * M_PI) + 2 * M_PI, 2 * M_PI);
        bool outf = Vector3f::dot(r.getDirection(), normal) < 0 ? true : false;
		normal *= outf ? 1 : -1;
        h.set(t, material, normal.normalized(), 
              material->getColor(theta / (2 * M_PI), (mu - pCurve->range[0]) / (pCurve->range[1] - pCurve->range[0])),
              outf, getIndex());
        return true;
    }

    bool solve(const Ray &r, const double tmin, double &t, double &theta, double &mu, 
               Vector3f &normal, Vector3f &point, int iters){
        double t_f = 1e38, theta_f, mu_f;
        bool isIntersect = false;
        for (int i = 0; i < iters; ++i){
            double _t = t, _theta = theta, _mu = mu;
            randomPerturb(_t, _theta, _mu);
            Vector3f _normal, _point;
            if (solveNewton(r, tmin, _t, _theta, _mu, _normal, _point) && _t < t_f){
                isIntersect = true; t_f = _t;
                theta_f = _theta; mu_f = _mu; normal = _normal; point = _point;
            }
        }
        if (isIntersect) {
            t = t_f; theta = theta_f; mu = mu_f;
        }
        return isIntersect;
    }

    Vector3f getPoint(const double &theta, const double &mu, Vector3f &dtheta, Vector3f &dmu) {
        Vector3f point;
        Quat4f rot;
        rot.setAxisAngle(theta, Vector3f::UP);
        Matrix3f rotMat = Matrix3f::rotation(rot);
        CurvePoint cp = pCurve->getVT(mu);
        point = rotMat * cp.V;
        dmu = rotMat * cp.T;
        dtheta = Vector3f(-cp.V.x() * sin(theta), 0, -cp.V.x() * cos(theta));
        return point;
    }

    bool solveNewton(const Ray &r, const double tmin, double &t, double &theta, double &mu, 
                     Vector3f &normal, Vector3f &point, double lr=1) {
        Vector3f dmu, dtheta;
        for (int i = 0; i < 5; ++i) {
            if (mu >= pCurve->range[1]) mu = pCurve->range[1] - DBL_EPSILON;
            if (mu <= pCurve->range[0]) mu = pCurve->range[0] + DBL_EPSILON;
            point = getPoint(theta, mu, dtheta, dmu);
            Vector3f f = r.pointAtParameter(t) - point;
            normal = Vector3f::cross(dmu, dtheta);
            if (f.length() < feps && t >= tmin) return true;
            float D = Vector3f::dot(r.getDirection(), normal);
            t -= Vector3f::dot(dmu, Vector3f::cross(dtheta, f)) / D * lr;
            mu -= Vector3f::dot(r.getDirection(), Vector3f::cross(dtheta, f)) / D * lr;
            theta += Vector3f::dot(r.getDirection(), Vector3f::cross(dmu, f)) / D * lr;
        }
        return false;
    }
    
    void getInitialPara(const Ray &r, double &t, double &theta, double &mu) {
        Hit d_hit;
        if (discrete_face->intersect(r, d_hit, 0.01)){
            t = d_hit.getT();
            Vector3f point(r.pointAtParameter(t));
            theta = atan2(-point.z(), point.x()) + M_PI;
            mu = mus[d_hit.getHash()];
        }
        else {
            t = 0;
            theta = double(rand()) / RAND_MAX * 2 * M_PI;
            mu = double(rand()) / RAND_MAX * (pCurve->range[1] - pCurve->range[0]) + pCurve -> range[0];
        }
    }

    void randomPerturb(double &t, double &theta, double &mu){
        t += n(e) / 1000;
        theta += n(e) * M_PI / 1000;
        mu += n(e) * (pCurve->range[1] - pCurve->range[0]) / 1000;
    }

    Mesh* discretize(int resolution=30, int steps=60)  {
        // Surface is just a struct that contains vertices, normals, and
        // faces.  VV[i] is the position of vertex i, and VN[i] is the normal
        // of vertex i.  A face is a triple i,j,k corresponding to a triangle
        // with (vertex i, normal i), (vertex j, normal j), ...
        // Currently this struct is computed every time when canvas refreshes.
        // You can store this as member function to accelerate rendering.

        std::vector<Vector3f> VV, VN;
        std::vector<TriangleIndex> VF;

        std::vector<CurvePoint> curvePoints;
        std::vector<double> ts;
        pCurve->discretize(resolution, curvePoints, ts);
        for (unsigned int ci = 0; ci < curvePoints.size(); ++ci) {    
            const CurvePoint &cp = curvePoints[ci];
            for (unsigned int i = 0; i < steps; ++i) {
                float theta = (float) i / steps;
                Quat4f rot;
                rot.setAxisAngle(theta * 2 * 3.14159, Vector3f::UP);
                Vector3f pnew = Matrix3f::rotation(rot) * cp.V;
                Vector3f pNormal = Vector3f::cross(cp.T, -Vector3f::FORWARD);
                Vector3f nnew = Matrix3f::rotation(rot) * pNormal;
                VV.push_back(pnew);
                VN.push_back(nnew);
                int i1 = (i + 1 == steps) ? 0 : i + 1;
                if (ci != curvePoints.size() - 1) {
                    VF.emplace_back(TriangleIndex((ci + 1) * steps + i, ci * steps + i1, ci * steps + i));
                    mus.emplace_back(ts[ci]);
                    VF.emplace_back(TriangleIndex((ci + 1) * steps + i, (ci + 1) * steps + i1, ci * steps + i1));
                    mus.emplace_back(ts[ci]);
                }
            }
        }

        Mesh* d_face = new Mesh(VV, VN, VF, material);
        for (int i = 0; i < VF.size(); ++i)
            d_face->faces[i]->setIndex(i);
        return d_face;
    }

protected:
    std::default_random_engine e {(unsigned)time(0)};
    std::normal_distribution<double> n {0, 1};

};

#endif //REVSURFACE_HPP
