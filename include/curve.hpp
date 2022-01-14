#ifndef CURVE_HPP
#define CURVE_HPP

#include "object3d.hpp"
#include <vecmath.h>
#include <vector>
#include <utility>

#include <algorithm>
#include <cstdlib>
#include <ctime>

// The CurvePoint object stores information about a point on a curve
// after it has been tesselated: the vertex (V) and the tangent (T)
// It is the responsiblility of functions that create these objects to fill in all the data.
struct CurvePoint {
    Vector3f V; // Vertex
    Vector3f T; // Tangent  (unit)
};

class Curve : public Object3D {
protected:
    std::vector<Vector3f> controls;
    int k_;

public:
    double ymin, ymax, radius;
    double range[2];

    explicit Curve(std::vector<Vector3f> points) : controls(std::move(points)) {
        ymin = 1e38, ymax = -1e38;
        radius = 0;
        for (auto &point : controls) {
            ymin = std::min(point.y(), ymin);
            ymax = std::max(point.y(), ymax);
            radius = std::max(radius, fabs(point.x()));
            radius = std::max(radius, fabs(point.z()));
        }
    }

    bool intersect(const Ray &r, Hit &h, double tmin) override {
        return false;
    }

    std::vector<Vector3f> &getControls() {
        return controls;
    }

    bool checkRange(double t){
        if (t < range[0] - feps || t > range[1] + feps) return false;
        return true; 
    }

    virtual CurvePoint getVT(double t) = 0;
    virtual void discretize(int resolution, std::vector<CurvePoint>& data, std::vector<double>& ts) = 0;
};

class BezierCurve : public Curve {
public:
    explicit BezierCurve(const std::vector<Vector3f> &points) : Curve(points) {
        // control points: k + 1
        range[0] = 0.0; range[1] = 1.0;
        k_ = points.size() - 1;
        for (int i = 0; i < k_; ++i)
            t_controls.push_back(controls[i+1] - controls[i]);
    }

    Vector3f de_Casteljau(const std::vector<Vector3f>& points, double t){
        int k = points.size() - 1;
        std::vector<Vector3f> cur_points(points), next_points;
        for(int i = 0; i < k; ++i){
            next_points.clear();
            for(int c = 0; c < k - i; ++c){
               next_points.push_back((1 - t) * cur_points[c] + t * cur_points[c+1]);
            }
            cur_points.assign(next_points.begin(), next_points.end());
        }
        return cur_points[0];
    }

    CurvePoint getVT(double t){
        CurvePoint cur_curvepoint = {de_Casteljau(controls, t), k_ * de_Casteljau(t_controls, t)};
        return cur_curvepoint;
    }

    void discretize(int resolution, std::vector<CurvePoint>& data, std::vector<double>& ts) override {
        data.clear(); ts.clear();
        for(int r = 0; r < resolution; r++){
            double t = double(r) / (resolution - 1);
            data.push_back(getVT(t));
            ts.push_back(t);        
        }
    }

protected:
    std::vector<Vector3f> t_controls;
};


class BsplineCurve : public Curve {
public:
    BsplineCurve(const std::vector<Vector3f> &points) : Curve(points) {
        k_ = 3;
        for(int i = 0; i < controls.size() + k_ + 1; ++i)
            knot_.push_back(double(i) / (controls.size() + k_));
        range[0] = knot_[k_];
        range[1] = knot_[controls.size()];
        if (points.size() < 4) {
            printf("Number of control points of BspineCurve must be more than 4!\n");
            exit(0);
        }
    }

    std::vector<double> de_Boor_Cox(const std::vector<double>& knot, double t, int k){
        int num_knot = knot.size();
        std::vector<double> cur_base, next_base;
        for (int i = 0; i < num_knot - 1; ++i){
            double v = (t >= knot[i]) && (t < knot[i+1]) ? 1.0f : 0.0f;
            cur_base.push_back(v);
        }
        for (int p = 1; p <= k; ++p){
            next_base.clear();
            for(int i = 0; i < num_knot - p - 1; ++i){
                double v = (t - knot[i]) * cur_base[i] / (knot[i+p] - knot[i]) \
                        + (knot[i+p+1] - t) * cur_base[i+1] / (knot[i+p+1] - knot[i+1]);
                next_base.push_back(v);
            }
            cur_base.assign(next_base.begin(), next_base.end());
        }
        return cur_base;
    }

    CurvePoint getVT(double t){
        Vector3f V = Vector3f::ZERO, T = Vector3f::ZERO;
        for(int rr = k_; rr < controls.size(); ++rr)
            if(t >= knot_[rr] && t < knot_[rr+1]){
                std::vector<Vector3f> cur_controls(controls.begin() + rr - k_, controls.begin() + rr + 1);
                std::vector<double> cur_knot(knot_.begin() + rr - k_, knot_.begin() + rr + k_ + 2);
                std::vector<double> cur_base = de_Boor_Cox(cur_knot, t, k_);
                std::vector<double> cur_t_base = de_Boor_Cox(cur_knot, t, k_ - 1);
                for (int i = 0; i < cur_controls.size(); ++i){
                    V += cur_controls[i] * cur_base[i];
                    T += k_ * (cur_t_base[i] / (cur_knot[i+k_] - cur_knot[i]) \
                              - cur_t_base[i+1] / (cur_knot[i+k_+1] - cur_knot[i+1])) * cur_controls[i];  
                }
                break;
            }
        CurvePoint cur_curvepoint = {V, T};
        return cur_curvepoint;
    }

    void discretize(int resolution, std::vector<CurvePoint>& data, std::vector<double>& ts) override {
        data.clear(); ts.clear();
        for (int rr = k_; rr < controls.size(); ++rr){
            std::vector<Vector3f> cur_controls(controls.begin() + rr - k_, controls.begin() + rr + 1);
            std::vector<double> cur_knot(knot_.begin() + rr - k_, knot_.begin() + rr + k_ + 2);
            for(int r = 0; r < resolution; r++){
                double t = knot_[rr] + double(r) / (resolution - 1) * (knot_[rr+1] - knot_[rr]);
                std::vector<double> cur_base = de_Boor_Cox(cur_knot, t, k_);
                std::vector<double> cur_t_base = de_Boor_Cox(cur_knot, t, k_ - 1);
                Vector3f V = Vector3f::ZERO, T = Vector3f::ZERO;
                for (int i = 0; i < cur_controls.size(); ++i){
                    V += cur_controls[i] * cur_base[i];
                    T += k_ * (cur_t_base[i] / (cur_knot[i+k_] - cur_knot[i]) \
                              - cur_t_base[i+1] / (cur_knot[i+k_+1] - cur_knot[i+1])) * cur_controls[i];  
                }
                CurvePoint cur_curvepoint = {V, T};
                data.push_back(cur_curvepoint);
                ts.push_back(t);
            }
        }
    }  

protected:
    std::vector<double> knot_;
};

#endif // CURVE_HPP
