#ifndef BOUND_H
#define BOUND_H

#include <vecmath.h>
#include <cmath>

// Cubic bounding box

class Bound{
public:
    Bound() {
        bounds[0] = Vector3f(1e38);
        bounds[1] = Vector3f(-1e38);
    }

    Bound(const Vector3f &vertex_0, const Vector3f &vertex_1){
        bounds[0] = vertex_0;
        bounds[1] = vertex_1;
    }

    ~Bound() = default;

    void set(const Vector3f &vertex_0, const Vector3f &vertex_1){
        bounds[0] = vertex_0;
        bounds[1] = vertex_1;
    }

    Vector3f getCentor(){
        return (bounds[0] + bounds[1]) / 2;
    }

    bool intersect(const Ray &r, double &t_hit){
        t_hit = 1e38;
        Vector3f origin(r.getOrigin()), invdir(1 / r.getDirection());
        double t_min = -1e38, t_max = 1e38, t_inter[2];
        for(int i=0; i<3; ++i){
            for(int j=0; j<2; ++j)
                t_inter[j] = (bounds[j][i] - origin[i]) * invdir[i];
            t_min = std::max(t_min, std::min(t_inter[0], t_inter[1]));
            t_max = std::min(t_max, std::max(t_inter[0], t_inter[1]));
        } 
        if (t_max < -feps) return false;
        if (t_min > t_max + feps) return false;
        t_hit = t_min;    
        return true;
    }

protected:
    Vector3f bounds[2];
};

#endif //PLANE_H
		

