#ifndef MATERIAL_H
#define MATERIAL_H

#include <cassert>
#include <vecmath.h>

#include "ray.hpp"
#include "hit.hpp"
#include "texture.hpp"
#include <iostream>
#include <algorithm>

class BRDF {
public:
    double diffuse, specular, refraction;
    double rho_d, rho_s, phong_s;
    double refractiveIndex;
    //BRDF() {}
    BRDF(double diff=1.0, double spec=0.0, double refr=0.0, 
         double r_d=1.0, double r_s=0.0, double p_s=0.0, double refr_i=1.0) : 
    diffuse(diff), specular(spec), refraction(refr), rho_d(r_d), rho_s(r_s), phong_s(p_s), refractiveIndex(refr_i) {}
};


class Material {
public:

    // Only for Phong model
    explicit Material(const Vector3f &d_color, const Vector3f &s_color=Vector3f::ZERO, double s=0) :
            diffuseColor(d_color), specularColor(s_color), brdf(BRDF()), texture(nullptr) {
                brdf.phong_s = s;
            }

    explicit Material(double diff=1.0, double spec=0.0, double refr=0.0, 
                      double r_d=1.0, double r_s=0.0, double p_s=0.0, double refr_i=1.0, 
                      Texture *t=nullptr, Texture *b=nullptr) :
            brdf(BRDF(diff, spec, refr, r_d, r_s, p_s, refr_i)), texture(t), bump(b) {}
        
    explicit Material(const BRDF &_brdf, Texture *t=nullptr, Texture* b=nullptr) : 
            brdf(_brdf), texture(t), bump(b) {}

    virtual ~Material() = default;

    // Only for Phong model
    virtual Vector3f getDiffuseColor() const {
        return diffuseColor;
    }

    Texture* getTexture(){
        return texture;
    }

    Texture* getBump(){
        return bump;
    }

    Vector3f getColor(){
        return texture->getColor();
    }
    Vector3f getColor(double u, double v){
        return texture->getColor(u, v);
    }

    const BRDF& getBRDF(){
        return brdf;
    }

    // Only for Phong model
    Vector3f Shade(const Ray &ray, const Hit &hit,
                   const Vector3f &dirToLight, const Vector3f &lightColor) {
        Vector3f shaded = Vector3f::ZERO;
        // Diffuse
        if (texture) shaded += brdf.diffuse * brdf.rho_d * getColor() * std::max(Vector3f::dot(dirToLight, hit.getNormal()), 0.0);
        else shaded += diffuseColor * std::max(Vector3f::dot(dirToLight, hit.getNormal()), 0.0);
        // Specular
        Vector3f R = 2 * Vector3f::dot(hit.getNormal(), dirToLight) * hit.getNormal() - dirToLight;
        if (texture) shaded += brdf.specular * brdf.rho_s * pow(std::max(Vector3f::dot(-ray.getDirection(), R), 0.0), brdf.phong_s);
        else shaded += specularColor * pow(std::max(Vector3f::dot(-ray.getDirection(), R), 0.0), brdf.phong_s);
        // Color
        shaded = shaded * lightColor;
        return shaded;
    }

protected:
    BRDF brdf;
    Texture *texture, *bump;
    
    // Only for Phong model
    Vector3f diffuseColor;
    Vector3f specularColor;
};


#endif // MATERIAL_H
