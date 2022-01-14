#include <cmath>
#include <algorithm>
#include <random>
#include <omp.h>

#include "scene_parser.hpp"
#include "light.hpp"
#include "group.hpp"
#include "render.hpp"
#include "pathtracing.hpp"

bool PathTracing::lightIntersect(const Ray& r, Hit& h, double tmin){
	bool isIntersect = false;
	for (int li = 0; li < scene->getNumLights(); ++li) {
        Light* light = scene->getLight(li);
		if (light->intersect(r, h, tmin))
			isIntersect = true;
	}
	return isIntersect;
}

Vector3f PathTracing::calReflection(const Ray& r, Hit& h, int depth){
	if (depth > max_depth) return scene->getBackgroundColor();
	Ray refl_ray = r.getReflectionRay(h.getT(), h.getNormal());
	return pathTrace(refl_ray, depth);
}

Vector3f PathTracing::calRefraction(const Ray& r, Hit& h, int depth){
	if (depth > max_depth) return scene->getBackgroundColor();
	double refr_iI, refr_iT;
	if (h.isOuterFace()) {
		refr_iI = 1.0; 
		refr_iT = h.getMaterial()->getBRDF().refractiveIndex;
	}
	else {
		refr_iI = h.getMaterial()->getBRDF().refractiveIndex; 
		refr_iT = 1.0;
	}
	Ray refr_ray = r.getRefractionRay(h.getT(), h.getNormal(), refr_iI, refr_iT);
	return pathTrace(refr_ray, depth);
}

Vector3f PathTracing::sampleRandomRay(const Vector3f& norm){
	Vector3f u = Vector3f::cross(Vector3f::RIGHT, norm);
    if (u.squaredLength() < feps) 
		u = Vector3f::cross(Vector3f::UP, norm);
    u.normalize();
    Vector3f v = Vector3f::cross(norm, u).normalized();
    double theta = rand() / double(RAND_MAX) * 2 * M_PI;
    double phi = asin(rand() / double(RAND_MAX));
    return (norm * cos(phi) + (u * cos(theta) + v * sin(theta)) * sin(phi)).normalized();
}

Vector3f PathTracing::calDiffusion(const Ray& r, Hit& h, int depth){
	if (depth > max_depth) return scene->getBackgroundColor();
	Vector3f hit_point = r.pointAtParameter(h.getT()), normal = h.getNormal();
	Vector3f color = Vector3f::ZERO, diffuse_factor = Vector3f::ZERO;
	BRDF hit_brdf = h.getMaterial()->getBRDF();
	// Direct light
	for (int li = 0; li < scene->getNumLights(); ++li) {
        Light* light = scene->getLight(li);
        Vector3f dirToLight, lightColor;
		light->getSampledIllumination(scene->getGroup(), hit_point, h.getNormal(), dirToLight, lightColor);
		color += hit_brdf.rho_d \
				* std::max(Vector3f::dot(dirToLight, h.getNormal()), 0.0) \
				* h.getColor() * lightColor ;
		Vector3f R = 2 * Vector3f::dot(h.getNormal(), dirToLight) * h.getNormal() - dirToLight;
		color += hit_brdf.rho_s \
				* pow(std::max(Vector3f::dot(-r.getDirection(), R), 0.0), hit_brdf.phong_s) \
				* lightColor;
	}	
	// RR to decide whether stop
	double p = rand() / double(RAND_MAX);
	// Indirect light
	if (p >= stop_p){
		Vector3f dir = sampleRandomRay(normal);
		diffuse_factor += hit_brdf.rho_d * std::max(Vector3f::dot(dir, normal), 0.0) * h.getColor();
		Vector3f R = 2 * Vector3f::dot(normal, dir) * normal - dir;
		diffuse_factor += hit_brdf.rho_s * pow(std::max(Vector3f::dot(-r.getDirection(), R), 0.0), hit_brdf.phong_s);
		Ray random_r = Ray(hit_point + normal * 2 * feps, dir);
		color += pathTrace(random_r, depth, false) * diffuse_factor / (1 - stop_p);// * 2 * M_PI;
	}
	return color;
}

Vector3f PathTracing::pathTrace(const Ray &r, int depth, bool cal_light) {
	Hit hit, light_hit;
	Vector3f color = Vector3f::ZERO;
	bool isIntersect = scene->getGroup()->intersect(r, hit, feps);
	bool isLightIntersect = lightIntersect(r, light_hit, feps);
	if (!isIntersect && !isLightIntersect){
		return scene->getBackgroundColor();
	}
	else if(!isIntersect || isIntersect && isLightIntersect && hit.getT() >= light_hit.getT() - feps){
		if (cal_light)
			return light_hit.getColor();
		else
			return Vector3f::ZERO;
	}
	else{
		// RR to decide random ray
		double action = rand() / double(RAND_MAX);
		BRDF hit_brdf = hit.getMaterial()->getBRDF();
		if(hit_brdf.specular > feps && action >= 0 && action < hit_brdf.specular)
			color += calReflection(r, hit, depth + 1);
		action -= hit_brdf.specular;
		if(hit_brdf.diffuse > feps && action >= 0 && action < hit_brdf.diffuse)
			color += calDiffusion(r, hit, depth + 1); 
		action -= hit_brdf.diffuse;
		if(hit_brdf.refraction > feps && action >= 0 && action < hit_brdf.refraction)
			color += calRefraction(r, hit, depth + 1);
		return color;
	}
}
	

void PathTracing::render() {
	// randn
	std::default_random_engine e {(unsigned)time(0)};
    std::normal_distribution<double> n {0, 1};
	
	Image* tmp_image = new Image(image->Width(), image->Height());
	for (int iter = 0; iter < num_iters; ++iter){
		for (int x = 0; x < camera->getWidth(); ++x) {
			#pragma omp parallel for schedule(dynamic, 1), num_threads(8)
        	for (int y = 0; y < camera->getHeight(); ++y) {
				// anti-aliasing
				Ray camRay = camera->generateRay(Vector2f(x + n(e) / 3.0, y + n(e) / 3.0));
				Vector3f color = pathTrace(camRay, 0);

				// update the value at this pixel
				Vector3f curColor = tmp_image->GetPixel(x, y);
				curColor = curColor * iter / (iter + 1) + color / (iter + 1);
				tmp_image->SetPixel(x, y, curColor);

				// write to image:
				image->SetPixel(x, y, curColor.clamp(0, 1));
			}
		}
		if ((iter + 1) % save_freq == 0) {
			printf("%d\n", iter + 1);
			image->SaveImage(save_path.data());
		}
	}

	// Save image
	image->SaveImage(save_path.data());
}