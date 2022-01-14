#include <cmath>
#include <algorithm>
#include <omp.h>

#include "scene_parser.hpp"
#include "light.hpp"
#include "group.hpp"
#include "render.hpp"
#include "raytracing.hpp"

bool RayTracing::lightIntersect(const Ray& r, Hit& h, double tmin){
	bool isIntersect = false;
	for (int li = 0; li < scene->getNumLights(); ++li) {
        Light* light = scene->getLight(li);
		if (light->intersect(r, h, tmin))
			isIntersect = true;
	}
	return isIntersect;
}

Vector3f RayTracing::calReflection(const Ray& r, Hit& h, int depth, unsigned& hash){
	if (depth > max_depth) {
		hash = hash * 17 + 233;
		return scene->getBackgroundColor();
	}
	hash = hash * 17 + h.getHash();
	Ray refl_ray = r.getReflectionRay(h.getT(), h.getNormal());
	return rayTrace(refl_ray, depth, hash);
}

Vector3f RayTracing::calRefraction(const Ray& r, Hit& h, int depth, unsigned& hash){
	if (depth > max_depth) {
		hash = hash * 19 + 233;
		return scene->getBackgroundColor();
	}
	hash = hash * 19 + h.getHash();
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
	return rayTrace(refr_ray, depth, hash);
}

Vector3f RayTracing::calDiffusion(const Ray& r, Hit& h, unsigned& hash){
	Vector3f hit_point = r.pointAtParameter(h.getT()), color = Vector3f::ZERO;
	BRDF hit_brdf = h.getMaterial()->getBRDF();
	for (int li = 0; li < scene->getNumLights(); ++li) {
        Light* light = scene->getLight(li);
        Vector3f dirToLight, lightColor;
		light->getIllumination(scene->getGroup(), hit_point, h.getNormal(), dirToLight, lightColor, hash);
		color += hit_brdf.rho_d \
				* std::max(Vector3f::dot(dirToLight, h.getNormal()), 0.0) \
				* h.getColor() * lightColor ;
		Vector3f R = 2 * Vector3f::dot(h.getNormal(), dirToLight) * h.getNormal() - dirToLight;
		color += hit_brdf.rho_s \
				* pow(std::max(Vector3f::dot(-r.getDirection(), R), 0.0), hit_brdf.phong_s) \
				* lightColor;
	}
	return color;
}

Vector3f RayTracing::rayTrace(const Ray &r, int depth, unsigned& hash) {
	Hit hit, light_hit;
	Vector3f color = Vector3f::ZERO;
	bool isIntersect = scene->getGroup()->intersect(r, hit, feps);
	bool isLightIntersect = lightIntersect(r, light_hit, feps);
	if (!isIntersect && !isLightIntersect){
		hash = hash * 13 + 235;
		color += scene->getBackgroundColor();
	}
	else if(!isIntersect || isIntersect && isLightIntersect && hit.getT() >= light_hit.getT() - feps){
		color += light_hit.getColor();
		hash = hash * 13 + light_hit.getHash();
	}
	else{
		BRDF hit_brdf = hit.getMaterial()->getBRDF();
		if(hit_brdf.specular > feps)
			color += calReflection(r, hit, depth + 1, hash) * hit_brdf.specular;
		if(hit_brdf.diffuse > feps)
			color += calDiffusion(r, hit, hash) * hit_brdf.diffuse;
		if(hit_brdf.refraction > feps)
			color += calRefraction(r, hit, depth + 1, hash) * hit_brdf.refraction;
	}
	return color.clamp(0.0, 1.0);
}

void RayTracing::render() {
	// Construct a hash table
	unsigned** hash_table = new unsigned*[camera->getWidth()];
	for (int x = 0; x < camera->getWidth(); x++)
		hash_table[x] = new unsigned[camera->getHeight()] {0};

	for (int x = 0; x < camera->getWidth(); ++x) {
		#pragma omp parallel for schedule(dynamic, 1), num_threads(8)
        for (int y = 0; y < camera->getHeight(); ++y) {
			Ray camRay = camera->generateRay(Vector2f(x, y));
			Vector3f finalColor = rayTrace(camRay, 0, hash_table[x][y]);
			image->SetPixel(x, y, finalColor);
		}
	}

	for (int x = 0; x < camera->getWidth(); ++x) {
		#pragma omp parallel for schedule(dynamic, 1), num_threads(8)
        for (int y = 0; y < camera->getHeight(); ++y) {
			bool flag = false;
			if (x != 0 && hash_table[x][y] != hash_table[x-1][y])
				flag = true;
			if (x != camera->getWidth() - 1 && hash_table[x][y] != hash_table[x+1][y])
				flag = true;
			if (y !=0 && hash_table[x][y] != hash_table[x][y-1])
				flag = true;
			if (y != camera->getHeight() - 1 && hash_table[x][y] != hash_table[x][y+1])
				flag = true;
			if (flag) {
				Vector3f color;
				for (int k1 = -1; k1 <= 1; k1++) {
					for (int k2 = -1; k2 <= 1; k2++) {
						unsigned hash = 0;
						Ray camRay = camera->generateRay(Vector2f(x + k1 / 3.0, y + k2 / 3.0));
						color += rayTrace(camRay, 0, hash);
					}
				}
				color = color / 9;
				image->SetPixel(x, y, color);
			}
		}
	}

	// Save image
	image->SaveImage(save_path.data());

	// Delete hash table
	for (int x = 0; x < camera->getWidth(); x++) {
		delete[] hash_table[x];
	}
	delete[] hash_table;
}