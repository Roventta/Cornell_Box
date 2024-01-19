//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include <random>

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

float custom_rand(float min, float max){

    return min + (float)(rand()) / ((float)(RAND_MAX/(max-min)));
}


// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Vector3f black;
    //if(depth > this->maxDepth){return black;}

    Intersection hit_event = intersect(ray);
    if(! hit_event.happened) {return black;}

    if(hit_event.obj->hasEmit()){return hit_event.m->getEmission();}
    
    Intersection light_position;
    float pdf = 0.0;
    sampleLight(light_position, pdf);

    Vector3f L_direct;

    Ray lightRay(hit_event.coords, normalize(light_position.coords - hit_event.coords));
    Intersection reflect_hit_event = intersect(lightRay);
    if(reflect_hit_event.happened){
    if(reflect_hit_event.obj->hasEmit()){
        Vector3f albedo = hit_event.m->eval(-lightRay.direction, -ray.direction, hit_event.normal);
        L_direct = reflect_hit_event.m->getEmission() * albedo * 
                    dotProduct(hit_event.normal, lightRay.direction) * dotProduct(light_position.normal, -lightRay.direction) 
                    / (distanceSquared(light_position.coords, hit_event.coords) * pdf);
        //printf("%f, %f, %f, %f\n", L_direct.x, L_direct.y, L_direct.z, pdf);
    } 
    }

    Vector3f L_indirect;
    if (get_random_float() < this->RussianRoulette){
        Vector3f outDir =  hit_event.m->sample(lightRay.direction, hit_event.normal);
        Ray ind_reflect_ray(hit_event.coords, outDir);
        reflect_hit_event = intersect(ind_reflect_ray);
        if(reflect_hit_event.happened){
        if(!reflect_hit_event.obj->hasEmit()){
            L_indirect = castRay(ind_reflect_ray, depth+1) * dotProduct(hit_event.normal, outDir) / this->RussianRoulette;
        }
        }
    }

    return L_direct + L_indirect;
    
}