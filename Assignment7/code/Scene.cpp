//
// Created by Göksu Güvendiren on 2019-05-14.
//

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

bool Scene::trace(const Ray &ray,const std::vector<Object*> &objects,float &tNear, uint32_t &index, Object **hitObject)
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

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    Vector3f hitColor = this->backgroundColor;

    //Find the Intersection of camera ray with the scene
    Intersection objInter = Scene::intersect(ray);

    if (objInter.happened) 
    {
        Material* m = objInter.m;
        
        //Return light emission if ligh is hit.
        if (m->hasEmission()) return m->getEmission();

        Vector3f N = objInter.normal;
        Vector3f light_dir = Vector3f(0.0, 0.0, 0.0);
        Vector3f light_indir = Vector3f(0.0, 0.0, 0.0);

        //Direct light:
        float pdf = 0.0f;
        Intersection lightInter;
        sampleLight(lightInter, pdf);

        Vector3f obj2light = lightInter.coords - objInter.coords;
        Vector3f ws = obj2light.normalized();
        float distancePow2 = obj2light.x * obj2light.x + obj2light.y * obj2light.y + obj2light.z * obj2light.z;

        Ray obj2lightray = {objInter.coords, ws};
        Intersection t = intersect(obj2lightray);
        if (t.distance - obj2light.norm() > -EPSILON)
        {
            light_dir = lightInter.emit * objInter.m->eval(ray.direction, ws, N) * dotProduct(ws, N) * dotProduct(-ws, lightInter.normal) / distancePow2 / pdf;
        }

        //Indirect light:
        if (get_random_float() < RussianRoulette) 
        {
            auto wi = objInter.m->sample(ray.direction, objInter.normal).normalized();
            Ray q = Ray(objInter.coords, wi);
            Intersection nextInter = intersect(q);
            if (nextInter.happened && !nextInter.m->hasEmission()) {
                float pdf = m->pdf(ray.direction, wi, N);
                light_indir = castRay(q, depth+1)* m->eval(ray.direction, wi, N)*dotProduct(wi, N)/pdf/RussianRoulette;
            }
        }

        hitColor = light_dir + light_indir;
    }
    return hitColor;
}
