#include "Scene.h"
#include <algorithm>
#include <stack>
#include "Objects.h"

void Scene :: add_object(std::shared_ptr<Geometry> obj) {
    objects.push_back(obj);
}	

bool Scene :: intersect(const Ray& r, Intersection& int_info) {
    Intersection best_int;
    best_int.t = INF;

    for(int i = 0; i < objects.size(); i++) {
        Intersection current_int;

        if (objects[i] -> intersect(r, current_int)) 
        {   
            if (current_int.t < best_int.t) 
            {
                best_int = current_int;
                best_int.obj_id = i;
            } 
        }
    }

    if (best_int.obj_id != -1)
    {
        objects[best_int.obj_id] -> get_intersection_info(r, best_int);
        int_info = best_int;
        return true;
    }
    
    return false;
}


Vector Scene :: get_colour(const Ray &initial_Ray, int depth){
    Vector color(0, 0, 0);
    Intersection current_int, light_int;
    if (depth < 0) return color;
    
    if (intersect(initial_Ray, current_int)) {

        if (objects[current_int.obj_id] -> material != OPAQUE) {
            
            
            double dp = dot(initial_Ray.u, current_int.N), rf_index = objects[current_int.obj_id] -> refr_index;


            // Invert normal and refraction index if exiting the sphere
            if (dp >= 0)
            {
                current_int.N = (-1) * current_int.N;
                dp = - dp;
                rf_index = 1 / rf_index;
            }
         
            // Offset the origin to avoid noice

            current_int.P = current_int.P + EPSILON * current_int.N;

            //Compute reflected ray

            Ray reflected_ray(current_int.P, initial_Ray.u - 2 * dp * current_int.N);

            if(objects[current_int.obj_id] -> material == MIRROR) return get_colour(reflected_ray, depth - 1);

            //Compute refracted ray

            Ray refracted_ray;
            Vector tangential_comp = rf_index * (initial_Ray.u - dp * current_int.N), normal_comp = Vector(0, 0, 0);
            double delta = 1 -  rf_index * rf_index * (1 - dp * dp);
            if (delta < 0) return get_colour(reflected_ray, depth - 1);
            normal_comp = current_int.N * (- sqrt(delta)); 
            refracted_ray = Ray(current_int.P - 2 * EPSILON * current_int.N, tangential_comp + normal_comp);

            // // Fresnel law

            double k0 = std :: pow((rf_index - 1) / (rf_index + 1), 2);
            double R = k0 + (1 - k0) * pow((1 - std :: abs(dp)), 5);
            if(unif(gen) < R) return get_colour(reflected_ray, depth - 1);
            else return get_colour(refracted_ray, depth - 1);
            
        } else {

            // Offset the origin to avoid noice
            
            current_int.P = current_int.P + EPSILON * current_int.N; 

            // Shoot ray towards light source

            Ray ray_to_light;
            double d2 = ray_to_light.create_ray(current_int.P, S);
            if(!intersect(ray_to_light, light_int) || (light_int.P - current_int.P).norm2() > d2)
                {
                    // Lambertian formula

                    color = I * std::max(0.0, dot(current_int.N, ray_to_light.u)) * objects[current_int.obj_id] -> albedo / (4 * M_PI * M_PI * d2);
                }

            // Indirect lighting 
            if (bIndirect_Lighting) {
                Ray randomRay = Ray(current_int.P, random_cos(current_int.N));
                color = color + comp_wise_mult(objects[current_int.obj_id] -> albedo, get_colour(randomRay, depth - 1));
            }        
            

            return color;
            
        }
    }

   
    return color;
}



std::vector<unsigned char> Scene :: take_picture() {

    std::vector<unsigned char> image(cmr.W * cmr.H * 3, 0);
    Vector color;

#pragma omp parallel for schedule(dynamic, 1)
    for (int i = 0; i < cmr.H; i++) {
        for (int j = 0; j < cmr.W; j++) {
             double r = 0, g = 0, b = 0;
             for(int k = 0; k < cmr.K; k++) {
                Vector color = get_colour(cmr.ray_from_pixel(i, j), max_ray_Depth);
                r += color[0];
                g += color[1];
                b += color[2];
            }

            /// Gamma correction
            image[(i * cmr.W + j) * 3 + 0] = int(255 * std::clamp(pow(std::max(r / cmr.K, 0.0), 1/GAMMA), 0.0, 1.0));
            image[(i * cmr.W + j) * 3 + 1] = int(255 * std::clamp(pow(std::max(g / cmr.K, 0.0), 1/GAMMA), 0.0, 1.0));
            image[(i * cmr.W + j) * 3 + 2] = int(255 * std::clamp(pow(std::max(b / cmr.K, 0.0), 1/GAMMA), 0.0, 1.0));
            }
    }

    return image;
}

    
