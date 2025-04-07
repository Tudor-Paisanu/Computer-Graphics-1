#include "Scene.h"
#include <algorithm>
#include <stack>
#include "Objects.h"
#include <random>



// Random number generator

std::random_device rd;
std::mt19937 gen(rd());
std::uniform_real_distribution<double> unif(0.0, 1.0);

Vector random_cos(const Vector &N) {
    float r1 = unif(gen);  // Uniform random [0,1]
    float r2 = unif(gen);
    float phi = 2.0f * M_PI * r1;

    float x = cos(phi) * sqrt(1.0f - r2);
    float y = sin(phi) * sqrt(1.0f - r2);
    float z = sqrt(r2);

    Vector T1;
    if (fabs(N[0]) < fabs(N[1]) && fabs(N[0]) < fabs(N[2]))
        T1 = Vector(0, -N[2], N[1]); // Zero x component
    else if (fabs(N[1]) < fabs(N[2]))
        T1 = Vector(-N[2], 0, N[0]); // Zero y component
    else
        T1 = Vector(-N[1], N[0], 0); // Zero z component

    T1.normalize();
    Vector T2 = cross(N, T1);

    // Step 3: Transform to world space
    Vector Result = x * T1 + y * T2 + z * N; Result.normalize();
    return Result;
}


void Scene :: add_object(Sphere s) {
    objects.push_back(s);
}	

bool Scene :: intersect(Ray r, Intersection& int_info) {
    int best_id = -1;
    double current_t, best_t;
    double min_dist = std::numeric_limits<double>::infinity();

    for(int i = 0; i < objects.size(); i++) {
        if (objects[i].intersect(r, current_t)) 
        {   
            double d = current_t * current_t;
            if (d < min_dist) 
            {
                min_dist = d;
                best_t = current_t;
                best_id = i;
            } 
        }
    }
    if (best_id == -1)
        return false;
    Vector P = r.o + best_t * r.dir, N = P - objects[best_id].center; N.normalize();
    if(objects[best_id].InvertNormals) N = (-1) * N;
    int_info = Intersection(P, N, best_t, best_id);
    return true;

}


Vector Scene :: get_colour(Ray &initial_Ray, int depth){
    Vector color(0, 0, 0);
    Intersection current_int, light_int;
    if (depth < 0) return color;
    
    if (intersect(initial_Ray, current_int)) {

        if (objects[current_int.id].material != Sphere :: OPAQUE) {
            
            
            double dp = dot(initial_Ray.dir, current_int.N), rf_index = objects[current_int.id].refr_index;


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

            Ray reflected_ray(current_int.P, initial_Ray.dir - 2 * dp * current_int.N);

            if(objects[current_int.id].material == Sphere :: MIRROR) return get_colour(reflected_ray, depth - 1);

            //Compute refracted ray

            Ray refracted_ray;
            Vector tangential_comp = rf_index * (initial_Ray.dir - dp * current_int.N), normal_comp = Vector(0, 0, 0);
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

            // Direct lighting 

            // Offset the origin to avoid noice
            
            current_int.P = current_int.P + EPSILON * current_int.N; 

            // Shoot ray towards light source

            Ray ray_to_light;
            double d2 = ray_to_light.create_ray(current_int.P, S);
            if(!intersect(ray_to_light, light_int) || (light_int.P - current_int.P).norm2() > d2)
                {
                    // Lambertian formula

                    color = I * std::max(0.0, dot(current_int.N, ray_to_light.dir)) * objects[current_int.id].albedo / (4 * M_PI * M_PI * d2);
                }

            // Indirect lighting

            Ray randomRay = Ray(current_int.P, random_cos(current_int.N));
            color = color + comp_wise_mult(objects[current_int.id].albedo, get_colour(randomRay, depth - 1));

            return color;
            
        }
    }

  
                
   
    return color;
}



std::vector<unsigned char> Scene :: take_picture() {

    std::vector<Ray> rays = cmr.cast_rays();
    std::vector<unsigned char> image(cmr.W * cmr.H * 3, 0);
    Vector color;

    #pragma omp parallel for

    for (int i = 0; i < cmr.H; i++) {
        for (int j = 0; j < cmr.W; j++) {
             double r = 0, g = 0, b = 0;
             for(int k = 0; k < cmr.K; k++) {
                Vector color = get_colour(rays[i * cmr.W + j], max_ray_Depth);
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

    
