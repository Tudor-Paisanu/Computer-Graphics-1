#include "Objects.h"
#include <cmath>


// Ray

double Ray :: create_ray(Vector O, Vector D){
    o = O;
    dir = D - O; double d = dir.norm2(); dir.normalize();
    return d;
}

// Sphere

bool Sphere :: intersect(Ray r, double& t){
    double delta = dot(r.dir, r.o - center) * dot(r.dir, r.o - center) - ((r.o - center).norm2() - radius * radius);
    if(delta < 0) return false;
    delta = sqrt(delta);
    double t2 = dot(r.dir, center - r.o) + delta;
    double t1 = t2 - 2 * delta;
    if (t2 < 0) return false;
    t = (t1 >=0 ? t1 : t2);
    return true;
}


// Camera

Ray Camera :: ray_from_pixel(int i, int j) {
    double x = j, y = H - i - 1;
    Vector V = Vector((x + 0.5 - W / 2), (y + 0.5 - H / 2), (- W / (2 * tan(a / 2))));
    V.normalize();
    return Ray(Q, V);  
}

std::vector<Ray> Camera :: cast_rays() {
    std::vector<Ray> rays;
    rays.reserve(W * H);
    for(int i = 0; i < H; i++){
        for(int j = 0; j < W; j++) {
            Ray ray = ray_from_pixel(i, j);
            rays.push_back(ray);
        }    
    }
            
    return rays;
}