#pragma once

#include "Vector.h"
#include "Objects.h"

class Scene{
    public:
        std::vector<Sphere> objects;
        Camera cmr;
        Vector S;
        double I;
        int max_ray_Depth;
        inline Scene(Camera cmr1, Vector S1, double I1 = 100000.0, std::vector<Sphere> objs = {}, int mx1 = 5) : cmr(cmr1), S(S1), I(I1), objects(objs), max_ray_Depth(mx1) {}
        void add_object(Sphere s); 
        bool intersect(Ray r, Intersection &int_info);
        std::vector<unsigned char> take_picture();
        Vector get_colour(Ray& ray, int n);
    };