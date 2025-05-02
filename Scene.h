#pragma once

#include <memory>
#include "Objects.h"
#include "Vector.h"


class Scene{
    public:
        std::vector<std::shared_ptr<Geometry>> objects;
        Camera cmr;
        Vector S;
        double I;
        int max_ray_Depth;
        bool bIndirect_Lighting = true;
        inline Scene(Camera cmr1, Vector S1, double I1 = 99900.0, std::vector<std::shared_ptr<Geometry>> objs = {}, int mx1 = 3) : cmr(cmr1), S(S1), I(I1), objects(objs), max_ray_Depth(mx1) {}
        void add_object(std::shared_ptr<Geometry> obj); 
        bool intersect(const Ray& r, Intersection &int_info);
        std::vector<unsigned char> take_picture();
        Vector get_colour(const Ray& ray, int n);
    };