#pragma once

#include "Vector.h"

class Ray {
    public:
        Vector o, dir;
        explicit Ray(Vector o1 = Vector(0,0,0), Vector dir1 =  Vector(0,0,0)): o(o1), dir(dir1) {}
        double create_ray(Vector S, Vector D); // Makes this ray to be from S to D and returns the distance between S,D
    };

class Intersection{
    public:
        Vector P, N;
        double t;
        int id;
        Intersection(Vector P1 = Vector(0, 0, 0), Vector N1 = Vector(0, 0, 0), double t1 = 0, int id1 = -1): P(P1), N(N1), t(t1), id(id1) {};
};


class Sphere {
    public:
        Vector center;
        double radius;
        Vector albedo;
        enum Material {
            OPAQUE,
            MIRROR,
            TRANSPARENT
        } material;

        double refr_index;
        bool InvertNormals;

        Sphere(Vector c, double r, Vector a, Material m = OPAQUE, double refra1 = 1. /1.5, bool inv = false): center(c), radius(r), albedo(a), material(m), refr_index(refra1), InvertNormals(inv){}
        bool intersect(Ray r, double& t);
    };

class Camera{
	public:
		Vector Q;
		double a;
		int W, H, K;
		Camera(Vector Q1, double a1, int W1, int H1, int K1 = 15): Q(Q1), a(a1), W(W1), H(H1), K(K1) {}
		Ray ray_from_pixel(int i, int j);
		std::vector<Ray> cast_rays();
	};
