#pragma once
#include "Vector.h"
#include "GenerateRandom.h"

class Ray {
    public:
        Vector O, u;
        explicit Ray(Vector O1 = Vector(0,0,0), Vector u1 =  Vector(0,0,0)): O(O1), u(u1) {}
        double create_ray(Vector S, Vector D); // Makes this ray to be from S to D and returns the distance between S,D
   };

class Intersection{
    public:
        Vector P, N;
        double t, a, b, c;
        int obj_id, tri_id;
        Intersection(Vector P1 = Vector(0, 0, 0), Vector N1 = Vector(0, 0, 0), double t1 = 0, 
        double a1 = 0, double b1 = 0, double c1 = 0, int obj_id1 = -1, int tri_id1 = -1 ): P(P1), N(N1), t(t1), a(a1), b(b1), c(c1), obj_id(obj_id1), tri_id(tri_id1) {};
    };

class Camera{
	public:
		Vector Q;
		double a;
		int W, H, K;
		Camera(Vector Q1, double a1, int W1, int H1, int K1 = 32): Q(Q1), a(a1), W(W1), H(H1), K(K1) {}
		Ray ray_from_pixel(int i, int j);
		std::vector<Ray> cast_rays();
	};

enum Material {
    OPAQUE,
    MIRROR,
    TRANSPARENT
    };

class Geometry {
    public:
        virtual bool intersect(const Ray& r, Intersection& int_info) = 0;
        virtual void get_intersection_info(const Ray& ray, Intersection& int_info) = 0;
        Vector albedo = Vector(0.8, 0.8, 0.8);
        Material material = OPAQUE;
        double refr_index = 1. /1.5;
        bool bInvertNormals = false;

    };

class TriangleIndices {
    public:
        TriangleIndices(int vtxi = -1, int vtxj = -1, int vtxk = -1, int ni = -1, int nj = -1, int nk = -1, int uvi = -1, int uvj = -1, int uvk = -1, int group = -1, bool added = false) : vtxi(vtxi), vtxj(vtxj), vtxk(vtxk), uvi(uvi), uvj(uvj), uvk(uvk), ni(ni), nj(nj), nk(nk), group(group) {
        };
        int vtxi, vtxj, vtxk; // indices within the vertex coordinates array
        int uvi, uvj, uvk;  // indices within the uv coordinates array
        int ni, nj, nk;  // indices within the normals array
        int group;       // face group
    };
    
class Bbox {
    public:
        Bbox(Vector min = Vector(std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()), Vector max = Vector(-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity())) : Bmin(min), Bmax(max) {};
        bool intersect(const Ray& r, double& t);
        Vector Bmin, Bmax;
};

class BVH {
    public:
        Bbox bbox;
        BVH *left = nullptr, *right = nullptr;
        int start, end;

        ~BVH() {
            delete left; delete right;
        }
};
    
class TriangleMesh : virtual public Geometry {
    public:
        ~TriangleMesh() {}
        TriangleMesh() {};
        void readOBJ(const char* obj);
        bool intersect_triangle(const Ray& r, Intersection& int_info, int id); // You only need to pass the id of the triangle in int_info
        Vector barycenter(int i);
        bool intersect(const Ray& r, Intersection& int_info) override;
        void get_intersection_info(const Ray& ray, Intersection& int_info) override;
        void compute_bbox(Bbox& bx, int start, int end);
        void compute_BVH(BVH* node, int start, int end);
        void set_BVH();
        void transform(const Vector& s, const Vector& t);
        std::vector<TriangleIndices> indices;
        std::vector<Vector> vertices;
        std::vector<Vector> normals;
        std::vector<Vector> uvs;
        std::vector<Vector> vertexcolors;
        BVH bvh;
    };

class Sphere : virtual public Geometry {
    public:
        Vector center;
        double radius;
        Sphere(Vector c, double r, Vector a, Material m = OPAQUE, double refra1 = 1. /1.5, bool inv = false): center(c), radius(r)
        {
            albedo = a;
            material = m;
            refr_index = refra1;
            bInvertNormals = inv;
        }

        bool intersect(const Ray& r, Intersection& int_info) override;
        void get_intersection_info(const Ray& ray, Intersection& int_info) override;
    };

        