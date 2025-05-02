#include "Objects.h"
#include "GenerateRandom.h"
#include <string.h>
#include <iostream>
#include <stdio.h>
#include <algorithm>
#include <vector>
#include <list>


// Ray

double Ray :: create_ray(Vector O1, Vector u1){
    O = O1;
    u = u1 - O1; double d = u.norm2(); u.normalize();
    return d;
}

// Camera

Ray Camera :: ray_from_pixel(int i, int j) {
    double x = 0, y = 0;
    boxMuller(x, y);
    x += j; y+= H - i - 1;
    Vector V = Vector(Q[0] + x + 0.5 - W / 2 , Q[1] + y + 0.5 - H / 2, Q[2] - W / (2 * tan(a / 2)));
    V.normalize();
    return Ray(Q, V);  
}


// Sphere

bool Sphere :: intersect(const Ray& r, Intersection& int_info) {
    double delta = dot(r.u, r.O - center) * dot(r.u, r.O - center) - ((r.O - center).norm2() - radius * radius);
    if(delta < 0) return false;
    delta = sqrt(delta);
    double t2 = dot(r.u, center - r.O) + delta;
    double t1 = t2 - 2 * delta;
    if (t2 < 0) return false;
    int_info.t = (t1 >=0 ? t1 : t2);
    return true;
}

void Sphere :: get_intersection_info(const Ray&r, Intersection& int_info) {
    Vector P = r.O + int_info.t * r.u, N = P - center; N.normalize();
    if(bInvertNormals) N = (-1) * N;
    int_info.P = P; int_info.N = N;
}


// Bbox

bool Bbox :: intersect(const Ray& r, double& t) {
    double tmin = -INF;
    double tmax =  INF;

    for (int i = 0; i < 3; ++i) {
		double t0 = (Bmin[i] - r.O[i]) / r.u[i];
		double t1 = (Bmax[i] - r.O[i]) / r.u[i];
		if (t0 > t1) std::swap(t0, t1);
		tmin = std::max(tmin, t0);
		tmax = std::min(tmax, t1);
		if (tmin > tmax) return false;
    }

	t = tmin;

    return true;
}


// Triangle 

Vector TriangleMesh :: barycenter(int id) {
	Vector A = vertices[indices[id].vtxi], B = vertices[indices[id].vtxj], C = vertices[indices[id].vtxk];
	return (A + B + C) / 3;

}

bool TriangleMesh::intersect_triangle(const Ray& r, Intersection& int_info, int id) {
    Vector A = vertices[indices[id].vtxi], B = vertices[indices[id].vtxj], C = vertices[indices[id].vtxk];
    Vector e1 = B - A, e2 = C - A, N = cross(e1, e2), crs = cross(A - r.O, r.u);
    double dp = dot(r.u, N), b = dot(e2, crs) / dp, c = - dot(e1, crs) / dp, a = 1 - b - c, t = dot(A - r.O, N) / dp;

    if (b > 0 && c > 0 && a > 0 && t > 0)
	{
		int_info.a = a; int_info.b = b; int_info.c = c; int_info.t = t; int_info.tri_id = id;
		return true;

	}

    return false;
}


bool TriangleMesh::intersect(const Ray& r, Intersection& int_info) {
    Intersection best_int;
    best_int.t = INF;

    double bbox_t;
    if (!bvh.bbox.intersect(r, bbox_t)) return false;

    std::list<BVH*> visit;
    visit.push_front(&bvh);

    while (!visit.empty()) {
        BVH* curNode = visit.back();
        visit.pop_back();
        if (!curNode->left) {
			Intersection current_int;
            for (int i = curNode->start; i < curNode->end; ++i) {
                if (intersect_triangle(r, current_int, i)) {
                    if (current_int.t < best_int.t) {
                        best_int = current_int;
                    }
                }
            }
        } else {
            if (curNode->left->bbox.intersect(r, bbox_t)) {
                if (bbox_t < best_int.t) {
                    visit.push_back(curNode->left);
                }
            }

            if (curNode->right->bbox.intersect(r, bbox_t)) {
                if (bbox_t < best_int.t) {
                    visit.push_back(curNode->right);
                }
            }
        }
    }

    if (best_int.tri_id != -1) int_info = best_int;
    return best_int.tri_id != -1;
}


void TriangleMesh :: get_intersection_info(const Ray&r, Intersection& int_info) {
	Vector P = r.O + int_info.t * r.u;
	Vector normi = normals[indices[int_info.tri_id].ni], normj = normals[indices[int_info.tri_id].nj], normk = normals[indices[int_info.tri_id].nk];
	Vector N = int_info.a * normi + int_info.b * normj + int_info.c * normk; N.normalize();
	if(bInvertNormals) N = (-1) * N;
	int_info.P = P; int_info.N = N; 
}


void TriangleMesh :: compute_bbox(Bbox& bx, int start, int end) {
    Bbox b;
    for (int i = start; i < end; i++) {
        Vector A = vertices[indices[i].vtxi];
        Vector B = vertices[indices[i].vtxj];
        Vector C = vertices[indices[i].vtxk];

        for (int j = 0; j < 3; j++) {
            b.Bmin[j] = std::min({b.Bmin[j], A[j], B[j], C[j]});
            b.Bmax[j] = std::max({b.Bmax[j], A[j], B[j], C[j]});
        }
    }

    bx = b;
}



void TriangleMesh :: compute_BVH(BVH* node, int start, int end) {
	compute_bbox(node->bbox, start, end);
	node->start = start;
	node->end = end;
	Vector diag = node->bbox.Bmax - node->bbox.Bmin;
	Vector middle_diag = node->bbox.Bmin + diag * 0.5 ;
	int longest_axis = diag.get_longest();
	int pivot = start;
	for (int id = start; id < end; id++) {
		Vector bary = barycenter(id);
		if (bary[longest_axis] < middle_diag[longest_axis])
		{
			std::swap (indices[id], indices[pivot]) ;
			pivot++;
		}
	}

	if (pivot <= start || pivot >= end - 1 || (end - start) < 5) return;
	node->left = new BVH; node->right = new BVH;
	compute_BVH(node->left, start, pivot);
	compute_BVH(node->right, pivot, end);

}

void TriangleMesh :: set_BVH() {

	compute_BVH(&bvh, 0, indices.size());
}


void TriangleMesh :: transform(const double s, const Vector& t) {
    for (auto v : vertices) {
		v = s * v + t;
	}
}

void TriangleMesh :: readOBJ(const char* obj) {

	char matfile[255];
	char grp[255];

	FILE* f;
	f = fopen(obj, "r");
	int curGroup = -1;
	while (!feof(f)) {
		char line[255];
		if (!fgets(line, 255, f)) break;

		std::string linetrim(line);
		linetrim.erase(linetrim.find_last_not_of(" \r\t") + 1);
		strcpy(line, linetrim.c_str());

		if (line[0] == 'u' && line[1] == 's') {
			sscanf(line, "usemtl %[^\n]\n", grp);
			curGroup++;
		}

		if (line[0] == 'v' && line[1] == ' ') {
			Vector vec;

			Vector col;
			if (sscanf(line, "v %lf %lf %lf %lf %lf %lf\n", &vec[0], &vec[1], &vec[2], &col[0], &col[1], &col[2]) == 6) {
				col[0] = std::min(1., std::max(0., col[0]));
				col[1] = std::min(1., std::max(0., col[1]));
				col[2] = std::min(1., std::max(0., col[2]));

				vertices.push_back(vec);
				vertexcolors.push_back(col);

			} else {
				sscanf(line, "v %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
				vertices.push_back(vec);
			}
		}
		if (line[0] == 'v' && line[1] == 'n') {
			Vector vec;
			sscanf(line, "vn %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
			normals.push_back(vec);
		}
		if (line[0] == 'v' && line[1] == 't') {
			Vector vec;
			sscanf(line, "vt %lf %lf\n", &vec[0], &vec[1]);
			uvs.push_back(vec);
		}
		if (line[0] == 'f') {
			TriangleIndices t;
			int i0, i1, i2, i3;
			int j0, j1, j2, j3;
			int k0, k1, k2, k3;
			int nn;
			t.group = curGroup;

			char* consumedline = line + 1;
			int offset;

			nn = sscanf(consumedline, "%u/%u/%u %u/%u/%u %u/%u/%u%n", &i0, &j0, &k0, &i1, &j1, &k1, &i2, &j2, &k2, &offset);
			if (nn == 9) {
				if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
				if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
				if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
				if (j0 < 0) t.uvi = uvs.size() + j0; else	t.uvi = j0 - 1;
				if (j1 < 0) t.uvj = uvs.size() + j1; else	t.uvj = j1 - 1;
				if (j2 < 0) t.uvk = uvs.size() + j2; else	t.uvk = j2 - 1;
				if (k0 < 0) t.ni = normals.size() + k0; else	t.ni = k0 - 1;
				if (k1 < 0) t.nj = normals.size() + k1; else	t.nj = k1 - 1;
				if (k2 < 0) t.nk = normals.size() + k2; else	t.nk = k2 - 1;
				indices.push_back(t);
			} else {
				nn = sscanf(consumedline, "%u/%u %u/%u %u/%u%n", &i0, &j0, &i1, &j1, &i2, &j2, &offset);
				if (nn == 6) {
					if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
					if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
					if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
					if (j0 < 0) t.uvi = uvs.size() + j0; else	t.uvi = j0 - 1;
					if (j1 < 0) t.uvj = uvs.size() + j1; else	t.uvj = j1 - 1;
					if (j2 < 0) t.uvk = uvs.size() + j2; else	t.uvk = j2 - 1;
					indices.push_back(t);
				} else {
					nn = sscanf(consumedline, "%u %u %u%n", &i0, &i1, &i2, &offset);
					if (nn == 3) {
						if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
						if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
						if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
						indices.push_back(t);
					} else {
						nn = sscanf(consumedline, "%u//%u %u//%u %u//%u%n", &i0, &k0, &i1, &k1, &i2, &k2, &offset);
						if (i0 < 0) t.vtxi = vertices.size() + i0; else	t.vtxi = i0 - 1;
						if (i1 < 0) t.vtxj = vertices.size() + i1; else	t.vtxj = i1 - 1;
						if (i2 < 0) t.vtxk = vertices.size() + i2; else	t.vtxk = i2 - 1;
						if (k0 < 0) t.ni = normals.size() + k0; else	t.ni = k0 - 1;
						if (k1 < 0) t.nj = normals.size() + k1; else	t.nj = k1 - 1;
						if (k2 < 0) t.nk = normals.size() + k2; else	t.nk = k2 - 1;
						indices.push_back(t);
					}
				}
			}

			consumedline = consumedline + offset;

			while (true) {
				if (consumedline[0] == '\n') break;
				if (consumedline[0] == '\0') break;
				nn = sscanf(consumedline, "%u/%u/%u%n", &i3, &j3, &k3, &offset);
				TriangleIndices t2;
				t2.group = curGroup;
				if (nn == 3) {
					if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
					if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
					if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
					if (j0 < 0) t2.uvi = uvs.size() + j0; else	t2.uvi = j0 - 1;
					if (j2 < 0) t2.uvj = uvs.size() + j2; else	t2.uvj = j2 - 1;
					if (j3 < 0) t2.uvk = uvs.size() + j3; else	t2.uvk = j3 - 1;
					if (k0 < 0) t2.ni = normals.size() + k0; else	t2.ni = k0 - 1;
					if (k2 < 0) t2.nj = normals.size() + k2; else	t2.nj = k2 - 1;
					if (k3 < 0) t2.nk = normals.size() + k3; else	t2.nk = k3 - 1;
					indices.push_back(t2);
					consumedline = consumedline + offset;
					i2 = i3;
					j2 = j3;
					k2 = k3;
				} else {
					nn = sscanf(consumedline, "%u/%u%n", &i3, &j3, &offset);
					if (nn == 2) {
						if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
						if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
						if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
						if (j0 < 0) t2.uvi = uvs.size() + j0; else	t2.uvi = j0 - 1;
						if (j2 < 0) t2.uvj = uvs.size() + j2; else	t2.uvj = j2 - 1;
						if (j3 < 0) t2.uvk = uvs.size() + j3; else	t2.uvk = j3 - 1;
						consumedline = consumedline + offset;
						i2 = i3;
						j2 = j3;
						indices.push_back(t2);
					} else {
						nn = sscanf(consumedline, "%u//%u%n", &i3, &k3, &offset);
						if (nn == 2) {
							if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
							if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
							if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
							if (k0 < 0) t2.ni = normals.size() + k0; else	t2.ni = k0 - 1;
							if (k2 < 0) t2.nj = normals.size() + k2; else	t2.nj = k2 - 1;
							if (k3 < 0) t2.nk = normals.size() + k3; else	t2.nk = k3 - 1;								
							consumedline = consumedline + offset;
							i2 = i3;
							k2 = k3;
							indices.push_back(t2);
						} else {
							nn = sscanf(consumedline, "%u%n", &i3, &offset);
							if (nn == 1) {
								if (i0 < 0) t2.vtxi = vertices.size() + i0; else	t2.vtxi = i0 - 1;
								if (i2 < 0) t2.vtxj = vertices.size() + i2; else	t2.vtxj = i2 - 1;
								if (i3 < 0) t2.vtxk = vertices.size() + i3; else	t2.vtxk = i3 - 1;
								consumedline = consumedline + offset;
								i2 = i3;
								indices.push_back(t2);
							} else {
								consumedline = consumedline + 1;
							}
						}
					}
				}
			}

		}

	}

	fclose(f);
}