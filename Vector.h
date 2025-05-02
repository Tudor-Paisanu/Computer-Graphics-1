#pragma once

#include <vector>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define EPSILON 1e-8
#define GAMMA 2.2
#define INF std::numeric_limits<double>::infinity()


class Vector {
    public:
        explicit Vector(double x = 0, double y = 0, double z = 0) {
            data[0] = x;
            data[1] = y;
            data[2] = z;
        }
        double norm2() const {
            return data[0] * data[0] + data[1] * data[1] + data[2] * data[2];
        }
        double norm() const {
            return sqrt(norm2());
        }
        void normalize() {
            double n = norm();
            data[0] /= n;
            data[1] /= n;
            data[2] /= n;
        }

        int get_longest() {
            if (data[0] > data[1] && data[0] > data[2]) return 0;
            if (data[1] > data[2]) return 1;
            return 2;
        }

        double operator[](int i) const { return data[i]; };
        double& operator[](int i) { return data[i]; };
        double data[3];
    };
    
    inline Vector operator+(const Vector& a, const Vector& b) {
        return Vector(a[0] + b[0], a[1] + b[1], a[2] + b[2]);
    }

    inline Vector operator-(const Vector& a, const Vector& b) {
        return Vector(a[0] - b[0], a[1] - b[1], a[2] - b[2]);
    }
    inline Vector operator*(const double a, const Vector& b) {
        return Vector(a*b[0], a*b[1], a*b[2]);
    }
    inline Vector operator*(const Vector& a, const double b) {
        return Vector(a[0]*b, a[1]*b, a[2]*b);
    }
    inline Vector operator/(const Vector& a, const double b) {
        return Vector(a[0] / b, a[1] / b, a[2] / b);
    }
    
    inline double dot(const Vector& a, const Vector& b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }
    inline Vector cross(const Vector& a, const Vector& b) {
        return Vector(a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]);
    }
    
    inline Vector comp_wise_mult(const Vector& a, const Vector& b) {
        return Vector(a[0] * b[0], a[1] * b[1], a[2] * b[2]);
    }

