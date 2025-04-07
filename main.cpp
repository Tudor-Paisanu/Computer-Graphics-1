#define _CRT_SECURE_NO_WARNINGS 1
#include <vector>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include "Scene.h"


int main() {
	int W = 512, H = 512;
	Camera cmr(Vector(0.0, 0.0, 55.0), M_PI/3, W, H);
	Vector S(-10.0, 20.0, 40.0);
	Scene scene(cmr, S, 100000);
	scene.add_object(Sphere(Vector(0, 0, 1000), 940,  Vector(0.9, 0.2, 0.9)));
	scene.add_object(Sphere(Vector(0, 0, 0), 7, Vector(0.8, 0.8, 0.8), Sphere :: Material (Sphere :: MIRROR)));
	scene.add_object(Sphere(Vector(-15, 0, 0), 7, Vector(0.8, 0.8, 0.8), Sphere :: Material (Sphere :: TRANSPARENT)));
	scene.add_object(Sphere(Vector(15, 0, 0), 7, Vector(0.8, 0.8, 0.8), Sphere :: Material (Sphere :: TRANSPARENT)));
	scene.add_object(Sphere(Vector(15, 0, 0), 6.9, Vector(0.8, 0.8, 0.8), Sphere :: Material (Sphere :: TRANSPARENT), 1.0 /1.5, true));
	scene.add_object(Sphere(Vector(0, 0, -1000), 940, Vector(0.4, 0.8, 0.7)));
	scene.add_object(Sphere(Vector(0, -1000, 0), 990, Vector(0.6, 0.5, 0.1)));
	scene.add_object(Sphere(Vector(0, 1000, 0), 940, Vector(0.9, 0.2, 0.9)));
	std::vector<unsigned char> image = scene.take_picture();
    stbi_write_png("image.png", W, H, 3, &image[0], 0);

	return 0;
}