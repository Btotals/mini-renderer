#include "./shader/shader.h"
#include "./util/string.h"
#include <iostream>

int main(int argc, char** argv) {
  Model* model = NULL;
  const int width = 800;
  const int height = 800;

  if (2 == argc) {
    if (endsWith(std::string(argv[1]), ".obj")) {
      model = new Model(argv[1]);
    } else {
      std::cout << "file is not .obj format" << std::endl;
      return 0;
    }
  } else {
    std::cout << "must enter path of .obj file" << std::endl;
    return 0;
  }

  // Vector3f light_dir(1, 1, 1);
  Vector3f eye(3, 1, 3);
  Vector3f center(0, 0, 0);
  Vector3f up(0, 1, 0);

  lookat(eye, center, up);
  viewport(width / 8, height / 8, width * 3 / 4, height * 3 / 4, 255);
  project(-1.f / (eye - center).length());

  // light_dir.normalize();

  TGAImage image(width, height, TGAImage::RGB);
  TGAImage zbuffer(width, height, TGAImage::GRAYSCALE);

  GouraudShader shader(model);

  for (int i = 0; i < model->nfaces(); i++) {
    // for (int i = 0; i < 1; i++) {
    Vector4f screen_coords[3];

    for (int j = 0; j < 3; j++) {
      screen_coords[j] = shader.vertex(i, j);
    }
    triangle(screen_coords, shader, image, zbuffer);
  }

  image.flip_vertically();
  zbuffer.flip_vertically();
  image.write_tga_file("output.tga");
  zbuffer.write_tga_file("zbuffer.tga");

  delete model;

  return 0;
}
