#include "./canvas/canvas.h"
#include "./shader/shader.h"
#include "./util/string.h"
#include "./util/transform.h"
#include <iostream>

int main(int argc, char** argv) {
  const int width = 1200;
  const int height = 1200;

  vector<Model*> model_list;
  for (int i = 1; i < argc; i++) {
    if (endsWith(std::string(argv[i]), ".obj")) {
      model_list.push_back(new Model(argv[i]));
    } else {
      std::cout << "file is not .obj format" << std::endl;
      return 0;
    }
  }

  Vector3f position(0, 0, 3);
  Vector3f to(0, 0, -1);
  Vector3f up(0, 1, 0);

  lookat(position, to, up);
  viewport(0, 0, width, height);

  // projectOrthographic(-2, 2, -2, 2, 0.1, 10);
  projectPerspective(45, width / height, 0.1, 10);

  set_light(Vector3f(-1, -1, 0), TGAColor(255, 255, 255));

  TGAImage image(width, height, TGAImage::RGB);

  // with float32 zbuffer we could get depth in higher precise
  // TGAImage zbuffer(width, height, TGAImage::GRAYSCALE);
  float* zbuffer = new float[width * height];
  float negative_max = -std::numeric_limits<float>::max();
  for (int i = width * height; i--; zbuffer[i] = negative_max) {}

  for (int begin = 0, end = model_list.size(); begin < end; begin++) {
    Model* model = model_list[begin];

    FirstShader shader(model);
    // GouraudShader shader(model);
    // PhongShader shader(model);

    for (int i = 0; i < model->nfaces(); i++) {
      // for (int i = 0; i < 20; i++) {
      Vector4f screen_coords[3];

      for (int j = 0; j < 3; j++) {
        screen_coords[j] = shader.vertex(i, j);
      }
      // triangle(screen_coords, shader, image, zbuffer);
      triangle(shader.varying_pts, shader, image, zbuffer);
    }
  }

  image.flip_vertically();
  // zbuffer.flip_vertically();
  image.write_tga_file("output.tga");
  // zbuffer.write_tga_file("zbuffer.tga");

  for (int begin = 0, end = model_list.size(); begin < end; begin++) {
    delete model_list[begin];
  }
  delete[] zbuffer;

  return 0;
}
