#ifndef __SHADER_H__
#define __SHADER_H__

#include "../lib/tga/tga_image.h"
#include "../model/model.h"
#include "../util/geo.h"
#include "../util/util.hpp"

extern Matrix44f model_view;
extern Matrix44f view_port;
extern Matrix44f projection;

void viewport(int x, int y, int w, int h, int depth);
void project(float coefficient = 0.f);  // coeff = -1/c
void lookat(Vector3f eye, Vector3f center, Vector3f up);

class IShader {
public:
  virtual ~IShader();
  virtual Vector4f vertex(int face, int nth_vert) = 0;
  virtual bool fragment(Vector3f bar, TGAColor& color) = 0;
};

void triangle(Vector4f* pts,
              IShader& shader,
              TGAImage& image,
              TGAImage& zbuffer);

class GouraudShader : public IShader {
public:
  Vector3f light_dir = Vector3f(1, 1, 1).normalize();

  GouraudShader(Model* model) : model(model) {}
  ~GouraudShader() {}

  // 这里是非标准的 vertex shader 顶点着色器
  // 标准的应该是接受 一组面的顶点参数, 返回处理后的顶点
  virtual Vector4f vertex(int iface, int nthvert);

  // 这里是非标准的 fragment shader 片元着色器
  // 标准的应该是接受 一组面的顶点参数, 返回处理后的顶点
  // pixel would not be discarded if it return false
  virtual bool fragment(Vector3f bar, TGAColor& color);

private:
  Model* model;
  // written by vertex shader, read by fragment shader
  Matrix<2, 3, float> varying_uv;
  // written by vertex shader, read by fragment shader
  Vector3f varying_intensity;
};

#endif
