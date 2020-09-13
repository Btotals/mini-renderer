#ifndef __SHADER_H__
#define __SHADER_H__

#include "../lib/tga/tga_image.h"
#include "../model/model.h"
#include "../util/geo.h"
#include "../util/util.hpp"

extern Matrix44f view;
extern Matrix44f view_port;
extern Matrix44f projection;

void set_light(Vector3f light_dir, TGAColor color);
void viewport(int x, int y, int w, int h);
void project(float coefficient = 0.f);  // coeff = -1/c
void projectOrthographic(float l,
                         float r,
                         float b,
                         float t,
                         float near,
                         float far);
void projectPerspective(float fov, float aspect, float near, float far);
void lookat(Vector3f eye, Vector3f center, Vector3f up);

class IShader {
public:
  // virtual ~IShader();
  virtual Vector4f vertex(int face, int nth_vert) = 0;
  virtual bool fragment(Vector3f bar, TGAColor& color) = 0;
};

void triangle(Vector4f* pts, IShader& shader, TGAImage& image, float* zbuffer);

// 增加透视校正
void triangle(Matrix<4, 3, float>& varying_pts,
              IShader& shader,
              TGAImage& image,
              float* zbuffer);

class FirstShader : public IShader {
public:
  FirstShader(Model* model) : model(model) {}
  ~FirstShader() {}

  // 这里是非标准的 vertex shader 顶点着色器
  // 标准的应该是接受 一组面的顶点参数, 返回处理后的顶点
  virtual Vector4f vertex(int iface, int nthvert);

  // 这里是非标准的 fragment shader 片元着色器
  // 标准的应该是接受 一组面的顶点参数, 返回处理后的顶点
  // pixel would not be discarded if it return false
  virtual bool fragment(Vector3f bar, TGAColor& color);

  // private:
  Model* model;

  // triangle uv coordinates
  // written by the vertex shader read by the fragment shader
  Matrix<2, 3, float> varying_uv;

  Matrix<3, 3, float> varying_normal;

  Matrix<4, 3, float> varying_pts;

  Matrix<3, 3, float> ndc_tri;  // triangle in normalized device coordinates
};

class GouraudShader : public IShader {
public:
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
  // triangle uv coordinates
  // written by the vertex shader read by the fragment shader
  Matrix<2, 3, float> varying_uv;

  // intensity of each point in triangle
  // written by vertex shader, read by fragment shader
  Vector3f varying_intensity;
};

class PhongShader : public IShader {
public:
  // triangle coordinates (clip coordinates)
  // written by vertex shader, read by fragment shader
  // Matrix<4, 3, float> varying_tri;

  PhongShader(Model* model) : model(model) {}
  ~PhongShader() {}

  virtual Vector4f vertex(int iface, int nthvert);

  virtual bool fragment(Vector3f bar, TGAColor& color);

private:
  Model* model;

  // triangle uv coordinates
  // written by the vertex shader read by the fragment shader
  Matrix<2, 3, float> varying_uv;

  // normal per vertex to be interpolated by fragment shader
  Matrix<3, 3, float> varying_nrm;
};

#endif
