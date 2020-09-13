#ifndef __MODEL_H__
#define __MODEL_H__

#include "../lib/tga/tga_image.h"
#include "../util/geo.h"
#include <vector>

using std::string;
using std::vector;

class Model {
private:
  vector<Vector3f> verts_;
  vector<vector<Vector3i>> faces_;
  vector<Vector3f> norms_;
  vector<Vector2f> uv_;
  TGAImage diffusemap_;
  TGAImage normalmap_;
  TGAImage specularmap_;

public:
  Model(const char* filename);
  ~Model();
  int nverts();
  int nfaces();
  Vector3f norm(int iface, int nvert);
  Vector3f norm(Vector2f uvf);
  Vector3f vert(int i);
  Vector3f vert(int iface, int nth_vert);
  Vector2f uv(int iface, int nvert);
  TGAColor diffuse(Vector2f uvf);
  float specular(Vector2f uvf);
  vector<int> face(int idx);

  void load_texture(const string& filename, const char* suffix, TGAImage& img);
};

#endif  //__MODEL_H__
