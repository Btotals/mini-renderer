#ifndef __MODEL_H__
#define __MODEL_H__

#include "../util/geometry.h"
#include <vector>

using std::vector;

class Model {
private:
  vector<Vector3f> verts_;
  vector<vector<int>> faces_;

public:
  Model(const char* filename);
  ~Model();
  int nverts();
  int nfaces();
  Vector3f vert(int i);
  vector<int> face(int idx);
};

#endif  //__MODEL_H__
