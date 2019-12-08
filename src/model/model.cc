#include "./model.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

Model::Model(const char* filename) : verts_(), faces_() {
  std::ifstream in;
  in.open(filename, std::ifstream::in);
  if (in.fail()) {
    return;
  }

  std::string line;
  while (!in.eof()) {
    // 按行获取 obj 文件中的顶点数据，以及面数据
    // 顶点数据格式 v x y z；其中 xyz 分别代表三个轴上的坐标
    // 面数据格式 f v/vt/vn v/vt/vn v/vt/vn
    // f 顶点索引 / 纹理坐标索引 / 顶点法向量索引
    std::getline(in, line);
    std::istringstream iss(line.c_str());
    char trash;
    if (!line.compare(0, 2, "v ")) {
      iss >> trash;
      Vector3f v;
      for (int i = 0; i < 3; i++)
        iss >> v.raw[i];
      verts_.push_back(v);
    } else if (!line.compare(0, 2, "f ")) {
      std::vector<int> f;
      int itrash, idx;
      iss >> trash;
      while (iss >> idx >> trash >> itrash >> trash >> itrash) {
        idx--;  // in wavefront obj all indices start at 1, not zero
        f.push_back(idx);
      }
      faces_.push_back(f);
    }
  }
  std::cerr << "# v# " << verts_.size() << " f# " << faces_.size() << std::endl;
}

Model::~Model() {}

int Model::nverts() {
  return (int)verts_.size();
}

int Model::nfaces() {
  return (int)faces_.size();
}

std::vector<int> Model::face(int idx) {
  return faces_[idx];
}

Vector3f Model::vert(int i) {
  return verts_[i];
}
