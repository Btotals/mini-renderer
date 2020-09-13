#include "./model.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

Model::Model(const char* filename)
  : verts_(), faces_(), norms_(), uv_(), diffusemap_(), normalmap_(),
    specularmap_() {
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
      for (int i = 0; i < 3; i++) {
        iss >> v[i];
      }
      verts_.push_back(v);
    } else if (!line.compare(0, 3, "vn ")) {
      iss >> trash >> trash;
      Vector3f n;
      for (int i = 0; i < 3; i++) {
        iss >> n[i];
      }
      norms_.push_back(n);
    } else if (!line.compare(0, 3, "vt ")) {
      iss >> trash >> trash;
      Vector2f uv;
      for (int i = 0; i < 2; i++) {
        iss >> uv[i];
      }
      uv_.push_back(uv);
    } else if (!line.compare(0, 2, "f ")) {
      vector<Vector3i> f;
      Vector3i tmp;
      iss >> trash;
      while (iss >> tmp[0] >> trash >> tmp[1] >> trash >> tmp[2]) {
        for (int i = 0; i < 3; i++) {
          tmp[i]--;  // in wavefront obj all indices start at 1, not zero
        }
        f.push_back(tmp);
      }
      faces_.push_back(f);
    }
  }
  std::cout << "# v# " << verts_.size() << " f# " << faces_.size() << " vt# "
            << uv_.size() << " vn# " << norms_.size() << std::endl;

  load_texture(filename, "_diffuse.tga", diffusemap_);
  load_texture(filename, "_nm.tga", normalmap_);
  load_texture(filename, "_spec.tga", specularmap_);
}

Model::~Model() {}

int Model::nverts() {
  return (int)verts_.size();
}

int Model::nfaces() {
  return (int)faces_.size();
}

vector<int> Model::face(int idx) {
  vector<int> face;
  vector<Vector3i> faces = faces_[idx];
  for (int i = 0, size = (int)faces.size(); i < size; i++) {
    face.push_back(faces[i][0]);
  }
  return face;
}

Vector3f Model::vert(int i) {
  return verts_[i];
}

Vector3f Model::vert(int iface, int nth_vert) {
  return verts_[faces_[iface][nth_vert][0]];
}

void Model::load_texture(const string& filename,
                         const char* suffix,
                         TGAImage& img) {
  std::string texfile(filename);
  size_t dot = texfile.find_last_of(".");
  if (dot != std::string::npos) {
    texfile = texfile.substr(0, dot) + std::string(suffix);
    std::cerr << "texture file " << texfile << " loading "
              << (img.read_tga_file(texfile.c_str()) ? "ok" : "failed")
              << std::endl;
    img.flip_vertically();
  }
}

TGAColor Model::diffuse(Vector2f uvf) {
  Vector2i uv(uvf[0] * diffusemap_.get_width(),
              uvf[1] * diffusemap_.get_height());
  return diffusemap_.get(uv.x, uv.y);
}

Vector2f Model::uv(int iface, int nthvert) {
  return uv_[faces_[iface][nthvert][1]];
}

Vector3f Model::norm(int iface, int nvert) {
  int idx = faces_[iface][nvert][2];
  return norms_[idx].normalize();
}

Vector3f Model::norm(Vector2f uvf) {
  Vector2i uv(uvf[0] * normalmap_.get_width(),
              uvf[1] * normalmap_.get_height());

  TGAColor c = normalmap_.get(uv[0], uv[1]);
  Vector3f res;
  for (int i = 0; i < 3; i++) {
    res[2 - i] = (float)c[i] / 255.f * 2.f - 1.f;
  }
  return res;
}

float Model::specular(Vector2f uvf) {
  Vector2i uv(uvf[0] * specularmap_.get_width(),
              uvf[1] * specularmap_.get_height());
  return specularmap_.get(uv[0], uv[1])[0] / 1.f;
}
