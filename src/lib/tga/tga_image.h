#ifndef __IMAGE_H__
#define __IMAGE_H__

#include <fstream>

#pragma pack(push, 1)
struct TGA_Header {
  char idlength;
  char colormaptype;
  char datatypecode;
  short colormaporigin;
  short colormaplength;
  char colormapdepth;
  short x_origin;
  short y_origin;
  short width;
  short height;
  char bitsperpixel;
  char imagedescriptor;
};
#pragma pack(pop)

class TGAColor {
public:
  union {
    struct {
      unsigned char b, g, r, a;
    };
    unsigned char raw[4];
    unsigned int val;
  };
  int bytespp;

  TGAColor() : val(0), bytespp(1) {}

  TGAColor(unsigned char R, unsigned char G, unsigned char B)
    : b(B), g(G), r(R), a(255), bytespp(4) {}

  TGAColor(unsigned char R, unsigned char G, unsigned char B, unsigned char A)
    : b(B), g(G), r(R), a(A), bytespp(4) {}

  TGAColor(unsigned char v) : raw(), bytespp(1) {
    for (int i = 0; i < 4; i++)
      raw[i] = 0;
    raw[0] = v;
  }

  TGAColor(unsigned int v, int bpp) : val(v), bytespp(bpp) {}

  TGAColor(const TGAColor& c) : val(c.val), bytespp(c.bytespp) {}

  TGAColor(const unsigned char* p, int bpp) : val(0), bytespp(bpp) {
    for (int i = 0; i < bpp; i++) {
      raw[i] = p[i];
    }
  }

  TGAColor operator+(TGAColor& color) const {
    return *this + static_cast<const TGAColor&>(color);
  }

  TGAColor operator+(const TGAColor& color) const {
    TGAColor res;
    res.bytespp = bytespp > color.bytespp ? bytespp : color.bytespp;
    for (int i = 0; i < 4; i++) {
      unsigned char c = raw[i] + color.raw[i];
      res.raw[i] = c >= 255 ? 255 : c;
    }
    return res;
  }

  TGAColor operator*(TGAColor& color) const {
    return *this * static_cast<const TGAColor&>(color);
  }

  TGAColor operator*(const TGAColor& color) const {
    TGAColor res;
    res.bytespp = bytespp > color.bytespp ? bytespp : color.bytespp;
    for (int i = 0; i < 4; i++) {
      unsigned char c = raw[i] * static_cast<float>(color.raw[i]) / 255.f;
      res.raw[i] = c;
    }
    return res;
  }

  TGAColor operator*(float intensity) const {
    TGAColor res;
    res.bytespp = bytespp;
    intensity = (intensity > 1.f ? 1.f : (intensity < 0.f ? 0.f : intensity));
    for (int i = 0; i < 4; i++) {
      unsigned char c = raw[i] * intensity;
      res.raw[i] = c >= 255 ? 255 : c;
    }
    return res;
  }

  unsigned char& operator[](const int i) {
    return raw[i];
  }

  TGAColor& operator=(const TGAColor& c) {
    if (this != &c) {
      bytespp = c.bytespp;
      val = c.val;
    }
    return *this;
  }

  static TGAColor lerp(const TGAColor& c1, const TGAColor& c2, float weight) {
    return c1 * weight + c2 * (1 - weight);
  }
};

typedef enum { GRAYSCALE = 1, RGB = 3, RGBA = 4 } TGAFormat;

class TGAImage {
protected:
  unsigned char* data;
  int width;
  int height;
  int size;
  int bytespp;

  bool load_rle_data(std::ifstream& in);
  bool unload_rle_data(std::ofstream& out);

public:
  enum Format { GRAYSCALE = 1, RGB = 3, RGBA = 4 };

  TGAImage();
  TGAImage(int w, int h, int bpp);
  TGAImage(const TGAImage& img);
  bool read_tga_file(const char* filename);
  bool write_tga_file(const char* filename, bool rle = true);
  bool flip_horizontally();
  bool flip_vertically();
  bool scale(int w, int h);
  TGAColor get(int x, int y);
  bool set(int x, int y, const TGAColor& c);
  ~TGAImage();
  TGAImage& operator=(const TGAImage& img);
  int get_width();
  int get_height();
  int get_bytespp();
  int get_size();
  unsigned char* buffer();
  void clear();
};

#endif  //__IMAGE_H__
