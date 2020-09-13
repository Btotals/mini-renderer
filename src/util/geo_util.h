#include "./geo.h"

Vector3f reflect(Vector3f in, Vector3f normal) {
  Vector3f i_n = normal.normalize();
  return in - i_n * (in * i_n) * 2.f;
}
