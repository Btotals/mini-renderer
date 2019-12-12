#ifndef __MATRIX_H__
#define __MATRIX_H__

#include "./geometry.h"
#include <iostream>
#include <vector>

#define MATRIX_SIZE 4

using std::vector;

typedef vector<float> Row;

class Matrix {
private:
  vector<Row> m_;
  int row_, col_;

public:
  static Matrix identity(int dimensions);
  static Vector3f m2v(Matrix m);
  static Matrix v2m(Vector3f v);

  Matrix();
  Matrix(int r = MATRIX_SIZE, int c = MATRIX_SIZE);
  Matrix(Vector3f v);

  int nrows();
  int ncols();

  Row& operator[](const int i);

  Matrix operator*(const Matrix& a);
  Matrix transpose();
  Matrix inverse();

  friend std::ostream& operator<<(std::ostream& s, Matrix& m);
};

#endif
