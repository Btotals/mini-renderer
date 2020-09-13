#ifndef __GEO_H__
#define __GEO_H__

#include <cassert>
#include <cmath>
#include <iostream>
#include <ostream>

template <size_t row, size_t col, typename T> class Matrix;

template <size_t dimension, typename T> class Vector {
public:
  union {
    T raw[dimension];
  };

  Vector() {
    for (size_t i = dimension; i--; raw[i] = T()) {}
  }

  T& operator[](size_t i) {
    assert(i < dimension);
    return raw[i];
  }
  const T& operator[](size_t i) const {
    assert(i < dimension);
    return raw[i];
  }
};

template <size_t dimension, typename T>
std::ostream& operator<<(std::ostream& s, Vector<dimension, T>& v) {
  s << static_cast<const Vector<dimension, T>&>(v);
  return s;
}

template <size_t dimension, typename T>
std::ostream& operator<<(std::ostream& s, const Vector<dimension, T>& v) {
  for (size_t i = 0; i < dimension; i++) {
    s << v[i] << " ";
  }
  s << std::endl;
  return s;
}

template <typename T> class Vector<2, T> {
public:
  union {
    struct {
      T x, y;
    };
    T raw[2];
  };
  // T x, y;

  Vector<2, T>() : x(0), y(0) {}
  Vector<2, T>(T _x, T _y) : x(_x), y(_y) {}

  template <class U> Vector<2, T>(const Vector<2, U>& v);

  float length() {
    return std::sqrt(x * x + y * y);
  }

  Vector<2, T>& normalize() {
    float l = length();
    // for (size_t i = 2; i--; this[i] = this[i] / l) {}
    x /= l;
    y /= l;
    return *this;
  }

  T& operator[](size_t i) {
    assert(i < 2);
    return i <= 0 ? x : y;
  }

  const T& operator[](size_t i) const {
    assert(i < 2);
    return i <= 0 ? x : y;
  }
};

template <typename T> class Vector<3, T> {
public:
  union {
    struct {
      T x, y, z;
    };
    T raw[3];
  };
  // T x, y, z;

  Vector<3, T>() : x(0), y(0), z(0) {}
  Vector<3, T>(T _x, T _y, T _z) : x(_x), y(_y), z(_z) {}

  template <class U> Vector<3, T>(const Vector<3, U>& v);

  float length() {
    return std::sqrt(x * x + y * y + z * z);
  }

  Vector<3, T>& normalize() {
    float l = length();
    // *this = (*this) * (1 / length());
    // for (size_t i = 3; i--; this[i] = this[i] / l) {}
    x /= l;
    y /= l;
    z /= l;
    return *this;
  }

  T& operator[](size_t i) {
    assert(i < 3);
    return i <= 0 ? x : (1 == i ? y : z);
  }

  const T& operator[](size_t i) const {
    assert(i < 3);
    return i <= 0 ? x : (1 == i ? y : z);
  }

  // friend std::ostream& operator<<(std::ostream& s, Vector<3, T>& v) {
  //   s << v.x << " " << v.y << " " << v.z << std::endl;
  //   return s;
  // }
};

template <> template <> Vector<3, int>::Vector(const Vector<3, float>& v);

template <> template <> Vector<3, float>::Vector(const Vector<3, int>& v);

template <> template <> Vector<2, int>::Vector(const Vector<2, float>& v);

template <> template <> Vector<2, float>::Vector(const Vector<2, int>& v);

// Vector Operations
template <size_t dimension, typename T>
Vector<dimension, T> operator+(Vector<dimension, T>& v1,
                               const Vector<dimension, T>& v2) {
  return static_cast<const Vector<dimension, T>&>(v1) + v2;
}

template <size_t dimension, typename T>
Vector<dimension, T> operator+(const Vector<dimension, T>& v1,
                               const Vector<dimension, T>& v2) {
  Vector<dimension, T> res;
  for (int i = dimension; i--; res[i] = v1[i] + v2[i]) {}
  return res;
}

template <size_t dimension, typename T, typename U>
Vector<dimension, T> operator+(Vector<dimension, T>& v1,
                               const Vector<dimension, U>& v2) {
  return static_cast<const Vector<dimension, T>&>(v1) + v2;
}

template <size_t dimension, typename T, typename U>
Vector<dimension, T> operator+(const Vector<dimension, T>& v1,
                               const Vector<dimension, U>& v2) {
  Vector<dimension, T> res;
  for (int i = dimension; i--; res[i] = v1[i] - v2[i]) {}
  return res;
}

template <size_t dimension, typename T>
Vector<dimension, T> operator-(Vector<dimension, T>& v1,
                               const Vector<dimension, T>& v2) {
  return static_cast<const Vector<dimension, T>&>(v1) - v2;
}

template <size_t dimension, typename T>
Vector<dimension, T> operator-(const Vector<dimension, T>& v1,
                               const Vector<dimension, T>& v2) {
  Vector<dimension, T> res;
  for (int i = dimension; i--; res[i] = v1[i] - v2[i]) {}
  return res;
}

template <size_t dimension, typename T>
T operator*(Vector<dimension, T>& v1, Vector<dimension, T>& v2) {
  return static_cast<const Vector<dimension, T>&>(v1) *
         static_cast<const Vector<dimension, T>&>(v2);
}

template <size_t dimension, typename T>
T operator*(Vector<dimension, T>& v1, const Vector<dimension, T>& v2) {
  return static_cast<const Vector<dimension, T>&>(v1) * v2;
}

template <size_t dimension, typename T>
T operator*(const Vector<dimension, T>& v1, const Vector<dimension, T>& v2) {
  T res = T();
  for (int i = dimension; i--; res += v1[i] * v2[i]) {}
  return res;
}

template <size_t dimension, typename T, typename U>
Vector<dimension, T> operator*(Vector<dimension, T>& v1, const U& ratio) {
  return static_cast<const Vector<dimension, T>&>(v1) * ratio;
}

template <size_t dimension, typename T, typename U>
Vector<dimension, T> operator*(const Vector<dimension, T>& v1, const U& ratio) {
  Vector<dimension, T> res;
  for (size_t i = dimension; i--; res[i] = v1[i] * ratio) {}
  return res;
}

template <size_t dimension, typename T, typename U>
Vector<dimension, T> operator/(Vector<dimension, T>& v1, const U& ratio) {
  Vector<dimension, T> res;
  for (size_t i = dimension; i--; res[i] = v1[i] / ratio) {}
  return res;
}

template <typename T> Vector<3, T> operator^(Vector<3, T> v1, Vector<3, T> v2) {
  return Vector<3, T>(v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z,
                      v1.x * v2.y - v1.y * v2.x);
}

template <size_t length, size_t dimension, typename T>
Vector<length, T> embed(const Vector<dimension, T>& v, T fill = 1) {
  Vector<length, T> res;
  for (size_t i = length; i--; res[i] = i < dimension ? v[i] : fill) {}
  return res;
}

template <size_t length, size_t dimension, typename T>
Vector<length, T> project(const Vector<dimension, T>& v) {
  Vector<length, T> res;
  for (size_t i = length; i--; res[i] = v[i]) {}
  return res;
}

// calculating det using Laplace expansion:
template <size_t dimension, typename T> struct dt {
  static T det(const Matrix<dimension, dimension, T>& m) {
    T res = T();
    for (size_t i = dimension; i--; res += m[0][i] * m.cofactor(0, i)) {}
    return res;
  }
};

template <typename T> struct dt<1, T> {
  static T det(const Matrix<1, 1, T>& m) {
    return m[0][0];
  }
};

// Matrix
template <size_t row, size_t col, typename T> class Matrix {
private:
  Vector<col, T> rows[row];

public:
  static Matrix<row, col, T> identity() {
    Matrix<row, col, T> res;
    for (size_t i = row; i--;)
      for (size_t j = col; j--; res[i][j] = i == j ? 1 : 0)
        ;
    return res;
  }

  Matrix() {}

  Vector<col, T>& operator[](size_t index) {
    assert(index < row);
    return rows[index];
  }

  const Vector<col, T>& operator[](size_t index) const {
    assert(index < row);
    return rows[index];
  }

  Vector<row, T> column(size_t index) {
    assert(index < col);
    Vector<row, T> res;
    for (size_t i = row; i--; res[i] = rows[i][index]) {}

    return res;
  }

  const Vector<row, T> column(size_t index) const {
    assert(index < col);
    Vector<row, T> res;
    for (size_t i = row; i--; res[i] = rows[i][index]) {}

    return res;
  }

  void set_column(size_t index, Vector<row, T>& v) {
    set_column(index, const_cast<const Vector<row, T>&>(v));
  }

  void set_column(size_t index, const Vector<row, T>& v) {
    assert(index < col);
    for (size_t i = row; i--; rows[i][index] = v[i]) {}
  }

  /**
   * @brief Get the minor object
   *
   * @param r row index to be removed
   * @param c col index to be removed
   * @return Matrix<row - 1, col - 1, T> minor matrix of origin
   */
  Matrix<row - 1, col - 1, T> get_minor(size_t r, size_t c) const {
    Matrix<row - 1, col - 1, T> res;
    for (size_t i = row - 1; i--;) {
      for (size_t j = col - 1; j--;
           res[i][j] = rows[i < r ? i : i + 1][j < c ? j : j + 1]) {}
    }

    return res;
  }

  T det() const {
    return dt<col, T>::det(*this);
  }

  T cofactor(size_t r, size_t c) const {
    return get_minor(r, c).det() * ((r + c) % 2 ? -1 : 1);
  }

  // classical adjoint matrix: which is transpose of cofactor matrix
  Matrix<row, col, T> adjugate() const {
    Matrix<row, col, T> res;
    for (size_t i = row; i--;) {
      for (size_t j = col; j--; res[i][j] = cofactor(i, j)) {}
    }

    return res;
  }

  Matrix<col, row, T> transpose() {
    Matrix<col, row, T> res;
    for (size_t i = col; i--; res[i] = this->column(i)) {}

    return res;
  }

  Matrix<row, col, T> invert_transpose() {
    Matrix<row, col, T> res = adjugate();
    T tmp = res[0] * rows[0];

    return res / tmp;
  }

  Matrix<row, col, T> invert() {
    return invert_transpose().transpose();
  }
};

template <size_t row, size_t col, typename T>
Vector<row, T> operator*(Matrix<row, col, T>& lhs, Vector<col, T>& rhs) {
  return static_cast<const Matrix<row, col, T>&>(lhs) *
         static_cast<const Vector<col, T>&>(rhs);
}

template <size_t row, size_t col, typename T>
Vector<row, T> operator*(Matrix<row, col, T>& lhs, const Vector<col, T>& rhs) {
  return static_cast<const Matrix<row, col, T>&>(lhs) * rhs;
}

template <size_t row, size_t col, typename T>
Vector<row, T> operator*(const Matrix<row, col, T>& lhs,
                         const Vector<col, T>& rhs) {
  Vector<row, T> res;
  for (size_t i = row; i--; res[i] = lhs[i] * rhs) {}

  return res;
}

template <size_t row, size_t col_1, size_t col_2, typename T>
Matrix<row, col_2, T> operator*(Matrix<row, col_1, T>& lhs,
                                Matrix<col_1, col_2, T>& rhs) {
  return static_cast<const Matrix<row, col_1, T>&>(lhs) *
         static_cast<const Matrix<col_1, col_2, T>&>(rhs);
}

template <size_t row, size_t col_1, size_t col_2, typename T>
Matrix<row, col_2, T> operator*(const Matrix<row, col_1, T>& lhs,
                                Matrix<col_1, col_2, T>& rhs) {
  return lhs * static_cast<const Matrix<col_1, col_2, T>&>(rhs);
}

template <size_t row, size_t col_1, size_t col_2, typename T>
Matrix<row, col_2, T> operator*(const Matrix<row, col_1, T>& lhs,
                                const Matrix<col_1, col_2, T>& rhs) {
  Matrix<row, col_2, T> res;
  for (size_t i = row; i--;)
    for (size_t j = col_2; j--; res[i][j] = lhs[i] * rhs.column(j)) {}

  return res;
}

template <size_t row, size_t col, typename T>
Matrix<row, col, T> operator/(Matrix<row, col, T>& lhs, const T& rhs) {
  for (size_t i = row; i--; lhs[i] = lhs[i] / rhs) {}

  return lhs;
}

template <size_t row, size_t col, class T>
std::ostream& operator<<(std::ostream& out, Matrix<row, col, T>& m) {
  for (size_t i = 0; i < row; i++) {
    out << m[i];
  }

  return out;
}

typedef Vector<2, float> Vector2f;
typedef Vector<2, int> Vector2i;
typedef Vector<3, float> Vector3f;
typedef Vector<3, int> Vector3i;
typedef Vector<4, float> Vector4f;
typedef Matrix<4, 4, float> Matrix44f;

#endif
