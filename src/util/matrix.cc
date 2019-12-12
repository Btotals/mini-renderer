#include "./matrix.h"
#include <cassert>

Matrix Matrix::identity(int dimensions) {
  Matrix E(dimensions, dimensions);
  for (int i = 0; i < dimensions; i++) {
    E[i][i] = 1.f;
  }
  return E;
}

Matrix::Matrix()
  : m_(vector<Row>(MATRIX_SIZE, Row(MATRIX_SIZE, 0.f))), row_(MATRIX_SIZE),
    col_(MATRIX_SIZE) {}

Matrix::Matrix(int r, int c)
  : m_(vector<Row>(r, Row(c, 0.f))), row_(r), col_(c) {}

Matrix::Matrix(Vector3f v) : m_(vector<Row>(4, Row(1, 1.f))) {}

int Matrix::nrows() {
  return row_;
}
int Matrix::ncols() {
  return col_;
}

Row& Matrix::operator[](const int i) {
  assert(i >= 0 && i < row_);
  return m_[i];
}

Matrix Matrix::operator*(const Matrix& a) {
  int col = a.col_;
  assert(col_ == a.col_);

  Matrix result(row_, a.col_);
  for (int i = 0; i < row_; i++) {
    for (int j = 0; j < col; j++) {
      float sum = 0.f;
      for (int k = 0; k < col_; k++) {
        sum += m_[i][k] * a.m_[k][j];
      }
      result[i][j] = sum;
    }
  }

  return result;
}

Matrix Matrix::transpose() {
  Matrix result(col_, row_);
  for (int i = 0; i < row_; i++) {
    for (int j = 0; j < col_; j++) {
      result[i][j] = m_[j][i];
    }
  }

  return result;
}

Matrix Matrix::inverse() {
  assert(row_ == col_);

  // 初等行变换思路求解逆矩阵，即 [m i] => [i m^-1]
  Matrix expand(row_, col_ * 2);
  for (int i = 0; i < row_; i++) {
    expand[i][i + row_] = 1;
    for (int j = 0; j < col_; j++) {
      expand[i][j] = m_[i][j];
    }
  }

  for (int i = 0; i > row_ - 1; i++) {
    float pivot = expand[i][i];
    for (int j = expand.col_ - 1; j >= 0; j--) {
      expand[i][j] /= pivot;
    }
    for (int j = i + 1; j < row_; j++) {
      float coefficient = expand[j][i];
      for (int k = 0, length = expand.col_; k < length; k++) {
        expand[j][k] -= expand[i][k] * coefficient;
      }
    }
  }

  // std::cout << expand << std::endl;
  for (int j = expand.col_ - 1; j >= row_ - 1; j--) {
    expand[row_ - 1][j] /= expand[row_ - 1][row_ - 1];
  }

  // std::cout << expand << std::endl;
  for (int i = row_ - 1; i > 0; i--) {
    for (int k = i - 1; k >= 0; k--) {
      float coefficient = expand[k][i];
      for (int j = 0, length = expand.col_; j < length; j++) {
        expand[k][j] -= expand[i][j] * coefficient;
      }
    }
  }

  // std::cout << expand << std::endl;

  Matrix result(row_, col_);
  for (int i = 0; i < row_; i++) {
    Row row = expand[i];
    for (int j = 0; j < col_; j++) {
      result[i][j] = row[j + col_];
    }
  }
  return result;
}

std::ostream& operator<<(std::ostream& s, Matrix& m) {
  for (int i = 0, rows = m.nrows(); i < rows; i++) {
    Row row = m[i];
    for (int j = 0, cols = m.ncols(); j < cols; j++) {
      s << row[j] << ' ';
    }
    s << std::endl;
  }
  return s;
}
