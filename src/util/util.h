#ifndef UTIL_H
#define UTIL_H

template <class T> void swap(T& x, T& y) {
  T temp(x);
  x = y;
  y = temp;
}

template <class T> T abs(T x) {
  return x > 0 ? x : -x;
}

template <class T> T max(T a, T b) {
  return a > b ? a : b;
}

template <class T> T min(T a, T b) {
  return a < b ? a : b;
}

#endif  // UTIL_H
