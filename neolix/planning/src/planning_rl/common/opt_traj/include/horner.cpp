#include "horner.h"

int horner(const std::vector<double>& a, double x, std::vector<double>& w) {
  int n = a.size() - 1;
  if ((int)w.size() != n + 1) {
    return -1;
  }

  w.assign(n + 1, a[n]);  // w[0] is also return value

  for (int i = n - 1; i >= 0; i--) {
    w[0] = w[0] * x + a[i];
    for (int j = 1; j <= i; j++) w[j] = w[j] * x + w[j - 1];
  }

  for (int i = 2; i <= n; i++) {
    for (int j = 2; j <= i; j++) w[i] = w[i] * j;  // multiply with i!
  }
  return 0;
}
