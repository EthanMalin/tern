#include "tern.h"
#include "tests.h"

#define PRECISION         0.000015

#define MAGIC_TEST_DOUBLE  (-0.777777)
#define MAGIC_TEST_VEC  {MAGIC_TEST_DOUBLE, MAGIC_TEST_DOUBLE, MAGIC_TEST_DOUBLE}
#define MAGIC_TEST_QUAT  {MAGIC_TEST_DOUBLE, MAGIC_TEST_DOUBLE, MAGIC_TEST_DOUBLE, MAGIC_TEST_DOUBLE}

static int dbl_equal(double f1, double f2);
static int vec_equal(const vec *v1, const vec *v2);
static int quat_equal(const quat *v1, const quat *v2);

static void fail_dbl_test(const char *msg, const double exp, const double rec);
static void fail_vec_test(const char *msg, const vec *exp, const vec *rec);
static void fail_quat_test(const char *msg, const quat *exp, const quat *rec);

int run_tests() {
  int all_passed = 1;

  if(!test_vec_add()) all_passed = 0;
  if(!test_vec_sub()) all_passed = 0;
  if(!test_vec_scalar_mult()) all_passed = 0;
  if(!test_vec_dot()) all_passed = 0;
  if(!test_vec_cross()) all_passed = 0;

  if(!test_quat_add()) all_passed = 0;
  if(!test_quat_sub()) all_passed = 0;
  if(!test_quat_prod()) all_passed = 0;
  // if(!test_quat_magnitude()) all_passed = 0;
  return all_passed;
}

int test_vec_add() {
  int passed = 1;
  uint32_t i = 0;
  uint32_t iters = 3;
  const vec a[3] = {
    {1.0f, 9.0f, -1.0f}, {0.0f, 0.0f, 0.0f}, {11.0f, -11.0f, 2.0f}
  };

  const vec b[3] = {
    {-0.01f, 9.0f, -1.5f}, {0.0f, 0.0f, 0.0f}, {11.1f, -11.1f, 2.9f}
  };

  const vec c[3] = {
    {0.99f, 18.0f, -2.5f}, {0.0f, 0.0f, 0.0f}, {22.1f, -22.1f, 4.9f}
  };

  vec res = MAGIC_TEST_VEC;

  for (i = 0; i < iters; i++) {
    vec_add(&(a[i]), &(b[i]), &res);
    if (!vec_equal(&res, &c[i])) {
      passed = 0;
      fail_vec_test("test_vec_add", &c[i], &res);
    }
  }
  return passed;
}

int test_vec_sub() {
  int passed = 1;
  uint32_t i = 0;
  uint32_t iters = 3;
  const vec a[3] = {
    {1.0f, 9.0f, -1.0f}, {0.0f, 0.0f, 0.0f}, {11.0f, -11.0f, 2.0f}
  };

  const vec b[3] = {
    {-0.01f, 9.0f, -1.5f}, {0.0f, 0.0f, 0.0f}, {11.1f, -11.1f, 2.9f}
  };

  const vec c[3] = {
    {1.01f, 0.0f, 0.5f}, {0.0f, 0.0f, 0.0f}, {-0.1f, 0.1f, -0.9f}
  };

  vec res = MAGIC_TEST_VEC;

  for (i = 0; i < iters; i++) {
    vec_sub(&(a[i]), &(b[i]), &res);
    if (!vec_equal(&res, &c[i])) {
      passed = 0;
      fail_vec_test("test_vec_sub", &c[i], &res);
    }
  }
  return passed;
}

int test_vec_scalar_mult() {
  int passed = 1;
  uint32_t i = 0;
  uint32_t iters = 3;

  const double a[3] = {
    1.0f, -1.0f, 2.9f
  };

  const vec b[3] = {
    {1.0f, 9.0f, -1.0f}, {0.0f, 0.0f, 0.0f}, {11.0f, -11.0f, 2.0f}
  };

  const vec c[3] = {
    {1.0f, 9.0f, -1.0f}, {0.0f, 0.0f, 0.0f}, {31.9f, -31.9f, 5.8f}
  };

  vec res = MAGIC_TEST_VEC;

  for (i = 0; i < iters; i++) {
    vec_scalar_mult(a[i], &(b[i]), &res);
    if (!vec_equal(&res, &c[i])) {
      passed = 0;
      fail_vec_test("test_vec_scalar_mult", &c[i], &res);
    }
  }
  return passed;
}

int test_vec_dot() {
  int passed = 1;
  uint32_t i = 0;
  uint32_t iters = 3;
  const vec a[3] = {
    {1.0f, 9.0f, -1.0f}, {0.0f, 0.0f, 0.0f}, {11.0f, -11.0f, 2.0f}
  };

  const vec b[3] = {
    {-0.01f, 9.0f, -1.5f}, {0.0f, 0.0f, 0.0f}, {11.1f, -11.1f, 2.9f}
  };

  const double c[3] = {
    82.49, 0.0f, 250.0
  };

  double res = MAGIC_TEST_DOUBLE;

  for (i = 0; i < iters; i++) {
    vec_dot(&(a[i]), &(b[i]), &res);
    if (!dbl_equal(res, c[i])) {
      passed = 0;
      fail_dbl_test("test_vec_dot", c[i], res);
    }
  }
  return passed;
}

int test_vec_cross() {
  int passed = 1;
  uint32_t i = 0;
  uint32_t iters = 3;
  const vec a[3] = {
    {1.0f, 9.0f, -1.0f}, {0.0f, 0.0f, 0.0f}, {11.0f, -11.0f, 2.0f}
  };

  const vec b[3] = {
    {-0.01f, 9.0f, -1.5f}, {0.0f, 0.0f, 0.0f}, {11.1f, -11.1f, 2.9f}
  };

  const vec c[3] = {
    {-4.5f, 1.51f, 9.09f}, {0.0f, 0.0f, 0.0f}, {-9.7f, -9.7f, 6.24945e-15}
  };

  vec res = MAGIC_TEST_VEC;

  for (i = 0; i < iters; i++) {
    vec_cross(&(a[i]), &(b[i]), &res);
    if (!vec_equal(&res, &c[i])) {
      passed = 0;
      fail_vec_test("test_vec_cross", &c[i], &res);
    }
  }
  return passed;
}

int test_quat_add() {
  int passed = 1;
  uint32_t i = 0;
  uint32_t iters = 3;
  const quat a[3] = {
    {1.0f, 9.0f, -1.0f, -0.82}, {0.0f, 0.0f, 0.0f, 0.0f}, {11.0f, -11.0f, 2.0f, 0.0f}
  };

  const quat b[3] = {
    {-0.01f, 9.0f, -1.5f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f}, {11.1f, -11.1f, 2.9f, 4.4f}
  };

  const quat c[3] = {
    {0.99f, 18.0f, -2.5f, -0.82f}, {0.0f, 0.0f, 0.0f, 0.0f}, {22.1f, -22.1f, 4.9f, 4.4f}
  };

  quat res = MAGIC_TEST_QUAT;

  for (i = 0; i < iters; i++) {
    quat_add(&(a[i]), &(b[i]), &res);
    if (!quat_equal(&res, &c[i])) {
      passed = 0;
      fail_quat_test("test_quat_add", &c[i], &res);
    }
  }
  return passed;
}

int test_quat_sub() {
  int passed = 1;
  uint32_t i = 0;
  uint32_t iters = 3;
  const quat a[3] = {
    {1.0f, 9.0f, -1.0f, -0.82}, {0.0f, 0.0f, 0.0f, 0.0f}, {11.0f, -11.0f, 2.0f, 0.0f}
  };

  const quat b[3] = {
    {-0.01f, 9.0f, -1.5f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f}, {11.1f, -11.1f, 2.9f, 4.4f}
  };

  const quat c[3] = {
    {1.01f, 0.0f, 0.5f, -0.82f}, {0.0f, 0.0f, 0.0f, 0.0f}, {-0.1f, 0.1f, -0.9f, -4.4f}
  };

  quat res = MAGIC_TEST_QUAT;

  for (i = 0; i < iters; i++) {
    quat_sub(&(a[i]), &(b[i]), &res);
    if (!quat_equal(&res, &c[i])) {
      passed = 0;
      fail_quat_test("test_quat_sub", &c[i], &res);
    }
  }
  return passed;
}

int test_quat_prod() {
  int passed = 1;
  uint32_t i = 0;
  uint32_t iters = 4;
  const quat a[] = {
    {1.0f, 9.0f, -1.0f, -0.82}, {-0.01f, 9.0f, -1.5f, 0.0f}, {0.0f, 0.0f, 0.0f, 0.0f}, {11.0f, -11.0f, 2.0f, 0.0f}
  };

  const quat b[] = {
    {-0.01f, 9.0f, -1.5f, 0.0f}, {1.0f, 9.0f, -1.0f, -0.82f}, {0.0f, 0.0f, 0.0f, 0.0f}, {11.1f, -11.1f, 2.9f, 4.4f}
  };

  const quat c[] = {
    {-82.51, 7.68f, -8.87f, -4.4918f}, {-82.51, 10.14f, 5.89f, 4.5082f}, {0.0f, 0.0f, 0.0f, 0.0f}, {-5.8f, -235.4f, 102.5f, 38.7f}
  };

  quat res = MAGIC_TEST_QUAT;

  for (i = 0; i < iters; i++) {
    quat_prod(&(a[i]), &(b[i]), &res);
    if (!quat_equal(&res, &c[i])) {
      passed = 0;
      fail_quat_test("test_quat_prod", &c[i], &res);
    }
  }
  return passed;
}

int dbl_equal(double f1, double f2) {
  return (((f1 - PRECISION) < f2) && ((f1 + PRECISION) > f2));
}

int vec_equal(const vec *v1, const vec *v2) {
  return dbl_equal(v1->x, v2->x) && dbl_equal(v1->y, v2->y) && dbl_equal(v1->z, v2->z);
}

int quat_equal(const quat *q1, const quat *q2) {
  return dbl_equal(q1->w, q2->w) && dbl_equal(q1->x, q2->x) && dbl_equal(q1->y, q2->y) && dbl_equal(q1->z, q2->z);
}

void fail_dbl_test(const char *msg, const double exp, const double rec) {
  printf("TEST FAILED: %s\n", msg);
  printf("\texpected: %f\n", exp);
  printf("\treceived: %f\n", rec);
}

void fail_vec_test(const char *msg, const vec *exp, const vec *rec) {
  printf("TEST FAILED: %s\n", msg);
  printf("\texpected: (%f, %f, %f)\n", exp->x, exp->y, exp->z);
  printf("\treceived: (%f, %f, %f)\n", rec->x, rec->y, rec->z);
}

void fail_quat_test(const char *msg, const quat *exp, const quat *rec) {
  printf("TEST FAILED: %s\n", msg);
  printf("\texpected: (%f, %f, %f, %f)\n", exp->w, exp->x, exp->y, exp->z);
  printf("\treceived: (%f, %f, %f, %f)\n", exp->w, rec->x, rec->y, rec->z);
}

