#ifndef TESTS_H
#define TESTS_H

#include <stdio.h>

extern int test_vec_add();
extern int test_vec_sub();
extern int test_vec_scalar_mult();
extern int test_vec_dot();
extern int test_vec_cross();

extern int test_quat_add();
extern int test_quat_sub();
extern int test_quat_prod();
extern int test_quat_magnitude();

extern int run_tests();

#endif /* TESTS_H */