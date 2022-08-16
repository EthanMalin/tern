#ifndef TERN_H
#define TERN_H

/* --- includes ---  */
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

/* --- macros --- */

/* --- data types ---  */
typedef struct bivec {
  double xy;
  double xz;
  double yz;
} bivec;

typedef struct rotor {
  double s;
  bivec b;
} rotor;

typedef struct quat {
  double w;
  double x;
  double y;
  double z;
} quat;

typedef struct vec {
  double x;
  double y;
  double z;
} vec;

/* --- variables --- */

/* --- prototypes --- */
const quat *quat_identity();

void vec_add(const vec *l, const vec *r, vec *s);
void vec_sub(const vec *l, const vec *r, vec *d);
void vec_scalar_mult(double f, const vec *v, vec *p);
void vec_scalar_div(double f, const vec *v, vec *p);
void vec_dot(const vec *l, const vec *r, double *p);
void vec_cross(const vec *l, const vec *r, vec *p);

void quat_add(const quat *l, const quat *r, quat *s);
void quat_sub(const quat *l, const quat *r, quat *d);
void quat_scalar_mult(double f, const quat *q, quat *p);
void quat_scalar_div(double f, const quat *q, quat *p);
void quat_prod(const quat *l, const quat *r, quat *p);
void quat_norm(const quat *q, double *n);
void quat_normed(const quat *q, quat *n);
void quat_conjugate(const quat *q, quat *c);
void quat_inverse(const quat *q, quat *i);
void quat_dot(const quat *l, const quat *r, double *p);
void quat_angular_diff(const quat *l, const quat *r, double *theta);
void quat_vector_prod(const quat *q, const vec *v, quat *p);
void quat_rot_around_axis(const vec *axis, double theta, quat *r);

void vec_outer_prod(const vec *l, const vec *r, bivec *p);
void vec_geom_prod(const vec *l, const vec *r, rotor *p);

#endif /* TERN_H */
