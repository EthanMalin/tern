#include "tern.h"

static const quat _identity = {1.0f, 0.0f, 0.0f, 0.0f};

const quat *quat_identity() {
  return &_identity;
}

/* l + r = s */
void vec_add(const vec *l, const vec *r, vec *s) {
  s->x = l->x + r->x;
  s->y = l->y + r->y;
  s->z = l->z + r->z;
}

/* l - r = s */
void vec_sub(const vec *l, const vec *r, vec *d) {
  d->x = l->x - r->x;
  d->y = l->y - r->y;
  d->z = l->z - r->z;
}

/* fv = p */
void vec_scalar_mult(const double f, const vec *v, vec *p) {
  p->x = f * v->x;
  p->y = f * v->y;
  p->z = f * v->z;
}

/* v/f = p */
void vec_scalar_div(const double f, const vec *q, vec *p) {
  vec_scalar_mult(1/f, q, p);
}

/* l . r = p */
void vec_dot(const vec *l, const vec *r, double *p) {
  *p = (l->x * r->x) + (l->y * r->y) + (l->z * r->z);
}

/* l x r = p */
void vec_cross(const vec *l, const vec *r, vec *p) {
  p->x = (l->y * r->z) - (l->z * r->y);
  p->y = -((l->x * r->z) - (l->z * r->x));
  p->z = (l->x * r->y) - (l->y * r->x);
}

/* l + r = s */
void quat_add(const quat *l, const quat *r, quat *s) {
  s->w = l->w + r->w;
  s->x = l->x + r->x;
  s->y = l->y + r->y;
  s->z = l->z + r->z;
}

/* l - r = d */
void quat_sub(const quat *l, const quat *r, quat *d) {
  d->w = l->w - r->w;
  d->x = l->x - r->x;
  d->y = l->y - r->y;
  d->z = l->z - r->z;
}

/* fq = p */
void quat_scalar_mult(const double f, const quat *q, quat *p) {
  p->w = f * q->w;
  p->x = f * q->x;
  p->y = f * q->y;
  p->z = f * q->z;
}

/* q/f = p */
void quat_scalar_div(const double f, const quat *q, quat *p) {
  quat_scalar_mult(1/f, q, p);
}

/* lr = p */
void quat_prod(const quat *l, const quat *r, quat *p) {
  /* Method taken from https://www.3dgep.com/understanding-quaternions/#quaternion-products
    Treat l and r as ordered pairs l = (sa, a) and r = (sb, b) where sa,sb from R and a,b from R^3
    (sa, a)(sb, b) = (sc, c)
    sc = sa*sb - a . b
    c = sa*b + sb*a + a x b
   */
  double sa = l->w;
  double sb = r->w;

  double a_dot_b = 0.0f;

  double sc = 0.0f;
  vec a = {l->x, l->y, l->z};
  vec b = {r->x, r->y, r->z};
  vec sa_times_b = {0};
  vec sb_times_a = {0};
  vec a_cross_b = {0};
  vec temp = {0};
  vec c = {0};

  /* compute sc */
  vec_dot(&a, &b, &a_dot_b);
  sc = (sa * sb)- a_dot_b;

  /* compute c */
  vec_scalar_mult(sa, &b, &sa_times_b);
  vec_scalar_mult(sb, &a, &sb_times_a);
  vec_cross(&a, &b, &a_cross_b);
  vec_add(&sa_times_b, &sb_times_a, &temp);
  vec_add(&temp, &a_cross_b, &c);

  p->w = sc;
  p->x = c.x;
  p->y = c.y;
  p->z = c.z;
}

/* |q| = n */
void quat_norm(const quat *q, double *n) {
  *n = sqrt((q->w*q->w + q->x*q->x + q->y*q->y + q->z*q->z));
}

/* q / |q| = n */
void quat_normed(const quat *q, quat *n) {
  double norm = 0.0f;

  quat_norm(q, &norm);
}

/* q* = c */
void quat_conjugate(const quat *q, quat *c) {
  c->w = q->w;
  c->x = -(q->x);
  c->y = -(q->y);
  c->z = -(q->z);
}

/* q^-1 = i */
void quat_inverse(const quat *q, quat *i) {
  quat conj = {0};
  double norm = 0;

  quat_conjugate(q, &conj);
  quat_norm(q, &norm);

  quat_scalar_div(norm*norm, &conj, i);
}

/* l . r = p */
void quat_dot(const quat *l, const quat *r, double *p) {
  *p = (l->w * r->w) + (l->x * r->x) + (l->y * r->y) + (l->z * r->z);
}

void quat_angular_diff(const quat *l, const quat *r, double *theta) {
  double dot = 0;
  double norm_l = 0;
  double norm_r = 0;

  quat_dot(l, r, &dot);
  quat_norm(l, &norm_l);
  quat_norm(r, &norm_r);
  *theta = acos(dot / (norm_l * norm_r));
}

void quat_vector_prod(const quat *q, const vec *v, quat *p) {
  quat pure_v = {0.0f, v->x, v->y, v->z};
  quat_prod(q, &pure_v, p);
}

void quat_rot_around_axis(const vec *axis, double theta, quat *r) {
  r->w = cos(theta / 2);
  r->x = sin(theta / 2) * axis->x;
  r->y = sin(theta / 2) * axis->y;
  r->z = sin(theta / 2) * axis->z;
}

void vec_outer_prod(const vec *l, const vec *r, bivec *p) {
  p->xy = (l->x*r->y) - (l->y*r->x);
  p->xz = (l->x*r->z) - (l->z*r->x);
  p->yz = (l->y*r->z) - (l->z*r->y);
}

void vec_geom_prod(const vec *l, const vec *r, rotor *p) {
  vec_dot(l, r, &(p->s));
  vec_outer_prod(l, r, &(p->b));
}
