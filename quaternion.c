/* * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "quaternion.h"
#include <assert.h>
#include <math.h>

quaternion_t q_add(quaternion_t *q1, quaternion_t *q2){
  assert(q1);
  assert(q2);

  quaternion_t ret;
  ret.real = q1->real + q2->real;
  ret.i = q1->i + q2->i;
  ret.j = q1->j + q2->j;
  ret.k = q1->k + q2->k;

  return ret;
}

quaternion_t q_mul(quaternion_t *q1, quaternion_t *q2){
  assert(q1);
  assert(q2);

  quaternion_t ret;

  ret.real = q1->real*q2->real - q1->i*q2->i - q1->j*q2->j - q1->k*q2->k;
  ret.i    = q1->real*q2->i + q1->i*q2->real + q1->j*q2->k - q1->k*q2->j;
  ret.j    = q1->real*q2->j - q1->i*q2->k + q1->j*q2->real + q1->k*q2->i;
  ret.k    = q1->real*q2->k + q1->i*q2->j - q1->j*q2->i + q1->k*q2->real;
  return ret;
}

quaternion_t q_conj(quaternion_t *q){
  assert(q);

  quaternion_t ret;

  ret.real = q->real;
  ret.i    = -q->i;
  ret.j    = -q->j;
  ret.k    = -q->k;
  return ret;
}

double q_norm(quaternion_t *q){
  assert(q);

  double ret;
  ret = sqrt(q->real*q->real + q->i*q->i + q->j*q->j + q->k*q->k);
  return ret;
}

quaternion_t q_normalize(quaternion_t *q){
  assert(q);

  double norm = q_norm(q);
  quaternion_t ret;

  ret.real = q->real / norm;
  ret.i = q->i / norm;
  ret.j = q->j / norm;
  ret.k = q->k / norm;

  return ret;
}

