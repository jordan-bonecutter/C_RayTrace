/* * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "vec3d.h"
#include <assert.h>
#include <math.h>

void v3d_add(vec3d_t* v1, vec3d_t* v2, vec3d_t* res){
  assert(v1);
  assert(v2);
  assert(res);

  /* add componentwise */
  res->x = v1->x + v2->x;
  res->y = v1->y + v2->y;
  res->z = v1->z + v2->z;
}

void v3d_sub(vec3d_t* v1, vec3d_t* v2, vec3d_t* res){
  assert(v1);
  assert(v2);
  assert(res);

  /* add componentwise */
  res->x = v1->x - v2->x;
  res->y = v1->y - v2->y;
  res->z = v1->z - v2->z;
}

void v3d_scale(vec3d_t* v, double s, vec3d_t* res){
  assert(v);
  assert(res);

  /* vector scale */
  res->x = v->x * s;
  res->y = v->y * s;
  res->z = v->z * s;
}

void v3d_hadamard(vec3d_t* v1, vec3d_t* v2, vec3d_t* res){
  assert(v1);
  assert(v2);
  assert(res);

  /* hadamard product */
  res->x = v1->x * v2->x;
  res->y = v1->y * v2->y;
  res->z = v1->z * v2->z;
}

void v3d_normalize(vec3d_t* v){
  assert(v);

  double mag = v3d_mag(v);
  v->x /= mag;
  v->y /= mag;
  v->z /= mag;
}

double v3d_dot(vec3d_t* v1, vec3d_t* v2){
  assert(v1);
  assert(v2);

  return v1->x*v2->x + v1->y*v2->y + v1->z*v2->z;
}

double v3d_mag(vec3d_t* v){
  assert(v);

  return sqrt(v->x*v->x + v->y*v->y + v->z*v->z);
}

