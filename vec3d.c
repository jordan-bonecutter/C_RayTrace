/* * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "vec3d.h"
#include <assert.h>

vec3d_t v3d_add(vec3d_t* v1, vec3d_t* v2){
  assert(v1);
  assert(v2);

  vec3d_t ret;

  /* add componentwise */
  ret.x = v1->x + v2->x;
  ret.y = v1->y + v2->y;
  ret.z = v1->z + v2->z;

  return ret;
}

vec3d_t v3d_scale(vec3d_t* v, double s){
  assert(v);

  vec3d_t ret;

  /* vector scale */
  ret.x = v->x * s;
  ret.y = v->y * s;
  ret.z = v->z * s;

  return ret;
}

double v3d_dot(vec3d_t* v1, vec3d_t* v2){
  assert(v1);
  assert(v2);

  double ret;

  ret = v1->x*v2->x + v1->y*v2->y + v1->z*v2->z;

  return ret;
}

