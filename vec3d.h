/* * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef VEC3D_H
#define VEC3D_H

typedef struct{
  double x, y, z;
}vec3d_t;

vec3d_t v3d_add(vec3d_t* v1, vec3d_t* v2);
vec3d_t v3d_scale(vec3d_t* v, double s);
double  v3d_dot(vec3d_t* v1, vec3d_t* v2);

#endif

