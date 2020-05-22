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

void   v3d_add(vec3d_t* v1, vec3d_t* v2, vec3d_t* res);
void   v3d_sub(vec3d_t* v1, vec3d_t* v2, vec3d_t* res);
void   v3d_scale(vec3d_t* v, double s, vec3d_t* res);
void   v3d_hadamard(vec3d_t* v1, vec3d_t* v2, vec3d_t* res);
void   v3d_normalize(vec3d_t* v);
double v3d_dot(vec3d_t* v1, vec3d_t* v2);
double v3d_mag(vec3d_t* v);

#endif

