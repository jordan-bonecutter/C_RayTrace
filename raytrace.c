/* * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "quaternion.h"
#include "vec3d.h"
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <openip.h>

typedef struct{
  vec3d_t origin, i, j, k;
  double fov;
} camera_t;

typedef struct{
  bool did_intersect;
  vec3d_t point, normal, specular; 
} intersection_t;

typedef struct{
  vec3d_t origin, direction, energy;
} ray_t;

typedef intersection_t (*intersection_nearest)(ray_t*, void* object_parameters);

typedef struct{
  void* parameters;
  intersection_nearest get_intersection;
  vec3d_t specular;
} scene_object_t;

struct sphere_params{
  vec3d_t origin;
  double radius;
};

struct plane_params{
  vec3d_t point;
  vec3d_t normal;
};

intersection_t closest_intersection(scene_object_t* scene_objects, int N, ray_t* ray){
  assert(scene_objects);
  assert(ray);

  int i;
  double mindist = 1e300, curr_dist;
  intersection_t ret, curr_intersect;
  vec3d_t tmp;
  ret.did_intersect = false;
  for(i = 0; i < N; i++){
    curr_intersect = scene_objects[i].get_intersection(ray, scene_objects[i].parameters);
    if(!curr_intersect.did_intersect){
      continue;
    }
    tmp = v3d_scale(&ray->origin, -1.);
    tmp = v3d_add(&tmp, &curr_intersect.point);
    curr_dist = v3d_dot(&tmp, &tmp);

    if(curr_dist < mindist){
      mindist = curr_dist;
      ret = curr_intersect;
      ret.specular = scene_objects[i].specular;
    }
  }

  return ret;
}

intersection_t plane_intersection(ray_t* ray, void* object_parameters){
  assert(ray);
  assert(object_parameters);

  intersection_t ret;
  double t, numerator, denominator;
  vec3d_t tmp;
  struct plane_params* plane = (struct plane_params*)object_parameters;

  tmp = v3d_scale(&ray->origin, -1.);
  tmp = v3d_add(&plane->point, &tmp);
  numerator = v3d_dot(&tmp, &plane->normal);
  denominator = v3d_dot(&ray->direction, &plane->normal);

  if(denominator == 0.){
    ret.did_intersect = false;
    return ret;
  }
  t = numerator / denominator;
  if(t <= 0){
    ret.did_intersect = false;
    return ret;
  }

  ret.did_intersect = true;
  tmp = v3d_scale(&ray->direction, t);
  ret.point = v3d_add(&ray->origin, &tmp);
  ret.normal = plane->normal;

  return ret;
}

intersection_t sphere_intersection(ray_t* ray, void* object_parameters){
  assert(ray);
  assert(object_parameters);
  
  intersection_t ret;
  double determinant, offset, t, normal_scale;
  struct sphere_params* sphere = (struct sphere_params*)object_parameters;
  vec3d_t tmp;
  tmp = v3d_scale(&sphere->origin, -1.);
  vec3d_t diff = v3d_add(&ray->origin, &tmp);
  
  determinant = v3d_dot(&ray->direction, &diff);
  determinant *= determinant;
  determinant -= v3d_dot(&diff, &diff);
  determinant += sphere->radius*sphere->radius;
  
  if (determinant < 0){
    ret.did_intersect = false;
    return ret;
  }
  determinant = sqrt(determinant);
  offset = -1.*(v3d_dot(&ray->direction, &diff));
  t = offset - determinant;
  if(t > 0){
    ret.did_intersect = true;
    tmp = v3d_scale(&ray->direction, t);
    ret.point = v3d_add(&ray->origin, &tmp);

    tmp = v3d_scale(&sphere->origin, -1.);
    ret.normal = v3d_add(&ret.point, &tmp);
    normal_scale = sqrt(v3d_dot(&ret.normal, &ret.normal));
    ret.normal = v3d_scale(&ret.normal, 1./normal_scale);
  } else {
    t = offset + determinant;
    if (t < 0)
    {
      ret.did_intersect = false;
      return ret;
    } 
    ret.did_intersect = true;
    tmp = v3d_scale(&ray->direction, t);
    ret.point = v3d_add(&ray->origin, &tmp);

    tmp = v3d_scale(&sphere->origin, -1.);
    ret.normal = v3d_add(&ret.point, &tmp);
    normal_scale = sqrt(v3d_dot(&ret.normal, &ret.normal));
    ret.normal = v3d_scale(&ret.normal, 1./normal_scale);
  }

  return ret;
}

camera_t camera_with(vec3d_t* origin, double theta, double phi, double alpha, double fov){
  assert(origin);
  
  camera_t ret;

  ret.origin = *origin;
  ret.fov    = fov;

  theta /= 2.;
  phi /= 2.;
  alpha /= 2.;

  // Setup rotation quaternions
  quaternion_t q1, q2, q3, q4, q5, q6, new_j, new_i, new_k;
  q1.real = cos(phi), q1.i = 0, q1.j = 0, q1.k = sin(phi);
  q2 = q_conj(&q1);

  new_j.real = 0., new_j.i = 0., new_j.j = 1., new_j.k = 0.;
  new_j = q_mul(&q1, &new_j);
  new_j = q_mul(&new_j, &q2);
  q3.real = cos(theta), q3.i = new_j.i*sin(theta), q3.j = new_j.j*sin(theta), q3.k = 0;
  q4 = q_conj(&q3);
  
  new_i.real = 0., new_i.i = 1., new_i.j = 0., new_i.k = 0.;
  new_i = q_mul(&q1, &new_i);
  new_i = q_mul(&new_i, &q2);
  new_i = q_mul(&q3, &new_i);
  new_i = q_mul(&new_i, &q4);
  q5.real = cos(alpha), q5.i = new_i.i*sin(alpha), q5.j = new_i.j*sin(alpha), q5.k = new_i.k*sin(alpha);
  q6 = q_conj(&q5);

  ret.i.x = new_i.i, ret.i.y = new_i.j, ret.i.z = new_i.k;

  new_j = q_mul(&q5, &new_j);
  new_j = q_mul(&new_j, &q6);

  ret.j.x = new_j.i, ret.j.y = new_j.j, ret.j.z = new_j.k;

  new_k.real = 0., new_k.i = 0., new_k.j = 0, new_k.k = 1.;
  new_k = q_mul(&q1, &new_k);
  new_k = q_mul(&new_k, &q2);
  new_k = q_mul(&q3, &new_k);
  new_k = q_mul(&new_k, &q4);
  new_k = q_mul(&q5, &new_k);
  new_k = q_mul(&new_k, &q6);

  ret.k.x = new_k.i, ret.k.y = new_k.j, ret.k.z = new_k.k;

  return ret;
}

ray_t camera_get_ray(camera_t* camera, double y, double x, int height, int width){
  assert(camera);
  
  ray_t ray;
  double l;
  vec3d_t ret, tmp;

  l = height / (2*tan(camera->fov/2));
  y = height/2. - y;
  x -= width/2.;

  ret = v3d_scale(&camera->i, l);
  tmp = v3d_scale(&camera->k, y);
  ret = v3d_add(&ret, &tmp);
  tmp = v3d_scale(&camera->j, x);
  ret = v3d_add(&ret, &tmp);

  l = sqrt(v3d_dot(&ret, &ret));
  ray.direction = v3d_scale(&ret, 1./l);
  ray.origin = camera->origin;
  ray.energy = (vec3d_t){1., 1., 1.};

  return ray;
}

vec3d_t ray_hit(ray_t* ray, intersection_t* intersection){
  assert(ray);
  assert(intersection);

  vec3d_t tmp;
  
  if(intersection->did_intersect){
    tmp = v3d_scale(&intersection->normal, 0.000001);
    ray->origin = v3d_add(&tmp, &intersection->point);
    tmp = v3d_scale(&intersection->normal, -2*v3d_dot(&ray->direction, &intersection->normal));
    ray->direction = v3d_add(&ray->direction, &tmp);
    ray->energy.x *= intersection->specular.x;
    ray->energy.y *= intersection->specular.y;
    ray->energy.z *= intersection->specular.z;

    return (vec3d_t){0., 0., 0.};
  } else {
    ray->energy = (vec3d_t){0., 0., 0.};
    return (vec3d_t){1.1*200., 1.1*200., 1.1*255.};
  }
}

unsigned char squash(double v){
  v = v > 0 ? v : 0;
  v = v < 255 ? v : 255;
  return (unsigned char)v;
}

#define BOUNCE_LIMIT  5
#define SAMPLES_PER_PIXEL 8
#define HEIGHT 100
#define WIDTH  100
#define SPHERES 21

int main()
{
  camera_t camera;
  vec3d_t tmp, curr_energy, curr_texture;
  vec3d_t curr_sample;
  ray_t curr_ray;
  intersection_t curr_intersection;
  scene_object_t scene_objects[SPHERES*SPHERES + 1];
  struct sphere_params sphere_params[SPHERES*SPHERES];
  struct plane_params plane_params;
  int i, iy, y, x, bounce, sample;
  double r, g, b, x_offset[SAMPLES_PER_PIXEL], y_offset[SAMPLES_PER_PIXEL];

  for(i = 0; i < SAMPLES_PER_PIXEL; i++){
    x_offset[i] = 0.5*cos(2.*M_PI*i/SAMPLES_PER_PIXEL);
    y_offset[i] = 0.5*sin(2.*M_PI*i/SAMPLES_PER_PIXEL);
  }

  img* image = img_new(WIDTH, HEIGHT);
 
  for(y = 0; y < SPHERES; y++){
    iy = y*SPHERES;
    for(x = 0; x < SPHERES; x++){
      i = iy + x;
      scene_objects[i].parameters = (struct sphere_params*)sphere_params + i;
      scene_objects[i].get_intersection = &sphere_intersection;
      scene_objects[i].specular = (vec3d_t){0.8, 0.8, 0.8};

      sphere_params[i].origin = (vec3d_t){(x-(SPHERES>>1))*120, (y-(SPHERES>>1))*120, 70.};
      sphere_params[i].radius = 45;
    }
  }

  scene_objects[SPHERES*SPHERES].parameters = &plane_params;
  scene_objects[SPHERES*SPHERES].get_intersection = &plane_intersection;
  scene_objects[SPHERES*SPHERES].specular = (vec3d_t){0.6, 0.6, 0.6};
  plane_params.normal = (vec3d_t){0., 0., 1.};
  plane_params.point  = (vec3d_t){0., 0., -10.};

  tmp = (vec3d_t){50, 50, 200};
  camera = camera_with(&tmp, M_PI/4., 5.*M_PI/4., 0., M_PI/1.5);

  printf("camera.i = (%lf, %lf, %lf)\n", camera.i.x, camera.i.y, camera.i.z);
  printf("camera.j = (%lf, %lf, %lf)\n", camera.j.x, camera.j.y, camera.j.z);
  printf("camera.k = (%lf, %lf, %lf)\n", camera.k.x, camera.k.y, camera.k.z);

  for(y = 0; y < HEIGHT; y++){
    for(x = 0; x < WIDTH; x++){
      r = 0, g = 0, b = 0;
      for(sample = 0; sample < SAMPLES_PER_PIXEL; sample++){
        curr_sample = (vec3d_t){0., 0., 0.};
        curr_ray = camera_get_ray(&camera, y + y_offset[sample], x + x_offset[sample], HEIGHT, WIDTH);
        for(bounce = 0; bounce < BOUNCE_LIMIT; bounce++){
          curr_intersection = closest_intersection(&scene_objects[0], SPHERES*SPHERES+1, &curr_ray);
          curr_energy = curr_ray.energy;
          curr_texture = ray_hit(&curr_ray, &curr_intersection);
          curr_sample.x += curr_energy.x * curr_texture.x;
          curr_sample.y += curr_energy.y * curr_texture.y;
          curr_sample.z += curr_energy.z * curr_texture.z;

          if(!curr_intersection.did_intersect){
            break;
          }
        }
        r += curr_sample.x/SAMPLES_PER_PIXEL;
        g += curr_sample.y/SAMPLES_PER_PIXEL;
        b += curr_sample.z/SAMPLES_PER_PIXEL;
      }  
      image->pix[y][x] = (pixel){squash(r), squash(g), squash(b), 255};
    }
  }

  (void)img_save_to_png(image, "test.png");
  img_free(image);

  return 0;
}

