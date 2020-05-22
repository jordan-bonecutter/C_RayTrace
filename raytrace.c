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
#include <pthread.h>

typedef struct _scene_object_ scene_object_t;

/* intersection struct:
 *  did_intersect: true if the ray meets object, false otherwise
 *  point: location where ray hits object
 *  normal: object normal at intersection point
 *  specular: object's specular function
 *  */
typedef struct{
  bool did_intersect, sky;
  vec3d_t point, normal, specular; 
  scene_object_t* object;
} intersection_t;

/* ray struct:
 *  origin: first ray point
 *  direction: ray travel direction
 *  energy: ray light energy for each rgb component
 *  */
typedef struct{
  vec3d_t origin, direction, energy;
} ray_t;

/* intersection_nearest function:
 *  should be defined for each object type. For instance,
 *  for a sphere this function should return the closest
 *  valid intersection for a given ray
 *  */
typedef intersection_t (*object_intersector)(ray_t*, void* object_parameters);

/* object_texturemap function:
 *  can have multiple definitions for each object type. 
 *  allows for objects to be textured with an image or constant
 *  */
typedef vec3d_t (*object_texturemap)(vec3d_t* object_point, void* object_parameters, img* itexture);

/* scene_object struct:
 *  parameters: object instance parameters
 *  get_intersection: object intersection function
 *  specular: object specular function
 *  
 *
 *  I like to think of this as a "base class" which all scene objects
 *  must "inherit". Obviously, C doesn't have real classes so this
 *  is just about the next best thing we can get.
 *
 *  When you want to create a new scene_object type, you need to 
 *  define a new object_params struct which parameters points to. 
 *  You also need to write an intersection function so that you
 *  can do intersection tests.*/
struct _scene_object_{
  void* parameters;
  object_intersector get_intersection;
  vec3d_t specular;
  bool sky;
};

/* scene struct:
 *  scene_objects: list of all scene objects
 *  n_scene_objects: number of scene objects in list
 *  */
typedef struct{
  scene_object_t* scene_objects;
  int n_scene_objects;
}scene_t;

/* render_sheet_t:
 *  iplane: image plane
 *  width, height: plane width & height in pixels
 *  */
typedef struct{
  vec3d_t** iplane;
  int width, height;
}render_sheet_t;

/* camera struct:
 *  origin: camera location
 *  i: camera forward vector
 *  j: camera right vector
 *  k: camera up vector
 *  */
typedef struct{
  vec3d_t origin, i, j, k;
  double fov;
  void* params;
} camera_t;

typedef struct{
  int samples_per_pixel, bounce_limit;
} render_params_t;

/* camera_bokeh_params struct:
 *  flen: focal length of camera lens
 *  fplane: focal plane distance from camera
 *  lrad: lens radius size
 *  */
struct camera_bokeh_params{
  double flen, fplane, lrad;
};

/* method for rendering a scene */
typedef void (*scene_renderer)(scene_t*, camera_t*, render_sheet_t*, render_params_t*);

/* render struct:
 *  camera: camera used to render
 *  scene: scene to capture
 *  renderer: */
typedef struct{
  camera_t camera;
  scene_t scene;
  scene_renderer renderer;
} render_t;

/* sphere_params struct:
 *  origin: sphere center
 *  radius: sphere radius
 *  */
struct sphere_params{
  vec3d_t origin;
  double radius;
};

struct sky_params{
  vec3d_t origin;
  double radius, height;
  img* skybox;
};

/* plane_params struct:
 *  point: a point on the plane
 *  normal: unit vector normal to the plane
 *  */
struct plane_params{
  vec3d_t point;
  vec3d_t normal;
};

struct thread_params{
  scene_t* scene;
  camera_t* camera;
  render_sheet_t* sheet;
  int tid, total;
  render_params_t* params;
};

/* takes in a list of all scene objects and returns the closest intersection */
// TODO: add  scene struct and make this function take in only the scene
intersection_t closest_intersection(scene_t* scene, ray_t* ray){
  assert(scene);
  assert(ray);

  int i;
  double mindist = 1e300, curr_dist;
  intersection_t ret, curr_intersect;
  vec3d_t tmp;
  ret.did_intersect = false;

  // Loop through each intersection and return the one that happened closest to the
  // origin of the current ray
  for(i = 0; i < scene->n_scene_objects; i++){
    curr_intersect = scene->scene_objects[i].get_intersection(ray, scene->scene_objects[i].parameters);
    if(!curr_intersect.did_intersect){
      continue;
    }
    v3d_sub(&curr_intersect.point, &ray->origin, &tmp);
    curr_dist = v3d_dot(&tmp, &tmp);

    if(curr_dist < mindist){
      mindist = curr_dist;
      ret = curr_intersect;
      ret.specular = scene->scene_objects[i].specular;
      ret.sky = scene->scene_objects[i].sky;
      ret.object = scene->scene_objects + i;
    }
  }

  return ret;
}

/* plane intersection function */
/* The algorithm I used can be found at https://en.wikipedia.org/wiki/Line–plane_intersection */
intersection_t plane_intersector(ray_t* ray, void* object_parameters){
  assert(ray);
  assert(object_parameters);

  intersection_t ret;
  double t, numerator, denominator;
  vec3d_t tmp;
  struct plane_params* plane = (struct plane_params*)object_parameters;

  v3d_sub(&plane->point, &ray->origin, &tmp);
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
  v3d_scale(&ray->direction, t, &tmp);
  v3d_add(&ray->origin, &tmp, &ret.point);
  ret.normal = plane->normal;

  return ret;
}

/* sphere intersection function */
/* The algorithm I used can be found at https://en.wikipedia.org/wiki/Line–sphere_intersection*/
intersection_t sphere_intersector(ray_t* ray, void* object_parameters){
  assert(ray);
  assert(object_parameters);
  
  intersection_t ret;
  double determinant, offset, t;
  struct sphere_params* sphere = (struct sphere_params*)object_parameters;
  vec3d_t tmp, diff;
  v3d_sub(&ray->origin, &sphere->origin, &diff);
  
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
    v3d_scale(&ray->direction, t, &tmp);
    v3d_add(&ray->origin, &tmp, &ret.point);

    v3d_sub(&ret.point, &sphere->origin, &ret.normal);
    v3d_normalize(&ret.normal);
  } else {
    t = offset + determinant;
    if (t < 0)
    {
      ret.did_intersect = false;
      return ret;
    } 
    ret.did_intersect = true;
    v3d_scale(&ray->direction, t, &tmp);
    v3d_add(&ray->origin, &tmp, &ret.point);

    v3d_sub(&ret.point, &sphere->origin, &ret.normal);
    v3d_normalize(&ret.normal);
  }

  return ret;
}

/* create a new camera with the parameters.
 * theta tilts the camera up and down
 * phi pans the camera side to side
 * alpha twists the camera clockwise 
 * fov is the vertical angle which the camera captures */
camera_t camera_with(vec3d_t* origin, double theta, double phi, double alpha, double fov){
  assert(origin);
  
  camera_t ret;

  ret.origin      = *origin;
  ret.fov         = fov;

  /* divide by two because of
   * our quaternion based algorithm */
  theta /= 2.;
  phi /= 2.;
  alpha /= 2.;

  /* the best way i've figured out to rotate the camera is using quaternions 
   * (I know, it's probably overkill but meh). I carry out the procedure with 
   * 3 quaternion rotations. I could do it with one but this way makes it far
   * easier to keep track of which axis I'm rotating about. If you are unfamiliar
   * with how quaternions are helpful in computing 3d rotations, I recommend 3blue1brown's
   * excellent YouTube series on them, but I'll give a brief explanation here. Quaternion
   * rotation is done via multiplication by two quaternions which are complex conjugates
   * of each other. The axis of rotation is about the axis perpenidular to the imaginary
   * part of the leftmost quaternion. The first quaternion pair I rotate about is pointing 
   * "up" so this pans the camera side to side (phi). The second pair is pointing to the
   * right, so this tilts the camera up and down(theta). Finally, the camera is rotated 
   * about the axis which it is facing, makng the scene appear to rotate (alpha). */

  quaternion_t q1, q2, q3, q4, q5, q6, new_j, new_i, new_k;

  // Rotate about k (phi)
  q1.real = cos(phi), q1.i = 0, q1.j = 0, q1.k = sin(phi);
  q2 = q_conj(&q1);

  // Roatate about j (theta)
  new_j.real = 0., new_j.i = 0., new_j.j = 1., new_j.k = 0.;
  new_j = q_mul(&q1, &new_j);
  new_j = q_mul(&new_j, &q2);
  q3.real = cos(theta), q3.i = new_j.i*sin(theta), q3.j = new_j.j*sin(theta), q3.k = 0;
  q4 = q_conj(&q3);
  
  // Rotate about i
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

camera_t camera_bokeh_with(vec3d_t* origin, double theta, double phi, double alpha, double fov, double flen, double fplane, double lrad){
  camera_t ret = camera_with(origin, theta, phi, alpha, fov);
  static struct camera_bokeh_params params;
  params.flen = flen, params.fplane = fplane, params.lrad = lrad;
  ret.params = &params;

  return ret;
}

ray_t camera_get_bokeh_ray(camera_t* camera, vec3d_t* point, vec3d_t* normal, int dx, int dy, double smear){
  assert(camera);
  
  ray_t ray;
  vec3d_t lenspoint, vdx, vdy, tmp;
  double fx, fy;
  struct camera_bokeh_params* bokeh = (struct camera_bokeh_params*)camera->params;

  fx = ((double)dx)/smear;
  fy = ((double)dy)/smear;

  v3d_scale(&camera->j, bokeh->lrad*fx, &vdx);
  v3d_scale(&camera->k, bokeh->lrad*fy, &vdy);
  v3d_add(&camera->origin, &vdx, &lenspoint);
  v3d_add(&lenspoint, &vdy, &lenspoint);
  v3d_sub(&lenspoint, point, &ray.direction);
  v3d_normalize(&ray.direction);
  v3d_scale(normal, 0.00001, &tmp);
  v3d_add(&tmp, point, &ray.origin);

  return ray;
}

/* get the ray originating at the camera passing through image plane point x, y */
ray_t camera_get_ray(camera_t* camera, double y, double x, int height, int width){
  assert(camera);
  
  ray_t ray;
  double l;
  vec3d_t ret, tmp;

  // We want the scene to always render with the same fov no matter
  // the resolution, so we do some trig to ensure this happens
  l = height / (2*tan(camera->fov/2));
  y = height/2. - y;
  x -= width/2.;

  v3d_scale(&camera->i, l, &ret);
  v3d_scale(&camera->k, y, &tmp);
  v3d_add(&ret, &tmp, &ret);
  v3d_scale(&camera->j, x, &tmp);
  v3d_add(&ret, &tmp, &ret);

  v3d_normalize(&ret);
  ray.origin = camera->origin;
  ray.energy = (vec3d_t){1., 1., 1.};
  ray.direction = ret;

  return ray;
}

double camera_bokeh_get_smear(camera_t* camera, vec3d_t* point){
  assert(camera);
  assert(point);

  struct camera_bokeh_params* bokeh = (struct camera_bokeh_params*)camera->params;

  double orth_dist, proj_dist;
  vec3d_t tmp;
  v3d_sub(point, &camera->origin, &tmp);
  orth_dist = v3d_dot(&tmp, &camera->i);
  proj_dist = 1./((1./bokeh->flen) - (1./orth_dist));
  return fabs((bokeh->lrad/proj_dist)*(proj_dist - 1./((1./bokeh->flen) - (1./bokeh->fplane))));
}

long mod(long a, long b){
  if(a < 0){
    return (a%b) + b - 1;
  } else {
    return a%b;
  }
}

/* bounce the ray and reduce its energy or return sky color */
vec3d_t ray_hit(ray_t* ray, intersection_t* intersection){
  assert(ray);
  assert(intersection);

  vec3d_t tmp;
  double t, sx, sy, skyrad, skyheight;
  pixel p;
  img* skybox;
  struct sky_params* sky_params;
  
  if(intersection->did_intersect){
    if(!intersection->sky){
      // Scooch the ray forward a little bit from the surface
      // so it doesn't double bounce
      v3d_scale(&intersection->normal, 0.000001, &tmp);
      v3d_add(&tmp, &intersection->point, &ray->origin);

      // Bounced ray = ray - 2*proj_normal(ray)
      v3d_scale(&intersection->normal, -2*v3d_dot(&ray->direction, &intersection->normal), &tmp);
      v3d_add(&ray->direction, &tmp, &ray->direction);

      // Reduce ray energy
      v3d_hadamard(&ray->energy, &intersection->specular, &ray->energy);

      return (vec3d_t){0., 0., 0.};
    } else {
      sky_params = (struct sky_params*)intersection->object->parameters;
      skyrad = sky_params->radius;
      skyheight = sky_params->height;
      skybox = sky_params->skybox;
      t = sqrt(2*skyrad*skyheight - skyheight*skyheight);
      sx = intersection->point.x, sy = intersection->point.y;
      sx *= 0.99, sy*= 0.99;
      sx += t, sy += t;
      sx /= t*2., sy /= t*2.;
      sx *= skybox->width - 1, sy *= skybox->height - 1;
      p = skybox->pix[(int)sy][(int)sx];
      tmp.x = p.r*1.3, tmp.y = p.g*1.3, tmp.z = p.b*1.4;
      ray->energy = (vec3d_t){0., 0., 0.};
      return tmp; 
    }
  } else {
    ray->energy = (vec3d_t){0., 0., 0.};
    return (vec3d_t){0.8, 0.8, 0.9};
  }
}

/* max(min(255, val), 0) so that we don't int overflow */
unsigned char squash(double v){
  v = v > 0 ? v : 0;
  v = v < 255 ? v : 255;
  return (unsigned char)v;
}

vec3d_t** alloc_double(int width, int height){
  int i;
  vec3d_t** ret = malloc(sizeof(vec3d_t*)*height);
  ret[0] = calloc(sizeof(vec3d_t), width*height);

  for(i = 1; i < height; ret[i] = ret[i-1] + width, i++);
  return ret;
}

void free_double(vec3d_t** a){
  free(a[0]);
  free(a);
}

void draw_smeared(vec3d_t** array, int cx, int cy, int width, int height, double dsmear, vec3d_t* color){
  int smear = 7*fabs(dsmear)+1;
  int dy, dx, y, x, count = 0;
  vec3d_t tmp;

  array[cy][cx] = (vec3d_t){squash(color->x), squash(color->y), squash(color->z)};
  return;
  
  for(dy = -smear; dy < smear; dy++){
    y = cy + dy;
    for(dx = -smear; dx < smear; dx++){
      x = cx + dx;
      if(dx*dx + dy*dy <= smear*smear){
        count++;
      }
    }
  }
  
  for(dy = -smear; dy < smear; dy++){
    y = cy + dy;
    if(y < 0 || y > height-1){
      continue;
    }
    for(dx = -smear; dx < smear; dx++){
      x = cx + dx;
      if(x < 0 || x > width - 1 || dx*dx + dy*dy > smear*smear){
        continue;
      }
      v3d_scale(color, 1./count, &tmp);
      v3d_add(array[y] + x, &tmp, array[y] + x);
    }
  }
}

void sheet2uchar(render_sheet_t* sheet, img* uchar, int width, int height){
  int x, y;
  pixel p;
  vec3d_t curr;

  for(y = 0; y < height; y++){
    for(x = 0; x < width; x++){
      curr = sheet->iplane[y][x];
      p.r = squash(curr.x);
      p.g = squash(curr.y);
      p.b = squash(curr.z);
      p.a = 255;

      uchar->pix[y][x] = p;
    }
  }
}

double circle_spread(int radius){
  int count = 0, y, x;
  for(y = -radius; y <= radius; y++){
    for(x = -radius; x <= radius; x++){
      if(x*x + y*y <= radius*radius){
        count++;
      }
    }
  }

  return 1./count;
}

double max(double v1, double v2){
  return v1 > v2 ? v1 : v2;
}

double min(double v1, double v2){
  return v1 < v2 ? v1 : v2;
}

void* bokeh_threadfunc(void* params){
  assert(params);

  struct thread_params* thread_params = (struct thread_params*)params;
  scene_t* scene = thread_params->scene;
  camera_t* camera = thread_params->camera;
  render_sheet_t* sheet = thread_params->sheet;
  int tid = thread_params->tid, total = thread_params->total;
  render_params_t* r_params = thread_params->params;

  int i, y, x, dx, dy, sample, height, width, bounce;
  double* y_offset, *x_offset, weight, percent;
  double smear, xl, xu, yl, yu;
  vec3d_t tmp, color, curr_sample, curr_energy, curr_texture, bounce_point, bounce_normal;
  ray_t curr_ray;
  intersection_t curr_intersection;
  struct camera_bokeh_params* bokeh = (struct camera_bokeh_params*)camera->params;

  y_offset = malloc(sizeof(double)*r_params->samples_per_pixel);
  x_offset = malloc(sizeof(double)*r_params->samples_per_pixel);

  for(i = 0; i < r_params->samples_per_pixel; i++){
    x_offset[i] = 0.5*cos(2.*M_PI*i/r_params->samples_per_pixel);
    y_offset[i] = 0.5*sin(2.*M_PI*i/r_params->samples_per_pixel);
  }

  height = sheet->height;
  width  = sheet->width;
  
  for(y = tid; y < sheet->height; y += total){
    for(x = 0; x < width; x++){
      fflush(stdout);
      color = (vec3d_t){0., 0., 0.};
      for(sample = 0; sample < r_params->samples_per_pixel; sample++){
        curr_sample = (vec3d_t){0., 0., 0.};
        curr_ray = camera_get_ray(camera, y + y_offset[sample], x + x_offset[sample], height, width);

        for(bounce = 0; bounce < r_params->bounce_limit; bounce++){
          curr_intersection = closest_intersection(scene, &curr_ray);
          if(bounce == 0){
            smear = camera_bokeh_get_smear(camera, &curr_intersection.point);
            bounce_point = curr_intersection.point;
            bounce_normal = curr_intersection.normal;
          } 
          curr_energy = curr_ray.energy;
          curr_texture = ray_hit(&curr_ray, &curr_intersection);
          v3d_hadamard(&curr_energy, &curr_texture, &curr_sample);

          if(!curr_intersection.did_intersect || curr_intersection.sky){
            break;
          }
        }
        v3d_scale(&curr_sample, 1./r_params->samples_per_pixel, &color);
        smear = smear < 0.25 ? 0.25 : smear;
        weight = 1./(4*smear*smear);
        v3d_scale(&color, 1./(4*smear*smear), &color);
        for(dy = -(smear+1); dy <= (smear+1); dy++){
          if(y + dy < 0 || y + dy >= height){
            continue;
          }
          for(dx = -(smear+1); dx <= (smear+1); dx++){
            if(x + dx < 0 || x + dx >= width){
              continue;
            }
            if(abs(dx) - smear > 0.5 || abs(dy) - smear > 0.5){
              continue;
            }
            xl = min(x + dx + 0.5, x + smear);
            xu = max(x + dx - 0.5, x - smear);
            yl = min(y + dy + 0.5, y + smear);
            yu = max(y + dy - 0.5, y - smear);
            curr_ray = camera_get_bokeh_ray(camera, &bounce_point, &bounce_normal, dx, dy, smear);
            curr_intersection = closest_intersection(scene, &curr_ray);
            if((!curr_intersection.did_intersect || curr_intersection.sky)){
              v3d_scale(&color, (xl - xu)*(yl - yu), &tmp);
              v3d_add(&sheet->iplane[dy + y][dx + x], &tmp, &sheet->iplane[dy + y][dx + x]);
            }
          }
        }
      }  
    }
  }

  free(y_offset);
  free(x_offset);

  return NULL;
}

void bokeh_renderer_multithread(scene_t* scene, camera_t* camera, render_sheet_t* sheet, render_params_t* r_params, int n_threads){
  int i;
  pthread_t* threads;
  threads = malloc(sizeof(pthread_t)*(n_threads-1));
  struct thread_params* thread_params;
  thread_params = malloc(sizeof(struct thread_params)*n_threads);
  double curr, prev;

  prev = 0.;
  for(i = 0; i < n_threads-1; i++){
    curr = prev + sheet->height/(double)n_threads;
    thread_params[i].scene = scene;
    thread_params[i].camera = camera;
    thread_params[i].sheet = sheet;
    thread_params[i].tid = i;
    thread_params[i].total = n_threads;
    thread_params[i].params = r_params;
    pthread_create(threads + i, NULL, bokeh_threadfunc, &thread_params[i]);
    prev = curr;
  }

  curr = prev + sheet->height/(double)n_threads;
  thread_params[n_threads-1].scene = scene;
  thread_params[n_threads-1].camera = camera;
  thread_params[n_threads-1].sheet = sheet;
  thread_params[n_threads-1].tid = n_threads - 1;
  thread_params[n_threads-1].total = n_threads;
  thread_params[n_threads-1].params = r_params;
  bokeh_threadfunc(thread_params + n_threads - 1);

  for(i = 0; i < n_threads-1; i++){
    pthread_join(threads[i], NULL);
  }

  free(threads);
  free(thread_params);
}

void bokeh_renderer(scene_t* scene, camera_t* camera, render_sheet_t* sheet, render_params_t* r_params){
  int i, y, x, dx, dy, sample, height, width, bounce;
  double* y_offset, *x_offset, weight, percent;
  double smear, xl, xu, yl, yu;
  vec3d_t tmp, color, curr_sample, curr_energy, curr_texture, bounce_point, bounce_normal;
  ray_t curr_ray;
  intersection_t curr_intersection;
  struct camera_bokeh_params* bokeh = (struct camera_bokeh_params*)camera->params;

  y_offset = malloc(sizeof(double)*r_params->samples_per_pixel);
  x_offset = malloc(sizeof(double)*r_params->samples_per_pixel);

  for(i = 0; i < r_params->samples_per_pixel; i++){
    x_offset[i] = 0.5*cos(2.*M_PI*i/r_params->samples_per_pixel);
    y_offset[i] = 0.5*sin(2.*M_PI*i/r_params->samples_per_pixel);
  }

  height = sheet->height;
  width  = sheet->width;
  
  for(y = 0; y < height; y++){
    for(x = 0; x < width; x++){
      printf("\r%lf", (y*width + x)/((double)height*width));
      fflush(stdout);
      color = (vec3d_t){0., 0., 0.};
      for(sample = 0; sample < r_params->samples_per_pixel; sample++){
        curr_sample = (vec3d_t){0., 0., 0.};
        curr_ray = camera_get_ray(camera, y + y_offset[sample], x + x_offset[sample], height, width);

        for(bounce = 0; bounce < r_params->bounce_limit; bounce++){
          curr_intersection = closest_intersection(scene, &curr_ray);
          if(bounce == 0){
            smear = camera_bokeh_get_smear(camera, &curr_intersection.point);
            bounce_point = curr_intersection.point;
            bounce_normal = curr_intersection.normal;
          } 
          curr_energy = curr_ray.energy;
          curr_texture = ray_hit(&curr_ray, &curr_intersection);
          v3d_hadamard(&curr_energy, &curr_texture, &curr_sample);

          if(!curr_intersection.did_intersect || curr_intersection.sky){
            break;
          }
        }
        v3d_scale(&curr_sample, 1./r_params->samples_per_pixel, &color);
        smear = smear < 0.25 ? 0.25 : smear;
        weight = 1./(4*smear*smear);
        v3d_scale(&color, 1./(4*smear*smear), &color);
        for(dy = -(smear+1); dy <= (smear+1); dy++){
          if(y + dy < 0 || y + dy >= height){
            continue;
          }
          for(dx = -(smear+1); dx <= (smear+1); dx++){
            if(x + dx < 0 || x + dx >= width){
              continue;
            }
            if(abs(dx) - smear > 0.5 || abs(dy) - smear > 0.5){
              continue;
            }
            xl = min(x + dx + 0.5, x + smear);
            xu = max(x + dx - 0.5, x - smear);
            yl = min(y + dy + 0.5, y + smear);
            yu = max(y + dy - 0.5, y - smear);
            curr_ray = camera_get_bokeh_ray(camera, &bounce_point, &bounce_normal, dx, dy, smear);
            curr_intersection = closest_intersection(scene, &curr_ray);
            if((!curr_intersection.did_intersect || curr_intersection.sky)){
              v3d_scale(&color, (xl - xu)*(yl - yu), &tmp);
              v3d_add(&sheet->iplane[dy + y][dx + x], &tmp, &sheet->iplane[dy + y][dx + x]);
            }
          }
        }
      }  
    }
  }

  free(y_offset);
  free(x_offset);
}

void* default_threadfunc(void* data){
  struct thread_params* thread_params = (struct thread_params*)data;
  scene_t* scene = thread_params->scene;
  camera_t* camera = thread_params->camera;
  render_sheet_t* sheet = thread_params->sheet;
  int tid = thread_params->tid, total = thread_params->total;
  render_params_t* r_params = thread_params->params;

  int i, y, x, sample, height, width, bounce;
  double* y_offset, *x_offset;
  vec3d_t color, curr_sample, curr_energy, curr_texture;
  ray_t curr_ray;
  intersection_t curr_intersection;

  y_offset = malloc(sizeof(double)*r_params->samples_per_pixel);
  x_offset = malloc(sizeof(double)*r_params->samples_per_pixel);

  for(i = 0; i < r_params->samples_per_pixel; i++){
    x_offset[i] = 0.5*cos(2.*M_PI*i/r_params->samples_per_pixel);
    y_offset[i] = 0.5*sin(2.*M_PI*i/r_params->samples_per_pixel);
  }

  height = sheet->height;
  width  = sheet->width;
  
  for(y = tid; y < height; y += total){
    for(x = 0; x < width; x++){
      color = (vec3d_t){0., 0., 0.};
      for(sample = 0; sample < r_params->samples_per_pixel; sample++){
        curr_sample = (vec3d_t){0., 0., 0.};
        curr_ray = camera_get_ray(camera, y + y_offset[sample], x + x_offset[sample], height, width);

        for(bounce = 0; bounce < r_params->bounce_limit; bounce++){
          curr_intersection = closest_intersection(scene, &curr_ray);
          curr_energy = curr_ray.energy;
          curr_texture = ray_hit(&curr_ray, &curr_intersection);
          v3d_hadamard(&curr_energy, &curr_texture, &curr_sample);

          if(!curr_intersection.did_intersect || curr_intersection.sky){
            break;
          }
        }
        color.x += curr_sample.x/r_params->samples_per_pixel;
        color.y += curr_sample.y/r_params->samples_per_pixel;
        color.z += curr_sample.z/r_params->samples_per_pixel;
      }  
      sheet->iplane[y][x] = color;
    }
  }

  free(y_offset);
  free(x_offset);
}

void default_renderer_multithread(scene_t* scene, camera_t* camera, render_sheet_t* sheet, render_params_t* r_params, int n_threads){
  int i;
  pthread_t* threads;
  threads = malloc(sizeof(pthread_t)*(n_threads-1));
  struct thread_params* thread_params;
  thread_params = malloc(sizeof(struct thread_params)*n_threads);
  double curr, prev;

  prev = 0.;
  for(i = 0; i < n_threads-1; i++){
    curr = prev + sheet->height/(double)n_threads;
    thread_params[i].scene = scene;
    thread_params[i].camera = camera;
    thread_params[i].sheet = sheet;
    thread_params[i].tid = i;
    thread_params[i].total = n_threads;
    thread_params[i].params = r_params;
    pthread_create(threads + i, NULL, default_threadfunc, &thread_params[i]);
    prev = curr;
  }

  curr = prev + sheet->height/(double)n_threads;
  thread_params[n_threads-1].scene = scene;
  thread_params[n_threads-1].camera = camera;
  thread_params[n_threads-1].sheet = sheet;
  thread_params[n_threads-1].tid = n_threads - 1;
  thread_params[n_threads-1].total = n_threads;
  thread_params[n_threads-1].params = r_params;
  default_threadfunc(thread_params + n_threads - 1);

  for(i = 0; i < n_threads-1; i++){
    pthread_join(threads[i], NULL);
  }

  free(threads);
  free(thread_params);
}

void default_renderer(scene_t* scene, camera_t* camera, render_sheet_t* sheet, render_params_t* r_params){
  int i, y, x, sample, height, width, bounce;
  double* y_offset, *x_offset;
  vec3d_t color, curr_sample, curr_energy, curr_texture;
  ray_t curr_ray;
  intersection_t curr_intersection;

  y_offset = malloc(sizeof(double)*r_params->samples_per_pixel);
  x_offset = malloc(sizeof(double)*r_params->samples_per_pixel);

  for(i = 0; i < r_params->samples_per_pixel; i++){
    x_offset[i] = 0.5*cos(2.*M_PI*i/r_params->samples_per_pixel);
    y_offset[i] = 0.5*sin(2.*M_PI*i/r_params->samples_per_pixel);
  }

  height = sheet->height;
  width  = sheet->width;
  
  for(y = 0; y < height; y++){
    for(x = 0; x < width; x++){
      color = (vec3d_t){0., 0., 0.};
      for(sample = 0; sample < r_params->samples_per_pixel; sample++){
        curr_sample = (vec3d_t){0., 0., 0.};
        curr_ray = camera_get_ray(camera, y + y_offset[sample], x + x_offset[sample], height, width);

        for(bounce = 0; bounce < r_params->bounce_limit; bounce++){
          curr_intersection = closest_intersection(scene, &curr_ray);
          curr_energy = curr_ray.energy;
          curr_texture = ray_hit(&curr_ray, &curr_intersection);
          v3d_hadamard(&curr_energy, &curr_texture, &curr_sample);

          if(!curr_intersection.did_intersect || curr_intersection.sky){
            break;
          }
        }
        color.x += curr_sample.x/r_params->samples_per_pixel;
        color.y += curr_sample.y/r_params->samples_per_pixel;
        color.z += curr_sample.z/r_params->samples_per_pixel;
      }  
      sheet->iplane[y][x] = color;
    }
  }

  free(y_offset);
  free(x_offset);
}

render_sheet_t* render_sheet_new(int width, int height){
  render_sheet_t* ret = malloc(sizeof(render_sheet_t));
  ret->iplane = malloc(sizeof(vec3d_t*)*height);
  ret->iplane[0] = calloc(width*height, sizeof(vec3d_t));
  for(int i = 1; i < height; ret->iplane[i] = ret->iplane[i-1] + width, i++);
  ret->width = width, ret->height = height;

  return ret;
}

void render_sheet_free(render_sheet_t* render_sheet){
  free(render_sheet->iplane[0]);
  free(render_sheet->iplane);
  free(render_sheet);
}

#define BOUNCE_LIMIT  5
#define SAMPLES_PER_PIXEL 5
#define HEIGHT 1000
#define WIDTH  1000
#define SPHERES 21
#define SKYHEIGHT 1000
#define SKYRADIUS 10000


int main()
{
  camera_t camera;
  scene_object_t scene_objects[SPHERES*SPHERES + 2];
  struct sphere_params sphere_params[SPHERES*SPHERES];
  struct plane_params plane_params;
  struct sky_params sky_params;
  img_ioerr err;
  img* skybox = img_from_png("skybox.png", &err);
  int i, iy, y, x;
  vec3d_t tmp;

  img* image = img_new(WIDTH, HEIGHT);
 
  for(y = 0; y < SPHERES; y++){
    iy = y*SPHERES;
    for(x = 0; x < SPHERES; x++){
      i = iy + x;
      scene_objects[i].parameters = (struct sphere_params*)sphere_params + i;
      scene_objects[i].get_intersection = &sphere_intersector;
      scene_objects[i].specular = (vec3d_t){0.8, 0.8, 0.8};
      scene_objects[i].sky      = false;

      sphere_params[i].origin = (vec3d_t){(x-(SPHERES>>1))*120, (y-(SPHERES>>1))*120, 70.};
      sphere_params[i].radius = 45;
    }
  }

  scene_objects[SPHERES*SPHERES].parameters = &plane_params;
  scene_objects[SPHERES*SPHERES].get_intersection = &plane_intersector;
  scene_objects[SPHERES*SPHERES].specular = (vec3d_t){0.6, 0.6, 0.6};
  scene_objects[SPHERES*SPHERES].sky = false;
  plane_params.normal = (vec3d_t){0., 0., 1.};
  plane_params.point  = (vec3d_t){0., 0., -10.};

  scene_objects[SPHERES*SPHERES+1].parameters = &sky_params;
  scene_objects[SPHERES*SPHERES+1].get_intersection = &sphere_intersector;
  scene_objects[SPHERES*SPHERES+1].sky = true;
  sky_params.origin = (vec3d_t){0., 0., SKYHEIGHT - SKYRADIUS};
  sky_params.radius = SKYRADIUS;
  sky_params.height = SKYHEIGHT;
  sky_params.skybox = skybox;

  tmp = (vec3d_t){50, 50, 200};
  camera = camera_bokeh_with(&tmp, M_PI/4., 5.*M_PI/4., 0., M_PI/1.5, 150, 200, 1);

  printf("camera.i = (%lf, %lf, %lf)\n", camera.i.x, camera.i.y, camera.i.z);
  printf("camera.j = (%lf, %lf, %lf)\n", camera.j.x, camera.j.y, camera.j.z);
  printf("camera.k = (%lf, %lf, %lf)\n", camera.k.x, camera.k.y, camera.k.z);

  scene_t scene;
  scene.scene_objects = &scene_objects[0];
  scene.n_scene_objects = SPHERES*SPHERES + 2;
  render_sheet_t* sheet = render_sheet_new(WIDTH, HEIGHT);
  render_params_t r_params;
  r_params.samples_per_pixel = SAMPLES_PER_PIXEL;
  r_params.bounce_limit      = BOUNCE_LIMIT;
  default_renderer_multithread(&scene, &camera, sheet, &r_params, 4);

  sheet2uchar(sheet, image, WIDTH, HEIGHT);
  (void)img_save_to_png(image, "test2.png");
  img_free(image);
  img_free(skybox);

  return 0;
}

