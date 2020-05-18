/* * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * */
/* * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef QUATERNION_H
#define QUATERNION_H

typedef struct{
  double real, i, j, k;
} quaternion_t;

quaternion_t q_add(quaternion_t *q1, quaternion_t *q2);
quaternion_t q_mul(quaternion_t *q1, quaternion_t *q2);
quaternion_t q_conj(quaternion_t *q);
double q_norm(quaternion_t *q);
quaternion_t q_normalize(quaternion_t *q);

#endif
