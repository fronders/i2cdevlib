// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class, 3D math helper
// 6/5/2012 by Jeff Rowberg <jeff@rowberg.net>
// 10/29/2015 by Alexander Olenyev <alexandes.zp@gmail.com>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2015-10-29 - ported to C for use with STM32

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef _HELPER_3DMATH_H_
#define _HELPER_3DMATH_H_

#include <stdint.h>
#include <math.h>

typedef struct {
	float w;
	float x;
	float y;
	float z;
} Quaternion;

void Quaternion_init(Quaternion *q) {
	q->w = 1.0f;
	q->x = 0.0f;
	q->y = 0.0f;
	q->z = 0.0f;
}

void Quaternion_initVal(Quaternion *q, float nw, float nx, float ny, float nz) {
	q->w = nw;
	q->x = nx;
	q->y = ny;
	q->z = nz;
}

Quaternion Quaternion_getProduct(Quaternion *q1, Quaternion *q2) {
	// Quaternion multiplication is defined by:
	//     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
	//     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
	//     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
	//     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2
	Quaternion q_result;
	Quaternion_initVal(&q_result,
		q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z,  // new w
		q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y,  // new x
		q1->w * q2->y - q1->x * q2->z + q1->y * q2->w + q1->z * q2->x,  // new y
		q1->w * q2->z + q1->x * q2->y - q1->y * q2->x + q1->z * q2->w); // new z
	return q_result;
}

Quaternion Quaternion_getConjugate(Quaternion *q) {
	Quaternion q_result;
	Quaternion_initVal(&q_result, q->w, - q->x, - q->y, - q->z);
	return q_result;
}

float Quaternion_getMagnitude(Quaternion *q) {
	return sqrtf((q->w) * (q->w) + (q->x) * (q->x) + (q->y) * (q->y) + (q->z) * (q->z));
}

void Quaternion_normalize(Quaternion *q) {
	float m = Quaternion_getMagnitude(q);
	q->w /= m;
	q->x /= m;
	q->y /= m;
	q->z /= m;
}

Quaternion Quaternion_getNormalized(Quaternion *q) {
	Quaternion r;
	Quaternion_initVal(&r, q->w, q->x, q->y, q->z);
	Quaternion_normalize(&r);
	return r;
}

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} VectorInt16;

void VectorInt16_init(VectorInt16 *v) {
	v->x = 0;
	v->y = 0;
	v->z = 0;
}

void VectorInt16_initVal(VectorInt16 *v, int16_t nx, int16_t ny, int16_t nz) {
	v->x = nx;
	v->y = ny;
	v->z = nz;
}

float VectorInt16_getMagnitude(VectorInt16 *v) {
	return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

void VectorInt16_normalize(VectorInt16 *v) {
	float m = VectorInt16_getMagnitude(v);
	v->x /= m;
	v->y /= m;
	v->z /= m;
}

VectorInt16 VectorInt16_getNormalized(VectorInt16 *v) {
	VectorInt16 r;
	VectorInt16_initVal(&r, v->x, v->y, v->z);
	VectorInt16_normalize(&r);
	return r;
}

void VectorInt16_rotate(VectorInt16 *v, Quaternion *q) {
	// http://www.cprogramming.com/tutorial/3d/quaternions.html
	// http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
	// http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation
	// ^ or: http://webcache.googleusercontent.com/search?q=cache:xgJAp3bDNhQJ:content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation&hl=en&gl=us&strip=1

	// P_out = q * P_in * conj(q)
	// - P_out is the output vector
	// - q is the orientation quaternion
	// - P_in is the input vector (a*aReal)
	// - conj(q) is the conjugate of the orientation quaternion (q=[w,x,y,z], q*=[w,-x,-y,-z])

	Quaternion p;
	Quaternion_initVal(&p, 0, v->x, v->y, v->z);

	// quaternion multiplication: q * p, stored back in p
	p = Quaternion_getProduct(q, &p);

    Quaternion q_conj;
    q_conj = Quaternion_getConjugate(q);
	// quaternion multiplication: p * conj(q), stored back in p
	p = Quaternion_getProduct(&p, &q_conj);

	// p quaternion is now [0, x', y', z']
    v->x = (int16_t) (&p)->x;
	v->y = (int16_t) (&p)->y;
	v->z = (int16_t) (&p)->z;
}

VectorInt16 VectorInt16_getRotated(VectorInt16 *v, Quaternion *q) {
	VectorInt16 r;
	VectorInt16_initVal(&r, v->x, v->y, v->z);
	VectorInt16_rotate(v, q);
	return r;
}

typedef struct {
	float x;
    float y;
    float z;
} VectorFloat;

void VectorFloat_init(VectorFloat *v) {
	v->x = 0;
	v->y = 0;
	v->z = 0;
}

void VectorFloat_initVal(VectorFloat *v, float nx, float ny, float nz) {
	v->x = nx;
	v->y = ny;
	v->z = nz;
}

float VectorFloat_getMagnitude(VectorFloat *v) {
	return sqrtf(v->x * v->x + v->y * v->y + v->z * v->z);
}

void VectorFloat_normalize(VectorFloat *v) {
	float m = VectorFloat_getMagnitude(v);
	v->x /= m;
	v->y /= m;
	v->z /= m;
}

VectorFloat VectorFloat_getNormalized(VectorFloat *v) {
	VectorFloat r;
	VectorFloat_initVal(&r, v->x, v->y, v->z);
	VectorFloat_normalize(&r);
	return r;
}

void VectorFloat_rotate(VectorFloat *v, Quaternion *q) {
	Quaternion p;
	Quaternion_initVal(&p, 0, v->x, v->y, v->z);

	// quaternion multiplication: q * p, stored back in p
	p = Quaternion_getProduct(q, &p);

    Quaternion q_conj;
    q_conj = Quaternion_getConjugate(q);
	// quaternion multiplication: p * conj(q), stored back in p
	p = Quaternion_getProduct(&p, &q_conj);

	// p quaternion is now [0, x', y', z']
	v->x = (&p)->x;
	v->y = (&p)->y;
	v->z = (&p)->z;
}

VectorFloat VectorFloat_getRotated(VectorFloat *v, Quaternion *q) {
	VectorFloat r;
	VectorFloat_initVal(&r, v->x, v->y, v->z);
	VectorFloat_rotate(v, q);
	return r;
}

#endif /* _HELPER_3DMATH_H_ */
