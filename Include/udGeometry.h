#ifndef UDGEOMETRY_H
#define UDGEOMETRY_H

#include "udMath.h"

//------------------------------------------------------------------------
// Constants
//------------------------------------------------------------------------

//#define UD_USE_EXACT_MATH

//Check individual geometry queries to determine possible return values.
enum udGeometryCode
{
  udGC_Success,
  udGC_Fail,
  udGC_Overlapping,
  udGC_Intersecting,
  udGC_NotIntersecting,
  udGC_CompletelyInside,
  udGC_CompletelyOutside
};

template<typename T>
using udPlane = udVector3<T>;

//Create a plane from 3 points that lie on the plane
template<typename T> udPlane<T> udGeometry_CreatePlane(const udVector3<T> &p0, const udVector3<T> &p1, const udVector3<T> &p2);

//Create a plane from a point and a normal. It is assumed the normal is a unit vector
template<typename T> udPlane<T> udGeometry_CreatePlane(const udVector3<T> &point, const udVector3<T> &normal);

//The scalar triple product is defined as ((v x u) . w), which is equivilent to the signed
//volume of the parallelepiped formed by the three vectors u, v and w.
template<typename T> T udGeometry_ScalarTripleProduct(const udVector3<T> &u, const udVector3<T> &v, const udVector3<T> &w);

//If UD_USE_EXACT_MATH is not defined, this function tests if value is within an epsilon of zero, as defined in udGetEpsilon().
//Otherwise it will test if value == T(0)
template<typename T> bool udIsZero(T value);

//Closest point between a point and a line in 3D.
//Returns: udGC_Success
template<typename T> udGeometryCode udGeometry_CPPointLine3(const udVector3<T> &lineOrigin, const udVector3<T> &lineDirection, const udVector3<T> &point, udVector3<T> &out, T *pU = nullptr);

//Closest point between a point and a line segment in 3D.
//Returns: udGC_Success
template<typename T> udGeometryCode udGeometry_CPPointSegment3(const udVector3<T> &ls0, const udVector3<T> &ls1, const udVector3<T> &point, udVector3<T> &out, T *pU = nullptr);

//Closest point between two line segments in 3D.
//Returns: udGC_Success
//         udGC_Overlapping if the segments are overlapping in a way that produces an infinite number of closest points. In this case, a point is chosen along this region to be the closest points set.
template<typename T> udGeometryCode udGeometry_CPSegmentSegment3(const udVector3<T> &a0, const udVector3<T> &a1, const udVector3<T> &b0, const udVector3<T> &b1, udVector3<T> &aOut, udVector3<T> &bOut, udVector2<T> *pU = nullptr);

//Intersection test between line segment and triangle.
//Returns: udGC_Intersecting,
//         udGC_NotIntersecting
template<typename T> udGeometryCode udGeometry_FISegmentTriangle3(const udVector3<T> &t0, const udVector3<T> &t1, const udVector3<T> &t2, const udVector3<T> &s0, const udVector3<T> &s1, udVector3<T> *pIntersect = nullptr);

#include "udGeometry_Inl.h"

#endif
