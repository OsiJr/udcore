#ifndef UDGEOMETRYTOOLS_H
#define UDGEOMETRYTOOLS_H

#include "udMath.h"

//------------------------------------------------------------------------
// Constants and Types
//------------------------------------------------------------------------

// If UD_USE_EXACT_MATH is defined, values are compared for exactness,
// otherwise a tolerance is used

// Check individual geometry queries to determine possible return values.
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
using udPlane = udVector4<T>;

//--------------------------------------------------------------------------------
// Utility
//--------------------------------------------------------------------------------

template<typename T> bool udAreEqual(T a, T b);
template<typename T> bool udAreEqual(const udVector3<T> &v0, const udVector3<T> &v1);

// If UD_USE_EXACT_MATH is not defined, this function tests if value is within an epsilon of zero, as defined in udGetEpsilon().
// Otherwise it will test if value == T(0)
template<typename T> bool udIsZero(T value);

// Utility function to sort two values
template<typename T> void udGeometry_SortLowToHigh(T &a, T &b);

// Utility function to sort vector elements
template<typename T> udVector3<T> udGeometry_SortLowToHigh(const udVector3<T> &a);

// Utility function to sum vector elements
template<typename T> T udGeometry_Sum(const udVector3<T> &v);

// The scalar triple product is defined as ((v x u) . w), which is equivilent to the signed
// volume of the parallelepiped formed by the three vectors u, v and w.
template<typename T> T udGeometry_ScalarTripleProduct(const udVector3<T> &u, const udVector3<T> &v, const udVector3<T> &w);

// Compute the barycentric coordinates of a point wrt a triangle.
template<typename T> udGeometryCode udGeometry_Barycentric(const udVector3<T> &t0, const udVector3<T> &t1, const udVector3<T> &t2, const udVector3<T> &p, T &u, T &v, T &w);

// Compute triangle area given triangle vertices
template<typename T> T udGeometry_GetTriangleArea(const udVector3<T> &t0, const udVector3<T> &t1, const udVector3<T> &t2);

// Compute triangle area given triangle vertices
template<typename T> T udGeometry_GetTriangleArea(const udVector2<T> &t0, const udVector2<T> &t1, const udVector2<T> &t2);

// Compute triangle area given triangle side lengths
template<typename T> T udGeometry_GetTriangleArea(const udVector3<T> &sideLengths);

// Compute the lengths of the triangle in the order [(t0 - t1), (t0 - t2), (t1 - t2)]
template<typename T> udVector3<T> udGeometry_GetTriangleSideLengths(const udVector3<T> &t0, const udVector3<T> &t1, const udVector3<T> &t2);

// Compute the lengths of the triangle in the order [(t0 - t1), (t0 - t2), (t1 - t2)]
template<typename T> udVector3<T> udGeometry_GetTriangleSideLengths(const udVector2<T> &t0, const udVector2<T> &t1, const udVector2<T> &t2);

//--------------------------------------------------------------------------------
// Object Creation
//--------------------------------------------------------------------------------

// Create a plane from 3 points that lie on the plane.
// The plane will have the form [(plane normal), offset from origin].
template<typename T> udGeometryCode udGeometry_CreatePlane(const udVector3<T> &p0, const udVector3<T> &p1, const udVector3<T> &p2, udPlane<T> &out);

// Create a plane from a point and a normal. It is assumed the normal is a unit vector.
// The plane will have the form [(plane normal), offset from origin].
template<typename T> udGeometryCode udGeometry_CreatePlane(const udVector3<T> &point, const udVector3<T> &normal, udPlane<T> &out);

//--------------------------------------------------------------------------------
// Distance Predicates
//--------------------------------------------------------------------------------

// Compute the signed distance between a plane and point
template<typename T> T udGeometry_SignedDistance(const udPlane<T> &plane, const udVector3<T> &point);

//--------------------------------------------------------------------------------
// Intersection Test Predicates
//--------------------------------------------------------------------------------

//--------------------------------------------------------------------------------
// Closest Points Predicates
//--------------------------------------------------------------------------------

// Find the closest point between a point and a plane
template<typename T> udGeometryCode udGeometry_CPPointPlane(const udPlane<T> &plane, const udVector3<T> &point, udVector3<T> &out);

// Closest point between a point and a line in 3D.
// Returns: udGC_Success
template<typename T> udGeometryCode udGeometry_CPPointLine3(const udVector3<T> &lineOrigin, const udVector3<T> &lineDirection, const udVector3<T> &point, udVector3<T> &out, T *pU = nullptr);

// Closest point between a point and a line segment in 3D.
// Returns: udGC_Success
template<typename T> udGeometryCode udGeometry_CPPointSegment3(const udVector3<T> &ls0, const udVector3<T> &ls1, const udVector3<T> &point, udVector3<T> &out, T *pU = nullptr);

// Closest point between two line segments in 3D.
// Returns: udGC_Success
//         udGC_Overlapping if the segments are overlapping in a way that produces an infinite number of closest points. In this case, a point is chosen along this region to be the closest points set.
template<typename T> udGeometryCode udGeometry_CPSegmentSegment3(const udVector3<T> &a0, const udVector3<T> &a1, const udVector3<T> &b0, const udVector3<T> &b1, udVector3<T> &aOut, udVector3<T> &bOut, udVector2<T> *pU = nullptr);

// Find the closest point on a triangle to a point in 3D space.
template<typename T> udGeometryCode udGeometry_CPPointTriangle3(const udVector3<T> &t0, const udVector3<T> &t1, const udVector3<T> &t2, const udVector3<T> &point, udVector3<T> &out);

//--------------------------------------------------------------------------------
// Find Intersection Predicates
//--------------------------------------------------------------------------------

// Intersection test between line segment and triangle.
//
// If the segment does not lie on the plane of the triangle
//   Returns: udGC_Intersecting, intersect0 set
//            udGC_NotIntersecting
//
// If the segment lies on the plane of the triangle
//   Returns: udGC_Intersecting, intersect0 and intersect1 set (intersection will be a segment)
//            udGC_NotIntersecting
template<typename T> udGeometryCode udGeometry_FISegmentTriangle3(const udVector3<T> &t0, const udVector3<T> &t1, const udVector3<T> &t2, const udVector3<T> &s0, const udVector3<T> &s1, udVector3<T> &intersect0, udVector3<T> &intersect1);

#include "udGeometryTools_Inl.h"

#endif
