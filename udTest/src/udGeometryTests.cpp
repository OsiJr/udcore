#include "udGeometry.h"
#include "gtest/gtest.h"
#include "udPlatform.h"

#define EXPECT_VEC3_NEAR(v0, v1, e) EXPECT_NEAR(v0.x, v1.x, e);\
EXPECT_NEAR(v0.y, v1.y, e);\
EXPECT_NEAR(v0.z, v1.z, e)

TEST(GeometryTests, GeometryLines)
{
  //point vs line
  {
    udDouble3 lineOrigin = {1.0, 1.0, 1.0};
    udDouble3 lineDirection = {1.0, 0.0, 0.0};
    double u;
    udDouble3 cp;
    udDouble3 point;
    udGeometryCode result;

    //Point lies 'before' line origin
    point = {-3.0, 1.0, 2.0};
    result = udGeometry_CPPointLine3(lineOrigin, lineDirection, point, cp, &u);
    EXPECT_EQ(result, udGC_Success);
    EXPECT_EQ(u, -4.0);
    EXPECT_EQ(cp, udDouble3::create(-3.0, 1.0, 1.0));

    //Point lies perpendicular to line origin
    point = {1.0, 1.0, 2.0};
    result = udGeometry_CPPointLine3(lineOrigin, lineDirection, point, cp, &u);
    EXPECT_EQ(result, udGC_Success);
    EXPECT_EQ(u, 0.0);
    EXPECT_EQ(cp, udDouble3::create(1.0, 1.0, 1.0));

    //Point lies 'after' line origin
    point = {7.0, 1.0, 2.0};
    result = udGeometry_CPPointLine3(lineOrigin, lineDirection, point, cp, &u);
    EXPECT_EQ(result, udGC_Success);
    EXPECT_EQ(u, 6.0);
    EXPECT_EQ(cp, udDouble3::create(7.0, 1.0, 1.0));
  }

  //point v segment
  {
    udDouble3 s0 = {1.0, 1.0, 1.0};
    udDouble3 s1 = {3.0, 1.0, 1.0};
    double u;
    udDouble3 cp;
    udDouble3 point;
    udGeometryCode result;

    //Point lies 'before' segment 0
    point = {-1.0, 1.0, 1.0};
    result = udGeometry_CPPointSegment3(s0, s1, point, cp, &u);
    EXPECT_EQ(result, udGC_Success);
    EXPECT_EQ(u, 0.0);
    EXPECT_EQ(cp, s0);

    //Point lies 'after' segment 1
    point = {5.0, 1.0, 1.0};
    result = udGeometry_CPPointSegment3(s0, s1, point, cp, &u);
    EXPECT_EQ(result, udGC_Success);
    EXPECT_EQ(u, 1.0);
    EXPECT_EQ(cp, s1);

    //Point lies along the segment line
    point = {2.0, 10.0, 42.0};
    result = udGeometry_CPPointSegment3(s0, s1, point, cp, &u);
    EXPECT_EQ(result, udGC_Success);
    EXPECT_EQ(u, 0.5);
    EXPECT_EQ(cp, udDouble3::create(2.0, 1.0, 1.0));
  }

  //segment v segment
  {
    udDouble3 a0, a1, b0, b1, cpa, cpb;
    udDouble2 u;
    udGeometryCode result;

    //Segments have zero length
    a0 = b0 = {1.0, 4.0, 12.0};
    a1 = b1 = {1.0, 4.0, 12.0};
    result = udGeometry_CPSegmentSegment3(a0, a1, b0, b1, cpa, cpb, &u);
    EXPECT_EQ(result, udGC_Success);
    EXPECT_EQ(u[0], 0.0);
    EXPECT_EQ(u[1], 0.0);
    EXPECT_EQ(cpa, a0);
    EXPECT_EQ(cpb, b0);

    a0 = {2.0, 0.0, 0.0};
    a1 = {6.0, 0.0, 0.0};

    //LineSegs parallel, no overlap, closest points a0, b0
    b0 = {-1.0, -4.0, 12.0};
    b1 = {-5.0, -4.0, 12.0};
    result = udGeometry_CPSegmentSegment3(a0, a1, b0, b1, cpa, cpb, &u);
    EXPECT_EQ(result, udGC_Success);
    EXPECT_EQ(u[0], 0.0);
    EXPECT_EQ(u[1], 0.0);
    EXPECT_EQ(cpa, a0);
    EXPECT_EQ(cpb, b0);

    //LineSegs parallel, no overlap, closest points a0, b1
    b0 = {-5.0, -4.0, 12.0};
    b1 = {-1.0, -4.0, 12.0};
    result = udGeometry_CPSegmentSegment3(a0, a1, b0, b1, cpa, cpb, &u);
    EXPECT_EQ(result, udGC_Success);
    EXPECT_EQ(u[0], 0.0);
    EXPECT_EQ(u[1], 1.0);
    EXPECT_EQ(cpa, a0);
    EXPECT_EQ(cpb, b1);

    //LineSegs parallel, no overlap, closest points a1, b0
    b0 = {9.0, -4.0, 12.0};
    b1 = {18.0, -4.0, 12.0};
    result = udGeometry_CPSegmentSegment3(a0, a1, b0, b1, cpa, cpb, &u);
    EXPECT_EQ(result, udGC_Success);
    EXPECT_EQ(u[0], 1.0);
    EXPECT_EQ(u[1], 0.0);
    EXPECT_EQ(cpa, a1);
    EXPECT_EQ(cpb, b0);

    //LineSegs parallel, no overlap, closest points a1, b1
    b0 = {10.0, -4.0, 12.0};
    b1 = {9.0, -4.0, 12.0};
    result = udGeometry_CPSegmentSegment3(a0, a1, b0, b1, cpa, cpb, &u);
    EXPECT_EQ(result, udGC_Success);
    EXPECT_EQ(u[0], 1.0);
    EXPECT_EQ(u[1], 1.0);
    EXPECT_EQ(cpa, a1);
    EXPECT_EQ(cpb, b1);

    //LineSegs parallel, overlap, a0---b0---a1---b1
    b0 = {4.0, -3.0, 4.0};
    b1 = {10.0, -3.0, 4.0};
    result = udGeometry_CPSegmentSegment3(a0, a1, b0, b1, cpa, cpb, &u);
    //Why udQCOverlapping and not udQC_Parallel? Because overlapping segments are already parallel.
    EXPECT_EQ(result, udGC_Overlapping);
    EXPECT_EQ(u[0], 0.5);
    EXPECT_EQ(u[1], 0.0);
    EXPECT_EQ(cpa, udDouble3::create(4.0, 0.0, 0.0));
    EXPECT_EQ(cpb, b0);

    //LineSegs parallel, overlap, a1---b0---a0---b1
    b0 = {4.0, -3.0, 4.0};
    b1 = {0.0, -3.0, 4.0};
    result = udGeometry_CPSegmentSegment3(a0, a1, b0, b1, cpa, cpb, &u);
    EXPECT_EQ(result, udGC_Overlapping);
    EXPECT_EQ(u[0], 0.0);
    EXPECT_EQ(u[1], 0.5);
    EXPECT_EQ(cpa, a0);
    EXPECT_EQ(cpb, udDouble3::create(2.0, -3.0, 4.0));

    //LineSegs parallel, overlap, a0---b0---b1---a1
    b0 = {4.0, -3.0, 4.0};
    b1 = {5.0, -3.0, 4.0};
    result = udGeometry_CPSegmentSegment3(a0, a1, b0, b1, cpa, cpb, &u);
    EXPECT_EQ(result, udGC_Overlapping);
    EXPECT_EQ(u[0], 0.5);
    EXPECT_EQ(u[1], 0.0);
    EXPECT_EQ(cpa, udDouble3::create(4.0, 0.0, 0.0));
    EXPECT_EQ(cpb, b0);

    //LineSegs parallel, overlap, a1---b0---b1---a0
    b0 = {4.0, -3.0, 4.0};
    b1 = {8.0, -3.0, 4.0};
    result = udGeometry_CPSegmentSegment3(a0, a1, b0, b1, cpa, cpb, &u);
    EXPECT_EQ(result, udGC_Overlapping);
    EXPECT_EQ(u[0], 0.5);
    EXPECT_EQ(u[1], 0.0);
    EXPECT_EQ(cpa, udDouble3::create(4.0, 0.0, 0.0));
    EXPECT_EQ(cpb, b0);

    //LineSegs not parallel, closest points: a0, b0
    b0 = {-2.0, -5.0, 20.0};
    b1 = {-2.0, -5.0, 23.0};
    result = udGeometry_CPSegmentSegment3(a0, a1, b0, b1, cpa, cpb, &u);
    EXPECT_EQ(result, udGC_Success);
    EXPECT_EQ(u[0], 0.0);
    EXPECT_EQ(u[1], 0.0);
    EXPECT_EQ(cpa, a0);
    EXPECT_EQ(cpb, b0);

    //LineSegs not parallel, closest points: a0, b1
    b0 = {-2.0, -5.0, 23.0};
    b1 = {-2.0, -5.0, 20.0};
    result = udGeometry_CPSegmentSegment3(a0, a1, b0, b1, cpa, cpb, &u);
    EXPECT_EQ(result, udGC_Success);
    EXPECT_EQ(u[0], 0.0);
    EXPECT_EQ(u[1], 1.0);
    EXPECT_EQ(cpa, a0);
    EXPECT_EQ(cpb, b1);

    //LineSegs not parallel, closest points: a1, b0
    b0 = {10.0, -5.0, 20.0};
    b1 = {10.0, -5.0, 23.0};
    result = udGeometry_CPSegmentSegment3(a0, a1, b0, b1, cpa, cpb, &u);
    EXPECT_EQ(result, udGC_Success);
    EXPECT_EQ(u[0], 1.0);
    EXPECT_EQ(u[1], 0.0);
    EXPECT_EQ(cpa, a1);
    EXPECT_EQ(cpb, b0);

    //LineSegs not parallel, closest points: a1, b1
    b0 = {10.0, -5.0, 23.0};
    b1 = {10.0, -5.0, 20.0};
    result = udGeometry_CPSegmentSegment3(a0, a1, b0, b1, cpa, cpb, &u);
    EXPECT_EQ(result, udGC_Success);
    EXPECT_EQ(u[0], 1.0);
    EXPECT_EQ(u[1], 1.0);
    EXPECT_EQ(cpa, a1);
    EXPECT_EQ(cpb, b1);

    //LineSegs not parallel, closest points: a0, ls1-along ls
    b0 = {-1.0, 4.0, -3.0};
    b1 = {-1.0, 4.0, 3.0};
    result = udGeometry_CPSegmentSegment3(a0, a1, b0, b1, cpa, cpb, &u);
    EXPECT_EQ(result, udGC_Success);
    EXPECT_EQ(u[0], 0.0);
    EXPECT_EQ(u[1],0.5);
    EXPECT_EQ(cpa, a0);
    EXPECT_EQ(cpb, udDouble3::create(-1.0, 4.0, 0.0));

    //LineSegs not parallel, closest points: a1, ls1-along ls
    b0 = {9.0, 4.0, -3.0};
    b1 = {9.0, 4.0, 3.0};
    result = udGeometry_CPSegmentSegment3(a0, a1, b0, b1, cpa, cpb, &u);
    EXPECT_EQ(result, udGC_Success);
    EXPECT_EQ(u[0], 1.0);
    EXPECT_EQ(u[1], 0.5);
    EXPECT_EQ(cpa, a1);
    EXPECT_EQ(cpb, udDouble3::create(9.0, 4.0, 0.0));

    //LineSegs not parallel, closest points: ls0-along ls, ls1-along ls
    b0 = {4.0, 4.0, -3.0};
    b1 = {4.0, -4.0, -3.0};
    result = udGeometry_CPSegmentSegment3(a0, a1, b0, b1, cpa, cpb, &u);
    EXPECT_EQ(result, udGC_Success);
    EXPECT_EQ(u[0], 0.5);
    EXPECT_EQ(u[1], 0.5);
    EXPECT_EQ(cpa, udDouble3::create(4.0, 0.0, 0.0));
    EXPECT_EQ(cpb, udDouble3::create(4.0, 0.0, -3.0));
  }
}

TEST(GeometryTests, GeometryEquivalence)
{
  EXPECT_TRUE(udAreEqual<double>(1.0, 1.0));
  EXPECT_TRUE(udAreEqual<double>(1.0, 1.0 + udGetEpsilon<double>() * 0.5));

  EXPECT_FALSE(udAreEqual<double>(2.0, 3.0));
  EXPECT_FALSE(udAreEqual<double>(2.0, 2.0 + udGetEpsilon<double>() * 1.5));

  udDouble3 a = {1.0, 2.0, 3.0};
  udDouble3 b = {1.01, 2.01, 3.01};

  EXPECT_TRUE(udAreEqual(a, a));
  EXPECT_FALSE(udAreEqual(a, b));
}

bool CheckBarycentricResult(udDouble3 t0, udDouble3 t1, udDouble3 t2, udDouble3 p, double u, double v, double w)
{
  double epsilon = 0.01;
  double ru, rv, rw;
  udGeometryCode result = udGeometry_Barycentric(t0, t1, t2, p, ru, rv, rw);
  if (result != udGC_Success)
    return false;

  if (udAbs(ru - u) > epsilon)
    return false;

  if (udAbs(rv - v) > epsilon)
    return false;

  if (udAbs(rw - w) > epsilon)
    return false;

  return true;
}

//Compared with results from https: //www.geogebra.org/m/ZuvmPjmy
TEST(GeometryTests, GeometryBaryCentric)
{
  udDouble3 t0 = {};
  udDouble3 t1 = {};
  udDouble3 t2 = {};

  double a = 1.0;
  double b = udSqrt(3.0) / 2.0;
  double c = 0.5;

  t0 = {0.0, a, 0.0};
  t1 = {b, -c, 0.0};
  t2 = {-b, -c, 0.0};

  EXPECT_TRUE(CheckBarycentricResult(t0, t1, t2, {0.0, 0.0, 0.0}, 0.33, 0.33, 0.33));
  EXPECT_TRUE(CheckBarycentricResult(t0, t1, t2, {0.0, 0.0, 345.0}, 0.33, 0.33, 0.33));
  EXPECT_TRUE(CheckBarycentricResult(t0, t1, t2, {0.0, 0.0, -327}, 0.33, 0.33, 0.33));

  EXPECT_TRUE(CheckBarycentricResult(t0, t1, t2, {1.0, 1.0, 0.0}, 1.0, 0.58, -0.58));
  EXPECT_TRUE(CheckBarycentricResult(t0, t1, t2, {1.0, 1.0, 345.0}, 1.0, 0.58, -0.58));
  EXPECT_TRUE(CheckBarycentricResult(t0, t1, t2, {1.0, 1.0, -327}, 1.0, 0.58, -0.58));

  EXPECT_TRUE(CheckBarycentricResult(t0, t1, t2, {0.0, 1.0, 0.0}, 1.0, 0.0, 0.0));
  EXPECT_TRUE(CheckBarycentricResult(t0, t1, t2, {0.0, 1.0, 345.0}, 1.0, 0.0, 0.0));
  EXPECT_TRUE(CheckBarycentricResult(t0, t1, t2, {0.0, 1.0, -327}, 1.0, 0.0, 0.0));

  EXPECT_TRUE(CheckBarycentricResult(t0, t1, t2, {0.0, 2.0, 0.0}, 1.67, -0.33, -0.33));
  EXPECT_TRUE(CheckBarycentricResult(t0, t1, t2, {0.0, 2.0, 345.0}, 1.67, -0.33, -0.33));
  EXPECT_TRUE(CheckBarycentricResult(t0, t1, t2, {0.0, 2.0, -327}, 1.67, -0.33, -0.33));

  EXPECT_TRUE(CheckBarycentricResult(t0, t1, t2, {-1.0, 1.0, 0.0}, 1.0, -0.58, 0.58));
  EXPECT_TRUE(CheckBarycentricResult(t0, t1, t2, {-1.0, 1.0, 345.0}, 1.0, -0.58, 0.58));
  EXPECT_TRUE(CheckBarycentricResult(t0, t1, t2, {-1.0, 1.0, -327}, 1.0, -0.58, 0.58));

  EXPECT_TRUE(CheckBarycentricResult(t0, t1, t2, {0.0, -1.0, 0.0}, -0.33, 0.67, 0.67));
  EXPECT_TRUE(CheckBarycentricResult(t0, t1, t2, {0.0, -1.0, 345.0}, -0.33, 0.67, 0.67));
  EXPECT_TRUE(CheckBarycentricResult(t0, t1, t2, {0.0, -1.0, -327}, -0.33, 0.67, 0.67));

  EXPECT_TRUE(CheckBarycentricResult(t0, t1, t2, {0.2, 0.2, 0.0}, 0.47, 0.38, 0.15));
  EXPECT_TRUE(CheckBarycentricResult(t0, t1, t2, {0.2, 0.2, 345.0}, 0.47, 0.38, 0.15));
  EXPECT_TRUE(CheckBarycentricResult(t0, t1, t2, {0.2, 0.2, -327}, 0.47, 0.38, 0.15));
}

TEST(GeometryTests, PlaneCreation)
{
  typedef udPlane<double> udDoublePlane;
  udDoublePlane plane;

  EXPECT_EQ(udGeometry_CreatePlane({1.0, 0.0, 0.0}, {1.0, 1.0, 0.0}, {1.0, 1.0, 1.0}, plane), udGC_Success);
  EXPECT_EQ(plane, udDoublePlane::create(1.0, 0.0, 0.0, -1.0));

  EXPECT_EQ(udGeometry_CreatePlane({1.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, {3.0, 0.0, 0.0}, plane), udGC_Fail);
}

// TODO this need many more tests!
TEST(GeometryTests, CPPointTriangle)
{
  udDouble3 cp = {};
  EXPECT_EQ(udGeometry_CPPointTriangle3({1.0, -1.0, -1.0}, {1.0, 1.0, -1.0}, {1.0, 0.0, 10.0}, {2.0, 0.0, 0.5}, cp), udGC_Success);
  EXPECT_EQ(cp, udDouble3::create(1.0, 0.0, 0.5));
}

TEST(GeometryTests, GeometrySegmentTriangle)
{
  double epsilon = 1e-12;
  udDouble3 t0 = {};
  udDouble3 t1 = {};
  udDouble3 t2 = {};

  udDouble3 p0 = {};
  udDouble3 p1 = {};

  udGeometryCode result = udGC_Fail;
  udDouble3 intersect0 = {};
  udDouble3 intersect1 = {};

  //------------------------------------------------------------------------
  // Intersecting
  //------------------------------------------------------------------------
  t0 = {0.0, -1.0, 0.0};
  t1 = {-1.0, 1.0, 0.0};
  t2 = {1.0, 1.0, 0.0};
  p0 = {0.5, 0.5, 1.0};
  p1 = {0.5, 0.5, -1.0};
  result = udGeometry_FISegmentTriangle3(t0, t1, t2, p0, p1, intersect0, intersect1);
  EXPECT_EQ(result, udGC_Intersecting);
  EXPECT_VEC3_NEAR(intersect0, udDouble3::create(0.5, 0.5, 0.0), epsilon);

  t0 = {0.0, -1.0, 0.0};
  t1 = {1.0, 1.0, 0.0};
  t2 = {-1.0, 1.0, 0.0};
  p0 = {0.5, 0.5, 1.0};
  p1 = {0.5, 0.5, -1.0};
  result = udGeometry_FISegmentTriangle3(t0, t1, t2, p0, p1, intersect0, intersect1);
  EXPECT_EQ(result, udGC_Intersecting);
  EXPECT_VEC3_NEAR(intersect0, udDouble3::create(0.5, 0.5, 0.0), epsilon);

  t0 = {0.0, -1.0, 0.0};
  t1 = {-1.0, 1.0, 0.0};
  t2 = {1.0, 1.0, 0.0};
  p0 = {0.5, 0.5, -1.0};
  p1 = {0.5, 0.5, 1.0};
  result = udGeometry_FISegmentTriangle3(t0, t1, t2, p0, p1, intersect0, intersect1);
  EXPECT_EQ(result, udGC_Intersecting);
  EXPECT_VEC3_NEAR(intersect0, udDouble3::create(0.5, 0.5, 0.0), epsilon);

  t0 = {0.0, -1.0, 0.0};
  t1 = {1.0, 1.0, 0.0};
  t2 = {-1.0, 1.0, 0.0};
  p0 = {0.5, 0.5, -1.0};
  p1 = {0.5, 0.5, 1.0};
  result = udGeometry_FISegmentTriangle3(t0, t1, t2, p0, p1, intersect0, intersect1);
  EXPECT_EQ(result, udGC_Intersecting);
  EXPECT_VEC3_NEAR(intersect0, udDouble3::create(0.5, 0.5, 0.0), epsilon);

  t0 = {0.0, 0.0, -1.0};
  t1 = {0.0, -1.0, 1.0};
  t2 = {0.0, 1.0, 1.0};
  p0 = {-2.0, -0.25, 0.25};
  p1 = {2.0, -0.25, 0.25};
  result = udGeometry_FISegmentTriangle3(t0, t1, t2, p0, p1, intersect0, intersect1);
  EXPECT_EQ(result, udGC_Intersecting);
  EXPECT_VEC3_NEAR(intersect0, udDouble3::create(0.0, -0.25, 0.25), epsilon);

  t0 = {0.0, 0.0, -1.0};
  t1 = {0.0, 1.0, 1.0};
  t2 = {0.0, -1.0, 1.0};
  p0 = {-2.0, -0.25, 0.25};
  p1 = {2.0, -0.25, 0.25};
  result = udGeometry_FISegmentTriangle3(t0, t1, t2, p0, p1, intersect0, intersect1);
  EXPECT_EQ(result, udGC_Intersecting);
  EXPECT_VEC3_NEAR(intersect0, udDouble3::create(0.0, -0.25, 0.25), epsilon);

  t0 = {0.0, 0.0, -1.0};
  t1 = {0.0, -1.0, 1.0};
  t2 = {0.0, 1.0, 1.0};
  p0 = {2.0, -0.25, 0.25};
  p1 = {-2.0, -0.25, 0.25};
  result = udGeometry_FISegmentTriangle3(t0, t1, t2, p0, p1, intersect0, intersect1);
  EXPECT_EQ(result, udGC_Intersecting);
  EXPECT_VEC3_NEAR(intersect0, udDouble3::create(0.0, -0.25, 0.25), epsilon);

  t0 = {0.0, 0.0, -1.0};
  t1 = {0.0, 1.0, 1.0};
  t2 = {0.0, -1.0, 1.0};
  p0 = {2.0, -0.25, 0.25};
  p1 = {-2.0, -0.25, 0.25};
  result = udGeometry_FISegmentTriangle3(t0, t1, t2, p0, p1, intersect0, intersect1);
  EXPECT_EQ(result, udGC_Intersecting);
  EXPECT_VEC3_NEAR(intersect0, udDouble3::create(0.0, -0.25, 0.25), epsilon);

  //------------------------------------------------------------------------
  // Non intersecting
  //------------------------------------------------------------------------
  t0 = {0.0, -1.0, 0.0};
  t1 = {-1.0, 1.0, 0.0};
  t2 = {1.0, 1.0, 0.0};
  p0 = {10.0, 0.5, 1.0};
  p1 = {10.0, 0.5, -1.0};
  result = udGeometry_FISegmentTriangle3(t0, t1, t2, p0, p1, intersect0, intersect1);
  EXPECT_EQ(result, udGC_NotIntersecting);

}
