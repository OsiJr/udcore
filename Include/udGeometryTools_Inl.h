
#ifdef UD_USE_EXACT_MATH
template<typename T> bool udIsZero(T value) { return value == T(0); }
#else
template<typename T> bool udIsZero(T value) { return udAbs(value) < udGetEpsilon<T>(); }
#endif

// ****************************************************************************
// Author: Frank Hart, July 2020
template<typename T>
bool udAreEqual(T a, T b)
{
  return udIsZero(a - b);
}

// ****************************************************************************
// Author: Frank Hart, July 2020
template<typename T>
bool udAreEqual(const udVector3<T> &a, const udVector3<T> &b)
{
  return udAreEqual(a[0], b[0]) && udAreEqual(a[1], b[1]) && udAreEqual(a[2], b[2]);
}

// ****************************************************************************
// Author: Frank Hart, July 2020
template<typename T>
T udGeometry_ScalarTripleProduct(const udVector3<T> &u, const udVector3<T> &v, const udVector3<T> &w)
{
  return udDot(udCross3(u, v), w);
}

// ****************************************************************************
// Author: Frank Hart, August 2020
template<typename T>
udGeometryCode udGeometry_CreatePlane(const udVector3<T> &p0, const udVector3<T> &p1, const udVector3<T> &p2, udPlane<T> &out)
{
  //Get plane vector
  udVector3<T> u = p1 - p0;
  udVector3<T> v = p2 - p0;
  udVector3<T> w = udCross3(u, v);

  //normalise for cheap distance checks
  T lensq = w.x * w.x + w.y * w.y + w.z * w.z;

  //recover gracefully
  if (udIsZero(lensq))
    return udGC_Fail;

  T recip = T(1) / udSqrt(lensq);
  udVector3<T> normal = {w.x* recip, w.y * recip, w.z * recip};
  T offset = udDot(-p0, normal);
  out = {normal.x, normal.y, normal.z, offset};

  return udGC_Success;
}

// ****************************************************************************
// Author: Frank Hart, August 2020
template<typename T>
void udGeometry_SortLowToHigh(T &a, T &b)
{
  if (b < a)
  {
    T temp = a;
    a = b;
    b = temp;
  }
}

// ****************************************************************************
// Author: Frank Hart, August 2020
template<typename T>
udVector3<T> udGeometry_SortLowToHigh(const udVector3<T> &a)
{
  udVector3<T> result = a;

  udGeometry_SortLowToHigh(result[0], result[1]);
  udGeometry_SortLowToHigh(result[0], result[2]);
  udGeometry_SortLowToHigh(result[1], result[2]);

  return result;
}

// ****************************************************************************
// Author: Frank Hart, August 2020
template<typename T>
T udGeometry_Sum(const udVector3<T> &v)
{
  return v[0] + v[1] + v[2];
}

// ****************************************************************************
// Author: Frank Hart, August 2020
template<typename T>
T udGeometry_GetTriangleArea(const udVector3<T> &t0, const udVector3<T> &t1, const udVector3<T> &t2)
{
  return udGeometry_GetTriangleArea(udGeometry_GetTriangleSideLengths(t0, t1, t2));
}

// ****************************************************************************
// Author: Frank Hart, August 2020
template<typename T>
T udGeometry_GetTriangleArea(const udVector2<T> &t0, const udVector2<T> &t1, const udVector2<T> &t2)
{
  return udGeometry_GetTriangleArea(udGeometry_GetTriangleSideLengths(t0, t1, t2));
}

// ****************************************************************************
// Author: Frank Hart, August 2020
template<typename T>
T udGeometry_GetTriangleArea(const udVector3<T> &sideLengths)
{
  T p = (sideLengths[0] + sideLengths[1] + sideLengths[2]) / T(2);

  // Theoritically these values should not be below zero, but due to floting point
  // error, they can be. So we need to check.
  T a = (p - sideLengths[0]);
  if (a <= T(0))
    return T(0);

  T b = (p - sideLengths[1]);
  if (b <= T(0))
    return T(0);

  T c = (p - sideLengths[2]);
  if (c <= T(0))
    return T(0);

  return udSqrt(p * a * b * c);
}

// ****************************************************************************
// Author: Frank Hart, August 2020
template<typename T>
udVector3<T> udGeometry_GetTriangleSideLengths(const udVector3<T> &t0, const udVector3<T> &t1, const udVector3<T> &t2)
{
  return udVector3<T>::create(udMag(t0 - t1), udMag(t0 - t2), udMag(t1 - t2));
}

// ****************************************************************************
// Author: Frank Hart, August 2020
template<typename T>
udVector3<T> udGeometry_GetTriangleSideLengths(const udVector2<T> &t0, const udVector2<T> &t1, const udVector2<T> &t2)
{
  return udVector3<T>::create(udMag(t0 - t1), udMag(t0 - t2), udMag(t1 - t2));
}

// ****************************************************************************
// Author: Frank Hart, August 2020
template<typename T>
udGeometryCode udGeometry_CreatePlane(const udVector3<T> &point, const udVector3<T> &normal, udPlane<T> &out)
{
  out = udPlane<T>::create(normal.x, normal.y, normal.z, -udDot(point, normal));
  return udGC_Success;
}

// ****************************************************************************
// Author: Frank Hart, August 2020
template<typename T> T udGeometry_SignedDistance(const udPlane<T> &plane, const udVector3<T> &point)
{
  return udDot(point, plane.toVector3()) + plane.w;
}

// ****************************************************************************
// Author: Frank Hart, August 2020
template<typename T> udGeometryCode udGeometry_CPPointPlane(const udPlane<T> &plane, const udVector3<T> &point, udVector3<T> &out)
{
  T signedDistance = udGeometry_SignedDistance(plane, point);
  out = point - signedDistance * plane.toVector3();

  return udGC_Success;
}

// ****************************************************************************
// Author: Frank Hart, June 2020
template<typename T>
udGeometryCode udGeometry_CPPointLine3(const udVector3<T> &lineOrigin, const udVector3<T> &lineDirection, const udVector3<T> &point, udVector3<T> &out, T * pU)
{
  udVector3<T> w;
  w = point - lineOrigin;
  T proj = udDot(w, lineDirection);
  out = lineOrigin + proj * lineDirection;
  if (pU != nullptr)
    *pU = proj;
  return udGC_Success;
}

// ****************************************************************************
// Author: Frank Hart, June 2020
template<typename T>
udGeometryCode udGeometry_CPPointSegment3(const udVector3<T> &ls0, const udVector3<T> &ls1, const udVector3<T> &point, udVector3<T> &out, T *pU)
{
  udVector3<T> w = point - ls0;
  udVector3<T> axis = ls1 - ls0;
  double u;

  double proj = udDot(w, ls1 - ls0);

  if (proj <= T(0))
  {
    u = T(0);
  }
  else
  {
    double vsq = udMagSq3(axis);

    if (proj >= vsq)
      u = T(1);
    else
      u = (proj / vsq);
  }

  out = ls0 + u * axis;

  if (pU != nullptr)
    *pU = u;

  return udGC_Success;
}

// ****************************************************************************
// Author: Frank Hart, June 2020
// Based of the work of James M. Van Verth and Lars M. Biship, taken from 'Essential Mathematics for Games and Interactive Applications: A Programmer's Guide, Second Edition
template<typename T>
udGeometryCode udGeometry_CPSegmentSegment3(const udVector3<T> &a0, const udVector3<T> &a1, const udVector3<T> &b0, const udVector3<T> &b1, udVector3<T> &aOut, udVector3<T> &bOut, udVector2<T> * pU)
{
  T ua, ub; //could be outputs?
  udGeometryCode result = udGC_Success;

  //directions
  udVector3<T> da = a1 - a0;
  udVector3<T> db = b1 - b0;

  //compute intermediate parameters
  udDouble3 w0(a0 - b0);
  double a = udDot(da, da);
  double b = udDot(da, db);
  double c = udDot(db, db);
  double d = udDot(da, w0);
  double e = udDot(db, w0);
  double denom = a*c - b*b;

  double sn, sd, tn, td;

  // if denom is zero, try finding closest point on segment1 to origin0
  if (udIsZero(denom))
  {
    if (udIsZero(a)) //length of a is 0
    {
      if (udIsZero(c)) //length of b is also 0
      {
        ua = T(0);
        ub = T(0);
        aOut = a0;
        bOut = b0;
      }
      else
      {
      udGeometry_CPPointSegment3(b0, b1, a0, bOut, &ub);
      aOut = a0;
      ua = T(0);
      }
      goto epilogue;
    }
    else if (udIsZero(c)) //length of b is 0
    {
    udGeometry_CPPointSegment3(a0, a1, b0, aOut, &ua);
    bOut = b0;
    ub = T(0);
    goto epilogue;
    }

    // clamp ua to 0
    sd = td = c;
    sn = T(0);
    tn = e;

    //Do the line segments overlap?
    udVector3<T> w1((a0 + da) - b0);
    udVector3<T> w2(a0 - (b0 + db));
    udVector3<T> w3((a0 + da) - (b0 + db));
    bool bse = (e < T(0));
    if (!(bse == (udDot(w1, db) < T(0)) && bse == (udDot(w2, db) < T(0)) && bse == (udDot(w3, db) < T(0))))
      result = udGC_Overlapping;
  }
  else
  {
  // clamp ua within [0,1]
  sd = td = denom;
  sn = b * e - c * d;
  tn = a * e - b * d;

  // clamp ua to 0
  if (sn < T(0))
  {
    sn = T(0);
    tn = e;
    td = c;
  }
  // clamp ua to 1
  else if (sn > sd)
  {
    sn = sd;
    tn = e + b;
    td = c;
  }
  }

  // clamp ub within [0,1]
  // clamp ub to 0
  if (tn < T(0))
  {
    ub = T(0);
    // clamp ua to 0
    if (-d < T(0))
      ua = T(0);
    // clamp ua to 1
    else if (-d > a)
      ua = T(1);
    else
      ua = -d / a;
  }
  // clamp ub to 1
  else if (tn > td)
  {
    ub = T(1);
    // clamp ua to 0
    if ((-d + b) < T(0))
      ua = T(0);
    // clamp ua to 1
    else if ((-d + b) > a)
      ua = T(1);
    else
      ua = (-d + b) / a;
  }
  else
  {
    ub = tn / td;
    ua = sn / sd;
  }

epilogue:
  aOut = a0 + ua * da;
  bOut = b0 + ub * db;

  if (pU != nullptr)
  {
    pU->x = ua;
    pU->y = ub;
  }

  return result;
}

// ****************************************************************************
// Author: Frank Hart, July 2020
// Based on Real Time Collision Detection, Christer Ericson p184
template<typename T>
udGeometryCode udGeometry_Barycentric(const udVector3<T> &t0, const udVector3<T> &t1, const udVector3<T> &t2, const udVector3<T> &p, T &u, T &v, T &w)
{
udVector3<T> v0 = t1 - t0;
udVector3<T> v1 = t2 - t0;
udVector3<T> v2 = p - t0;

T d00 = udDot(v0, v0);
T d01 = udDot(v0, v1);
T d11 = udDot(v1, v1);
T d20 = udDot(v2, v0);
T d21 = udDot(v2, v1);

T denom = d00 * d11 - d01 * d01;

//Check for demon == 0?

v = (d11 * d20 - d01 * d21) / denom;
w = (d00 * d21 - d01 * d20) / denom;
u = T(1) - v - w;

return udGC_Success;
}

// ****************************************************************************
// Author: Frank Hart, July 2020
template<typename T>
udGeometryCode udGeometry_FindQuaternion(const udPlane<T> &plane, udQuaternion<T> &q)
{

}

// ****************************************************************************
// Author: Frank Hart, July 2020
// Based on Real Time Collision Detection, Christer Ericson p184
template<typename T>
udGeometryCode udGeometry_FISegmentTriangle3(const udVector3<T> &t0, const udVector3<T> &t1, const udVector3<T> &t2, const udVector3<T> &s0, const udVector3<T> &s1, udVector3<T> &intersect0, udVector3<T> &intersect1)
{
  udVector3<T> s0s1 = s1 - s0;
  udVector3<T> s0t0 = t0 - s0;
  udVector3<T> s0t1 = t1 - s0;
  udVector3<T> s0t2 = t2 - s0;

  T u = udGeometry_ScalarTripleProduct(s0s1, s0t2, s0t1);
  T v = udGeometry_ScalarTripleProduct(s0s1, s0t0, s0t2);
  T w = udGeometry_ScalarTripleProduct(s0s1, s0t1, s0t0);

  // TODO Line is on triangle plane
  if (udIsZero(u) && udIsZero(v) && udIsZero(w))
  {
    // Flag as fail for now...
    return udGC_Fail;
  }

  int sign = 0;
  sign |= (u < T(0) ? 1 : 0);
  sign |= (v < T(0) ? 2 : 0);
  sign |= (w < T(0) ? 4 : 0);

  if (sign > 0 && sign < 7)
    return udGC_NotIntersecting;

  T denom = T(1) / (u + v + w);
  u *= denom;
  v *= denom;
  w *= denom;

  intersect0 = u * t0 + v * t1 + w * t2;

  return udGC_Intersecting;
}

template<typename T>
udGeometryCode udGeometry_CPPointTriangle3(const udVector3<T> &t0, const udVector3<T> &t1, const udVector3<T> &t2, const udVector3<T> &point, udVector3<T> &out)
{
  udVector3<T> v01 = t1 - t0;
  udVector3<T> v02 = t2 - t0;
  udVector3<T> v0p = point - t0;

  T d1 = udDot(v01, v0p);
  T d2 = udDot(v02, v0p);

  udGeometryCode result = udGC_Success;

  do
  {
    if (d1 <= T(0) && d2 <= T(0))
    {
      out = t0;
      break;
    }

    udVector3<T> v1p = point - t1;
    T d3 = udDot(v01, v1p);
    T d4 = udDot(v02, v1p);
    if (d3 >= T(0) && d4 <= d3)
    {
      out = t1;
      break;
    }

    T v2 = d1 * d4 - d3 * d2;
    if (v2 <= T(0) && d1 >= T(0) && d3 <= T(0))
    {
      T v = d1 / (d1 - d3);
      out = t0 + v * v01;
      break;
    }

    udVector3<T> v2p = point - t2;
    T d5 = udDot(v01, v2p);
    T d6 = udDot(v02, v2p);
    if (d6 >= T(0) && d5 <= d6)
    {
      out = t2;
      break;
    }

    T v1 = d5 * d2 - d1 * d6;
    if (v1 <= T(0) && d2 >= T(0) && d6 <= T(0))
    {
      T w = d2 / (d2 - d6);
      out = t0 + w * v02;
      break;
    }

    T v0 = d3 * d6 - d5 * d4;
    if (v0 <= T(0) && (d4 - d3) >= T(0) && (d5 - d6) >= T(0))
    {
      T w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
      out = t1 + w * (t2 - t1);
      break;
    }

    T denom = T(1) / (v0 + v1 + v2);
    T v = v1 * denom;
    T w = v2 * denom;
    out = t0 + v01 * v + v02 * w;

  } while (false);

  return result;
}
