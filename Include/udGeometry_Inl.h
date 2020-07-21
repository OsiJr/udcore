
#ifdef UD_USE_EXACT_MATH
template<typename T> bool udIsZero(T value) { return value == T(0); }
#else
template<typename T> bool udIsZero(T value) { return udAbs(value) < udGetEpsilon<T>(); }
#endif

// ****************************************************************************
// Author: Frank Hart, July 2020
template<typename T>
T udGeometry_ScalarTripleProduct(const udVector3<T> &u, const udVector3<T> &v, const udVector3<T> &w)
{
  return udDot(udCross3(u, v), w);
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
udGeometryCode udGeometry_Barycentric(const udVector3<T> &t0, const udVector3<T> &t1, const udVector3<T> &t2, const udVector3<T> &p, udVector3<T> &uvw)
{
  udVector3<T> v0 = t1 - t0;
  udVector3<T> v1 = t2 - t0;
  udVector3<T> v2 = p - t0;

  float d00 = Dot(v0, v0);
  float d01 = Dot(v0, v1);
  float d11 = Dot(v1, v1);
  float d20 = Dot(v2, v0);
  float d21 = Dot(v2, v1);

  float denom = d00 * d11 - d01 * d01;

  uvw.x = (d11 * d20 - d01 * d21) / denom;
  uvw.y = (d00 * d21 - d01 * d20) / denom;
  uvw.z = T(1) - v - w;
}

// ****************************************************************************
// Author: Frank Hart, July 2020
// Based on Real Time Collision Detection, Christer Ericson p184
template<typename T>
udGeometryCode udGeometry_FISegmentTriangle3(const udVector3<T> &t0, const udVector3<T> &t1, const udVector3<T> &t2, const udVector3<T> &s0, const udVector3<T> &s1, udVector3<T> *pIntersect)
{
  udVector3<T> s0s1 = s1 - s0;
  udVector3<T> s0t0 = t0 - s0;
  udVector3<T> s0t1 = t1 - s0;
  udVector3<T> s0t2 = t2 - s0;

  T u = udGeometry_ScalarTripleProduct(s0s1, s0t2, s0t1);
  T v = udGeometry_ScalarTripleProduct(s0s1, s0t0, s0t2);
  T w = udGeometry_ScalarTripleProduct(s0s1, s0t1, s0t0);

  //Line is on triangle plane
  if (udIsZero(u) && udIsZero(v) && udIsZero(w))
  {
    //Segment end points inside triangle


    //Segment intersects triangle edge
    /*udVector3<T> pt = {};
    udVector3<T> ps = {};
    udGeometry_CPSegmentSegment3(t0, t1, s0, s1, pt, ps);
    T minDistSq = udMag3Sq(pt - ps);
    udVector<T> */

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

  if (pIntersect)
    *pIntersect = u * t0 + v * t1 + w * t2;

  return udGC_Intersecting;
}
