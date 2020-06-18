#include "udMath.h"
#include "gtest/gtest.h"
#include "udPlatform.h"

// Due to IEEE-754 supporting two different ways to specify 8.f and 2.f, the next three EXPECT_* calls are different
// This appears to only occur when using Clang in Release, and only for udFloat4 - SIMD optimizations perhaps?
#define EXPECT_UDFLOAT2_EQ(expected, _actual) { udFloat2 actual = _actual; EXPECT_FLOAT_EQ(expected.x, actual.x); EXPECT_FLOAT_EQ(expected.y, actual.y); }
#define EXPECT_UDFLOAT3_EQ(expected, _actual) { udFloat3 actual = _actual; EXPECT_FLOAT_EQ(expected.x, actual.x); EXPECT_FLOAT_EQ(expected.y, actual.y); EXPECT_FLOAT_EQ(expected.z, actual.z); }
#define EXPECT_UDFLOAT4_EQ(expected, _actual) { udFloat4 actual = _actual; EXPECT_FLOAT_EQ(expected.x, actual.x); EXPECT_FLOAT_EQ(expected.y, actual.y); EXPECT_FLOAT_EQ(expected.z, actual.z); EXPECT_FLOAT_EQ(expected.w, actual.w); }


// This test slerps 2 quaterions rotating about the -Y axis (backwards) q0 theta PI/4 and q1 theta 3PI/4
template <typename T>
bool udQuaternion_SlerpBasicUnitTest()
{
  udVector3<T> axis = { T(0.0), T(-1.0), T(0.0) };
  const T rad0 = T(UD_PI / 4.0);
  const T rad1 = T(3.0 * UD_PI / 4.0);
  udQuaternion<T> q0 = udQuaternion<T>::create(axis, rad0);
  udQuaternion<T> q1 = udQuaternion<T>::create(axis, rad1);
  udVector3<T> v = { T(1.0), T(0.0),  T(0.0) };

  const int32_t count = 1024;
  const T tCount = T(count);
  for (int32_t i = 0; i <= count; ++i)
  {
    T inc = T(i) / tCount;
    udQuaternion<T> qr = udSlerp(q0, q1, T(inc));
    udMatrix4x4<T> mr = udMatrix4x4<T>::rotationAxis(axis, rad0 * (T(1) - inc) + inc * rad1);
    udVector3<T> vqr = qr.apply(v);
    udVector3<T> vmr = udMul(mr, v);

    if (!udEqualApprox(vqr, vmr, T(1.0 / 262144)))
      return false;
  }
  return true;
}

// This function is the foundation of all but one of the slerp unit tests. It works as
// follows, the rotation that represents rotating from q0 to q1 is found, we'll call this qi.
// We then extract the axis of rotation (A) and the angle (theta) from qi.  We then iterate
// from 0 to 1 in steps of 1/incrementCount using this for 't' in the slerp of q0 and q1. At
// the same time using the same 't' we linearly interpolate theta and build a rotation
// matrix from A.  This matrix is then concatenated with the matrix built from q0. This new
// matrix represents the same rotation as the result of the slerped quaternions, each of which
// is now applied to a vector and the results compared with an error tollerance.

template <typename T>
bool udQuaternion_SlerpAxisAngleUnitTest(udQuaternion<T> q0, udQuaternion<T> q1, udVector3<T> v, int incrementCount, const T epsilon)
{
  const T thetaEpsilon = T(UD_PI / (180.0 * 100.0)); // 1/100 of a degree
  const T tIncrementCount = T(incrementCount);

  udQuaternion<T> nlerpq1 = q1;

  T cosHalfTheta = udDotQ(q0, q1); // Dot product of 2 quaterions results in cos(theta/2)

  // udSlerp will use a normalized lerp if the absolute of the dot of the two quaternions is within 1/100 of a degree
  if ((T(1) - udAbs(cosHalfTheta)) < thetaEpsilon)
  {
    if (cosHalfTheta < T(0))
      nlerpq1 = -q1;

    for (int i = 0; i < incrementCount; ++i)
    {
      T inc = T(i) / tIncrementCount;
      udQuaternion<T> nl = udNormalize(udLerp(q0, nlerpq1, inc));
      udQuaternion<T> q = udSlerp(q0, q1, inc);

      udVector3<T> vnl = nl.apply(v);
      udVector3<T> vq = q.apply(v);

      if (!udEqualApprox(vnl, vq, epsilon))
        return false;
    }

    return true;
  }

  udMatrix4x4<T> m0 = udMatrix4x4<T>::rotationQuat(q0);

  if (!udEqualApprox(q0.apply(v), udMul(m0, v), epsilon))
    return false;

  T q0SinHalfTheta = udSqrt(T(1) - q0.w * q0.w);
  udVector3<T> q0Axis = { q0.x / q0SinHalfTheta, q0.y / q0SinHalfTheta, q0.z / q0SinHalfTheta };

  T q0Theta = udACos(q0.w) * T(2.0);

  udMatrix4x4<T> q0AxisM = udMatrix4x4<T>::rotationAxis(q0Axis, q0Theta);

  {
    udVector3<T> v0 = udMul(m0, v);
    udVector3<T> v1 = udMul(q0AxisM, v);
    if (!udEqualApprox(v0, v1, epsilon))
      return false;
  }

  T halfTheta = udACos(cosHalfTheta);

  // If the angle between q1 and q2 is too close to PI to slerp just return true as this is an invalid test
  if (udAbs(2.0 * halfTheta - T(UD_PI)) < thetaEpsilon)
    return true;

  // If the rotation is greater than PI (180°) neqate q1 to take the shortest path along the arc
  udQuaternion<T> q1c = cosHalfTheta < T(0) ? -q1 : q1;
  udQuaternion<T> qr = udMul(udConjugate(q0), q1c);

  {
    udVector3<T> vt0 = qr.apply(v);
    vt0 = q0.apply(vt0);

    udVector3<T> vt1 = q1.apply(v);
    if (!udEqualApprox(vt0, vt1, epsilon))
      return false;
  }

  T sinHalfTheta = udSqrt(T(1) - cosHalfTheta * cosHalfTheta);
  udVector3<T> axis = { qr.x / sinHalfTheta, qr.y / sinHalfTheta, qr.z / sinHalfTheta };

  T theta = udACos(qr.w) * T(2.0);

  for (int i = 0; i < incrementCount; ++i)
  {
    T inc = T(i) / tIncrementCount;
    udMatrix4x4<T> m = udMatrix4x4<T>::rotationAxis(axis, theta * inc);
    udQuaternion<T> q = udSlerp(q0, q1, inc);

    // Concatenate the interpolated rotation
    m = udMul(m0, m);

    udVector3<T> vm = udMul(m, v);
    udVector3<T> vq = q.apply(v);

    if (!udEqualApprox(vm, vq, epsilon))
      return false;
  }

  return true;
}

// This simple test checks clockwise, counter clockwise spherical interpolations.  The angle between the quaternions
// has been chosen such that half the tests must negate the second quaternion to ensure the shortest path is taken.
// This test overlaps with udQuaternion_SlerpRandomInputsTest(), however if a serious error occurs its likly to be
// caught in this test first and with its constant inputs it will be much easer to debug the problem.
template <typename T>
bool udQuaternion_SlerpDefinedInputsTest(int incrementCount)
{
  // The positive rotations in this test are counter clockwise about the Y axis
  // Note that rotating counter clockwise requires the axis of rotation to point out of the screen thus -Y.

  udVector3<T> v = { T(1.0), T(1.0), T(1.0) };
  udVector3<T> axis = { T(0.0), T(-1.0), T(0.0) };
  const T epsilon = sizeof(T) == 4 ? T(1.0 / 1024.0) : T(1.0 / 4096.0);

  T tPI = T(UD_PI);
  // Counter clockwise
  {
    T theta = tPI / T(8.0);
    udQuaternion<T> q0 = udQuaternion<T>::create(axis, theta);
    udQuaternion<T> q1 = udQuaternion<T>::create(axis, theta * T(2.0));

    if (!udQuaternion_SlerpAxisAngleUnitTest<T>(q0, q1, v, incrementCount, epsilon))
      return false;
  }

  // Clockwise
  {
    T theta = tPI / T(-8.0);
    udQuaternion<T> q0 = udQuaternion<T>::create(axis, theta);
    udQuaternion<T> q1 = udQuaternion<T>::create(axis, theta * T(2.0));

    if (!udQuaternion_SlerpAxisAngleUnitTest<T>(q0, q1, v, incrementCount, epsilon))
      return false;
  }

  // Clockwise because the shortest path is the opposite direction
  {
    T theta = tPI / T(8.0);
    udQuaternion<T> q0 = udQuaternion<T>::create(axis, theta);
    udQuaternion<T> q1 = udQuaternion<T>::create(axis, theta + tPI * T(1.25));

    if (!udQuaternion_SlerpAxisAngleUnitTest<T>(q0, q1, v, incrementCount, epsilon))
      return false;
  }

  // Counter clockwise because the shortest path is the opposite direction
  {
    T theta = tPI / T(-8.0);
    udQuaternion<T> q0 = udQuaternion<T>::create(axis, theta);
    udQuaternion<T> q1 = udQuaternion<T>::create(axis, theta + tPI * T(1.25));

    if (!udQuaternion_SlerpAxisAngleUnitTest<T>(q0, q1, v, incrementCount, epsilon))
      return false;
  }

  // Test when quaternions are equal or approximately equal
  {
    T theta = tPI / T(-8.0);
    const T thetaApprox = theta - T(UD_PI / (180.0 * 100.0) * 0.5); // 1/200 of a degree
    udQuaternion<T> q0 = udQuaternion<T>::create(axis, theta);
    udQuaternion<T> q1 = udQuaternion<T>::create(axis, thetaApprox);

    if (!udQuaternion_SlerpAxisAngleUnitTest<T>(q0, q0, v, incrementCount, epsilon))
      return false;

    if (!udQuaternion_SlerpAxisAngleUnitTest<T>(q0, q1, v, incrementCount, epsilon))
      return false;

    if (!udQuaternion_SlerpAxisAngleUnitTest<T>(q0, -q1, v, incrementCount, epsilon))
      return false;
  }


  return true;
}

template <typename T>
bool EqualApproxUnitTest()
{
  typedef typename T::ElementType ET;
  const ET epsilon = ET(0.01);
  T a = T::zero();
  T b = T::zero();
  T c = T::zero();

  const int count = T::ElementCount;

  for (int i = 0; i < count; ++i)
  {
    b[i] = ET(0.001) / ET(count);
    c[i] = ET(0.02) / ET(count);
  }

  if (!udEqualApprox(a, a, epsilon))
    return false;

  if (!udEqualApprox(a, b, epsilon))
    return false;

  if (udEqualApprox(a, c, epsilon))
    return false;

  return true;
}

template <typename T>
bool MatrixEqualApproxUnitTest()
{
  const T epsilon = T(0.01);
  udMatrix4x4<T> a;
  udMatrix4x4<T> b;
  udMatrix4x4<T> c;

  const int count = udMatrix4x4<T>::ElementCount;

  for (int i = 0; i < count; ++i)
  {
    a.a[i] = T(0);
    b.a[i] = T(0.001);
    c.a[i] = T(0.02);
  }

  if (!udMatrixEqualApprox(a, a, epsilon))
    return false;

  if (!udMatrixEqualApprox(a, b, epsilon))
    return false;

  if (udMatrixEqualApprox(a, c, epsilon))
    return false;

  return true;
}

template <typename T>
bool IsUnitLengthUnitTest()
{
  typedef typename T::ElementType ET;
  const ET epsilon = ET(0.01);

  T unitLength = T::zero();
  unitLength.x = ET(1);
  if (!udIsUnitLength(unitLength, epsilon))
    return false;

  T v = T::zero();
  v.x = (ET)(0.999);
  if (!udIsUnitLength(v, epsilon))
    return false;

  v.x = (ET)(1.011);
  if (udIsUnitLength(v, epsilon))
    return false;

  return true;
}

// FAIL Test unit length asserts
template <typename T>
void udQuaternion_SlerpAssertUnitLengthQ0()
{
  udVector3<T> axis = { T(0.0), T(2.0), T(0.0) };
  udQuaternion<T> qi = udQuaternion<T>::identity();
  udQuaternion<T> q0 = udQuaternion<T>::create(axis, T(0.1));
  udSlerp(q0, qi, T(0.5)); // This should assert
}

template <typename T>
void udQuaternion_SlerpAssertUnitLengthQ1()
{
  udVector3<T> axis = { T(0.0), T(2.0), T(0.0) };
  udQuaternion<T> qi = udQuaternion<T>::identity();
  udQuaternion<T> q0 = udQuaternion<T>::create(axis, T(UD_HALF_PI));
  udSlerp(qi, q0, T(0.5)); // This should assert
}

// FAIL Test assert for slerping quaternions separated by an angle of PI (180°)
template <typename T>
void udQuaternion_SlerpAssertPITheta()
{
  udVector3<T> axis = { T(0.0), T(1.0), T(0.0) };
  udQuaternion<T> q0 = udQuaternion<T>::create(axis, T(UD_PI / 4.0));
  udQuaternion<T> q1 = udQuaternion<T>::create(axis, T(5.0 * UD_PI / 4.0));
  udSlerp(q0, q1, T(0.5)); // This should assert
}

template <typename T>
void udQuaternion_SlerpAlmostPITheta()
{
  udVector3<T> axis = { T(0.0), T(1.0), T(0.0) };
  udQuaternion<T> q0 = udQuaternion<T>::create(axis, T(UD_PI / 4.0));
  udQuaternion<T> q1 = udQuaternion<T>::create(axis, T(5.0 * UD_PI / 4.0) - T(2.0 * UD_PI / (180.0 * 100.0))); // 2/100 of a degree

  udSlerp(q0, q1, T(0.5)); // This should not assert
}

// This test was added as it was a bug case in udCDKEditor.
// When the quaternions are close together (falls back to nlerp) and also cross an axis the quat flips direction (but otherwise takes the 'shortest' path)
template <typename T>
bool udQuaternion_Slerp2DegOverZero()
{
  udQuaternion<T> q0 = udQuaternion<T>::create((T)UD_DEG2RAD(1), 0, 0);
  udQuaternion<T> q1 = udQuaternion<T>::create((T)UD_DEG2RAD(359), 0, 0);

  udVector3<T> testVector = udVector3<T>::create(0, 1, 0);

  udVector3<T> vExpected = (udQuaternion<T>::create(0, 0, 0)).apply(testVector);
  udVector3<T> vResult = udSlerp(q0, q1, T(0.5)).apply(testVector);

  return udEqualApprox(vResult, vExpected);
}

TEST(MathTests, MathCPPSuite)
{
  udFloat4 x = udFloat4::create(0, 1, 2, 3);
  udFloat4x4 m = udFloat4x4::create(x, x, x, x);

  udFloat3 v = udFloat3::create(4, 5, 6);
  udFloat4 v4 = udFloat4::create(7, 8, 9, 10);

  float f = udDot3(v, v);
  udPow(f, (float)UD_PI);

  v = 2.f*v;
  v = v*v + v;

  m.transpose();
  m.determinant();
  m.inverse();

  udCross3(v, v4.toVector3());

  v += v;
  v /= 2.f;

  v *= v.one();

  udMul(m, 1.f);
  udMul(m, v);
  udMul(m, v4);
  udMul(m, m);

  EXPECT_EQ(1.f, udAbs(-1.f));
  EXPECT_EQ(1.0, udAbs(-1.0));
  EXPECT_EQ(1, udAbs(-1));
  EXPECT_EQ(udFloat3::create(udAbs(v.x), udAbs(v.y), udAbs(v.z)), udAbs(v));
}

TEST(MathTests, UnitLengthCanaries)
{
  EXPECT_TRUE(EqualApproxUnitTest<udVector2<float> >());
  EXPECT_TRUE(EqualApproxUnitTest<udVector2<double> >());
  EXPECT_TRUE(EqualApproxUnitTest<udVector3<float> >());
  EXPECT_TRUE(EqualApproxUnitTest<udVector3<double> >());
  EXPECT_TRUE(EqualApproxUnitTest<udVector4<float> >());
  EXPECT_TRUE(EqualApproxUnitTest<udVector4<double> >());
  EXPECT_TRUE(EqualApproxUnitTest<udQuaternion<float> >());
  EXPECT_TRUE(EqualApproxUnitTest<udQuaternion<double> >());

  EXPECT_TRUE(MatrixEqualApproxUnitTest<float>());
  EXPECT_TRUE(MatrixEqualApproxUnitTest<double>());

  EXPECT_TRUE(IsUnitLengthUnitTest<udVector2<float> >());
  EXPECT_TRUE(IsUnitLengthUnitTest<udVector2<double> >());
  EXPECT_TRUE(IsUnitLengthUnitTest<udVector3<float> >());
  EXPECT_TRUE(IsUnitLengthUnitTest<udVector3<double> >());
  EXPECT_TRUE(IsUnitLengthUnitTest<udVector4<float> >());
  EXPECT_TRUE(IsUnitLengthUnitTest<udVector4<double> >());
  EXPECT_TRUE(IsUnitLengthUnitTest<udQuaternion<float> >());
  EXPECT_TRUE(IsUnitLengthUnitTest<udQuaternion<double> >());
}

TEST(MathTests, QuaternionCanaries)
{
  EXPECT_TRUE(udQuaternion_SlerpBasicUnitTest<double>());
  EXPECT_TRUE(udQuaternion_SlerpBasicUnitTest<float>());
  EXPECT_TRUE(udQuaternion_Slerp2DegOverZero<float>());

  int incrementCount = 256;
  EXPECT_TRUE(udQuaternion_SlerpDefinedInputsTest<double>(incrementCount));
  EXPECT_TRUE(udQuaternion_SlerpDefinedInputsTest<float>(incrementCount));
}

TEST(MathTests, BasicMathFunctions)
{
  // Floats
  EXPECT_FLOAT_EQ(4.f, udPow(2.f, 2.f));
  EXPECT_FLOAT_EQ(2.30258512f, udLogN(10.f));
  EXPECT_FLOAT_EQ(2.f, udLog2(4.f));
  EXPECT_FLOAT_EQ(1.f, udLog10(10.f));
  EXPECT_FLOAT_EQ(0.25f, udRSqrt(16.f));
  EXPECT_FLOAT_EQ(2.f, udSqrt(4.f));
  EXPECT_FLOAT_EQ(1.f, udSin(UD_PIf / 2.f));
  EXPECT_FLOAT_EQ(-1.f, udCos(UD_PIf));
  EXPECT_FLOAT_EQ(1.f, udTan(UD_PIf / 4.f));
  EXPECT_FLOAT_EQ(1.1752011936438f, udSinh(1.f));
  EXPECT_FLOAT_EQ(1.5430806348152f, udCosh(1.f));
  EXPECT_FLOAT_EQ(0.7615941559557f, udTanh(1.f));
  EXPECT_FLOAT_EQ(1.57079637f, udASin(1.f));
  EXPECT_FLOAT_EQ(0.f, udACos(1.f));
  EXPECT_FLOAT_EQ(0.785398185f, udATan(1.f));
  EXPECT_FLOAT_EQ(0.321750551f, udATan2(0.25f, 0.75f));
  EXPECT_FLOAT_EQ(1.f, udASinh(1.1752011936438f));
  EXPECT_FLOAT_EQ(1.f, udACosh(1.5430806348152f));
  EXPECT_FLOAT_EQ(1.f, udATanh(0.7615941559557f));

  EXPECT_FLOAT_EQ(UD_PIf / 2.f, udNormaliseRotation(UD_PIf / 2.f, UD_PIf));
  EXPECT_FLOAT_EQ(0.f, udNormaliseRotation(UD_PIf * 2.f, UD_PIf));

  // Doubles
  EXPECT_DOUBLE_EQ(4.0, udPow(2.0, 2.0));
  EXPECT_DOUBLE_EQ(2.3025850929940459, udLogN(10.0));
  EXPECT_DOUBLE_EQ(2.0, udLog2(4.0));
  EXPECT_DOUBLE_EQ(1.0, udLog10(10.0));
  EXPECT_DOUBLE_EQ(0.25, udRSqrt(16.0));
  EXPECT_DOUBLE_EQ(2.0, udSqrt(4.0));
  EXPECT_DOUBLE_EQ(1.0, udSin(UD_PI / 2.0));
  EXPECT_DOUBLE_EQ(-1.0, udCos(UD_PI));
  EXPECT_DOUBLE_EQ(1.0, udTan(UD_PI / 4.0));
  EXPECT_DOUBLE_EQ(1.1752011936438014, udSinh(1.0));
  EXPECT_DOUBLE_EQ(1.5430806348152437, udCosh(1.0));
  EXPECT_DOUBLE_EQ(0.7615941559557648, udTanh(1.0));
  EXPECT_DOUBLE_EQ(1.5707963267948966, udASin(1.0));
  EXPECT_DOUBLE_EQ(0.0, udACos(1.0));
  EXPECT_DOUBLE_EQ(0.78539816339744828, udATan(1.0));
  EXPECT_DOUBLE_EQ(0.32175055439664219, udATan2(0.25, 0.75));
  EXPECT_DOUBLE_EQ(1.0, udASinh(1.1752011936438014));
  EXPECT_DOUBLE_EQ(1.0, udACosh(1.5430806348152437));
  EXPECT_DOUBLE_EQ(1.0, udATanh(0.7615941559557648));
  EXPECT_DOUBLE_EQ(UD_PI / 2.0, udNormaliseRotation(UD_PI / 2.0, UD_PI));
  EXPECT_DOUBLE_EQ(0.0, udNormaliseRotation(UD_PI * 2.0, UD_PI));
}

TEST(MathTests, RoundingFunctions)
{
  // Floats
  EXPECT_FLOAT_EQ(5.f, udRound(4.5f));
  EXPECT_FLOAT_EQ(5.f, udRound(4.6f));
  EXPECT_FLOAT_EQ(5.f, udRound(5.4f));
  EXPECT_FLOAT_EQ(-5.f, udRound(-4.5f));
  EXPECT_FLOAT_EQ(-5.f, udRound(-4.6f));
  EXPECT_FLOAT_EQ(-5.f, udRound(-5.4f));
  EXPECT_FLOAT_EQ(5.f, udFloor(5.9f));
  EXPECT_FLOAT_EQ(-6.f, udFloor(-5.9f));
  EXPECT_FLOAT_EQ(5.f, udCeil(4.01f));
  EXPECT_FLOAT_EQ(-4.f, udCeil(-4.01f));
  EXPECT_FLOAT_EQ(5.f, udMod(35.f, 6.f));
  EXPECT_FLOAT_EQ(4.f, udRoundEven(4.5f));
  EXPECT_FLOAT_EQ(4.f, udRoundEven(3.5f));
  EXPECT_FLOAT_EQ(4.f, udRoundEven(4.1f));
  EXPECT_FLOAT_EQ(-4.f, udRoundEven(-4.5f));
  EXPECT_FLOAT_EQ(-4.f, udRoundEven(-3.5f));

  // Doubles
  EXPECT_DOUBLE_EQ(5.0, udRound(4.5));
  EXPECT_DOUBLE_EQ(5.0, udRound(4.6));
  EXPECT_DOUBLE_EQ(5.0, udRound(5.4));
  EXPECT_DOUBLE_EQ(-5.0, udRound(-4.5));
  EXPECT_DOUBLE_EQ(-5.0, udRound(-4.6));
  EXPECT_DOUBLE_EQ(-5.0, udRound(-5.4));
  EXPECT_DOUBLE_EQ(5.0, udFloor(5.9));
  EXPECT_DOUBLE_EQ(-6.0, udFloor(-5.9));
  EXPECT_DOUBLE_EQ(5.0, udCeil(4.01));
  EXPECT_DOUBLE_EQ(-4.0, udCeil(-4.01));
  EXPECT_DOUBLE_EQ(5.0, udMod(35.0, 6.0));
  EXPECT_DOUBLE_EQ(4.0, udRoundEven(4.5));
  EXPECT_DOUBLE_EQ(4.0, udRoundEven(3.5));
  EXPECT_DOUBLE_EQ(4.0, udRoundEven(4.1));
  EXPECT_DOUBLE_EQ(-4.0, udRoundEven(-4.5));
  EXPECT_DOUBLE_EQ(-4.0, udRoundEven(-3.5));

  // Signed Integers
  EXPECT_EQ(4, udPowerOfTwoAbove(3));
  EXPECT_EQ(8, udPowerOfTwoAbove(5));
  EXPECT_TRUE(udIsPowerOfTwo(4));
  EXPECT_EQ(16, udHighestBitValue(18));

  // Unsigned Integers
  EXPECT_EQ(uint32_t(4), udPowerOfTwoAbove(uint32_t(3)));
  EXPECT_EQ(uint32_t(8), udPowerOfTwoAbove(uint32_t(5)));
  EXPECT_TRUE(udIsPowerOfTwo(uint32_t(4)));
  EXPECT_EQ(uint32_t(16), udHighestBitValue(uint32_t(18)));
}

TEST(MathTests, LinearAlgebraFunctions_Abs)
{
  // Floats
  EXPECT_EQ(4.f, udAbs(-4.f));
  EXPECT_EQ(4.f, udAbs(4.f));
  EXPECT_EQ(udFloat2::create(4.f), udAbs(udFloat2::create(-4.f)));
  EXPECT_EQ(udFloat2::create(4.f), udAbs(udFloat2::create(4.f)));
  EXPECT_EQ(udFloat3::create(4.f), udAbs(udFloat3::create(-4.f)));
  EXPECT_EQ(udFloat3::create(4.f), udAbs(udFloat3::create(4.f)));
  EXPECT_EQ(udFloat4::create(4.f), udAbs(udFloat4::create(-4.f)));
  EXPECT_EQ(udFloat4::create(4.f), udAbs(udFloat4::create(4.f)));
  {
    udFloatQuat quat = udFloatQuat::identity();
    quat.x = -4.f;
    quat.y = -4.f;
    quat.z = -4.f;
    quat.w = -4.f;
    EXPECT_EQ(udFloat4::create(4.f), udAbs(quat).toVector4());
  }
  {
    udFloatQuat quat = udFloatQuat::identity();
    quat.x = 4.f;
    quat.y = 4.f;
    quat.z = 4.f;
    quat.w = 4.f;
    EXPECT_EQ(udFloat4::create(4.f), udAbs(quat).toVector4());
  }

  // Doubles
  EXPECT_EQ(4.0, udAbs(-4.0));
  EXPECT_EQ(4.0, udAbs(4.0));
  EXPECT_EQ(udDouble2::create(4.0), udAbs(udDouble2::create(-4.0)));
  EXPECT_EQ(udDouble2::create(4.0), udAbs(udDouble2::create(4.0)));
  EXPECT_EQ(udDouble3::create(4.0), udAbs(udDouble3::create(-4.0)));
  EXPECT_EQ(udDouble3::create(4.0), udAbs(udDouble3::create(4.0)));
  EXPECT_EQ(udDouble4::create(4.0), udAbs(udDouble4::create(-4.0)));
  EXPECT_EQ(udDouble4::create(4.0), udAbs(udDouble4::create(4.0)));
  {
    udDoubleQuat quat = udDoubleQuat::identity();
    quat.x = -4.0;
    quat.y = -4.0;
    quat.z = -4.0;
    quat.w = -4.0;
    EXPECT_EQ(udDouble4::create(4.0), udAbs(quat).toVector4());
  }
  {
    udDoubleQuat quat = udDoubleQuat::identity();
    quat.x = 4.0;
    quat.y = 4.0;
    quat.z = 4.0;
    quat.w = 4.0;
    EXPECT_EQ(udDouble4::create(4.0), udAbs(quat).toVector4());
  }
}

TEST(MathTests, LinearAlgebraFunctions_MinMax)
{
  // Floats
  EXPECT_EQ(4.f, udMin(4.f, 8.f));
  EXPECT_EQ(4.f, udMin(8.f, 4.f));
  EXPECT_EQ(8.f, udMax(4.f, 8.f));
  EXPECT_EQ(8.f, udMax(8.f, 4.f));
  EXPECT_EQ(udFloat2::create(4.f), udMin(udFloat2::create(4.f), udFloat2::create(8.f)));
  EXPECT_EQ(udFloat2::create(4.f), udMin(udFloat2::create(8.f), udFloat2::create(4.f)));
  EXPECT_EQ(udFloat2::create(8.f), udMax(udFloat2::create(4.f), udFloat2::create(8.f)));
  EXPECT_EQ(udFloat2::create(8.f), udMax(udFloat2::create(8.f), udFloat2::create(4.f)));
  EXPECT_EQ(udFloat3::create(4.f), udMin(udFloat3::create(4.f), udFloat3::create(8.f)));
  EXPECT_EQ(udFloat3::create(4.f), udMin(udFloat3::create(8.f), udFloat3::create(4.f)));
  EXPECT_EQ(udFloat3::create(8.f), udMax(udFloat3::create(4.f), udFloat3::create(8.f)));
  EXPECT_EQ(udFloat3::create(8.f), udMax(udFloat3::create(8.f), udFloat3::create(4.f)));
  EXPECT_EQ(udFloat4::create(4.f), udMin(udFloat4::create(4.f), udFloat4::create(8.f)));
  EXPECT_EQ(udFloat4::create(4.f), udMin(udFloat4::create(8.f), udFloat4::create(4.f)));
  EXPECT_EQ(udFloat4::create(8.f), udMax(udFloat4::create(4.f), udFloat4::create(8.f)));
  EXPECT_EQ(udFloat4::create(8.f), udMax(udFloat4::create(8.f), udFloat4::create(4.f)));
  EXPECT_EQ(1.f, udMinElement(udFloat4::create(1.f, 2.f, 4.f, 8.f)));
  EXPECT_EQ(-8.f, udMinElement(udFloat4::create(1.f, 2.f, 4.f, -8.f)));
  EXPECT_EQ(8.f, udMaxElement(udFloat4::create(1.f, 2.f, 4.f, 8.f)));
  EXPECT_EQ(4.f, udMaxElement(udFloat4::create(1.f, 2.f, 4.f, -8.f)));

  // Doubles
  EXPECT_EQ(4.0, udMin(4.0, 8.0));
  EXPECT_EQ(4.0, udMin(8.0, 4.0));
  EXPECT_EQ(8.0, udMax(4.0, 8.0));
  EXPECT_EQ(8.0, udMax(8.0, 4.0));
  EXPECT_EQ(udDouble2::create(4.0), udMin(udDouble2::create(4.0), udDouble2::create(8.0)));
  EXPECT_EQ(udDouble2::create(4.0), udMin(udDouble2::create(8.0), udDouble2::create(4.0)));
  EXPECT_EQ(udDouble2::create(8.0), udMax(udDouble2::create(4.0), udDouble2::create(8.0)));
  EXPECT_EQ(udDouble2::create(8.0), udMax(udDouble2::create(8.0), udDouble2::create(4.0)));
  EXPECT_EQ(udDouble3::create(4.0), udMin(udDouble3::create(4.0), udDouble3::create(8.0)));
  EXPECT_EQ(udDouble3::create(4.0), udMin(udDouble3::create(8.0), udDouble3::create(4.0)));
  EXPECT_EQ(udDouble3::create(8.0), udMax(udDouble3::create(4.0), udDouble3::create(8.0)));
  EXPECT_EQ(udDouble3::create(8.0), udMax(udDouble3::create(8.0), udDouble3::create(4.0)));
  EXPECT_EQ(udDouble4::create(4.0), udMin(udDouble4::create(4.0), udDouble4::create(8.0)));
  EXPECT_EQ(udDouble4::create(4.0), udMin(udDouble4::create(8.0), udDouble4::create(4.0)));
  EXPECT_EQ(udDouble4::create(8.0), udMax(udDouble4::create(4.0), udDouble4::create(8.0)));
  EXPECT_EQ(udDouble4::create(8.0), udMax(udDouble4::create(8.0), udDouble4::create(4.0)));
  EXPECT_EQ(1.0, udMinElement(udDouble4::create(1.f, 2.f, 4.f, 8.f)));
  EXPECT_EQ(-8.0, udMinElement(udDouble4::create(1.f, 2.f, 4.f, -8.f)));
  EXPECT_EQ(8.0, udMaxElement(udDouble4::create(1.f, 2.f, 4.f, 8.f)));
  EXPECT_EQ(4.0, udMaxElement(udDouble4::create(1.f, 2.f, 4.f, -8.f)));

  // Signed Integers
  EXPECT_EQ(4, udMin(4, 8));
  EXPECT_EQ(4, udMin(8, 4));
  EXPECT_EQ(8, udMax(4, 8));
  EXPECT_EQ(8, udMax(8, 4));
  EXPECT_EQ(udInt2::create(4), udMin(udInt2::create(4), udInt2::create(8)));
  EXPECT_EQ(udInt2::create(4), udMin(udInt2::create(8), udInt2::create(4)));
  EXPECT_EQ(udInt2::create(8), udMax(udInt2::create(4), udInt2::create(8)));
  EXPECT_EQ(udInt2::create(8), udMax(udInt2::create(8), udInt2::create(4)));
  EXPECT_EQ(udInt3::create(4), udMin(udInt3::create(4), udInt3::create(8)));
  EXPECT_EQ(udInt3::create(4), udMin(udInt3::create(8), udInt3::create(4)));
  EXPECT_EQ(udInt3::create(8), udMax(udInt3::create(4), udInt3::create(8)));
  EXPECT_EQ(udInt3::create(8), udMax(udInt3::create(8), udInt3::create(4)));
  EXPECT_EQ(udInt4::create(4), udMin(udInt4::create(4), udInt4::create(8)));
  EXPECT_EQ(udInt4::create(4), udMin(udInt4::create(8), udInt4::create(4)));
  EXPECT_EQ(udInt4::create(8), udMax(udInt4::create(4), udInt4::create(8)));
  EXPECT_EQ(udInt4::create(8), udMax(udInt4::create(8), udInt4::create(4)));

  // Unsigned Integers
  EXPECT_EQ(uint32_t(4), udMin(uint32_t(4), uint32_t(8)));
  EXPECT_EQ(uint32_t(4), udMin(uint32_t(8), uint32_t(4)));
  EXPECT_EQ(uint32_t(8), udMax(uint32_t(4), uint32_t(8)));
  EXPECT_EQ(uint32_t(8), udMax(uint32_t(8), uint32_t(4)));
  EXPECT_EQ(udUInt2::create(4), udMin(udUInt2::create(4), udUInt2::create(8)));
  EXPECT_EQ(udUInt2::create(4), udMin(udUInt2::create(8), udUInt2::create(4)));
  EXPECT_EQ(udUInt2::create(8), udMax(udUInt2::create(4), udUInt2::create(8)));
  EXPECT_EQ(udUInt2::create(8), udMax(udUInt2::create(8), udUInt2::create(4)));
  EXPECT_EQ(udUInt3::create(4), udMin(udUInt3::create(4), udUInt3::create(8)));
  EXPECT_EQ(udUInt3::create(4), udMin(udUInt3::create(8), udUInt3::create(4)));
  EXPECT_EQ(udUInt3::create(8), udMax(udUInt3::create(4), udUInt3::create(8)));
  EXPECT_EQ(udUInt3::create(8), udMax(udUInt3::create(8), udUInt3::create(4)));
  EXPECT_EQ(udUInt4::create(4), udMin(udUInt4::create(4), udUInt4::create(8)));
  EXPECT_EQ(udUInt4::create(4), udMin(udUInt4::create(8), udUInt4::create(4)));
  EXPECT_EQ(udUInt4::create(8), udMax(udUInt4::create(4), udUInt4::create(8)));
  EXPECT_EQ(udUInt4::create(8), udMax(udUInt4::create(8), udUInt4::create(4)));
}

TEST(MathTests, LinearAlgebraFunctions_Clamp)
{
  // Floats
  EXPECT_EQ(4.f, udClamp(3.f, 4.f, 8.f));
  EXPECT_EQ(8.f, udClamp(9.f, 4.f, 8.f));
  EXPECT_EQ(6.f, udClamp(6.f, 4.f, 8.f));
  EXPECT_EQ(udFloat2::create(4.f), udClamp(udFloat2::create(3.f), udFloat2::create(4.f), udFloat2::create(8.f)));
  EXPECT_EQ(udFloat2::create(8.f), udClamp(udFloat2::create(9.f), udFloat2::create(4.f), udFloat2::create(8.f)));
  EXPECT_EQ(udFloat2::create(6.f), udClamp(udFloat2::create(6.f), udFloat2::create(4.f), udFloat2::create(8.f)));
  EXPECT_EQ(udFloat3::create(4.f), udClamp(udFloat3::create(3.f), udFloat3::create(4.f), udFloat3::create(8.f)));
  EXPECT_EQ(udFloat3::create(8.f), udClamp(udFloat3::create(9.f), udFloat3::create(4.f), udFloat3::create(8.f)));
  EXPECT_EQ(udFloat3::create(6.f), udClamp(udFloat3::create(6.f), udFloat3::create(4.f), udFloat3::create(8.f)));
  EXPECT_EQ(udFloat4::create(4.f), udClamp(udFloat4::create(3.f), udFloat4::create(4.f), udFloat4::create(8.f)));
  EXPECT_EQ(udFloat4::create(8.f), udClamp(udFloat4::create(9.f), udFloat4::create(4.f), udFloat4::create(8.f)));
  EXPECT_EQ(udFloat4::create(6.f), udClamp(udFloat4::create(6.f), udFloat4::create(4.f), udFloat4::create(8.f)));

  // Doubles
  EXPECT_EQ(4.0, udClamp(3.0, 4.0, 8.0));
  EXPECT_EQ(8.0, udClamp(9.0, 4.0, 8.0));
  EXPECT_EQ(6.0, udClamp(6.0, 4.0, 8.0));
  EXPECT_EQ(udDouble2::create(4.0), udClamp(udDouble2::create(3.0), udDouble2::create(4.0), udDouble2::create(8.0)));
  EXPECT_EQ(udDouble2::create(8.0), udClamp(udDouble2::create(9.0), udDouble2::create(4.0), udDouble2::create(8.0)));
  EXPECT_EQ(udDouble2::create(6.0), udClamp(udDouble2::create(6.0), udDouble2::create(4.0), udDouble2::create(8.0)));
  EXPECT_EQ(udDouble3::create(4.0), udClamp(udDouble3::create(3.0), udDouble3::create(4.0), udDouble3::create(8.0)));
  EXPECT_EQ(udDouble3::create(8.0), udClamp(udDouble3::create(9.0), udDouble3::create(4.0), udDouble3::create(8.0)));
  EXPECT_EQ(udDouble3::create(6.0), udClamp(udDouble3::create(6.0), udDouble3::create(4.0), udDouble3::create(8.0)));
  EXPECT_EQ(udDouble4::create(4.0), udClamp(udDouble4::create(3.0), udDouble4::create(4.0), udDouble4::create(8.0)));
  EXPECT_EQ(udDouble4::create(8.0), udClamp(udDouble4::create(9.0), udDouble4::create(4.0), udDouble4::create(8.0)));
  EXPECT_EQ(udDouble4::create(6.0), udClamp(udDouble4::create(6.0), udDouble4::create(4.0), udDouble4::create(8.0)));

  // Signed Integers
  EXPECT_EQ(4, udClamp(3, 4, 8));
  EXPECT_EQ(8, udClamp(9, 4, 8));
  EXPECT_EQ(6, udClamp(6, 4, 8));
  EXPECT_EQ(udInt2::create(4), udClamp(udInt2::create(3), udInt2::create(4), udInt2::create(8)));
  EXPECT_EQ(udInt2::create(8), udClamp(udInt2::create(9), udInt2::create(4), udInt2::create(8)));
  EXPECT_EQ(udInt2::create(6), udClamp(udInt2::create(6), udInt2::create(4), udInt2::create(8)));
  EXPECT_EQ(udInt3::create(4), udClamp(udInt3::create(3), udInt3::create(4), udInt3::create(8)));
  EXPECT_EQ(udInt3::create(8), udClamp(udInt3::create(9), udInt3::create(4), udInt3::create(8)));
  EXPECT_EQ(udInt3::create(6), udClamp(udInt3::create(6), udInt3::create(4), udInt3::create(8)));
  EXPECT_EQ(udInt4::create(4), udClamp(udInt4::create(3), udInt4::create(4), udInt4::create(8)));
  EXPECT_EQ(udInt4::create(8), udClamp(udInt4::create(9), udInt4::create(4), udInt4::create(8)));
  EXPECT_EQ(udInt4::create(6), udClamp(udInt4::create(6), udInt4::create(4), udInt4::create(8)));

  // Unsigned Integers
  EXPECT_EQ(uint32_t(4), udClamp(uint32_t(3), uint32_t(4), uint32_t(8)));
  EXPECT_EQ(uint32_t(8), udClamp(uint32_t(9), uint32_t(4), uint32_t(8)));
  EXPECT_EQ(uint32_t(6), udClamp(uint32_t(6), uint32_t(4), uint32_t(8)));
  EXPECT_EQ(udUInt2::create(4), udClamp(udUInt2::create(3), udUInt2::create(4), udUInt2::create(8)));
  EXPECT_EQ(udUInt2::create(8), udClamp(udUInt2::create(9), udUInt2::create(4), udUInt2::create(8)));
  EXPECT_EQ(udUInt2::create(6), udClamp(udUInt2::create(6), udUInt2::create(4), udUInt2::create(8)));
  EXPECT_EQ(udUInt3::create(4), udClamp(udUInt3::create(3), udUInt3::create(4), udUInt3::create(8)));
  EXPECT_EQ(udUInt3::create(8), udClamp(udUInt3::create(9), udUInt3::create(4), udUInt3::create(8)));
  EXPECT_EQ(udUInt3::create(6), udClamp(udUInt3::create(6), udUInt3::create(4), udUInt3::create(8)));
  EXPECT_EQ(udUInt4::create(4), udClamp(udUInt4::create(3), udUInt4::create(4), udUInt4::create(8)));
  EXPECT_EQ(udUInt4::create(8), udClamp(udUInt4::create(9), udUInt4::create(4), udUInt4::create(8)));
  EXPECT_EQ(udUInt4::create(6), udClamp(udUInt4::create(6), udUInt4::create(4), udUInt4::create(8)));
}

TEST(MathTests, LinearAlgebraFunctions_Saturate)
{
  // Floats
  EXPECT_EQ(1.f, udSaturate(2.f));
  EXPECT_EQ(0.f, udSaturate(-2.f));
  EXPECT_EQ(0.5f, udSaturate(0.5f));
  EXPECT_EQ(udFloat2::create(1.f), udSaturate(udFloat2::create(2.f)));
  EXPECT_EQ(udFloat2::create(0.f), udSaturate(udFloat2::create(-2.f)));
  EXPECT_EQ(udFloat2::create(0.5f), udSaturate(udFloat2::create(0.5f)));
  EXPECT_EQ(udFloat3::create(1.f), udSaturate(udFloat3::create(2.f)));
  EXPECT_EQ(udFloat3::create(0.f), udSaturate(udFloat3::create(-2.f)));
  EXPECT_EQ(udFloat3::create(0.5f), udSaturate(udFloat3::create(0.5f)));
  EXPECT_EQ(udFloat4::create(1.f), udSaturate(udFloat4::create(2.f)));
  EXPECT_EQ(udFloat4::create(0.f), udSaturate(udFloat4::create(-2.f)));
  EXPECT_EQ(udFloat4::create(0.5f), udSaturate(udFloat4::create(0.5f)));

  // Doubles
  EXPECT_EQ(1.0, udSaturate(2.0));
  EXPECT_EQ(0.0, udSaturate(-2.0));
  EXPECT_EQ(0.5, udSaturate(0.5));
  EXPECT_EQ(udDouble2::create(1.0), udSaturate(udDouble2::create(2.0)));
  EXPECT_EQ(udDouble2::create(0.0), udSaturate(udDouble2::create(-2.0)));
  EXPECT_EQ(udDouble2::create(0.5), udSaturate(udDouble2::create(0.5)));
  EXPECT_EQ(udDouble3::create(1.0), udSaturate(udDouble3::create(2.0)));
  EXPECT_EQ(udDouble3::create(0.0), udSaturate(udDouble3::create(-2.0)));
  EXPECT_EQ(udDouble3::create(0.5), udSaturate(udDouble3::create(0.5)));
  EXPECT_EQ(udDouble4::create(1.0), udSaturate(udDouble4::create(2.0)));
  EXPECT_EQ(udDouble4::create(0.0), udSaturate(udDouble4::create(-2.0)));
  EXPECT_EQ(udDouble4::create(0.5), udSaturate(udDouble4::create(0.5)));

  // Signed Integers
  EXPECT_EQ(1, udSaturate(2));
  EXPECT_EQ(0, udSaturate(-2));
  EXPECT_EQ(udInt2::create(1), udSaturate(udInt2::create(2)));
  EXPECT_EQ(udInt2::create(0), udSaturate(udInt2::create(-2)));
  EXPECT_EQ(udInt3::create(1), udSaturate(udInt3::create(2)));
  EXPECT_EQ(udInt3::create(0), udSaturate(udInt3::create(-2)));
  EXPECT_EQ(udInt4::create(1), udSaturate(udInt4::create(2)));
  EXPECT_EQ(udInt4::create(0), udSaturate(udInt4::create(-2)));

  // Unsigned Integers
  EXPECT_EQ(uint32_t(1), udSaturate(uint32_t(2)));
  EXPECT_EQ(udUInt2::create(1), udSaturate(udUInt2::create(2)));
  EXPECT_EQ(udUInt3::create(1), udSaturate(udUInt3::create(2)));
  EXPECT_EQ(udUInt4::create(1), udSaturate(udUInt4::create(2)));
}

TEST(MathTests, LinearAlgebraFunctions_IsUnitLength)
{
  // Floats
  EXPECT_TRUE(udIsUnitLength(udFloat2::create(1.f, 0.f), UD_EPSILON));
  EXPECT_TRUE(udIsUnitLength(udFloat2::create(0.70710678f, 0.70710678f), UD_EPSILON));
  EXPECT_FALSE(udIsUnitLength(udFloat2::create(0.f, 0.f), UD_EPSILON));
  EXPECT_TRUE(udIsUnitLength(udFloat3::create(1.f, 0.f, 0.f), UD_EPSILON));
  EXPECT_TRUE(udIsUnitLength(udFloat3::create(0.5773214f, 0.5773214f, 0.5773214f), UD_EPSILON));
  EXPECT_FALSE(udIsUnitLength(udFloat3::create(0.f, 0.f, 0.f), UD_EPSILON));
  EXPECT_TRUE(udIsUnitLength(udFloat4::create(1.f, 0.f, 0.f, 0.f), UD_EPSILON));
  EXPECT_TRUE(udIsUnitLength(udFloat4::create(0.5f, 0.5f, 0.5f, 0.5f), UD_EPSILON));
  EXPECT_FALSE(udIsUnitLength(udFloat4::create(0.f, 0.f, 0.f, 0.f), UD_EPSILON));

  // Doubles
  EXPECT_TRUE(udIsUnitLength(udDouble2::create(1.0, 0.0), UD_EPSILON));
  EXPECT_TRUE(udIsUnitLength(udDouble2::create(0.70710678, 0.70710678), UD_EPSILON));
  EXPECT_FALSE(udIsUnitLength(udDouble2::create(0.0, 0.0), UD_EPSILON));
  EXPECT_TRUE(udIsUnitLength(udDouble3::create(1.0, 0.0, 0.0), UD_EPSILON));
  EXPECT_TRUE(udIsUnitLength(udDouble3::create(0.5773214, 0.5773214, 0.5773214), UD_EPSILON));
  EXPECT_FALSE(udIsUnitLength(udDouble3::create(0.0, 0.0, 0.0), UD_EPSILON));
  EXPECT_TRUE(udIsUnitLength(udDouble4::create(1.0, 0.0, 0.0, 0.0), UD_EPSILON));
  EXPECT_TRUE(udIsUnitLength(udDouble4::create(0.5, 0.5, 0.5, 0.5), UD_EPSILON));
  EXPECT_FALSE(udIsUnitLength(udDouble4::create(0.0, 0.0, 0.0, 0.0), UD_EPSILON));
}

TEST(MathTests, BasicVectorFunctions)
{
  // udVector2<>
  {
    udDouble2 other = udDouble2::one();
    udFloat2 a = udFloat2::zero();
    udFloat2 b = udFloat2::one();
    udFloat2 c = udFloat2::create(1.f, 0.f);
    const udFloat2 constC = udFloat2::create(1.f, 0.f);
    udFloat2 d = udFloat2::create(2.f);
    udFloat2 e = udFloat2::create(d);
    udFloat2 f = udFloat2::create(other);
    udFloat2 x = udFloat2::xAxis();
    udFloat2 y = udFloat2::yAxis();
    udFloat2 res = udFloat2::zero();

    EXPECT_EQ(udFloat2::create(-1.f), -b);
    EXPECT_EQ(udFloat2::create(3.f), b + d);
    EXPECT_EQ(udFloat2::create(-1.f), b - d);
    EXPECT_EQ(udFloat2::create(2.f), b * d);
    EXPECT_EQ(udFloat2::create(4.f), b * 4);
    EXPECT_UDFLOAT2_EQ(udFloat2::create(0.5f), b / d);
    EXPECT_EQ(udFloat2::create(0.25f), b / 4);
    EXPECT_EQ(d, e);
    EXPECT_EQ(udFloat2::one(), f);
    EXPECT_FALSE(a == b);
    EXPECT_TRUE(a != b);
    EXPECT_EQ(1.f, c[0]);
    EXPECT_EQ(0.f, c[1]);
    EXPECT_EQ(1.f, constC[0]);
    EXPECT_EQ(0.f, constC[1]);
    EXPECT_EQ(1.f, x.x);
    EXPECT_EQ(0.f, x.y);
    EXPECT_EQ(0.f, y.x);
    EXPECT_EQ(1.f, y.y);
    res -= d;
    EXPECT_EQ(udFloat2::create(-2.f), res);
    res = udFloat2::zero();
    res += d;
    EXPECT_EQ(udFloat2::create(2.f), res);
    res *= d;
    EXPECT_EQ(udFloat2::create(4.f), res);
    res *= 4;
    EXPECT_EQ(udFloat2::create(16.f), res);
    res /= d;
    EXPECT_UDFLOAT2_EQ(udFloat2::create(8.f), res);
    res /= 4;
    EXPECT_UDFLOAT2_EQ(udFloat2::create(2.f), res);
    EXPECT_UDFLOAT2_EQ(udFloat2::create(8.f), 4.f * res);
  }

  // udVector3<>
  {
    udDouble3 other = udDouble3::one();
    udFloat3 a = udFloat3::zero();
    udFloat3 b = udFloat3::one();
    udFloat3 c = udFloat3::create(1.f, 0.f, 2.f);
    const udFloat3 constC = udFloat3::create(1.f, 0.f, 2.f);
    udFloat3 d = udFloat3::create(2.f);
    udFloat3 e = udFloat3::create(d);
    udFloat3 f = udFloat3::create(other);
    udFloat3 g = udFloat3::create(udFloat2::create(1.f, 0.f), 2.f);
    udFloat3 res = udFloat3::zero();

    EXPECT_EQ(udFloat3::create(-1.f), -b);
    EXPECT_EQ(udFloat3::create(3.f), b + d);
    EXPECT_EQ(udFloat3::create(-1.f), b - d);
    EXPECT_EQ(udFloat3::create(2.f), b * d);
    EXPECT_EQ(udFloat3::create(4.f), b * 4);
    EXPECT_UDFLOAT3_EQ(udFloat3::create(0.5f), b / d);
    EXPECT_EQ(udFloat3::create(0.25f), b / 4);
    EXPECT_EQ(udFloat2::create(1.f, 0.f), c.toVector2());
    EXPECT_EQ(udFloat2::create(1.f, 0.f), constC.toVector2());
    EXPECT_EQ(d, e);
    EXPECT_EQ(udFloat3::one(), f);
    EXPECT_EQ(c, g);
    EXPECT_FALSE(a == b);
    EXPECT_TRUE(a != b);
    EXPECT_EQ(1.f, c[0]);
    EXPECT_EQ(0.f, c[1]);
    EXPECT_EQ(2.f, c[2]);
    EXPECT_EQ(1.f, constC[0]);
    EXPECT_EQ(0.f, constC[1]);
    EXPECT_EQ(2.f, constC[2]);
    res -= d;
    EXPECT_EQ(udFloat3::create(-2.f), res);
    res = udFloat3::zero();
    res += d;
    EXPECT_EQ(udFloat3::create(2.f), res);
    res *= d;
    EXPECT_EQ(udFloat3::create(4.f), res);
    res *= 4;
    EXPECT_EQ(udFloat3::create(16.f), res);
    res /= d;
    EXPECT_UDFLOAT3_EQ(udFloat3::create(8.f), res);
    res /= 4;
    EXPECT_UDFLOAT3_EQ(udFloat3::create(2.f), res);
    EXPECT_UDFLOAT3_EQ(udFloat3::create(8.f), 4.f * res);
  }

  // udVector4<>
  {
    udDouble4 other = udDouble4::one();
    udFloat4 a = udFloat4::zero();
    udFloat4 b = udFloat4::one();
    udFloat4 c = udFloat4::create(1.f, 0.f, 2.f, 3.f);
    const udFloat4 constC = udFloat4::create(1.f, 0.f, 2.f, 3.f);
    udFloat4 d = udFloat4::create(2.f);
    udFloat4 e = udFloat4::create(d);
    udFloat4 f = udFloat4::create(other);
    udFloat4 g = udFloat4::create(udFloat2::create(1.f, 0.f), 2.f, 3.f);
    udFloat4 h = udFloat4::create(udFloat3::create(1.f, 0.f, 2.f), 3.f);
    udFloat4 i = udFloat4::identity();
    udFloat4 res = udFloat4::zero();

    EXPECT_EQ(udFloat4::create(-1.f), -b);
    EXPECT_EQ(udFloat4::create(3.f), b + d);
    EXPECT_EQ(udFloat4::create(-1.f), b - d);
    EXPECT_EQ(udFloat4::create(2.f), b * d);
    EXPECT_EQ(udFloat4::create(4.f), b * 4);
    EXPECT_UDFLOAT4_EQ(udFloat4::create(0.5f), b / d);
    EXPECT_EQ(udFloat4::create(0.25f), b / 4);
    EXPECT_EQ(udFloat2::create(1.f, 0.f), c.toVector2());
    EXPECT_EQ(udFloat2::create(1.f, 0.f), constC.toVector2());
    EXPECT_EQ(udFloat3::create(1.f, 0.f, 2.f), c.toVector3());
    EXPECT_EQ(udFloat3::create(1.f, 0.f, 2.f), constC.toVector3());
    EXPECT_EQ(d, e);
    EXPECT_EQ(udFloat4::one(), f);
    EXPECT_EQ(c, g);
    EXPECT_EQ(c, h);
    EXPECT_EQ(udFloat4::create(0.f, 0.f, 0.f, 1.f), i);
    EXPECT_FALSE(a == b);
    EXPECT_TRUE(a != b);
    EXPECT_EQ(1.f, c[0]);
    EXPECT_EQ(0.f, c[1]);
    EXPECT_EQ(2.f, c[2]);
    EXPECT_EQ(3.f, c[3]);
    EXPECT_EQ(1.f, constC[0]);
    EXPECT_EQ(0.f, constC[1]);
    EXPECT_EQ(2.f, constC[2]);
    EXPECT_EQ(3.f, constC[3]);
    res -= d;
    EXPECT_EQ(udFloat4::create(-2.f), res);
    res = udFloat4::zero();
    res += d;
    EXPECT_EQ(udFloat4::create(2.f), res);
    res *= d;
    EXPECT_EQ(udFloat4::create(4.f), res);
    res *= 4;
    EXPECT_EQ(udFloat4::create(16.f), res);
    res /= d;
    EXPECT_UDFLOAT4_EQ(udFloat4::create(8.f), res);
    res /= 4;
    EXPECT_UDFLOAT4_EQ(udFloat4::create(2.f), res);
    EXPECT_UDFLOAT4_EQ(udFloat4::create(8.f), (4.f * res));
  }
}

TEST(MathTests, BasicMatrixFunctions)
{
  udFloat4x4 a = udFloat4x4::create(udFloat4::create(1.f, 2.f, 3.f, 4.f), udFloat4::create(5.f, 6.f, 7.f, 8.f), udFloat4::create(9.f, 10.f, 11.f, 12.f), udFloat4::create(13.f, 14.f, 15.f, 16.f));
  udFloat4x4 b = udFloat4x4::create(1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f, 10.f, 11.f, 12.f, 13.f, 14.f, 15.f, 16.f);
  float cArray[16] = { 1.f, 2.f, 3.f, 4.f, 5.f, 6.f, 7.f, 8.f, 9.f, 10.f, 11.f, 12.f, 13.f, 14.f, 15.f, 16.f };
  udFloat4x4 c = udFloat4x4::create(cArray);
  udFloat4x4 d = udFloat4x4::create(c);
  const udFloat4x4 constD = udFloat4x4::create(d);

  for (int i = 0; i < 16; i++)
  {
    EXPECT_FLOAT_EQ(float(i + 1), a.a[i]);
    EXPECT_FLOAT_EQ(float(i + 1), b.a[i]);
    EXPECT_FLOAT_EQ(float(i + 1), c.a[i]);
    EXPECT_FLOAT_EQ(float(i + 1), d.a[i]);
    EXPECT_FLOAT_EQ(float(i + 1), constD.a[i]);
  }

  udFloat4x4 ret = a + b;
  for (int i = 0; i < 16; i++)
  {
    EXPECT_FLOAT_EQ(float((i + 1) * 2), ret.a[i]);
  }

  ret = a - b;
  for (int i = 0; i < 16; i++)
  {
    EXPECT_FLOAT_EQ(float(0), ret.a[i]);
  }

  ret = a * b;
  for (int row = 0; row < 4; row++)
  {
    for (int col = 0; col < 4; col++)
    {
      float expected = float((row * 4 + 1) * (col + 1) + (row * 4 + 2) * (col + 5) + (row * 4 + 3) * (col + 9) + (row * 4 + 4) * (col + 13));
      EXPECT_FLOAT_EQ(expected, ret.a[row * 4 + col]);
    }
  }

  udFloat4 vecRet = a * udFloat4::create(1.f, 2.f, 3.f, 4.f);
  for (int i = 0; i < 4; i++)
  {
    float expected = float(1 * (i + 1) + 2 * (i + 5) + 3 * (i + 9) + 4 * (i + 13));
    EXPECT_FLOAT_EQ(expected, vecRet[i]);
  }

  ret = a * 2.f;
  for (int i = 0; i < 16; i++)
  {
    EXPECT_FLOAT_EQ(float((i + 1) * 2), ret.a[i]);
  }

  ret = udFloat4x4::rotationX(UD_PIf);
  float rotX[16] = { 1.f, 0.f, 0.f, 0.f, 0.f, udCos(UD_PIf), udSin(UD_PIf), 0.f, 0.f, -udSin(UD_PIf), udCos(UD_PIf), 0.f, 0.f, 0.f, 0.f, 1.f };
  for (int i = 0; i < 16; i++)
  {
    EXPECT_FLOAT_EQ(rotX[i], ret.a[i]);
  }

  ret = udFloat4x4::rotationY(UD_PIf);
  float rotY[16] = { udCos(UD_PIf), 0.f, -udSin(UD_PIf), 0.f, 0.f, 1.f, 0.f, 0.f, udSin(UD_PIf), 0.f, udCos(UD_PIf), 0.f, 0.f, 0.f, 0.f, 1.f };
  for (int i = 0; i < 16; i++)
  {
    EXPECT_FLOAT_EQ(rotY[i], ret.a[i]);
  }

  ret = udFloat4x4::rotationZ(UD_PIf);
  float rotZ[16] = { udCos(UD_PIf), udSin(UD_PIf), 0.f, 0.f, -udSin(UD_PIf), udCos(UD_PIf), 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 1.f };
  for (int i = 0; i < 16; i++)
  {
    EXPECT_FLOAT_EQ(rotZ[i], ret.a[i]);
  }
}

TEST(MathTests, MatrixExtraction)
{
  const udDouble3 euler = udDouble3::create(UD_PI / 4.0, UD_PI / 6.0, UD_PI / 8.0);

  const udDouble3 position = udDouble3::create(-23.7f, 0, 40.5f);
  const udDoubleQuat orientation = udDoubleQuat::create(euler);
  const udDouble3 scaleFactor = udDouble3::create(2, 4, 1);

  const udDouble4x4 matrix = udDouble4x4::rotationQuat(orientation, position) * udDouble4x4::scaleNonUniform(scaleFactor);

  udDouble3 outPosition;
  udDoubleQuat outOrientation;
  udDouble3 outScale;

  matrix.extractTransforms(outPosition, outScale, outOrientation);

  EXPECT_DOUBLE_EQ(position.x, outPosition.x);
  EXPECT_DOUBLE_EQ(position.y, outPosition.y);
  EXPECT_DOUBLE_EQ(position.z, outPosition.z);

  EXPECT_DOUBLE_EQ(scaleFactor.x, outScale.x);
  EXPECT_DOUBLE_EQ(scaleFactor.y, outScale.y);
  EXPECT_DOUBLE_EQ(scaleFactor.z, outScale.z);

  EXPECT_DOUBLE_EQ(orientation.x, outOrientation.x);
  EXPECT_DOUBLE_EQ(orientation.y, outOrientation.y);
  EXPECT_DOUBLE_EQ(orientation.z, outOrientation.z);
  EXPECT_DOUBLE_EQ(orientation.w, outOrientation.w);

  udDouble3 outMatYPR = matrix.extractYPR();
  udDouble3 outQuatYPR = outOrientation.eulerAngles();

  EXPECT_DOUBLE_EQ(outQuatYPR.x, outMatYPR.x);
  EXPECT_DOUBLE_EQ(outQuatYPR.y, outMatYPR.y);
  EXPECT_DOUBLE_EQ(outQuatYPR.z, outMatYPR.z);
}

udDouble2 ProjectAABB(const udDouble3 &aabbMin, double aabbSize, const udDouble3 &vector)
{
  udDouble2 result;
  result[0] = udDot(aabbMin, vector);
  result[1] = result[0];
  double u;

  u = udDot(aabbMin + udDouble3::create(0.0, aabbSize, 0.0), vector);
  if (u < result[0]) result[0] = u;
  else if (u > result[1]) result[1] = u;

  u = udDot(aabbMin + udDouble3::create(0.0, 0.0, aabbSize), vector);
  if (u < result[0]) result[0] = u;
  else if (u > result[1]) result[1] = u;

  u = udDot(aabbMin + udDouble3::create(0.0, aabbSize, aabbSize), vector);
  if (u < result[0]) result[0] = u;
  else if (u > result[1]) result[1] = u;

  u = udDot(aabbMin + udDouble3::create(aabbSize, 0.0, 0.0), vector);
  if (u < result[0]) result[0] = u;
  else if (u > result[1]) result[1] = u;

  u = udDot(aabbMin + udDouble3::create(aabbSize, aabbSize, 0.0), vector);
  if (u < result[0]) result[0] = u;
  else if (u > result[1]) result[1] = u;

  u = udDot(aabbMin + udDouble3::create(aabbSize, 0.0, aabbSize), vector);
  if (u < result[0]) result[0] = u;
  else if (u > result[1]) result[1] = u;

  u = udDot(aabbMin + udDouble3::create(aabbSize, aabbSize, aabbSize), vector);
  if (u < result[0]) result[0] = u;
  else if (u > result[1]) result[1] = u;

  return result;
}

double SqDistLinePoint(udDouble3 lsOrigin, udDouble3 lsAxis, udDouble3 point)
{
  udDouble3 w;
  w = point - lsOrigin;
  double proj = udDot(w, lsAxis);
  udDouble3 closest = lsOrigin + proj * lsAxis;
  udDouble3 v = (closest - point);
  return udMagSq3(v);
}

//TODO, operate in aabb space (aabb center == [0, 0, 0])
bool AreIntersecting(udDouble3 aabbMin, double aabbSize, udDouble3 p1, udDouble3 p2, double radius)
{
  //------------------------------------------------------------------------
  // Setup (this can be precalculated)
  //------------------------------------------------------------------------

  //Todo rework this...
  udDouble3 cylinderAxis = p2 - p1;
  udDouble3 axisSq = cylinderAxis * cylinderAxis;
  double c0 = axisSq.x + axisSq.y + axisSq.z;
  double axisSq_xy = axisSq.x + axisSq.y;
  double axisSq_xz = axisSq.x + axisSq.z;
  double axisSq_zy = axisSq.z + axisSq.y;
  double cylinderLength = udMag3(cylinderAxis);
  cylinderAxis /= cylinderLength;
  udDouble3 offset = {};

  //the span of the end disk on the x-axis
  double denom = axisSq_zy * c0;
  if (denom == 0.0)
    offset[0] = 0.0;
  else
    offset[0] = (axisSq_zy) / sqrt(denom) * radius;

  //the span of the end disk on the y-axis
  denom = axisSq_xz * c0;
  if (denom == 0.0)
    offset[1] = 0.0;
  else
    offset[1] = (axisSq_xz) / sqrt(denom) * radius;

  //the span of the end disk on the z-axis
  denom = axisSq_xy * c0;
  if (denom == 0.0)
    offset[2] = 0.0;
  else
    offset[2] = (axisSq_xy) / sqrt(denom) * radius;


  //------------------------------------------------------------------------
  // Test
  //------------------------------------------------------------------------
  bool result = false;

  //separating axis = aabb axis
  for (int i = 0; i < 3; ++i)
  {
    udDouble2 cylinderRange = (p1[i] < p2[i]) ? udDouble2::create(p1[i] - offset[i], p2[i] + offset[i]) :  udDouble2::create(p2[i] - offset[i], p1[i] + offset[i]);
    udDouble2 aabbRange = udDouble2::create(aabbMin[i], aabbMin[i] + aabbSize);
    if (aabbRange[1] < cylinderRange[0] || aabbRange[0] > cylinderRange[1])
      goto epilogue;
  }

  //separating axis = cylinder axis
  udDouble2 aabbRange = ProjectAABB(aabbMin, aabbSize, cylinderAxis);
  udDouble2 cylinderRange = udDouble2::create(udDot3(p1, cylinderAxis), udDot3(p2, cylinderAxis)); //cylinderRange[1] could just be cylinderRange[0] + cylinderLength
  if (cylinderRange[0] > cylinderRange[1])
  {
    double temp = cylinderRange[0];
    cylinderRange[0] = cylinderRange[1];
    cylinderRange[1] = temp;
  }
  if (aabbRange[1] < cylinderRange[0] || aabbRange[0] > cylinderRange[1])
    goto epilogue;

  //separating axis = aabb edges x cylinder axis
  int parallelAxis = -1;
  for (int i = 0; i < 3; ++i)
  {
    udDouble3 edge = {};
    edge[i] = 1.0;
    udDouble3 separatingAxis = udCross3(edge, cylinderAxis);
    double saLen = udMag3(separatingAxis);
    if (saLen == 0.0)
    {
      //Might be the case the cylinder is very close to, and aligned with an edge
      parallelAxis = i;
      break;
    }
    separatingAxis /= saLen; 

    //TODO can be optimised; don't have to project all 8 vertices. Indeed, we only need to project 2... but how to determine which 2?
    udDouble2 aabbProj = ProjectAABB(aabbMin, aabbSize, separatingAxis);

    double cylCenterProj = udDot(p1, separatingAxis); 
    udDouble2 cylinderProj = udDouble2::create(cylCenterProj - radius, cylCenterProj + radius); //As cylinder axis is always perpendicular to separating axis

    if (cylinderProj[0] > aabbProj[1] || cylinderProj[1] < aabbProj[0])
      goto epilogue;
  }

  //Check if the cylinder is parallel to an axis, within the aabb range and close to an edge.
  //We need to check the case where the cylinder is just off an aabb edge.
  //This can be seen as the separating axis is the vector from a point on the edge, perpendicular to the cylinder axis.
  if (parallelAxis != -1)
  {
    int indFirst = parallelAxis + 1;
    int indSecond = parallelAxis + 2;
    if (indFirst > 2) indFirst -= 3;
    if (indSecond > 2) indSecond -= 3;

    //Only need to worry about the closest edge
    udDouble3 cylinderCenter = {}; //Can these be udDouble2??
    cylinderCenter[indFirst] = p1[indFirst];
    cylinderCenter[indSecond] = p1[indSecond];
    double minDistSq = DBL_MAX;
    double testDistSq;
    udDouble3 vert = {};

    vert[indFirst] = aabbMin[indFirst];
    vert[indSecond] = aabbMin[indSecond];
    testDistSq = udMagSq3(vert - cylinderCenter);
    if (testDistSq < minDistSq)
      minDistSq = testDistSq;

    vert[indFirst] = aabbMin[indFirst] + aabbSize;
    vert[indSecond] = aabbMin[indSecond];
    testDistSq = udMagSq3(vert - cylinderCenter);
    if (testDistSq < minDistSq)
      minDistSq = testDistSq;

    vert[indFirst] = aabbMin[indFirst];
    vert[indSecond] = aabbMin[indSecond] + aabbSize;
    testDistSq = udMagSq3(vert - cylinderCenter);
    if (testDistSq < minDistSq)
      minDistSq = testDistSq;

    vert[indFirst] = aabbMin[indFirst] + aabbSize;
    vert[indSecond] = aabbMin[indSecond] + aabbSize;
    testDistSq = udMagSq3(vert - cylinderCenter);
    if (testDistSq < minDistSq)
      minDistSq = testDistSq;

    if (minDistSq > radius * radius) //precalculate radius * radius? Is it used anywhere else?
      goto epilogue; //No need for further testing.
  }

  //aabb vertex to cylinder edge
  //Find closest vertex to cylinder axis
  //This can be seen as the separating axis is the vector from the aabb vertex, perpendicular to the cylinder axis.
  {
    double minDistSq = DBL_MAX;
    double testDistSq;

    testDistSq = SqDistLinePoint(p1, cylinderAxis, aabbMin);
    if (testDistSq < minDistSq) minDistSq = testDistSq;

    testDistSq = SqDistLinePoint(p1, cylinderAxis, aabbMin + udDouble3::create(0.0, aabbSize, 0.0));
    if (testDistSq < minDistSq) minDistSq = testDistSq;

    testDistSq = SqDistLinePoint(p1, cylinderAxis, aabbMin + udDouble3::create(0.0, 0.0, aabbSize));
    if (testDistSq < minDistSq) minDistSq = testDistSq;

    testDistSq = SqDistLinePoint(p1, cylinderAxis, aabbMin + udDouble3::create(0.0, aabbSize, aabbSize));
    if (testDistSq < minDistSq) minDistSq = testDistSq;

    testDistSq = SqDistLinePoint(p1, cylinderAxis, aabbMin + udDouble3::create(aabbSize, 0.0, 0.0));
    if (testDistSq < minDistSq) minDistSq = testDistSq;

    testDistSq = SqDistLinePoint(p1, cylinderAxis, aabbMin + udDouble3::create(aabbSize, aabbSize, 0.0));
    if (testDistSq < minDistSq) minDistSq = testDistSq;

    testDistSq = SqDistLinePoint(p1, cylinderAxis, aabbMin + udDouble3::create(aabbSize, 0.0, aabbSize));
    if (testDistSq < minDistSq) minDistSq = testDistSq;

    testDistSq = SqDistLinePoint(p1, cylinderAxis, aabbMin + udDouble3::create(aabbSize, aabbSize, aabbSize));
    if (testDistSq < minDistSq) minDistSq = testDistSq;

    if (testDistSq > radius * radius)
      goto epilogue;
  }

  result = true;

epilogue:
  return result;
}

TEST(MathTests, GeometryCylinderAABB_NotIntersecting)
{
  //------------------------------------------------------------------------------------------
  //Axis aligned cylinder, off aabb face
  //------------------------------------------------------------------------------------------

  //x-axis
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {3.0, 0.0, 0.0}, {4.0, 0.0, 0.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {4.0, 0.0, 0.0}, {3.0, 0.0, 0.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-3.0, 0.0, 0.0}, {-4.0, 0.0, 0.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-4.0, 0.0, 0.0}, {-3.0, 0.0, 0.0}, 0.5));

  //y-axis
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {0.0, 3.0, 0.0}, {0.0, 4.0, 0.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {0.0, 4.0, 0.0}, {0.0, 3.0, 0.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {0.0, -3.0, 0.0}, {0.0, -4.0, 0.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {0.0, -4.0, 0.0}, {0.0, -3.0, 0.0}, 0.5));

  //z-axis
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {0.0, 3.0, 0.5}, {0.0, 4.0, 1.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {0.0, 4.0, 0.5}, {0.0, 3.0, 1.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {0.0, -3.0, 0.5}, {0.0, -4.0, 1.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {0.0, -4.0, 0.5}, {0.0, -3.0, 1.0}, 0.5));

  //------------------------------------------------------------------------------------------
  //Cylinder end close to aabb edge
  //------------------------------------------------------------------------------------------
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {1.001, 1.001, 0.0}, {3.0, 3.0, 0.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-1.001, 1.001, 0.0}, {-3.0, 3.0, 0.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {1.001, -1.001, 0.0}, {3.0, -3.0, 0.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-1.001, -1.001, 0.0}, {-3.0, -3.0, 0.0}, 0.5));

  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {1.001, 0.0, 1.001}, {3.0, 0.0, 3.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {1.001, 0.0, -1.001}, {3.0, 0.0, -3.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-1.001, 0.0, 1.001}, {-3.0, 0.0, 3.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-1.001, 0.0, -1.001}, {-3.0, 0.0, -3.0}, 0.5));

  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {0.0, 1.001, 1.001}, {0.0, 3.0, 3.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {0.0, 1.001, -1.001}, {0.0, 3.0, -3.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {0.0, -1.001, 1.001}, {0.0, -3.0, 3.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {0.0, -1.001, -1.001}, {0.0, -3.0, -3.0}, 0.5));

  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {3.0, 3.0, 0.0}, {1.001, 1.001, 0.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-3.0, 3.0, 0.0}, {-1.001, 1.001, 0.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {3.0, -3.0, 0.0}, {1.001, -1.001, 0.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-3.0, -3.0, 0.0}, {-1.001, -1.001, 0.0}, 0.5));

  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {3.0, 0.0, 3.0}, {1.001, 0.0, 1.001}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {3.0, 0.0, -3.0}, {1.001, 0.0, -1.001}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-3.0, 0.0, 3.0}, {-1.001, 0.0, 1.001}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-3.0, 0.0, -3.0}, {-1.001, 0.0, -1.001}, 0.5));

  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {0.0, 3.0, 3.0}, {0.0, 1.001, 1.001}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {0.0, 3.0, -3.0}, {0.0, 1.001, -1.001}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {0.0, -3.0, 3.0}, {0.0, -1.001, 1.001}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {0.0, -3.0, -3.0}, {0.0, -1.001, -1.001}, 0.5));

  //------------------------------------------------------------------------------------------
  //Cylinder end close to aabb vertex
  //------------------------------------------------------------------------------------------
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {1.001, 1.001, 1.001}, {5.0, 5.0, 5.0}, 1.2));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {1.001, 1.001, -1.001}, {5.0, 5.0, -5.0}, 1.2));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {1.001, -1.001, 1.001}, {5.0, -5.0, 5.0}, 1.2));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {1.001, -1.001, -1.001}, {5.0, -5.0, -5.0}, 1.2));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-1.001, 1.001, 1.001}, {-5.0, 5.0, 5.0}, 1.2));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-1.001, 1.001, -1.001}, {-5.0, 5.0, -5.0}, 1.2));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-1.001, -1.001, 1.001}, {-5.0, -5.0, 5.0}, 1.2));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-1.001, -1.001, -1.001}, {-5.0, -5.0, -5.0}, 1.2));

  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {5.0, 5.0, 5.0}, {1.001, 1.001, 1.001}, 1.2));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {5.0, 5.0, -5.0}, {1.001, 1.001, -1.001}, 1.2));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {5.0, -5.0, 5.0}, {1.001, -1.001, 1.001}, 1.2));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {5.0, -5.0, -5.0}, {1.001, -1.001, -1.001}, 1.2));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-5.0, 5.0, 5.0}, {-1.001, 1.001, 1.001}, 1.2));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-5.0, 5.0, -5.0}, {-1.001, 1.001, -1.001}, 1.2));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-5.0, -5.0, 5.0}, {-1.001, -1.001, 1.001}, 1.2));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-5.0, -5.0, -5.0}, {-1.001, -1.001, -1.001}, 1.2));

  //------------------------------------------------------------------------------------------
  //Cylinder crossing aabb edge
  //------------------------------------------------------------------------------------------
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {4.0, 1.0, 0.0}, {1.0, 4.0, 0.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-4.0, 1.0, 0.0}, {-1.0, 4.0, 0.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {4.0, -1.0, 0.0}, {1.0, -4.0, 0.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-4.0, -1.0, 0.0}, {-1.0, -4.0, 0.0}, 0.5));

  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {4.0, 0.0, 1.0}, {1.0, 0.0, 4.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-4.0, 0.0, 1.0}, {-1.0, 0.0, 4.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {4.0, 0.0, -1.0}, {1.0, 0.0, -4.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-4.0, 0.0, -1.0}, {-1.0, 0.0, -4.0}, 0.5));

  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {0.0, 4.0, 1.0}, {0.0, 1.0, 4.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {0.0, -4.0, 1.0}, {0.0, -1.0, 4.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {0.0, 4.0, -1.0}, {0.0, 1.0, -4.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {0.0, -4.0, -1.0}, {0.0, -1.0, -4.0}, 0.5));

  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {1.0, 4.0, 0.0}, {4.0, 1.0, 0.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-1.0, 4.0, 0.0}, {-4.0, 1.0, 0.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {1.0, -4.0, 0.0}, {4.0, -1.0, 0.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-1.0, -4.0, 0.0}, {-4.0, -1.0, 0.0}, 0.5));

  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {1.0, 0.0, 4.0}, {4.0, 0.0, 1.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-1.0, 0.0, 4.0}, {-4.0, 0.0, 1.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {1.0, 0.0, -4.0}, {4.0, 0.0, -1.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-1.0, 0.0, -4.0}, {-4.0, 0.0, -1.0}, 0.5));

  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {0.0, 1.0, 4.0}, {0.0, 4.0, 1.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {0.0, -1.0, 4.0}, {0.0, -4.0, 1.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {0.0, 1.0, -4.0}, {0.0, 4.0, -1.0}, 0.5));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {0.0, -1.0, -4.0}, {0.0, -4.0, -1.0}, 0.5));

  //------------------------------------------------------------------------------------------
  //Cylinder side just off aabb corner
  //------------------------------------------------------------------------------------------
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {2.0, 2.0, 1.0}, {1.0, 1.0, 2.5}, 0.98));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-2.0, 2.0, 1.0}, {-1.0, 1.0, 2.5}, 0.98));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {2.0, -2.0, 1.0}, {1.0, -1.0, 2.5}, 0.98));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-2.0, -2.0, 1.0}, {-1.0, -1.0, 2.5}, 0.98));

  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {2.0, 2.0, -1.0}, {1.0, 1.0, -2.5}, 0.98));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-2.0, 2.0, -1.0}, {-1.0, 1.0, -2.5}, 0.98));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {2.0, -2.0, -1.0}, {1.0, -1.0, -2.5}, 0.98));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-2.0, -2.0, -1.0}, {-1.0, -1.0, -2.5}, 0.98));

  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {1.0, 1.0, 2.5}, {2.0, 2.0, 1.0}, 0.98));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-1.0, 1.0, 2.5}, {-2.0, 2.0, 1.0}, 0.98));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {1.0, -1.0, 2.5}, {2.0, -2.0, 1.0}, 0.98));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-1.0, -1.0, 2.5}, {-2.0, -2.0, 1.0}, 0.98));

  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {1.0, 1.0, -2.5}, {2.0, 2.0, -1.0}, 0.98));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-1.0, 1.0, -2.5}, {-2.0, 2.0, -1.0}, 0.98));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {1.0, -1.0, -2.5}, {2.0, -2.0, -1.0}, 0.98));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-1.0, -1.0, -2.5}, {-2.0, -2.0, -1.0}, 0.98));

  //------------------------------------------------------------------------------------------
  ////Cylinder parallel to axis, just of aabb edge
  //------------------------------------------------------------------------------------------
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {2.0, 2.0, -1.0}, {2.0, 2.0, 1.0}, 1.414));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-2.0, 2.0, -1.0}, {-2.0, 2.0, 1.0}, 1.414));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {2.0, -2.0, -1.0}, {2.0, -2.0, 1.0}, 1.414));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-2.0, -2.0, -1.0}, {-2.0, -2.0, 1.0}, 1.414));

  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-1.0, 2.0, 2.0}, {1.0, 2.0, 2.0}, 1.414));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-1.0, -2.0, 2.0}, {1.0, -2.0, 2.0}, 1.414));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-1.0, 2.0, -2.0}, {1.0, 2.0, -2.0}, 1.414));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-1.0, -2.0, -2.0}, {1.0, -2.0, -2.0}, 1.414));

  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {2.0, -1.0, 2.0}, {2.0, 1.0, 2.0}, 1.414));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-2.0, -1.0, 2.0}, {-2.0, 1.0, 2.0}, 1.414));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {2.0, -1.0, -2.0}, {2.0, 1.0, -2.0}, 1.414));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-2.0, -1.0, -2.0}, {-2.0, 1.0, -2.0}, 1.414));

  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {2.0, 2.0, 1.0}, {2.0, 2.0, -1.0}, 1.414));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-2.0, 2.0, 1.0}, {-2.0, 2.0, -1.0}, 1.414));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {2.0, -2.0, 1.0}, {2.0, -2.0, -1.0}, 1.414));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-2.0, -2.0, 1.0}, {-2.0, -2.0, -1.0}, 1.414));

  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {1.0, 2.0, 2.0}, {-1.0, 2.0, 2.0}, 1.414));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {1.0, -2.0, 2.0}, {-1.0, -2.0, 2.0}, 1.414));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {1.0, 2.0, -2.0}, {-1.0, 2.0, -2.0}, 1.414));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {1.0, -2.0, -2.0}, {-1.0, -2.0, -2.0}, 1.414));

  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {2.0, 1.0, 2.0}, {2.0, -1.0, 2.0}, 1.414));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-2.0, 1.0, 2.0}, {-2.0, -1.0, 2.0}, 1.414));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {2.0, 1.0, -2.0}, {2.0, -1.0, -2.0}, 1.414));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {-2.0, 1.0, -2.0}, {-2.0, -1.0, -2.0}, 1.414));

  //------------------------------------------------------------------------------------------
  //Cylinder at some other interesting orientations
  //------------------------------------------------------------------------------------------
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {1.2, 0.0, 0.0}, {6.0, 0.0, 1.0}, 0.7));
  EXPECT_FALSE(AreIntersecting({-1.0, -1.0, -1.0}, 2.0, {1.22, -0.03, 1.0}, {6.0, -1.31, 2.83}, 0.7));
}

TEST(MathTests, GeometryCylinderAABB_Intersecting)
{
  //------------------------------------------------------------------------------------------
  //Axis aligned cylinder, end penetrating aabb face
  //------------------------------------------------------------------------------------------

  //------------------------------------------------------------------------------------------
  //Cylinder end penetrating aabb edge
  //------------------------------------------------------------------------------------------

  //------------------------------------------------------------------------------------------
  //Cylinder end penetrating aabb vertex
  //------------------------------------------------------------------------------------------

  //------------------------------------------------------------------------------------------
  //Cylinder crossing aabb edge
  //------------------------------------------------------------------------------------------

  //------------------------------------------------------------------------------------------
  //aabb vertex penetrating cylinder side
  //------------------------------------------------------------------------------------------

  //------------------------------------------------------------------------------------------
  ////Cylinder parallel to axis, penetrating aabb edge
  //------------------------------------------------------------------------------------------

  //------------------------------------------------------------------------------------------
  //Cylinder at some other interesting orientations
  //------------------------------------------------------------------------------------------
}
