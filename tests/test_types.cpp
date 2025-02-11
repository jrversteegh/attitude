#define BOOST_TEST_MODULE types_test
#include "attitude/types.h"

#include "test_common.h"

using namespace attitude;

BOOST_AUTO_TEST_SUITE(cpptest);

BOOST_AUTO_TEST_CASE(vector_mul_test) {
  Vector3 v1{1.f, 2.f, 3.f};
  Vector3 v2{2.f, 3.f, 4.f};
  auto result = v1 * v2;
  auto expected = 20.f;
  BOOST_TEST(result == expected);
}

BOOST_AUTO_TEST_CASE(vector_add_test) {
  Vector3 v1{1.f, 2.f, 3.f};
  Vector3 v2{2.f, 3.f, 4.f};
  auto result = v1 + v2;
  auto expected = Vector3{3.f, 5.f, 7.f};
  BOOST_TEST(result == expected);
}

BOOST_AUTO_TEST_CASE(quaternion_mul_test) {
  Quaternion q1{1.f, 2.f, 3.f, 4.f};
  Quaternion q2{2.f, 2.f, 1.f, 4.f};
  auto result = q1 * q2;
  auto expected = Quaternion{-21.f, -2.f, 7.f, 16.f};
  BOOST_TEST(result == expected);
  auto result2 = q1.mul(q2);
  BOOST_TEST(result2 == expected);
}

BOOST_AUTO_TEST_CASE(matrix_mul_test) {
  Matrix3 m{1.f, 2.f, 3.f, 2.f, 4.f, 6.f, 3.f, 6.f, 9.f};
  Vector3 v{1.f, 2.f, 3.f};
  auto result1 = m * v;
  auto expected1 = Vector3{14.f, 28.f, 42.f};
  BOOST_TEST(result1 == expected1);
  Matrix3 result2 = m * m;
  Matrix3 expected2{14.f, 28.f, 42.f, 28.f, 56.f, 84.f, 42.f, 84.f, 126.f};
  BOOST_TEST(result2 == expected2);
}

BOOST_AUTO_TEST_CASE(rotation_matrix_mul_test) {
  RotationMatrix m{1.f, 2.f, 3.f, 2.f, 4.f, 6.f};
  Vector3 v{1.f, 2.f, 3.f};
  auto result1 = m * v;
  auto expected1 = Vector3{14.f, 18.f, 29.f};
  BOOST_TEST(result1 == expected1);
  auto result2 = m * m;
  auto expected2 = RotationMatrix{14.f, 18.f, 29.f, 24.f, 38.f, 61.f};
  BOOST_TEST(result2 == expected2);
}

BOOST_AUTO_TEST_SUITE_END();
