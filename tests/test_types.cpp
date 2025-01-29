#define BOOST_TEST_MODULE types_test
#include "types.h"

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
  Quaternion q1{1.f,2.f,3.f,4.f};
  Quaternion q2{2.f,2.f,1.f,4.f};
  auto result = q1 * q2;
  auto expected = Quaternion{-21.f, -2.f, 7.f, 16.f};
  BOOST_TEST(result == expected);
}

BOOST_AUTO_TEST_SUITE_END();
