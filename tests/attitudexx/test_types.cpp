#define BOOST_TEST_MODULE types_test
#include "types.h"

#include "test_common.h"

using namespace attitude;


BOOST_AUTO_TEST_SUITE(cpptest);


BOOST_AUTO_TEST_CASE(vector_mul_test) {
  Vector3 v1{1, 2, 3};
  Vector3 v2{2, 3, 4};
  auto result = v1 * v2;
  auto expected = 20;
  BOOST_TEST(result == expected);
}

BOOST_AUTO_TEST_CASE(vector_add_test) {
  Vector3 v1{1, 2, 3};
  Vector3 v2{2, 3, 4};
  auto result = v1 + v2;
  auto expected = Vector3{3, 5, 7};
  BOOST_TEST(result == expected);
}

BOOST_AUTO_TEST_CASE(quaternion_mul_test) {
  Quaternion q1{1,2,3,4};
  Quaternion q2{2,2,1,4};
  auto result = q1 * q2;
  auto expected = Quaternion{8, 8, 8, 8};
  BOOST_TEST(result == expected);
}

BOOST_AUTO_TEST_SUITE_END();
