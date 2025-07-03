#include "test_common.h"

#include "attitude/quaternion.h"

namespace ut = boost::unit_test;

using namespace attitude;

BOOST_AUTO_TEST_CASE(quaternion_mul_test) {
  auto q1 = Quaternion{1., 2., 3., 4.};
  auto q2 = Quaternion{2., 2., 1., 4.};
  auto result = q1 * q2;
  auto expected = Quaternion{-21., -2., 7., 16.};
  BOOST_TEST(result == expected);
}

BOOST_AUTO_TEST_CASE(quaternion_equality) {
  Quaternion<> q1(0., 1., 0., 0.);
  Quaternion<> q2{0., 1., 0., 0.};
  Quaternion<> q3{q2};
  BOOST_TEST(q1 == q2);
  BOOST_TEST(q1 == q3);
}

BOOST_AUTO_TEST_CASE(unit_quaternion_equality) {
  UnitQuaternion<> q1(0., 1., 0., 0.);
  UnitQuaternion<> q2{0., 1., 0., 0.};
  UnitQuaternion<> q3{q2};
  BOOST_TEST(q1 == q2);
  BOOST_TEST(q1 == q3);
}

BOOST_AUTO_TEST_CASE(unit_quaternion_readonlyness) {
  Quaternion<> q1(0., 1., 0., 0.);
  UnitQuaternion<> q2(0., 1., 0., 0.);
  q1[0] = 1.0;
  // Doesn't (and shouldn't!) compile (how to test for that?)
  // q2[0] = 1.0f;
}

ut::test_suite* init_unit_test_suite(int, char*[]) {
  return 0;
}
