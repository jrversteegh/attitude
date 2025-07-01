#include "test_common.h"

#include "attitude/types.h"

namespace ut = boost::unit_test;
using namespace attitude;

BOOST_AUTO_TEST_CASE(quaternion_mul_test) {
  auto q1 = Quaternion{1., 2., 3., 4.};
  auto q2 = Quaternion{2., 2., 1., 4.};
  auto result = q1 * q2;
  auto expected = Quaternion{-21., -2., 7., 16.};
  BOOST_TEST(result == expected);
}

BOOST_AUTO_TEST_CASE(tensor_mul_test) {
  Tensor<> t{1., 2., 3., 2., 4., 6.};
  Vector<> v{1., 2., 3.};
  auto result_vector = t * v;
  auto expected_vector = Vector<>{14., 18., 29.};
  BOOST_TEST(result_vector == expected_vector);
  auto result_tensor = t * t;
  auto expected_tensor = Tensor<>{14., 18., 29., 24., 38., 61.};
  auto expected_matrix = Matrix<>{14., 18., 29., 18., 24., 38., 29., 38., 61.};
  auto failing_matrix = Matrix<>{14., 18., 29., 18., 24., 38., 28., 38., 61.};

  BOOST_TEST(result_tensor == expected_tensor);
  BOOST_TEST(result_tensor == expected_matrix);
  BOOST_TEST(result_tensor != failing_matrix);
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
  /*
  UnitQuaternion uqs[] = {
      UnitQuaternion{std::sqrt(0.5f), 0.5f, 0.5f, 0.0f},
      UnitQuaternion{0.5f, 0.0f, 0.5f, -1.f * std::sqrt(0.5f)}};

  void test_qmq_conversion(UnitQuaternion q) {
    RotationMatrix m{q};
    UnitQuaternion result = static_cast<UnitQuaternion>(m);
    BOOST_TEST((result == q || result == -q));
  }

  */
  //  ut::framework::master_test_suite().add(BOOST_PARAM_TEST_CASE(
  //      &test_qmq_conversion, uqs, uqs + sizeof(uqs) / sizeof(uqs[0])));
  return 0;
}
