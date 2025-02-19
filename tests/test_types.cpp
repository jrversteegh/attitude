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

BOOST_AUTO_TEST_CASE(vector_component_test) {
  Vector3 v1{1.f, 2.f, 3.f};
  auto c2 = v1[1];
  BOOST_TEST(c2 == 2.f);
  c2 = 3.f;
  BOOST_TEST(c2 == 3.f);
  c2 = v1[1];
  BOOST_TEST(c2 == 2.f);
  v1[1] = 3.f;
  c2 = v1[1];
  BOOST_TEST(c2 == 3.f);
}

BOOST_AUTO_TEST_CASE(quaternion_mul_test) {
  Quaternion q1{1.f, 2.f, 3.f, 4.f};
  Quaternion q2{2.f, 2.f, 1.f, 4.f};
  auto result = q1 * q2;
  auto expected = Quaternion{-21.f, -2.f, 7.f, 16.f};
  BOOST_TEST(result == expected);
}

BOOST_AUTO_TEST_CASE(matrix_mul_test) {
  Matrix3 m{1.f, 2.f, 3.f, 2.f, 4.f, 6.f, 3.f, 6.f, 9.f};
  Vector3 v{1.f, 2.f, 3.f};
  auto result_vector = m * v;
  auto expected_vector = Vector3{14.f, 28.f, 42.f};
  BOOST_TEST(result_vector == expected_vector);
  Matrix3 result_matrix = m * m;
  Matrix3 expected_matrix{14.f, 28.f, 42.f, 28.f, 56.f,
                          84.f, 42.f, 84.f, 126.f};
  BOOST_TEST(result_matrix == expected_matrix);
}

BOOST_AUTO_TEST_CASE(tensor_mul_test) {
  Tensor t{1.f, 2.f, 3.f, 2.f, 4.f, 6.f};
  Vector3 v{1.f, 2.f, 3.f};
  auto result_vector = t * v;
  auto expected_vector = Vector3{14.f, 18.f, 29.f};
  BOOST_TEST(result_vector == expected_vector);
  auto result_tensor = t * t;
  auto expected_tensor = Tensor{14.f, 18.f, 29.f, 24.f, 38.f, 61.f};
  auto expected_matrix =
      Matrix3{14.f, 18.f, 29.f, 18.f, 24.f, 38.f, 29.f, 38.f, 61.f};
  auto failing_matrix =
      Matrix3{14.f, 18.f, 29.f, 18.f, 24.f, 38.f, 28.f, 38.f, 61.f};

  BOOST_TEST(result_tensor == expected_tensor);
  BOOST_TEST(result_tensor == expected_matrix);
  BOOST_TEST(result_tensor != failing_matrix);
}

BOOST_AUTO_TEST_CASE(quaternion_equality) {
  Quaternion q1(0.f, 1.f, 0.f, 0.f);
  Quaternion q2{0.f, 1.f, 0.f, 0.f};
  Quaternion q3{q2};
  BOOST_TEST(q1 == q2);
  BOOST_TEST(q1 == q3);
}

BOOST_AUTO_TEST_CASE(unit_quaternion_equality) {
  UnitQuaternion q1(0.f, 1.f, 0.f, 0.f);
  UnitQuaternion q2{0.f, 1.f, 0.f, 0.f};
  UnitQuaternion q3{q2};
  BOOST_TEST(q1.equals(q2));
  BOOST_TEST(q1 == q2);
  BOOST_TEST(q1 == q3);
}

BOOST_AUTO_TEST_CASE(unit_quaternion_readonlyness) {
  Quaternion q1(0.f, 1.f, 0.f, 0.f);
  UnitQuaternion q2(0.f, 1.f, 0.f, 0.f);
  q1[0] = 1.0f;
  // Doesn't (and shouldn't!) compile (how to test for that?)
  // q2[0] = 1.0f;
}

BOOST_AUTO_TEST_SUITE_END();
