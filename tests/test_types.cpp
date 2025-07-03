#include "test_common.h"

#include "attitude/types.h"

namespace ut = boost::unit_test;
using namespace attitude;

BOOST_AUTO_TEST_CASE(tensor_mul_test) {
  Tensor<> t{1., 2., 3., 2., 4., 6.};
  Vector<> v{1., 2., 3.};
  auto result_vector = t * v;
  auto expected_vector = Vector<>{11., 20., 28.};
  BOOST_TEST(result_vector == expected_vector);
  auto result_tensor = t * t;
  auto expected_tensor = Tensor<>{9., 16., 29., 22., 40., 56.};
  auto expected_matrix = Matrix<>{9., 16., 22., 16., 29., 40., 22., 40., 56.};
  auto failing_matrix = Matrix<>{9., 16., 22., 16., 29., 41., 22., 40., 56.};

  BOOST_TEST(result_tensor == expected_tensor);
  BOOST_TEST(result_tensor == expected_matrix);
  BOOST_TEST(result_tensor != failing_matrix);
}

ut::test_suite* init_unit_test_suite(int, char*[]) {
  return 0;
}
