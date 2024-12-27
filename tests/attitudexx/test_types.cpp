#define BOOST_TEST_MODULE types_test
#include "types.h"

#include "test_common.h"


BOOST_AUTO_TEST_SUITE(cpptest);


BOOST_AUTO_TEST_CASE(vector_mul_test) {
  Vector3 v1{1, 2, 3};
  Vector3 v2{2, 3, 4};
  BOOST_TEST((v1 * v2) == 20);
}

BOOST_AUTO_TEST_CASE(vector_add_test) {
  Vector3 v1{1, 2, 3};
  Vector3 v2{2, 3, 4};
  BOOST_TEST((v1 + v2) == Vector3{3, 5, 7});
}


BOOST_AUTO_TEST_SUITE_END();
