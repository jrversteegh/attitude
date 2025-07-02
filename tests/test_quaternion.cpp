#include <boost/test/included/unit_test.hpp>
#include <boost/test/parameterized_test.hpp>

#include "attitude/types.h"

namespace ut = boost::unit_test;

using namespace attitude;
using UQ = UnitQuaternion<>;
using RM = RotationMatrix<>;

UQ uqs[] = {UQ{std::sqrt(0.5), 0.5, 0.5, 0.0},
            UQ{0.0, std::sqrt(0.5), 0.5, 0.5},
            UQ{0.0, 0.5, std::sqrt(0.5), 0.5},
            UQ{0.5, 0.0, 0.5, std::sqrt(0.5)},
            UQ{std::sqrt(0.5), -0.5, 0.5, 0.0},
            UQ{0.0, std::sqrt(0.5), -0.5, 0.5},
            UQ{0.0, 0.5, std::sqrt(0.5), -0.5},
            UQ{-0.5, 0.0, 0.5, std::sqrt(0.5)},
            UQ{-1. * std::sqrt(0.5), 0.5, 0.5, 0.0},
            UQ{0.0, -1. * std::sqrt(0.5), 0.5, 0.5},
            UQ{0.0, 0.5, -1. * std::sqrt(0.5), 0.5},
            UQ{0.5, 0.0, 0.5, -1. * std::sqrt(0.5)}};

void test_qmq_conversion(UQ q) {
  RM m{q};
  UQ result = static_cast<UQ>(m);
  BOOST_TEST_CONTEXT("result = " << result << ", q = " << q) {
    BOOST_TEST((result == q || result == -q));
  }
}

ut::test_suite* init_unit_test_suite(int /*argc*/, char* /*argv*/[]) {
  ut::framework::master_test_suite().add(BOOST_PARAM_TEST_CASE(
      &test_qmq_conversion, uqs, uqs + sizeof(uqs) / sizeof(uqs[0])));
  return 0;
}
