#include <boost/test/included/unit_test.hpp>
#include <boost/test/parameterized_test.hpp>

#include "attitude/types.h"

namespace ut = boost::unit_test;

using namespace attitude;
using UQ = UnitQuaternion<>;
using RM = RotationMatrix<>;

UQ uqs[] = {UQ{std::sqrt(0.5f), 0.5f, 0.5f, 0.0f},
            UQ{0.0f, std::sqrt(0.5f), 0.5f, 0.5f},
            UQ{0.0f, 0.5f, std::sqrt(0.5f), 0.5f},
            UQ{0.5f, 0.0f, 0.5f, std::sqrt(0.5f)},
            UQ{std::sqrt(0.5f), -0.5f, 0.5f, 0.0f},
            UQ{0.0f, std::sqrt(0.5f), -0.5f, 0.5f},
            UQ{0.0f, 0.5f, std::sqrt(0.5f), -0.5f},
            UQ{-0.5f, 0.0f, 0.5f, std::sqrt(0.5f)},
            UQ{-1.f * std::sqrt(0.5f), 0.5f, 0.5f, 0.0f},
            UQ{0.0f, -1.f * std::sqrt(0.5f), 0.5f, 0.5f},
            UQ{0.0f, 0.5f, -1.f * std::sqrt(0.5f), 0.5f},
            UQ{0.5f, 0.0f, 0.5f, -1.f * std::sqrt(0.5f)}};

void test_qmq_conversion(UQ q) {
  RM m{q};
  UQ result = static_cast<UQ>(m);
  BOOST_TEST((result == q || result == -q));
}

ut::test_suite* init_unit_test_suite(int /*argc*/, char* /*argv*/[]) {
  ut::framework::master_test_suite().add(BOOST_PARAM_TEST_CASE(
      &test_qmq_conversion, uqs, uqs + sizeof(uqs) / sizeof(uqs[0])));
  return 0;
}
