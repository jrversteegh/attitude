#include <boost/test/included/unit_test.hpp>
#include <boost/test/parameterized_test.hpp>

#include "attitude/types.h"

namespace ut = boost::unit_test;

using namespace attitude;

UnitQuaternion uqs[] = {
    UnitQuaternion{std::sqrt(0.5f), 0.5f, 0.5f, 0.0f},
    UnitQuaternion{0.0f, std::sqrt(0.5f), 0.5f, 0.5f},
    UnitQuaternion{0.0f, 0.5f, std::sqrt(0.5f), 0.5f},
    UnitQuaternion{0.5f, 0.0f, 0.5f, std::sqrt(0.5f)},
    UnitQuaternion{std::sqrt(0.5f), -0.5f, 0.5f, 0.0f},
    UnitQuaternion{0.0f, std::sqrt(0.5f), -0.5f, 0.5f},
    UnitQuaternion{0.0f, 0.5f, std::sqrt(0.5f), -0.5f},
    UnitQuaternion{-0.5f, 0.0f, 0.5f, std::sqrt(0.5f)},
    UnitQuaternion{-1.f * std::sqrt(0.5f), 0.5f, 0.5f, 0.0f},
    UnitQuaternion{0.0f, -1.f * std::sqrt(0.5f), 0.5f, 0.5f},
    UnitQuaternion{0.0f, 0.5f, -1.f * std::sqrt(0.5f), 0.5f},
    UnitQuaternion{0.5f, 0.0f, 0.5f, -1.f * std::sqrt(0.5f)}};

void test_qmq_conversion(UnitQuaternion q) {
  RotationMatrix m{q};
  UnitQuaternion result = static_cast<UnitQuaternion>(m);
  BOOST_TEST((result == q || result == -q));
}

ut::test_suite* init_unit_test_suite(int /*argc*/, char* /*argv*/[]) {
  ut::framework::master_test_suite().add(BOOST_PARAM_TEST_CASE(
      &test_qmq_conversion, uqs, uqs + sizeof(uqs) / sizeof(uqs[0])));
  return 0;
}
