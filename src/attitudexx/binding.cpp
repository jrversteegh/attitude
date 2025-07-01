/**
 * \file
 * \brief Provide python bindings for C++ objects and functions
 *
 * \author J.R. Versteegh <j.r.versteegh@gmail.com>
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>

#include "attitude/types.h"

namespace py = pybind11;
using namespace pybind11::literals;
using namespace attitude;

PYBIND11_MODULE(attitudexx, m) {
  using V = Vector<Number>;
  m.doc() = "C++ attitude module";
  py::class_<V>(m, "Vector")
      .def(py::init<Number, Number, Number>(), "x"_a, "y"_a, "z"_a, "Construct 3 component vector")
      .def("__str__", [](V const& self) { trix::to_string(self); }, "Convert to string");
}
