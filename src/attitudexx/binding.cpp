/**
 * \file
 * \brief Provide python bindings for C++ objects and functions
 *
 * \author J.R. Versteegh <j.r.versteegh@gmail.com>
 */

#include <pybind11/pybind11.h>

#include "../../include/types.h"

namespace py = pybind11;
using namespace attitude;

PYBIND11_MODULE(attitudexx, m) {
  m.doc() = "C++ attitude module";
  py::class_<Vector3>(m, "Vector3")
      .def(py::init<Number, Number, Number>())
      .def("__str__", &Vector3::to_string);
}
