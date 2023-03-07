#include <pybind11/pybind11.h>
#include "control/ASMC.h"
#include "utils/ControllerUtils.h"
#include "model/dynamic_model.h"

namespace py = pybind11;
PYBIND11_MODULE(usv_libs_py, m){
  m.doc() = "USV Libs";
  py::module controller = m.def_submodule("controller", "Controller modules");
  py::class_<ASMCParams>(controller, "ASMCParams")
          .def(py::init())
          .def_readwrite("k_u", &ASMCParams::k_u)
          .def_readwrite("k_psi", &ASMCParams::k_psi)
          .def_readwrite("kmin_u", &ASMCParams::kmin_u)
          .def_readwrite("k2_u", &ASMCParams::k2_u)
          .def_readwrite("k2_psi", &ASMCParams::k2_psi)
          .def_readwrite("mu_u", &ASMCParams::mu_u)
          .def_readwrite("mu_psi", &ASMCParams::mu_psi)
          .def_readwrite("lambda_u", &ASMCParams::lambda_u)
          .def_readwrite("lambda_psi", &ASMCParams::lambda_psi);

  py::class_<ASMCSetpoint>(controller, "ASMCSetpoint")
          .def(py::init())
          .def_readwrite("heading", &ASMCSetpoint::heading_setpoint)
          .def_readwrite("velocity", &ASMCSetpoint::velocity_setpoint);

  py::class_<ASMCOutput>(controller, "ASMCOutput")
          .def(py::init())
          .def_readwrite("left_thruster", &ASMCOutput::left_thruster)
          .def_readwrite("right_thruster", &ASMCOutput::right_thruster)
          .def_readonly("speed_gain", &ASMCOutput::speed_gain)
          .def_readonly("speed_error", &ASMCOutput::speed_error)
          .def_readonly("speed_sigma", &ASMCOutput::speed_sigma)
          .def_readonly("heading_gain", &ASMCOutput::heading_gain)
          .def_readonly("heading_error", &ASMCOutput::heading_error)
          .def_readonly("heading_sigma", &ASMCOutput::heading_sigma)
          .def_readonly("Tx", &ASMCOutput::Tx)
          .def_readonly("Tz", &ASMCOutput::Tz);

  py::class_<ASMC>(controller, "ASMC")
          .def(py::init<ASMCParams>())
          .def("defaultParams", &ASMC::defaultParams)
          .def("update", &ASMC::update);

  py::module model = m.def_submodule("model", "Model classes");
  py::class_<DynamicModel>(model, "DynamicModel")
          .def(py::init<>())
          .def(py::init<double,double,double>())
          .def("update", &DynamicModel::update);

  py::module utils = m.def_submodule("utils", "Utility functions");
  utils.def("update_controller_and_model", &ControllerUtils::update);
  utils.def("update_controller_and_model_n", &ControllerUtils::update_n);
}