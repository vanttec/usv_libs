#include <pybind11/pybind11.h>
#include "control/ASMC.h"
#include "utils/ControllerUtils.h"
#include "model/dynamic_model.h"
#include <pybind11/stl.h>

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
          .def_readwrite("velocity", &ASMCSetpoint::velocity_setpoint)
          .def(py::pickle(
                  [](const ASMCSetpoint &s){ //__getstate__
                    return py::make_tuple(s.heading_setpoint, s.velocity_setpoint);
                  },
                  [](py::tuple t){  // __setstate__
                    if(t.size() != 2){
                      throw std::runtime_error("Invalid state!");
                    }

                    ASMCSetpoint s{};
                    s.heading_setpoint = t[0].cast<double>();
                    s.velocity_setpoint = t[1].cast<double>();

                    return s;
                  }
                  ));

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
          .def_readonly("Tz", &ASMCOutput::Tz)
          .def_readonly("speed_setpoint", &ASMCOutput::speed_setpoint)
          .def_readonly("heading_setpoint", &ASMCOutput::heading_setpoint)
          .def(py::pickle(
                  [](const ASMCOutput &o){
                    return py::make_tuple(o.left_thruster, o.right_thruster, o.speed_gain, o.speed_error,
                                          o.speed_sigma, o.heading_gain, o.heading_error, o.heading_sigma,
                                          o.Tx, o.Tz, o.speed_setpoint, o.heading_setpoint);
                  },
                  [](py::tuple t){
                    if (t.size() != 12)
                      throw std::runtime_error("Invalid state, tuple size expected to be 12.");
                    ASMCOutput o{};
                    o.left_thruster = t[0].cast<double>();
                    o.right_thruster = t[1].cast<double>();
                    o.speed_gain = t[2].cast<double>();
                    o.speed_error = t[3].cast<double>();
                    o.speed_sigma = t[4].cast<double>();
                    o.heading_gain = t[5].cast<double>();
                    o.heading_error = t[6].cast<double>();
                    o.heading_sigma = t[7].cast<double>();
                    o.Tx = t[8].cast<double>();
                    o.Tz = t[9].cast<double>();
                    o.speed_setpoint = t[10].cast<double>();
                    o.heading_setpoint = t[11].cast<double>();
                    return o;
                  }
                  ));

  py::class_<ASMC>(controller, "ASMC")
          .def(py::init<ASMCParams>())
          .def("defaultParams", &ASMC::defaultParams)
          .def("update", &ASMC::update);

  py::module model = m.def_submodule("model", "Model classes");
  py::class_<DynamicModel>(model, "DynamicModel")
          .def(py::init<>())
          .def(py::init<double,double,double>())
          .def("update", &DynamicModel::update)
          .def("currentState", &DynamicModel::currentState);

  py::class_<DynamicModelOutput>(model, "DynamicModelOutput")
          .def(py::init<>())
          .def_readwrite("pose_x", &DynamicModelOutput::pose_x)
          .def_readwrite("pose_y", &DynamicModelOutput::pose_y)
          .def_readwrite("pose_psi", &DynamicModelOutput::pose_psi)
          .def_readwrite("vel_x", &DynamicModelOutput::vel_x)
          .def_readwrite("vel_y", &DynamicModelOutput::vel_y)
          .def_readwrite("vel_r", &DynamicModelOutput::vel_r)
          .def(py::pickle(
                  [](const DynamicModelOutput &o){
                    return py::make_tuple(o.pose_x, o.pose_y, o.pose_psi, o.vel_x, o.vel_y, o.vel_r);
                  },
                  [](py::tuple t) {
                    DynamicModelOutput o;
                    o.pose_x = t[0].cast<double>();
                    o.pose_y = t[1].cast<double>();
                    o.pose_psi = t[2].cast<double>();
                    o.vel_x = t[3].cast<double>();
                    o.vel_y = t[4].cast<double>();
                    o.vel_r = t[5].cast<double>();
                    return o;
                  }
          ));

  py::module utils = m.def_submodule("utils", "Utility functions");
  utils.def("update_controller_and_model", &ControllerUtils::update);
  utils.def("update_controller_and_model_n", &ControllerUtils::update_n);
}