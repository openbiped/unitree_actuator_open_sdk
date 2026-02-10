// Unitree Actuator SDK compatible Python bindings.
//
// This binding intentionally mirrors the public API surface of the upstream
// Unitree Actuator SDK Python wrapper (module: unitree_actuator_open_sdk)

#include <pybind11/pybind11.h>

#include "serialPort/SerialPort.h"
#include "unitreeMotor/unitreeMotor.h"

#define EXPORT_API __attribute__((visibility("default")))

namespace py = pybind11;

EXPORT_API
PYBIND11_MODULE(_unitree_actuator_open_sdk, m) {
    m.doc() = "Unitree Actuator SDK compatible wrapper";

    // ---------------------------------------------------------------------
    // Enums
    // ---------------------------------------------------------------------
    py::enum_<MotorType>(m, "MotorType")
        .value("A1", MotorType::A1, "A1 motor")
        .value("B1", MotorType::B1, "B1 motor")
        .value("GO_M8010_6", MotorType::GO_M8010_6, "GO-M8010-6 motor")
        .value("GO_2", MotorType::GO_2, "GO2 motor")
        .export_values();

    py::enum_<MotorMode>(m, "MotorMode")
        .value("BRAKE", MotorMode::BRAKE, "Brake")
        .value("FOC", MotorMode::FOC, "FOC")
        .value("CALIBRATE", MotorMode::CALIBRATE, "Calibrate")
        .export_values();

    // ---------------------------------------------------------------------
    // Data structures
    // ---------------------------------------------------------------------
    py::class_<MotorCmd>(m, "MotorCmd")
        .def(py::init([]() {
            MotorCmd c;
            c.motorType = MotorType::A1;
            c.hex_len = 0;
            c.id = 0;
            c.mode = 0;
            c.tau = 0.0f;
            c.dq = 0.0f;
            c.q = 0.0f;
            c.kp = 0.0f;
            c.kd = 0.0f;
            return c;
        }))
        .def_readwrite("motorType", &MotorCmd::motorType)
        .def_readwrite("hex_len", &MotorCmd::hex_len)
        .def_readwrite("id", &MotorCmd::id)
        .def_readwrite("mode", &MotorCmd::mode)
        .def_readwrite("tau", &MotorCmd::tau)
        .def_readwrite("dq", &MotorCmd::dq)
        .def_readwrite("q", &MotorCmd::q)
        .def_readwrite("kp", &MotorCmd::kp)
        .def_readwrite("kd", &MotorCmd::kd)
        .def("__repr__", [](const MotorCmd& c) {
            return "MotorCmd(id=" + std::to_string(c.id) + ", mode=" + std::to_string(c.mode) + ")";
        });

    py::class_<MotorData>(m, "MotorData")
        .def(py::init([]() {
            MotorData d;
            d.motorType = MotorType::A1;
            d.hex_len = 0;
            d.motor_id = 0;
            d.mode = 0;
            d.temp = 0;
            d.merror = 0;
            d.tau = 0.0f;
            d.dq = 0.0f;
            d.q = 0.0f;
            d.correct = false;
            return d;
        }))
        .def_readwrite("motorType", &MotorData::motorType)
        .def_readwrite("hex_len", &MotorData::hex_len)
        .def_readwrite("motor_id", &MotorData::motor_id)
        .def_readwrite("mode", &MotorData::mode)
        .def_readwrite("temp", &MotorData::temp)
        .def_readwrite("merror", &MotorData::merror)
        .def_readwrite("tau", &MotorData::tau)
        .def_readwrite("dq", &MotorData::dq)
        .def_readwrite("q", &MotorData::q)
        .def_readwrite("correct", &MotorData::correct)
        // Extra fields (harmless to add; keeps compatibility while adding utility)
        .def_readwrite("footForce", &MotorData::footForce)
        .def("__repr__", [](const MotorData& d) {
            return "MotorData(id=" + std::to_string(d.motor_id) + ", q=" + std::to_string(d.q) + ")";
        });

    // ---------------------------------------------------------------------
    // SerialPort
    // ---------------------------------------------------------------------
    py::class_<SerialPort>(m, "SerialPort")
        .def(py::init<const std::string&>(), py::arg("port"))
        .def("test", &SerialPort::test)
        // Match the Unitree wrapper call style: sendRecv(cmd, data)
        .def(
            "sendRecv",
            [](SerialPort& self, MotorCmd& cmd, MotorData& data) {
                py::gil_scoped_release release;
                return self.sendRecv(&cmd, &data);
            },
            py::arg("cmd"),
            py::arg("data"),
            "Send one motor command and receive one motor reply")
        // Optional: expose resetSerial for advanced users
        .def(
            "resetSerial",
            [](SerialPort& self,
               size_t recvLength,
               uint32_t baudrate,
               size_t timeOutUs) {
                self.resetSerial(recvLength, baudrate, timeOutUs);
            },
            py::arg("recvLength") = 16,
            py::arg("baudrate") = 4800000,
            py::arg("timeOutUs") = 20000);

    // ---------------------------------------------------------------------
    // Utility functions
    // ---------------------------------------------------------------------
    m.def("queryMotorMode", &queryMotorMode, py::arg("motorType"), py::arg("motorMode"),
          "Query the protocol-specific integer mode value for a given MotorType/MotorMode");
    m.def("queryGearRatio", &queryGearRatio, py::arg("motorType"),
          "Query the gear ratio");
}
