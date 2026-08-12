#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <iostream>
#include <vector>

namespace py = pybind11;

class AmbienteRobot {
public:
    AmbienteRobot() { std::cout << "[C++] Simulatore inizializzato!" << std::endl; }
    
    std::vector<float> reset() {
        std::cout << "[C++] Reset: pronti per un nuovo episodio." << std::endl;
        return {0.0f, 10.5f, 2.0f}; 
    }
    
    std::tuple<std::vector<float>, float, bool> step(std::vector<float> azione) {
        std::cout << "[C++] Eseguo azione: muovo verso X=" << azione[0] << std::endl;
        std::vector<float> nuovo_stato = {azione[0], 0.0f, 0.0f};
        return std::make_tuple(nuovo_stato, 1.0f, false);
    }
};

PYBIND11_MODULE(mio_simulatore, m) {
    py::class_<AmbienteRobot>(m, "AmbienteRobot")
        .def(py::init<>())
        .def("reset", &AmbienteRobot::reset)
        .def("step", &AmbienteRobot::step);
}