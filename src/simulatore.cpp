#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <iostream>
#include <vector>

// Includiamo la libreria principale di Bullet
#include <btBulletDynamicsCommon.h>

namespace py = pybind11;

class AmbienteRobot {
private:
    // I componenti fondamentali del motore fisico
    btDefaultCollisionConfiguration* collisionConfiguration;
    btCollisionDispatcher* dispatcher;
    btBroadphaseInterface* overlappingPairCache;
    btSequentialImpulseConstraintSolver* solver;
    btDiscreteDynamicsWorld* dynamicsWorld;

public:
    AmbienteRobot() {
        std::cout << "[C++] Inizializzazione simulatore..." << std::endl;

        // 1. Setup dell'infrastruttura di Bullet
        collisionConfiguration = new btDefaultCollisionConfiguration();
        dispatcher = new btCollisionDispatcher(collisionConfiguration);
        overlappingPairCache = new btDbvtBroadphase();
        solver = new btSequentialImpulseConstraintSolver;
        
        // 2. Creazione del "Mondo"
        dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
        
        // 3. Impostiamo la gravità (Asse Y verso il basso a 9.81 m/s^2)
        dynamicsWorld->setGravity(btVector3(0, -9.81, 0));

        std::cout << "[C++] Mondo fisico creato con successo! Gravita' impostata." << std::endl;
    }

    ~AmbienteRobot() {
        // Pulizia della memoria (il C++ richiede ordine!)
        delete dynamicsWorld;
        delete solver;
        delete overlappingPairCache;
        delete dispatcher;
        delete collisionConfiguration;
        std::cout << "[C++] Memoria liberata." << std::endl;
    }
    
    std::vector<float> reset() {
        return {0.0f, 10.5f, 2.0f}; 
    }
    
    std::tuple<std::vector<float>, float, bool> step(std::vector<float> azione) {
        // Nel prossimo step faremo calcolare un fotogramma di fisica qui dentro!
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