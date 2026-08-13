#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <btBulletDynamicsCommon.h>

#include "../include/Package.h"
#include "../include/ConveyorBelt.h"
#include "../include/RoboticArm.h"

namespace py = pybind11;

class AmbienteRobot {
private:
    btDefaultCollisionConfiguration* collisionConfig;
    btCollisionDispatcher* dispatcher;
    btBroadphaseInterface* overlappingPairCache;
    btSequentialImpulseConstraintSolver* solver;
    btDiscreteDynamicsWorld* dynamicsWorld;
    
    btRigidBody* floorBody;
    RoboticArm* mainArm;
    Package* dynamicBox;
    ConveyorBelt* feedBelt;

public:
    AmbienteRobot() {
        std::cout << "[C++] Initializing Phase 6: Dynamic Conveyor Belt..." << std::endl;

        collisionConfig = new btDefaultCollisionConfiguration();
        dispatcher = new btCollisionDispatcher(collisionConfig);
        overlappingPairCache = new btDbvtBroadphase();
        solver = new btSequentialImpulseConstraintSolver;
        dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfig);
        dynamicsWorld->setGravity(btVector3(0, -9.81, 0));

        btCollisionShape* floorShape = new btBoxShape(btVector3(50.0f, 0.5f, 50.0f)); 
        btTransform floorTransform;
        floorTransform.setIdentity();
        floorTransform.setOrigin(btVector3(0, -0.5f, 0)); 
        btDefaultMotionState* floorMotionState = new btDefaultMotionState(floorTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfoFloor(0.0f, floorMotionState, floorShape, btVector3(0,0,0));
        floorBody = new btRigidBody(rbInfoFloor);
        dynamicsWorld->addRigidBody(floorBody);

        mainArm = new RoboticArm(dynamicsWorld);
        // SPAWN LONTANO: a 2 metri di distanza lungo l'asse X
        dynamicBox = new Package(dynamicsWorld, 2.0f, 0.15f, 0.0f);
        feedBelt = new ConveyorBelt(dynamicsWorld, 1.5f, -0.1f, 0.0f);
    }

    ~AmbienteRobot() {
        delete mainArm;
        delete dynamicBox;
        delete feedBelt;
        dynamicsWorld->removeRigidBody(floorBody);
        delete floorBody->getMotionState();
        delete floorBody->getCollisionShape();
        delete floorBody;
        delete dynamicsWorld;
        delete solver;
        delete overlappingPairCache;
        delete dispatcher;
        delete collisionConfig;
    }
    
    std::vector<float> reset() {
        // Il pacco riparte da 2 metri
        dynamicBox->resetPosition(2.0f, 0.15f, 0.0f);
        mainArm->reset();
        
        return {
            mainArm->getShoulderAngle(), mainArm->getElbowAngle(),
            0.0f, 0.0f,
            dynamicBox->getX(), dynamicBox->getY(),
            0.0f,
            0.0f
        };
    }
    
    std::tuple<std::vector<float>, float, bool> step(std::vector<float> action) {
        float velSpalla = action[0] * 3.0f;
        float velGomito = action[1] * 3.0f;
        bool grip_command = (action[2] > 0.0f);

        mainArm->applyMotorVelocities(velSpalla, velGomito);
        
        bool isHoldingPrima = mainArm->isHolding();
        
        // AGGIORNAMENTO FISICA DINAMICA: se non lo stiamo tenendo, il nastro lo sposta
        if (!isHoldingPrima) {
            feedBelt->update(dynamicBox);
        }

        dynamicsWorld->stepSimulation(1.0f / 60.0f, 10);
        mainArm->handleVacuum(grip_command, dynamicBox);

        float eeX = mainArm->getEndEffectorX();
        float eeY = mainArm->getEndEffectorY();
        float boxX = dynamicBox->getX();
        float boxY = dynamicBox->getY();
        float dist_to_box = std::sqrt(std::pow(eeX - boxX, 2) + std::pow(eeY - boxY, 2));

        float reward = 0.0f;
        bool done = false; 
        bool isHolding = mainArm->isHolding();

        if (!isHolding) {
            reward = -dist_to_box; 
        } else {
            reward = 5.0f; 
        }

        // CONDIZIONI DI VITTORIA E SCONFITTA
        if (boxY > 1.0f) {
            reward += 2000.0f; 
            done = true;      
        } else if (eeY < 0.1f) {
            reward -= 2000.0f; // Scontro a terra
            done = true;      
        } else if (boxX < -0.2f && !isHolding) {
            // IL PACCO E' CADUTO DAL NASTRO! Ha superato il robot.
            reward -= 2000.0f;
            done = true;
        }

        // ... (resto del codice sopra invariato)
        
        // ESTRAIAMO LA VELOCITA' DEL PACCO
        float boxVX = dynamicBox->getBody()->getLinearVelocity().getX();

        std::vector<float> state = {
            mainArm->getShoulderAngle(), mainArm->getElbowAngle(),
            velSpalla, velGomito,
            boxX, boxY, 
            boxVX, // <--- IL NUOVO SENSO: VELOCITA' ASSE X
            isHolding ? 1.0f : 0.0f
        };

        return std::make_tuple(state, reward, done);
    }
};

PYBIND11_MODULE(mio_simulatore, m) {
    py::class_<AmbienteRobot>(m, "AmbienteRobot")
        .def(py::init<>())
        .def("reset", &AmbienteRobot::reset)
        .def("step", &AmbienteRobot::step);
}