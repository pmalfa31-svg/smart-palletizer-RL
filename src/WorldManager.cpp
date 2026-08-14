#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <iostream>
#include <vector>
#include <cmath>
#include <btBulletDynamicsCommon.h>

#include "../include/Package.h"
#include "../include/ConveyorBelt.h"
#include "../include/RoboticArm.h"
#include "../include/LayerCompensator.h"

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
    LayerCompensator* compensatorRobot;

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
        
        // Spawn distance set to 2.0m on X axis, operational height at 0.6m
        dynamicBox = new Package(dynamicsWorld, 2.0f, 0.6f, 0.0f);
        feedBelt = new ConveyorBelt(dynamicsWorld, 1.5f, 0.35f, 0.0f);
        compensatorRobot = new LayerCompensator(dynamicsWorld, 0.0f, 0.0f, 1.5f);
    }

    ~AmbienteRobot() {
        delete mainArm;
        delete dynamicBox;
        delete feedBelt;
        delete compensatorRobot;
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
        // Il pacco riparte da 2 metri, alla nuova altezza operativa
        dynamicBox->resetPosition(2.0f, 0.6f, 0.0f);
        mainArm->reset();
        compensatorRobot->reset();
        
        // Estraiamo tutte le coordinate necessarie
        float elbowX = mainArm->getElbowX();
        float elbowY = mainArm->getElbowY();
        float eeX = mainArm->getEndEffectorX();
        float eeY = mainArm->getEndEffectorY();

        return {
            mainArm->getShoulderAngle(), // 0
            mainArm->getElbowAngle(),    // 1
            0.0f,                        // 2: velSpalla iniziale
            0.0f,                        // 3: velGomito iniziale
            elbowX,                      // 4: Gomito X
            elbowY,                      // 5: Gomito Y
            eeX,                         // 6: Ventosa X
            eeY,                         // 7: Ventosa Y
            dynamicBox->getX(),          // 8: Pacco X
            dynamicBox->getY(),          // 9: Pacco Y
            0.0f,                        // 10: boxVX iniziale
            0.0f                         // 11: isHolding iniziale
        };
    }
    
    std::tuple<std::vector<float>, float, bool> step(std::vector<float> action) {
        float velSpalla = action[0] * 3.0f;
        float velGomito = action[1] * 3.0f;
        bool grip_command = (action[2] > 0.0f);

        mainArm->applyMotorVelocities(velSpalla, velGomito);
        
        bool isHoldingPrima = mainArm->isHolding();
        if (!isHoldingPrima) {
            feedBelt->update(dynamicBox);
        }

        dynamicsWorld->stepSimulation(1.0f / 60.0f, 10);
        mainArm->handleVacuum(grip_command, dynamicBox);

        float elbowX = mainArm->getElbowX(); // NUOVO
        float elbowY = mainArm->getElbowY();
        float eeX = mainArm->getEndEffectorX();
        float eeY = mainArm->getEndEffectorY();
        float boxX = dynamicBox->getX();
        float boxY = dynamicBox->getY();
        float dist_to_box = std::sqrt(std::pow(eeX - boxX, 2) + std::pow(eeY - boxY, 2));

        float reward = 0.0f;
        bool done = false; 
        bool isHolding = mainArm->isHolding();

        if (!isHolding) {
            reward = 1.0f / (1.0f + dist_to_box); 
        } else {
            reward = boxY * 20.0f; 
            
            if (boxX > 1.0f) {
                reward -= 5.0f; 
            }
        }

        // Win/Loss conditions
        if (boxY > 1.0f) {
            reward += 10000.0f;
            done = true;      
        } else if (eeY < 0.02f) { // Adjusted real crash threshold
            reward -= 20.0f;
            done = true;      
        } else if (boxX < -0.2f && !isHolding) {
            reward -= 20.0f;
            done = true;
        }

        float boxVX = dynamicBox->getBody()->getLinearVelocity().getX();

        std::vector<float> state = {
            mainArm->getShoulderAngle(),
            mainArm->getElbowAngle(),
            velSpalla,
            velGomito,
            elbowX,                 // Indice 4
            elbowY,                 // Indice 5
            eeX,                    // Indice 6
            eeY,                    // Indice 7
            boxX,                   // Indice 8
            boxY,                   // Indice 9
            boxVX,                  // Indice 10
            isHolding ? 1.0f : 0.0f // Indice 11
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