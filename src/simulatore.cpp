#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <iostream>
#include <vector>
#include <btBulletDynamicsCommon.h>

namespace py = pybind11;

class AmbienteRobot {
private:
    btDefaultCollisionConfiguration* collisionConfiguration;
    btCollisionDispatcher* dispatcher;
    btBroadphaseInterface* overlappingPairCache;
    btSequentialImpulseConstraintSolver* solver;
    btDiscreteDynamicsWorld* dynamicsWorld;
    
    btRigidBody* pavimentoBody;
    btRigidBody* palletBody; // Il nostro nuovo bancale!
    btRigidBody* paccoBody;

public:
    AmbienteRobot() {
        std::cout << "[C++] Inizializzazione magazzino logistico..." << std::endl;

        collisionConfiguration = new btDefaultCollisionConfiguration();
        dispatcher = new btCollisionDispatcher(collisionConfiguration);
        overlappingPairCache = new btDbvtBroadphase();
        solver = new btSequentialImpulseConstraintSolver;
        dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
        dynamicsWorld->setGravity(btVector3(0, -9.81, 0));

        // 1. PAVIMENTO (Infinito, Y=0)
        btCollisionShape* pavimentoShape = new btBoxShape(btVector3(50.0f, 0.5f, 50.0f)); 
        btTransform pavimentoTransform;
        pavimentoTransform.setIdentity();
        pavimentoTransform.setOrigin(btVector3(0, -0.5f, 0)); 
        btDefaultMotionState* pavimentoMotionState = new btDefaultMotionState(pavimentoTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfoPavimento(0.0f, pavimentoMotionState, pavimentoShape, btVector3(0,0,0));
        pavimentoBody = new btRigidBody(rbInfoPavimento);
        dynamicsWorld->addRigidBody(pavimentoBody);

        // 2. EUROPALLET (Massa 0 = Immobile. Dimensioni: 1.2m x 0.15m x 0.8m)
        // In Bullet usiamo le "mezze misure" dal centro: X=0.6, Y=0.075, Z=0.4
        btCollisionShape* palletShape = new btBoxShape(btVector3(0.6f, 0.075f, 0.4f)); 
        btTransform palletTransform;
        palletTransform.setIdentity();
        palletTransform.setOrigin(btVector3(0.0f, 0.075f, 0.0f)); // Appoggiato a terra
        btDefaultMotionState* palletMotionState = new btDefaultMotionState(palletTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfoPallet(0.0f, palletMotionState, palletShape, btVector3(0,0,0));
        palletBody = new btRigidBody(rbInfoPallet);
        dynamicsWorld->addRigidBody(palletBody);

        // 3. PACCO REALISTICO (Dimensioni: 40x20x30 cm. Massa: 5kg)
        // Mezze misure: X=0.2, Y=0.1, Z=0.15
        btCollisionShape* paccoShape = new btBoxShape(btVector3(0.2f, 0.1f, 0.15f)); 
        btTransform paccoTransform;
        paccoTransform.setIdentity();
        paccoTransform.setOrigin(btVector3(0, 5.0f, 0)); // Parte da 5 metri di altezza
        
        btScalar massaPacco(5.0f);
        btVector3 inerziaPacco(0, 0, 0);
        paccoShape->calculateLocalInertia(massaPacco, inerziaPacco);
        
        btDefaultMotionState* paccoMotionState = new btDefaultMotionState(paccoTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfoPacco(massaPacco, paccoMotionState, paccoShape, inerziaPacco);
        paccoBody = new btRigidBody(rbInfoPacco);
        dynamicsWorld->addRigidBody(paccoBody);

        std::cout << "[C++] Magazzino pronto: Pavimento, Europallet e Pacco operativi!" << std::endl;
    }

    ~AmbienteRobot() {
        dynamicsWorld->removeRigidBody(paccoBody);
        delete paccoBody->getMotionState();
        delete paccoBody->getCollisionShape();
        delete paccoBody;

        dynamicsWorld->removeRigidBody(palletBody);
        delete palletBody->getMotionState();
        delete palletBody->getCollisionShape();
        delete palletBody;

        dynamicsWorld->removeRigidBody(pavimentoBody);
        delete pavimentoBody->getMotionState();
        delete pavimentoBody->getCollisionShape();
        delete pavimentoBody;

        delete dynamicsWorld;
        delete solver;
        delete overlappingPairCache;
        delete dispatcher;
        delete collisionConfiguration;
    }
    
    std::vector<float> reset() {
        btTransform resetTransform;
        resetTransform.setIdentity();
        resetTransform.setOrigin(btVector3(0.0f, 5.0f, 0.0f));
        paccoBody->setWorldTransform(resetTransform);
        paccoBody->getMotionState()->setWorldTransform(resetTransform);
        
        paccoBody->setLinearVelocity(btVector3(0, 0, 0));
        paccoBody->setAngularVelocity(btVector3(0, 0, 0));
        paccoBody->clearForces();

        return {0.0f, 5.0f, 0.0f};
    }
    
    std::tuple<std::vector<float>, float, bool> step(std::vector<float> azione) {
        paccoBody->activate(true);

        btVector3 spinta(azione[0] * 50.0f, 0.0f, azione[1] * 50.0f);
        paccoBody->applyCentralForce(spinta);

        dynamicsWorld->stepSimulation(1.0f / 60.0f, 10);

        btTransform trans;
        paccoBody->getMotionState()->getWorldTransform(trans);
        
        float pos_x = trans.getOrigin().getX();
        float altezza_y = trans.getOrigin().getY();
        float pos_z = trans.getOrigin().getZ();

        std::vector<float> nuovo_stato = {pos_x, altezza_y, pos_z};

        bool done = false;
        float reward = 0.0f;

        // --- REWARD SHAPING ---
        float distanza_dal_centro = (pos_x * pos_x) + (pos_z * pos_z);
        reward -= distanza_dal_centro * 0.1f; 

        float spreco_energia = (azione[0] * azione[0]) + (azione[1] * azione[1]);
        reward -= spreco_energia * 0.01f;

        // --- REWARD TERMINALE CORRETTA ---
        if (altezza_y <= 0.251f) {
            done = true;
            
            bool dentro_x = (pos_x >= -0.6f && pos_x <= 0.6f);
            bool dentro_z = (pos_z >= -0.4f && pos_z <= 0.4f);
            
            // Se le coordinate X e Z sono quelle del bancale, è un centro perfetto.
            // Ignoriamo la compenetrazione su Y causata dai "salti" di frame!
            if (dentro_x && dentro_z) {
                // Strike! È sul pallet.
                reward += 10.0f; 
            } else {
                // Crash sul cemento.
                reward -= 5.0f;  
            }
        }

        return std::make_tuple(nuovo_stato, reward, done);
    }
};

PYBIND11_MODULE(mio_simulatore, m) {
    py::class_<AmbienteRobot>(m, "AmbienteRobot")
        .def(py::init<>())
        .def("reset", &AmbienteRobot::reset)
        .def("step", &AmbienteRobot::step);
}