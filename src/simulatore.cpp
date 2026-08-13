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
    btRigidBody* palletBody;
    btRigidBody* paccoStaticoBody; // NOVITÀ: La prima scatola già sul bancale
    btRigidBody* paccoBody;        // Il pacco controllato dall'IA

public:
    AmbienteRobot() {
        std::cout << "[C++] Inizializzazione fase 2: Stacking (Incastro)..." << std::endl;

        collisionConfiguration = new btDefaultCollisionConfiguration();
        dispatcher = new btCollisionDispatcher(collisionConfiguration);
        overlappingPairCache = new btDbvtBroadphase();
        solver = new btSequentialImpulseConstraintSolver;
        dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
        dynamicsWorld->setGravity(btVector3(0, -9.81, 0));

        // 1. PAVIMENTO
        btCollisionShape* pavimentoShape = new btBoxShape(btVector3(50.0f, 0.5f, 50.0f)); 
        btTransform pavimentoTransform;
        pavimentoTransform.setIdentity();
        pavimentoTransform.setOrigin(btVector3(0, -0.5f, 0)); 
        btDefaultMotionState* pavimentoMotionState = new btDefaultMotionState(pavimentoTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfoPavimento(0.0f, pavimentoMotionState, pavimentoShape, btVector3(0,0,0));
        pavimentoBody = new btRigidBody(rbInfoPavimento);
        dynamicsWorld->addRigidBody(pavimentoBody);

        // 2. EUROPALLET (Centro Y=0.075)
        btCollisionShape* palletShape = new btBoxShape(btVector3(0.6f, 0.075f, 0.4f)); 
        btTransform palletTransform;
        palletTransform.setIdentity();
        palletTransform.setOrigin(btVector3(0.0f, 0.075f, 0.0f)); 
        btDefaultMotionState* palletMotionState = new btDefaultMotionState(palletTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfoPallet(0.0f, palletMotionState, palletShape, btVector3(0,0,0));
        palletBody = new btRigidBody(rbInfoPallet);
        dynamicsWorld->addRigidBody(palletBody);

        // 3. PACCO STATICO (Il bersaglio!). Dimensioni 40x20x30 cm.
        // Mezze misure: X=0.2, Y=0.1, Z=0.15. 
        // Appoggiato sul pallet (altezza 0.15). Il suo centro Y sarà 0.15 + 0.1 = 0.25.
        btCollisionShape* paccoStaticoShape = new btBoxShape(btVector3(0.2f, 0.1f, 0.15f)); 
        btTransform paccoStaticoTransform;
        paccoStaticoTransform.setIdentity();
        paccoStaticoTransform.setOrigin(btVector3(0.0f, 0.25f, 0.0f)); 
        btDefaultMotionState* paccoStaticoMotionState = new btDefaultMotionState(paccoStaticoTransform);
        
        // Massa 0.0f = Oggetto Statico (non cade, non subisce spinte)
        btRigidBody::btRigidBodyConstructionInfo rbInfoPaccoStatico(0.0f, paccoStaticoMotionState, paccoStaticoShape, btVector3(0,0,0));
        paccoStaticoBody = new btRigidBody(rbInfoPaccoStatico);
        dynamicsWorld->addRigidBody(paccoStaticoBody);

        // 4. PACCO DINAMICO (L'agente)
        btCollisionShape* paccoShape = new btBoxShape(btVector3(0.2f, 0.1f, 0.15f)); 
        btTransform paccoTransform;
        paccoTransform.setIdentity();
        paccoTransform.setOrigin(btVector3(0, 5.0f, 0)); 
        
        btScalar massaPacco(5.0f);
        btVector3 inerziaPacco(0, 0, 0);
        paccoShape->calculateLocalInertia(massaPacco, inerziaPacco);
        
        btDefaultMotionState* paccoMotionState = new btDefaultMotionState(paccoTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfoPacco(massaPacco, paccoMotionState, paccoShape, inerziaPacco);
        paccoBody = new btRigidBody(rbInfoPacco);
        dynamicsWorld->addRigidBody(paccoBody);
    }

    ~AmbienteRobot() {
        dynamicsWorld->removeRigidBody(paccoBody);
        delete paccoBody->getMotionState();
        delete paccoBody->getCollisionShape();
        delete paccoBody;

        dynamicsWorld->removeRigidBody(paccoStaticoBody);
        delete paccoStaticoBody->getMotionState();
        delete paccoStaticoBody->getCollisionShape();
        delete paccoStaticoBody;

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

        // Reward Shaping (uguale a prima, la teniamo vicino al centro X=0, Z=0)
        float distanza_dal_centro = (pos_x * pos_x) + (pos_z * pos_z);
        reward -= distanza_dal_centro * 0.1f; 

        float spreco_energia = (azione[0] * azione[0]) + (azione[1] * azione[1]);
        reward -= spreco_energia * 0.01f;

        // NUOVA LOGICA: Atterrare sulla prima scatola.
        // La scatola sotto ha il tetto a Y=0.35. 
        // Il pacco sopra ha mezza altezza 0.1. Se è perfettamente appoggiato, il suo centro è a 0.45.
        // Applichiamo il correttivo per il tunneling a 0.451f.
        if (altezza_y <= 0.451f) {
            done = true;
            
            // L'area sicura non è più il bancale intero, ma solo la scatola sotto!
            // I bordi della scatola sono a -0.2/0.2 (X) e -0.15/0.15 (Z).
            bool sopra_scatola_x = (pos_x >= -0.2f && pos_x <= 0.2f);
            bool sopra_scatola_z = (pos_z >= -0.15f && pos_z <= 0.15f);
            
            if (sopra_scatola_x && sopra_scatola_z) {
                // Strike! Scatola impilata con successo.
                reward += 10.0f; 
            } else {
                // Crash. Ha mancato la prima scatola, magari è caduta sul pallet o a terra.
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