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
    
    // I nostri due oggetti fisici!
    btRigidBody* pavimentoBody;
    btRigidBody* paccoBody;

public:
    AmbienteRobot() {
        std::cout << "[C++] Inizializzazione simulatore..." << std::endl;

        collisionConfiguration = new btDefaultCollisionConfiguration();
        dispatcher = new btCollisionDispatcher(collisionConfiguration);
        overlappingPairCache = new btDbvtBroadphase();
        solver = new btSequentialImpulseConstraintSolver;
        dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
        dynamicsWorld->setGravity(btVector3(0, -9.81, 0));

        // --- 1. CREAZIONE PAVIMENTO (Statico, Massa = 0) ---
        // Un box largo 50x50 metri, spesso 1 metro. In Bullet le misure sono "mezze larghezze"
        btCollisionShape* pavimentoShape = new btBoxShape(btVector3(50.0f, 0.5f, 50.0f)); 
        btTransform pavimentoTransform;
        pavimentoTransform.setIdentity();
        pavimentoTransform.setOrigin(btVector3(0, -0.5f, 0)); // Spostiamo giù di 0.5 così la superficie è a quota Y=0
        
        btDefaultMotionState* pavimentoMotionState = new btDefaultMotionState(pavimentoTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfoPavimento(0.0f, pavimentoMotionState, pavimentoShape, btVector3(0,0,0));
        pavimentoBody = new btRigidBody(rbInfoPavimento);
        dynamicsWorld->addRigidBody(pavimentoBody);

        // --- 2. CREAZIONE PACCO (Dinamico, Massa = 1 kg) ---
        btCollisionShape* paccoShape = new btBoxShape(btVector3(0.5f, 0.5f, 0.5f)); // Cubo di 1x1x1 metri
        btTransform paccoTransform;
        paccoTransform.setIdentity();
        paccoTransform.setOrigin(btVector3(0, 10.0f, 0)); // Il pacco parte da 10 metri di altezza!
        
        btScalar massaPacco(1.0f);
        btVector3 inerziaPacco(0, 0, 0);
        paccoShape->calculateLocalInertia(massaPacco, inerziaPacco);
        
        btDefaultMotionState* paccoMotionState = new btDefaultMotionState(paccoTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfoPacco(massaPacco, paccoMotionState, paccoShape, inerziaPacco);
        paccoBody = new btRigidBody(rbInfoPacco);
        dynamicsWorld->addRigidBody(paccoBody);

        std::cout << "[C++] Mondo fisico creato: Pavimento e Pacco pronti!" << std::endl;
    }

    ~AmbienteRobot() {
        // Pulizia della memoria
        dynamicsWorld->removeRigidBody(paccoBody);
        delete paccoBody->getMotionState();
        delete paccoBody->getCollisionShape();
        delete paccoBody;

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
        // Questa funzione ora "teletrasporta" il pacco di nuovo a 10 metri di altezza all'inizio di ogni episodio
        btTransform resetTransform;
        resetTransform.setIdentity();
        resetTransform.setOrigin(btVector3(0, 10.0f, 0));
        paccoBody->setWorldTransform(resetTransform);
        paccoBody->getMotionState()->setWorldTransform(resetTransform);
        
        // Fermiamo qualsiasi movimento residuo (togliamo la velocità della caduta precedente)
        paccoBody->setLinearVelocity(btVector3(0, 0, 0));
        paccoBody->setAngularVelocity(btVector3(0, 0, 0));
        paccoBody->clearForces();

        return {0.0f, 10.0f, 0.0f}; // Restituiamo le coordinate [X, Y, Z]
    }
    
    std::tuple<std::vector<float>, float, bool> step(std::vector<float> azione) {
        // LA MAGIA: facciamo avanzare il tempo nel motore fisico di 1/60 di secondo
        dynamicsWorld->stepSimulation(1.0f / 60.0f, 10);

        // Leggiamo dove si trova il pacco dopo questo step di tempo
        btTransform trans;
        paccoBody->getMotionState()->getWorldTransform(trans);
        float altezza_y = trans.getOrigin().getY();

        std::vector<float> nuovo_stato = {
            (float)trans.getOrigin().getX(), 
            altezza_y, 
            (float)trans.getOrigin().getZ()
        };

        // Calcoliamo se l'episodio è finito (se il centro del pacco scende a 0.5, significa che tocca terra a Y=0)
        bool done = (altezza_y <= 0.501f);
        float reward = done ? 10.0f : 0.0f; // Diamo una reward quando tocca terra (solo per prova)

        return std::make_tuple(nuovo_stato, reward, done);
    }
};

PYBIND11_MODULE(mio_simulatore, m) {
    py::class_<AmbienteRobot>(m, "AmbienteRobot")
        .def(py::init<>())
        .def("reset", &AmbienteRobot::reset)
        .def("step", &AmbienteRobot::step);
}