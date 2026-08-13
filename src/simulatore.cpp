#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <iostream>
#include <vector>
#include <btBulletDynamicsCommon.h>

namespace py = pybind11;

class AmbienteRobot {
private:
    btDefaultCollisionConfiguration* collisionConfig;
    btCollisionDispatcher* dispatcher;
    btBroadphaseInterface* overlappingPairCache;
    btSequentialImpulseConstraintSolver* solver;
    btDiscreteDynamicsWorld* dynamicsWorld;
    
    btRigidBody* floorBody;
    btRigidBody* basePillarBody;
    btRigidBody* bicepBody;
    btRigidBody* forearmBody;
    
    // I nostri giunti robotici!
    btHingeConstraint* shoulderJoint;
    btHingeConstraint* elbowJoint;



public:
    AmbienteRobot() {
        std::cout << "[C++] Inizializzazione Fase 3: Braccio Articolato Verticale..." << std::endl;

        collisionConfig = new btDefaultCollisionConfiguration();
        dispatcher = new btCollisionDispatcher(collisionConfig);
        overlappingPairCache = new btDbvtBroadphase();
        solver = new btSequentialImpulseConstraintSolver;
        dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfig);
        dynamicsWorld->setGravity(btVector3(0, -9.81, 0)); // La gravità ora è il nemico n.1

        // 1. PAVIMENTO
        btCollisionShape* floorShape = new btBoxShape(btVector3(50.0f, 0.5f, 50.0f)); 
        btTransform floorTransform;
        floorTransform.setIdentity();
        floorTransform.setOrigin(btVector3(0, -0.5f, 0)); 
        btDefaultMotionState* floorMotionState = new btDefaultMotionState(floorTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfoFloor(0.0f, floorMotionState, floorShape, btVector3(0,0,0));
        floorBody = new btRigidBody(rbInfoFloor);
        dynamicsWorld->addRigidBody(floorBody);

        // 2. PILASTRO DI BASE (Statico, massa 0)
        btCollisionShape* baseShape = new btBoxShape(btVector3(0.2f, 0.5f, 0.2f)); 
        btTransform baseTransform;
        baseTransform.setIdentity();
        baseTransform.setOrigin(btVector3(0.0f, 0.5f, 0.0f));
        btDefaultMotionState* baseMotionState = new btDefaultMotionState(baseTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfoBase(0.0f, baseMotionState, baseShape, btVector3(0,0,0));
        basePillarBody = new btRigidBody(rbInfoBase);
        dynamicsWorld->addRigidBody(basePillarBody);

        // 3. BICIPITE (Braccio 1, Dinamico, massa 2.0)
        btCollisionShape* bicepShape = new btBoxShape(btVector3(0.1f, 0.5f, 0.1f));
        btTransform bicepTransform;
        bicepTransform.setIdentity();
        bicepTransform.setOrigin(btVector3(0.0f, 1.5f, 0.0f)); // Posizionato in piedi sopra il pilastro
        
        btScalar bicepMass(2.0f);
        btVector3 bicepInertia(0, 0, 0);
        bicepShape->calculateLocalInertia(bicepMass, bicepInertia);
        btDefaultMotionState* bicepMotionState = new btDefaultMotionState(bicepTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfoBicep(bicepMass, bicepMotionState, bicepShape, bicepInertia);
        bicepBody = new btRigidBody(rbInfoBicep);
        dynamicsWorld->addRigidBody(bicepBody);

        // 4. AVAMBRACCIO (Braccio 2, Dinamico, massa 1.5)
        btCollisionShape* forearmShape = new btBoxShape(btVector3(0.08f, 0.5f, 0.08f));
        btTransform forearmTransform;
        forearmTransform.setIdentity();
        forearmTransform.setOrigin(btVector3(0.0f, 2.5f, 0.0f)); // Posizionato in piedi sopra il bicipite
        
        btScalar forearmMass(1.5f);
        btVector3 forearmInertia(0, 0, 0);
        forearmShape->calculateLocalInertia(forearmMass, forearmInertia);
        btDefaultMotionState* forearmMotionState = new btDefaultMotionState(forearmTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfoForearm(forearmMass, forearmMotionState, forearmShape, forearmInertia);
        forearmBody = new btRigidBody(rbInfoForearm);
        dynamicsWorld->addRigidBody(forearmBody);

        // --- GIUNTI E MOTORI ---
        
        // A. SPALLA (Asse Z per rotazione verticale planare)
        btVector3 axisZ(0, 0, 1);
        btVector3 pivotInBase(0, 0.5f, 0);     // Aggancio: Cima del pilastro
        btVector3 pivotInBicep(0, -0.5f, 0);   // Aggancio: Fondo del bicipite
        
        shoulderJoint = new btHingeConstraint(*basePillarBody, *bicepBody, pivotInBase, pivotInBicep, axisZ, axisZ);
        shoulderJoint->enableAngularMotor(true, 0.0f, 150.0f); // Motore acceso! Max Torque 150
        dynamicsWorld->addConstraint(shoulderJoint, true);

        // B. GOMITO (Asse Z)
        btVector3 pivotInBicepTop(0, 0.5f, 0);       // Aggancio: Cima del bicipite
        btVector3 pivotInForearmBottom(0, -0.5f, 0); // Aggancio: Fondo dell'avambraccio
        
        elbowJoint = new btHingeConstraint(*bicepBody, *forearmBody, pivotInBicepTop, pivotInForearmBottom, axisZ, axisZ);
        elbowJoint->enableAngularMotor(true, 0.0f, 100.0f); // Motore acceso! Max Torque 100
        dynamicsWorld->addConstraint(elbowJoint, true);
    }

    ~AmbienteRobot() {
        dynamicsWorld->removeConstraint(elbowJoint);
        delete elbowJoint;
        dynamicsWorld->removeConstraint(shoulderJoint);
        delete shoulderJoint;

        dynamicsWorld->removeRigidBody(forearmBody);
        delete forearmBody->getMotionState();
        delete forearmBody->getCollisionShape();
        delete forearmBody;

        dynamicsWorld->removeRigidBody(bicepBody);
        delete bicepBody->getMotionState();
        delete bicepBody->getCollisionShape();
        delete bicepBody;

        dynamicsWorld->removeRigidBody(basePillarBody);
        delete basePillarBody->getMotionState();
        delete basePillarBody->getCollisionShape();
        delete basePillarBody;

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
        // Per ora facciamo semplicemente ripartire i motori da fermi.
        // Lo stato ora ha 4 sensori: [angolo_spalla, angolo_gomito, velocità_spalla, velocità_gomito]
        return {0.0f, 0.0f, 0.0f, 0.0f};
    }
    
    std::tuple<std::vector<float>, float, bool> step(std::vector<float> action) {
        // L'IA non controlla più X e Z, ma decide la VELOCITA' DEI MOTORI (rad/s)
        float velSpalla = action[0] * 2.0f; // Da -2 a +2 rad/s
        float velGomito = action[1] * 2.0f;
        
        shoulderJoint->setMotorTargetVelocity(velSpalla);
        elbowJoint->setMotorTargetVelocity(velGomito);

        dynamicsWorld->stepSimulation(1.0f / 60.0f, 10);

        // Leggiamo i sensori interni (Encoder dei giunti)
        float angSpalla = shoulderJoint->getHingeAngle();
        float angGomito = elbowJoint->getHingeAngle();
        
        std::vector<float> stato = {angSpalla, angGomito, velSpalla, velGomito};

        // Calcoliamo dove si trova la punta dell'avambraccio (Forward Kinematics) per la reward
        btTransform trans;
        forearmBody->getMotionState()->getWorldTransform(trans);
        float altezza_polso = trans.getOrigin().getY(); 

        // OBIETTIVO TEMPORANEO: Combattere la gravità!
        // Diamo punti all'IA se riesce a tenere il polso in aria.
        float reward = altezza_polso * 0.1f; 
        bool done = false; 

        // Se l'IA spegne i motori e il braccio crolla a terra, fine episodio e penalità!
        if(altezza_polso < 0.2f) {
            reward -= 10.0f;
            done = true;
        }

        return std::make_tuple(stato, reward, done);
    }
};

PYBIND11_MODULE(mio_simulatore, m) {
    py::class_<AmbienteRobot>(m, "AmbienteRobot")
        .def(py::init<>())
        .def("reset", &AmbienteRobot::reset)
        .def("step", &AmbienteRobot::step);
}