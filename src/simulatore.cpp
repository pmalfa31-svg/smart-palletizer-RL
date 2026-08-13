#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <iostream>
#include <vector>
#include <cmath>
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
    btRigidBody* dynamicBoxBody;
    
    btHingeConstraint* shoulderJoint;
    btHingeConstraint* elbowJoint;
    btFixedConstraint* vacuumConstraint;

public:
    AmbienteRobot() : vacuumConstraint(nullptr) {
        std::cout << "[C++] Initializing Phase 4: Vacuum Grasping (Auto-Grip) & Reward Shaping..." << std::endl;

        collisionConfig = new btDefaultCollisionConfiguration();
        dispatcher = new btCollisionDispatcher(collisionConfig);
        overlappingPairCache = new btDbvtBroadphase();
        solver = new btSequentialImpulseConstraintSolver;
        dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfig);
        dynamicsWorld->setGravity(btVector3(0, -9.81, 0));

        // 1. FLOOR
        btCollisionShape* floorShape = new btBoxShape(btVector3(50.0f, 0.5f, 50.0f)); 
        btTransform floorTransform;
        floorTransform.setIdentity();
        floorTransform.setOrigin(btVector3(0, -0.5f, 0)); 
        btDefaultMotionState* floorMotionState = new btDefaultMotionState(floorTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfoFloor(0.0f, floorMotionState, floorShape, btVector3(0,0,0));
        floorBody = new btRigidBody(rbInfoFloor);
        dynamicsWorld->addRigidBody(floorBody);

        // 2. BASE PILLAR
        btCollisionShape* baseShape = new btBoxShape(btVector3(0.2f, 0.5f, 0.2f)); 
        btTransform baseTransform;
        baseTransform.setIdentity();
        baseTransform.setOrigin(btVector3(0.0f, 0.5f, 0.0f));
        btDefaultMotionState* baseMotionState = new btDefaultMotionState(baseTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfoBase(0.0f, baseMotionState, baseShape, btVector3(0,0,0));
        basePillarBody = new btRigidBody(rbInfoBase);
        dynamicsWorld->addRigidBody(basePillarBody);

        // 3. BICEP
        btCollisionShape* bicepShape = new btBoxShape(btVector3(0.1f, 0.5f, 0.1f));
        btTransform bicepTransform;
        bicepTransform.setIdentity();
        bicepTransform.setOrigin(btVector3(0.0f, 1.5f, 0.0f)); 
        btScalar bicepMass(2.0f);
        btVector3 bicepInertia(0, 0, 0);
        bicepShape->calculateLocalInertia(bicepMass, bicepInertia);
        btDefaultMotionState* bicepMotionState = new btDefaultMotionState(bicepTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfoBicep(bicepMass, bicepMotionState, bicepShape, bicepInertia);
        bicepBody = new btRigidBody(rbInfoBicep);
        dynamicsWorld->addRigidBody(bicepBody);

        // 4. FOREARM
        btCollisionShape* forearmShape = new btBoxShape(btVector3(0.08f, 0.5f, 0.08f));
        btTransform forearmTransform;
        forearmTransform.setIdentity();
        forearmTransform.setOrigin(btVector3(0.0f, 2.5f, 0.0f)); 
        btScalar forearmMass(1.5f);
        btVector3 forearmInertia(0, 0, 0);
        forearmShape->calculateLocalInertia(forearmMass, forearmInertia);
        btDefaultMotionState* forearmMotionState = new btDefaultMotionState(forearmTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfoForearm(forearmMass, forearmMotionState, forearmShape, forearmInertia);
        forearmBody = new btRigidBody(rbInfoForearm);
        dynamicsWorld->addRigidBody(forearmBody);

        // 5. THE PACKAGE
        btCollisionShape* dynamicBoxShape = new btBoxShape(btVector3(0.15f, 0.15f, 0.15f)); 
        btTransform dynamicBoxTransform;
        dynamicBoxTransform.setIdentity();
        dynamicBoxTransform.setOrigin(btVector3(1.0f, 0.15f, 0.0f));
        btScalar boxMass(1.0f);
        btVector3 boxInertia(0, 0, 0);
        dynamicBoxShape->calculateLocalInertia(boxMass, boxInertia);
        btDefaultMotionState* dynamicBoxMotionState = new btDefaultMotionState(dynamicBoxTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfoDynamicBox(boxMass, dynamicBoxMotionState, dynamicBoxShape, boxInertia);
        dynamicBoxBody = new btRigidBody(rbInfoDynamicBox);
        dynamicsWorld->addRigidBody(dynamicBoxBody);

        // JOINTS
        btVector3 axisZ(0, 0, 1);
        btVector3 pivotInBase(0, 0.5f, 0);     
        btVector3 pivotInBicep(0, -0.5f, 0);   
        shoulderJoint = new btHingeConstraint(*basePillarBody, *bicepBody, pivotInBase, pivotInBicep, axisZ, axisZ);
        shoulderJoint->enableAngularMotor(true, 0.0f, 250.0f); 
        dynamicsWorld->addConstraint(shoulderJoint, true);

        btVector3 pivotInBicepTop(0, 0.5f, 0);       
        btVector3 pivotInForearmBottom(0, -0.5f, 0); 
        elbowJoint = new btHingeConstraint(*bicepBody, *forearmBody, pivotInBicepTop, pivotInForearmBottom, axisZ, axisZ);
        elbowJoint->enableAngularMotor(true, 0.0f, 150.0f); 
        dynamicsWorld->addConstraint(elbowJoint, true);
    }

    ~AmbienteRobot() {
        if (vacuumConstraint) {
            dynamicsWorld->removeConstraint(vacuumConstraint);
            delete vacuumConstraint;
        }
        dynamicsWorld->removeConstraint(elbowJoint);
        delete elbowJoint;
        dynamicsWorld->removeConstraint(shoulderJoint);
        delete shoulderJoint;

        dynamicsWorld->removeRigidBody(dynamicBoxBody);
        delete dynamicBoxBody->getMotionState();
        delete dynamicBoxBody->getCollisionShape();
        delete dynamicBoxBody;

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
        if (vacuumConstraint) {
            dynamicsWorld->removeConstraint(vacuumConstraint);
            delete vacuumConstraint;
            vacuumConstraint = nullptr;
        }

        // 1. Reset Box
        btTransform boxTrans;
        boxTrans.setIdentity();
        boxTrans.setOrigin(btVector3(1.0f, 0.15f, 0.0f));
        dynamicBoxBody->setWorldTransform(boxTrans);
        dynamicBoxBody->getMotionState()->setWorldTransform(boxTrans);
        dynamicBoxBody->setLinearVelocity(btVector3(0,0,0));
        dynamicBoxBody->setAngularVelocity(btVector3(0,0,0));
        dynamicBoxBody->clearForces();

        // 2. Reset Arms
        btTransform bicepTrans;
        bicepTrans.setIdentity();
        bicepTrans.setOrigin(btVector3(0.0f, 1.5f, 0.0f));
        bicepBody->setWorldTransform(bicepTrans);
        bicepBody->getMotionState()->setWorldTransform(bicepTrans);
        bicepBody->setLinearVelocity(btVector3(0,0,0));
        bicepBody->setAngularVelocity(btVector3(0,0,0));
        bicepBody->clearForces();

        btTransform forearmTrans;
        forearmTrans.setIdentity();
        forearmTrans.setOrigin(btVector3(0.0f, 2.5f, 0.0f));
        forearmBody->setWorldTransform(forearmTrans);
        forearmBody->getMotionState()->setWorldTransform(forearmTrans);
        forearmBody->setLinearVelocity(btVector3(0,0,0));
        forearmBody->setAngularVelocity(btVector3(0,0,0));
        forearmBody->clearForces();

        shoulderJoint->setMotorTargetVelocity(0.0f);
        elbowJoint->setMotorTargetVelocity(0.0f);
        
        // 3. Return true sensor data
        return {
            shoulderJoint->getHingeAngle(), elbowJoint->getHingeAngle(),
            0.0f, 0.0f,
            1.0f, 0.15f,
            0.0f
        };
    }
    
    std::tuple<std::vector<float>, float, bool> step(std::vector<float> action) {
        float velSpalla = action[0] * 3.0f;
        float velGomito = action[1] * 3.0f;
        bool grip_command = (action[2] > 0.0f); // IL RITORNO DEL CONTROLLO MANUALE

        shoulderJoint->setMotorTargetVelocity(velSpalla);
        elbowJoint->setMotorTargetVelocity(velGomito);

        dynamicsWorld->stepSimulation(1.0f / 60.0f, 10);

        btTransform eeLocalTrans;
        eeLocalTrans.setIdentity();
        eeLocalTrans.setOrigin(btVector3(0, 0.5f, 0)); 
        btTransform eeWorldTrans = forearmBody->getWorldTransform() * eeLocalTrans;
        float eeX = eeWorldTrans.getOrigin().getX();
        float eeY = eeWorldTrans.getOrigin().getY();

        btTransform boxTrans;
        dynamicBoxBody->getMotionState()->getWorldTransform(boxTrans);
        float boxX = boxTrans.getOrigin().getX();
        float boxY = boxTrans.getOrigin().getY();

        float dist_to_box = std::sqrt(std::pow(eeX - boxX, 2) + std::pow(eeY - boxY, 2));

        // VACUUM LOGIC (MANUALE E DI PRECISIONE)
        if (grip_command && !vacuumConstraint && dist_to_box < 0.2f) { // Tolleranza ridotta a 20cm
            btTransform frameInA = forearmBody->getWorldTransform().inverse() * dynamicBoxBody->getWorldTransform();
            btTransform frameInB;
            frameInB.setIdentity();
            vacuumConstraint = new btFixedConstraint(*forearmBody, *dynamicBoxBody, frameInA, frameInB);
            dynamicsWorld->addConstraint(vacuumConstraint, true);
        } else if (!grip_command && vacuumConstraint) {
            // Se l'IA spegne l'interruttore, il pacco cade
            dynamicsWorld->removeConstraint(vacuumConstraint);
            delete vacuumConstraint;
            vacuumConstraint = nullptr;
        }

        // REWARD SHAPING & TERMINATION CONDITIONS (Invariato, ora è perfetto)
        float reward = 0.0f;
        bool done = false; 
        bool isHolding = (vacuumConstraint != nullptr);

        if (!isHolding) {
            reward = -dist_to_box; 
        } else {
            reward = 5.0f; 
        }

        if (boxY > 1.0f) {
            reward += 2000.0f; 
            done = true;      
        } else if (eeY < 0.1f) {
            reward -= 2000.0f; 
            done = true;      
        }

        std::vector<float> state = {
            shoulderJoint->getHingeAngle(), elbowJoint->getHingeAngle(),
            velSpalla, velGomito,
            boxX, boxY,
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