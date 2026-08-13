#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <iostream>
#include <vector>
#include <random> // Required for randomization
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
    btRigidBody* palletBody;
    btRigidBody* staticBoxBody;
    btRigidBody* dynamicBoxBody;

    // Random Number Generator for initial positions
    std::mt19937 rng;
    std::uniform_real_distribution<float> pos_distribution;

public:
    AmbienteRobot() {
        std::cout << "[C++] Initializing Phase 2: Spatial Randomization..." << std::endl;

        // Initialize the random generator for coordinates between -2.0m and 2.0m
        rng.seed(std::random_device{}());
        pos_distribution = std::uniform_real_distribution<float>(-0.5f, 0.5f);

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

        // 2. EUROPALLET
        btCollisionShape* palletShape = new btBoxShape(btVector3(0.6f, 0.075f, 0.4f)); 
        btTransform palletTransform;
        palletTransform.setIdentity();
        palletTransform.setOrigin(btVector3(0.0f, 0.075f, 0.0f)); 
        btDefaultMotionState* palletMotionState = new btDefaultMotionState(palletTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfoPallet(0.0f, palletMotionState, palletShape, btVector3(0,0,0));
        palletBody = new btRigidBody(rbInfoPallet);
        dynamicsWorld->addRigidBody(palletBody);

        // 3. STATIC BOX (Target)
        btCollisionShape* staticBoxShape = new btBoxShape(btVector3(0.2f, 0.1f, 0.15f)); 
        btTransform staticBoxTransform;
        staticBoxTransform.setIdentity();
        staticBoxTransform.setOrigin(btVector3(0.0f, 0.25f, 0.0f)); 
        btDefaultMotionState* staticBoxMotionState = new btDefaultMotionState(staticBoxTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfoStaticBox(0.0f, staticBoxMotionState, staticBoxShape, btVector3(0,0,0));
        staticBoxBody = new btRigidBody(rbInfoStaticBox);
        dynamicsWorld->addRigidBody(staticBoxBody);

        // 4. DYNAMIC BOX (Agent)
        btCollisionShape* dynamicBoxShape = new btBoxShape(btVector3(0.2f, 0.1f, 0.15f)); 
        btTransform dynamicBoxTransform;
        dynamicBoxTransform.setIdentity();
        dynamicBoxTransform.setOrigin(btVector3(0, 5.0f, 0)); 
        
        btScalar boxMass(5.0f);
        btVector3 boxInertia(0, 0, 0);
        dynamicBoxShape->calculateLocalInertia(boxMass, boxInertia);
        
        btDefaultMotionState* dynamicBoxMotionState = new btDefaultMotionState(dynamicBoxTransform);
        btRigidBody::btRigidBodyConstructionInfo rbInfoDynamicBox(boxMass, dynamicBoxMotionState, dynamicBoxShape, boxInertia);
        dynamicBoxBody = new btRigidBody(rbInfoDynamicBox);
        dynamicsWorld->addRigidBody(dynamicBoxBody);
    }

    ~AmbienteRobot() {
        dynamicsWorld->removeRigidBody(dynamicBoxBody);
        delete dynamicBoxBody->getMotionState();
        delete dynamicBoxBody->getCollisionShape();
        delete dynamicBoxBody;

        dynamicsWorld->removeRigidBody(staticBoxBody);
        delete staticBoxBody->getMotionState();
        delete staticBoxBody->getCollisionShape();
        delete staticBoxBody;

        dynamicsWorld->removeRigidBody(palletBody);
        delete palletBody->getMotionState();
        delete palletBody->getCollisionShape();
        delete palletBody;

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
        // Generate random starting coordinates between -2.0m and 2.0m
        float start_x = pos_distribution(rng);
        float start_z = pos_distribution(rng);

        btTransform resetTransform;
        resetTransform.setIdentity();
        resetTransform.setOrigin(btVector3(start_x, 5.0f, start_z));
        dynamicBoxBody->setWorldTransform(resetTransform);
        dynamicBoxBody->getMotionState()->setWorldTransform(resetTransform);
        
        dynamicBoxBody->setLinearVelocity(btVector3(0, 0, 0));
        dynamicBoxBody->setAngularVelocity(btVector3(0, 0, 0));
        dynamicBoxBody->clearForces();

        // Feed the new random position to the Python Gym Environment
        return {start_x, 5.0f, start_z};
    }
    
    std::tuple<std::vector<float>, float, bool> step(std::vector<float> action) {
        dynamicBoxBody->activate(true);

        btVector3 thrust(action[0] * 50.0f, 0.0f, action[1] * 50.0f);
        dynamicBoxBody->applyCentralForce(thrust);

        dynamicsWorld->stepSimulation(1.0f / 60.0f, 10);

        btTransform trans;
        dynamicBoxBody->getMotionState()->getWorldTransform(trans);
        
        float pos_x = trans.getOrigin().getX();
        float height_y = trans.getOrigin().getY();
        float pos_z = trans.getOrigin().getZ();

        std::vector<float> new_state = {pos_x, height_y, pos_z};

        bool done = false;
        float reward = 0.0f;

        // Reward Shaping
        float distance_from_center = (pos_x * pos_x) + (pos_z * pos_z);
        reward -= distance_from_center * 0.1f; 

        float energy_waste = (action[0] * action[0]) + (action[1] * action[1]);
        reward -= energy_waste * 0.01f;

        // Terminal Reward
        if (height_y <= 0.451f) {
            done = true;
            
            bool inside_x = (pos_x >= -0.2f && pos_x <= 0.2f);
            bool inside_z = (pos_z >= -0.15f && pos_z <= 0.15f);
            
            if (inside_x && inside_z) {
                reward += 10.0f; // Strike
            } else {
                reward -= 5.0f;  // Crash
            }
        }

        return std::make_tuple(new_state, reward, done);
    }
};

PYBIND11_MODULE(mio_simulatore, m) {
    py::class_<AmbienteRobot>(m, "AmbienteRobot")
        .def(py::init<>())
        .def("reset", &AmbienteRobot::reset)
        .def("step", &AmbienteRobot::step);
}