#include "PhysicsWorld.h"

PhysicsWorld::PhysicsWorld() {
    collisionConfig = std::make_unique<btDefaultCollisionConfiguration>();
    dispatcher = std::make_unique<btCollisionDispatcher>(collisionConfig.get());
    broadphase = std::make_unique<btDbvtBroadphase>();
    solver = std::make_unique<btMultiBodyConstraintSolver>();

    world = std::make_unique<btMultiBodyDynamicsWorld>(
        dispatcher.get(), broadphase.get(), solver.get(), collisionConfig.get());
    world->setGravity(btVector3(0, -9.81f, 0));
}

PhysicsWorld::~PhysicsWorld() {
    if (groundBody) {
        world->removeRigidBody(groundBody);
        delete groundBody->getMotionState();
        delete groundBody;
    }
    delete groundShape;
}

void PhysicsWorld::stepSimulation(float dt, int maxSubSteps) {
    world->stepSimulation(dt, maxSubSteps);
}

void PhysicsWorld::addGroundPlane(float y) {
    groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), y);
    btTransform t;
    t.setIdentity();
    btDefaultMotionState* motionState = new btDefaultMotionState(t);
    btRigidBody::btRigidBodyConstructionInfo info(0.0f, motionState, groundShape, btVector3(0, 0, 0));
    groundBody = new btRigidBody(info);
    world->addRigidBody(groundBody);
}
