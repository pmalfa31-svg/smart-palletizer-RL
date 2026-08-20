#include "PhysicsWorld.h"

PhysicsWorld::PhysicsWorld() {
    collisionConfig = std::make_unique<btDefaultCollisionConfiguration>();
    dispatcher = std::make_unique<btCollisionDispatcher>(collisionConfig.get());
    broadphase = std::make_unique<btDbvtBroadphase>();
    solver = std::make_unique<btMultiBodyConstraintSolver>();

    world = std::make_unique<btMultiBodyDynamicsWorld>(
        dispatcher.get(), broadphase.get(), solver.get(), collisionConfig.get());
    world->setGravity(btVector3(0, -9.81f, 0));

    // NOTA: avevo alzato questo a 100 pensando che il default (10) fosse
    // insufficiente per far propagare la correzione dei motori lungo la
    // catena. Test empirico ha mostrato il CONTRARIO: con 100 iterazioni
    // anche i giunti che tenevano bene (shoulder_pan/lift) sono peggiorati
    // — piu' iterazioni possono amplificare un overshoot invece di
    // smorzarlo, se il motore e' gia' vicino al limite di stabilita'.
    // Tornato al default: il vero problema e' altrove (probabilmente il
    // gain "kd" di setVelocityTarget, non l'iterazione del solver).
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
