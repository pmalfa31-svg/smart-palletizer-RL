#include "../include/ConveyorBelt.h"

ConveyorBelt::ConveyorBelt(btDiscreteDynamicsWorld* dynamicsWorld, float startX, float startY, float startZ) {
    world = dynamicsWorld;
    beltSpeed = -0.5f; // Velocità negativa: spinge verso il robot (X=0)

    btCollisionShape* shape = new btBoxShape(btVector3(2.0f, 0.1f, 0.25f)); 
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(btVector3(startX, startY, startZ)); 
    btDefaultMotionState* motionState = new btDefaultMotionState(transform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(0.0f, motionState, shape, btVector3(0,0,0));
    body = new btRigidBody(rbInfo);
    body->setFriction(0.8f); 
    world->addRigidBody(body);
}

ConveyorBelt::~ConveyorBelt() {
    world->removeRigidBody(body);
    delete body->getMotionState();
    delete body->getCollisionShape();
    delete body;
}

void ConveyorBelt::setSpeed(float speed) { beltSpeed = speed; }

void ConveyorBelt::update(Package* targetBox) {
    btRigidBody* boxBody = targetBox->getBody();
    btTransform trans;
    boxBody->getMotionState()->getWorldTransform(trans);
    float boxY = trans.getOrigin().getY();

    // Se il pacco è fisicamente appoggiato sul nastro
    if (boxY < 0.2f) {
        btVector3 currentVel = boxBody->getLinearVelocity();
        // Override della sola velocità sull'asse X per simulare lo scorrimento
        boxBody->setLinearVelocity(btVector3(beltSpeed, currentVel.getY(), currentVel.getZ()));
    }
}