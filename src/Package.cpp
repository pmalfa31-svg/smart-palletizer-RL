#include "../include/Package.h"

Package::Package(btDiscreteDynamicsWorld* dynamicsWorld, float startX, float startY, float startZ) {
    world = dynamicsWorld;
    startPos = btVector3(startX, startY, startZ);

    btCollisionShape* shape = new btBoxShape(btVector3(0.15f, 0.15f, 0.15f)); 
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(startPos);
    
    btScalar mass(1.0f); 
    btVector3 inertia(0, 0, 0);
    shape->calculateLocalInertia(mass, inertia);
    
    btDefaultMotionState* motionState = new btDefaultMotionState(transform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motionState, shape, inertia);
    
    body = new btRigidBody(rbInfo);

    body->setActivationState(DISABLE_DEACTIVATION);
    world->addRigidBody(body);
}

Package::~Package() {
    world->removeRigidBody(body);
    delete body->getMotionState();
    delete body->getCollisionShape();
    delete body;
}

void Package::resetPosition(float newX, float newY, float newZ) {
    startPos = btVector3(newX, newY, newZ);
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(startPos);
    body->setWorldTransform(transform);
    body->getMotionState()->setWorldTransform(transform);
    body->setLinearVelocity(btVector3(0, 0, 0));
    body->setAngularVelocity(btVector3(0, 0, 0));
    body->clearForces();
}

float Package::getX() {
    btTransform trans;
    body->getMotionState()->getWorldTransform(trans);
    return trans.getOrigin().getX();
}

float Package::getY() {
    btTransform trans;
    body->getMotionState()->getWorldTransform(trans);
    return trans.getOrigin().getY();
}

btRigidBody* Package::getBody() { return body; }