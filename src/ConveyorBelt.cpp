#include "../include/ConveyorBelt.h"

ConveyorBelt::ConveyorBelt(btDiscreteDynamicsWorld* dynamicsWorld, float startX, float startY, float startZ) {
    world = dynamicsWorld;

    // Dimensions: 2 meters long, 0.2m thick, 0.5m wide
    btCollisionShape* shape = new btBoxShape(btVector3(2.0f, 0.1f, 0.25f)); 
    
    btTransform transform;
    transform.setIdentity();
    transform.setOrigin(btVector3(startX, startY, startZ)); 
    
    btDefaultMotionState* motionState = new btDefaultMotionState(transform);
    
    // Mass 0 means it's a static object (kinematic)
    btRigidBody::btRigidBodyConstructionInfo rbInfo(0.0f, motionState, shape, btVector3(0,0,0));
    
    body = new btRigidBody(rbInfo);
    
    // Add some friction so packages don't slide off uncontrollably
    body->setFriction(0.8f); 
    
    world->addRigidBody(body);
}

ConveyorBelt::~ConveyorBelt() {
    world->removeRigidBody(body);
    delete body->getMotionState();
    delete body->getCollisionShape();
    delete body;
}