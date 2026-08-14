#include "../include/LayerCompensator.h"

LayerCompensator::LayerCompensator(btDiscreteDynamicsWorld* dynamicsWorld, float startX, float startY, float startZ) {
    world = dynamicsWorld;

    // 1. La Base (Statica)
    btCollisionShape* baseShape = new btBoxShape(btVector3(0.2f, 0.5f, 0.2f));
    btTransform baseTransform;
    baseTransform.setIdentity();
    baseTransform.setOrigin(btVector3(startX, startY, startZ));
    btDefaultMotionState* baseMotion = new btDefaultMotionState(baseTransform);
    btRigidBody::btRigidBodyConstructionInfo baseInfo(0.0f, baseMotion, baseShape, btVector3(0,0,0));
    baseBody = new btRigidBody(baseInfo);
    world->addRigidBody(baseBody);

    // 2. La Piastra per il Void Filling (Dinamica)
    btCollisionShape* plateShape = new btBoxShape(btVector3(0.4f, 0.05f, 0.4f));
    btTransform plateTransform;
    plateTransform.setIdentity();
    plateTransform.setOrigin(btVector3(startX, startY + 0.6f, startZ));
    
    btScalar plateMass = 2.0f;
    btVector3 plateInertia(0, 0, 0);
    plateShape->calculateLocalInertia(plateMass, plateInertia);
    
    btDefaultMotionState* plateMotion = new btDefaultMotionState(plateTransform);
    btRigidBody::btRigidBodyConstructionInfo plateInfo(plateMass, plateMotion, plateShape, plateInertia);
    plateBody = new btRigidBody(plateInfo);
    
    // Non facciamo addormentare la piastra (lezione imparata dal pacco!)
    plateBody->setActivationState(DISABLE_DEACTIVATION);
    world->addRigidBody(plateBody);

    // 3. Il Giunto (Collega base e piastra)
    btVector3 pivotInA(0, 0.5f, 0);
    btVector3 pivotInB(0, -0.05f, 0);
    btVector3 axisInA(0, 1, 0);
    btVector3 axisInB(0, 1, 0);
    joint = new btHingeConstraint(*baseBody, *plateBody, pivotInA, pivotInB, axisInA, axisInB);
    
    // Limiti di rotazione del giunto
    joint->setLimit(-1.57f, 1.57f);
    world->addConstraint(joint, true);
}

LayerCompensator::~LayerCompensator() {
    world->removeConstraint(joint);
    delete joint;
    
    world->removeRigidBody(plateBody);
    delete plateBody->getMotionState();
    delete plateBody->getCollisionShape();
    delete plateBody;

    world->removeRigidBody(baseBody);
    delete baseBody->getMotionState();
    delete baseBody->getCollisionShape();
    delete baseBody;
}

void LayerCompensator::reset() {
    // La logica per rimettere la piastra in posizione iniziale la scriveremo a breve
}