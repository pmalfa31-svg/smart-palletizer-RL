#include "../include/RoboticArm.h"
#include <cmath>

RoboticArm::RoboticArm(btDiscreteDynamicsWorld* dynamicsWorld) : vacuumConstraint(nullptr) {
    world = dynamicsWorld;

    // 1. BASE PILLAR
    btCollisionShape* baseShape = new btBoxShape(btVector3(0.2f, 0.5f, 0.2f)); 
    btTransform baseTransform;
    baseTransform.setIdentity();
    baseTransform.setOrigin(btVector3(0.0f, 0.5f, 0.0f));
    btDefaultMotionState* baseMotionState = new btDefaultMotionState(baseTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfoBase(0.0f, baseMotionState, baseShape, btVector3(0,0,0));
    basePillarBody = new btRigidBody(rbInfoBase);
    world->addRigidBody(basePillarBody);

    // 2. BICEP
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
    world->addRigidBody(bicepBody);

    // 3. FOREARM
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
    world->addRigidBody(forearmBody);

    // 4. JOINTS
    btVector3 axisZ(0, 0, 1);
    btVector3 pivotInBase(0, 0.5f, 0);     
    btVector3 pivotInBicep(0, -0.5f, 0);   
    shoulderJoint = new btHingeConstraint(*basePillarBody, *bicepBody, pivotInBase, pivotInBicep, axisZ, axisZ);
    shoulderJoint->enableAngularMotor(true, 0.0f, 250.0f); 
    world->addConstraint(shoulderJoint, true);

    btVector3 pivotInBicepTop(0, 0.5f, 0);       
    btVector3 pivotInForearmBottom(0, -0.5f, 0); 
    elbowJoint = new btHingeConstraint(*bicepBody, *forearmBody, pivotInBicepTop, pivotInForearmBottom, axisZ, axisZ);
    elbowJoint->enableAngularMotor(true, 0.0f, 150.0f); 
    world->addConstraint(elbowJoint, true);
}

RoboticArm::~RoboticArm() {
    if (vacuumConstraint) {
        world->removeConstraint(vacuumConstraint);
        delete vacuumConstraint;
    }
    world->removeConstraint(elbowJoint);
    delete elbowJoint;
    world->removeConstraint(shoulderJoint);
    delete shoulderJoint;

    world->removeRigidBody(forearmBody);
    delete forearmBody->getMotionState();
    delete forearmBody->getCollisionShape();
    delete forearmBody;

    world->removeRigidBody(bicepBody);
    delete bicepBody->getMotionState();
    delete bicepBody->getCollisionShape();
    delete bicepBody;

    world->removeRigidBody(basePillarBody);
    delete basePillarBody->getMotionState();
    delete basePillarBody->getCollisionShape();
    delete basePillarBody;
}

void RoboticArm::reset() {
    if (vacuumConstraint) {
        world->removeConstraint(vacuumConstraint);
        delete vacuumConstraint;
        vacuumConstraint = nullptr;
    }

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
}

void RoboticArm::applyMotorVelocities(float velSpalla, float velGomito) {
    shoulderJoint->setMotorTargetVelocity(velSpalla);
    elbowJoint->setMotorTargetVelocity(velGomito);
}

void RoboticArm::handleVacuum(bool grip_command, Package* targetBox) {
    float eeX = getEndEffectorX();
    float eeY = getEndEffectorY();
    float boxX = targetBox->getX();
    float boxY = targetBox->getY();

    float dist = std::sqrt(std::pow(eeX - boxX, 2) + std::pow(eeY - boxY, 2));

    if (grip_command && !vacuumConstraint && dist < 0.2f) {
        btTransform frameInA = forearmBody->getWorldTransform().inverse() * targetBox->getBody()->getWorldTransform();
        btTransform frameInB;
        frameInB.setIdentity();
        vacuumConstraint = new btFixedConstraint(*forearmBody, *(targetBox->getBody()), frameInA, frameInB);
        world->addConstraint(vacuumConstraint, true);
    } else if (!grip_command && vacuumConstraint) {
        world->removeConstraint(vacuumConstraint);
        delete vacuumConstraint;
        vacuumConstraint = nullptr;
    }
}

float RoboticArm::getEndEffectorX() {
    btTransform eeLocalTrans;
    eeLocalTrans.setIdentity();
    eeLocalTrans.setOrigin(btVector3(0, 0.5f, 0)); 
    btTransform eeWorldTrans = forearmBody->getWorldTransform() * eeLocalTrans;
    return eeWorldTrans.getOrigin().getX();
}

float RoboticArm::getEndEffectorY() {
    btTransform eeLocalTrans;
    eeLocalTrans.setIdentity();
    eeLocalTrans.setOrigin(btVector3(0, 0.5f, 0)); 
    btTransform eeWorldTrans = forearmBody->getWorldTransform() * eeLocalTrans;
    return eeWorldTrans.getOrigin().getY();
}

float RoboticArm::getShoulderAngle() { return shoulderJoint->getHingeAngle(); }
float RoboticArm::getElbowAngle() { return elbowJoint->getHingeAngle(); }
bool RoboticArm::isHolding() { return vacuumConstraint != nullptr; }
float RoboticArm::getElbowX() {
    btTransform transform;
    // Se non usi i MotionState, puoi usare direttamente: 
    // transform = forearmBody->getWorldTransform();
    if (forearmBody && forearmBody->getMotionState()) {
        forearmBody->getMotionState()->getWorldTransform(transform);
    } else {
        transform = forearmBody->getWorldTransform();
    }
    return transform.getOrigin().getX();
}

float RoboticArm::getElbowY() {
    btTransform transform;
    if (forearmBody && forearmBody->getMotionState()) {
        forearmBody->getMotionState()->getWorldTransform(transform);
    } else {
        transform = forearmBody->getWorldTransform();
    }
    return transform.getOrigin().getY();
}