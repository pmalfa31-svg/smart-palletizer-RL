#pragma once
#include <btBulletDynamicsCommon.h>
#include "Package.h"

class RoboticArm {
private:
    btDiscreteDynamicsWorld* world;
    
    btRigidBody* basePillarBody;
    btRigidBody* bicepBody;
    btRigidBody* forearmBody;
    
    btHingeConstraint* shoulderJoint;
    btHingeConstraint* elbowJoint;
    btFixedConstraint* vacuumConstraint;

public:
    RoboticArm(btDiscreteDynamicsWorld* dynamicsWorld);
    ~RoboticArm();

    void reset();
    void applyMotorVelocities(float velSpalla, float velGomito);
    void handleVacuum(bool grip_command, Package* targetBox);

    float getElbowX();
    float getElbowY();
    float getEndEffectorX();
    float getEndEffectorY();
    float getShoulderAngle();
    float getElbowAngle();
    bool isHolding();
};