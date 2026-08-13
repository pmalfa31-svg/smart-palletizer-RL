#pragma once
#include <btBulletDynamicsCommon.h>

class ConveyorBelt {
private:
    btRigidBody* body;
    btDiscreteDynamicsWorld* world;

public:
    ConveyorBelt(btDiscreteDynamicsWorld* dynamicsWorld, float startX, float startY, float startZ);
    ~ConveyorBelt();
    
    // Future methods will go here (e.g., setSpeed, turnOn, turnOff)
};