#pragma once
#include <btBulletDynamicsCommon.h>
#include "Package.h"

class ConveyorBelt {
private:
    btRigidBody* body;
    btDiscreteDynamicsWorld* world;
    float beltSpeed;

public:
    ConveyorBelt(btDiscreteDynamicsWorld* dynamicsWorld, float startX, float startY, float startZ);
    ~ConveyorBelt();
    
    void setSpeed(float speed);
    void update(Package* targetBox);
};