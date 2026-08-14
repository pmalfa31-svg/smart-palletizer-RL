#pragma once
#include <btBulletDynamicsCommon.h>

class LayerCompensator {
private:
    btDiscreteDynamicsWorld* world;
    btRigidBody* baseBody;
    btRigidBody* plateBody;
    btHingeConstraint* joint; 

public:
    LayerCompensator(btDiscreteDynamicsWorld* dynamicsWorld, float startX, float startY, float startZ);
    ~LayerCompensator();
    
    void reset();
};