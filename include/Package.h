#pragma once
#include <btBulletDynamicsCommon.h>

class Package {
private:
    btRigidBody* body;
    btDiscreteDynamicsWorld* world;
    btVector3 startPos;

public:
    Package(btDiscreteDynamicsWorld* dynamicsWorld, float startX, float startY, float startZ);
    ~Package();

    void resetPosition(float newX, float newY, float newZ);
    float getX();
    float getY();
    btRigidBody* getBody();
};