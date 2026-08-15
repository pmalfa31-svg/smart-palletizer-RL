#pragma once
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>
#include <btBulletCollisionCommon.h>
#include <memory>

// Incapsula un mondo fisico Bullet basato su btMultiBodyDynamicsWorld.
//
// Perche' MultiBody e non i vecchi btRigidBody+btHingeConstraint (come nel
// il precedente approccio a catene di btHingeConstraint): un braccio a 6 assi e' una catena cinematica articolata,
// esattamente il caso d'uso per cui esiste l'algoritmo di Featherstone.
// E' piu' stabile numericamente, piu' veloce con catene lunghe, ed e' lo
// stesso approccio che usa PyBullet per importare robot via URDF.
class PhysicsWorld {
public:
    PhysicsWorld();
    ~PhysicsWorld();

    void stepSimulation(float dt, int maxSubSteps = 10);
    btMultiBodyDynamicsWorld* raw() { return world.get(); }

    // Aggiunge un piano statico (pavimento) a una certa quota Y.
    void addGroundPlane(float y = 0.0f);

private:
    std::unique_ptr<btDefaultCollisionConfiguration> collisionConfig;
    std::unique_ptr<btCollisionDispatcher> dispatcher;
    std::unique_ptr<btBroadphaseInterface> broadphase;
    std::unique_ptr<btMultiBodyConstraintSolver> solver;
    std::unique_ptr<btMultiBodyDynamicsWorld> world;

    btRigidBody* groundBody = nullptr;
    btCollisionShape* groundShape = nullptr;
};
