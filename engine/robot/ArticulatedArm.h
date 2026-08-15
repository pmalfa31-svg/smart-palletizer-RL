#pragma once
#include <BulletDynamics/Featherstone/btMultiBody.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h>
#include <string>
#include <vector>
#include "../core/PhysicsWorld.h"

// Specifica di un singolo link/giunto, prodotta da tools/urdf_parser.py
// e passata dal binding Python. NON facciamo parsing XML in C++: il motore
// riceve solo numeri gia' pronti.
struct JointSpec {
    std::string name;
    btVector3 axis;              // asse di rotazione nel frame del parent
    btVector3 pivotInParent;     // offset del giunto nel frame del parent
    btVector3 pivotInChild;      // offset del giunto nel frame del child (di norma origine link)
    float lowerLimit = 1.0f;     // lowerLimit > upperLimit => giunto libero (convenzione Bullet)
    float upperLimit = -1.0f;
    float maxMotorForce = 200.0f;
    float linkMass = 1.0f;
    btVector3 linkInertia;       // diagonale del tensore d'inerzia (da mesh/CAD, non stimata a mano)
    // Collision shape del link: elenco di hull convessi pre-decomposti
    // (vedi tools/mesh_preprocess.py — V-HACD offline, mai mesh concava
    // su un corpo dinamico).
    std::vector<std::vector<btVector3>> convexHulls;
};

class ArticulatedArm {
public:
    ArticulatedArm(PhysicsWorld& world, const std::string& baseName,
                    const btVector3& basePosition);
    ~ArticulatedArm();

    // Aggiunge link in ordine catena cinematica (dalla base all'end-effector).
    void addLink(const JointSpec& spec);

    // Da chiamare una volta finita la catena: registra i collider nel mondo.
    void finalizeBuild();

    void setJointTargetVelocity(int jointIndex, float velocity);
    void reset();

    // Trasformazione mondo dell'end-effector (ultimo link aggiunto).
    btTransform getEndEffectorTransform() const;
    int numLinks() const { return static_cast<int>(links.size()); }

private:
    PhysicsWorld& physicsWorld;
    btMultiBody* multiBody = nullptr;
    std::vector<JointSpec> links;
    std::vector<btMultiBodyLinkCollider*> colliders;
    btVector3 basePos;
    bool built = false;
};
