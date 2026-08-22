#pragma once
#include <BulletDynamics/Featherstone/btMultiBody.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointMotor.h>
#include <string>
#include <vector>
#include <utility>
#include "../core/PhysicsWorld.h"

struct JointSpec {
    std::string name;
    std::string jointType = "revolute";
    btVector3 axis;
    btQuaternion rotParentToThis = btQuaternion::getIdentity();
    btVector3 pivotInParent;
    btVector3 pivotInChild;
    float lowerLimit = 1.0f;
    float upperLimit = -1.0f;
    float maxMotorForce = 200.0f;
    float linkMass = 1.0f;
    btVector3 linkInertia;
    std::vector<std::vector<btVector3>> convexHulls;
};

class ArticulatedArm {
public:
    ArticulatedArm(PhysicsWorld& world, const std::string& baseName,
                    const btVector3& basePosition);
    ~ArticulatedArm();

    void addLink(const JointSpec& spec);
    void finalizeBuild();

    void setJointTargetVelocity(int jointIndex, float velocity);
    void setJointTargetPosition(int jointIndex, float position, float kp = 0.3f);
    void reset();

    btTransform getEndEffectorTransform() const;
    std::pair<btVector3, btQuaternion> getEndEffectorPose() const;
    int numLinks() const { return static_cast<int>(links.size()); }
    std::vector<float> getJointPositions() const;

private:
    PhysicsWorld& physicsWorld;
    btMultiBody* multiBody = nullptr;
    std::vector<JointSpec> links;
    std::vector<btMultiBodyLinkCollider*> colliders;
    std::vector<btMultiBodyJointMotor*> motors;
    std::vector<btMultiBodyConstraint*> ownedConstraints;
    btVector3 basePos;
    bool built = false;
};
