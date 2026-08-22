#include "ArticulatedArm.h"
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>

ArticulatedArm::ArticulatedArm(PhysicsWorld& world, const std::string& baseName,
                                 const btVector3& basePosition)
    : physicsWorld(world), basePos(basePosition) {
    (void)baseName;
}

ArticulatedArm::~ArticulatedArm() {
    for (auto* c : ownedConstraints) {
        physicsWorld.raw()->removeMultiBodyConstraint(c);
        delete c;
    }
    if (multiBody) {
        physicsWorld.raw()->removeMultiBody(multiBody);
        delete multiBody;
    }
    for (auto* c : colliders) {
        physicsWorld.raw()->removeCollisionObject(c);
        delete c->getCollisionShape();
        delete c;
    }
}

void ArticulatedArm::addLink(const JointSpec& spec) {
    links.push_back(spec);
}

void ArticulatedArm::finalizeBuild() {
    if (built || links.empty()) return;

    const int n = static_cast<int>(links.size());
    multiBody = new btMultiBody(n, /*mass*/ 0.0f, btVector3(0, 0, 0),
                                 /*fixedBase*/ true, /*canSleep*/ false);
    multiBody->setBasePos(basePos);
    multiBody->setBaseWorldTransform(btTransform(btQuaternion::getIdentity(), basePos));

    for (int i = 0; i < n; ++i) {
        const JointSpec& s = links[i];
        int parentIndex = i - 1;
        btVector3 inertia = s.linkInertia.length2() > 0.0f
                                 ? s.linkInertia
                                 : btVector3(s.linkMass, s.linkMass, s.linkMass);

        if (s.jointType == "fixed") {
            multiBody->setupFixed(
                i, s.linkMass, inertia, parentIndex,
                s.rotParentToThis,
                s.pivotInParent,
                s.pivotInChild);
            motors.push_back(nullptr);
        } else {
            multiBody->setupRevolute(
                i, s.linkMass, inertia, parentIndex,
                s.rotParentToThis,
                s.axis,
                s.pivotInParent,
                s.pivotInChild,
                /*disableParentCollision*/ true);

            if (s.lowerLimit <= s.upperLimit) {
                auto* limit = new btMultiBodyJointLimitConstraint(multiBody, i, s.lowerLimit, s.upperLimit);
                physicsWorld.raw()->addMultiBodyConstraint(limit);
                ownedConstraints.push_back(limit);
            }

            auto* motor = new btMultiBodyJointMotor(multiBody, i, /*desiredVelocity*/ 0.0f, s.maxMotorForce);
            physicsWorld.raw()->addMultiBodyConstraint(motor);
            motors.push_back(motor);
            ownedConstraints.push_back(motor);
        }
    }

    multiBody->finalizeMultiDof();
    physicsWorld.raw()->addMultiBody(multiBody);

    for (int i = 0; i < n; ++i) {
        const JointSpec& s = links[i];
        btCompoundShape* compound = new btCompoundShape();
        for (const auto& hull : s.convexHulls) {
            auto* hullShape = new btConvexHullShape();
            for (const auto& v : hull) hullShape->addPoint(v, false);
            hullShape->recalcLocalAabb();
            btTransform id;
            id.setIdentity();
            compound->addChildShape(id, hullShape);
        }

        auto* collider = new btMultiBodyLinkCollider(multiBody, i);
        collider->setCollisionShape(compound);
        physicsWorld.raw()->addCollisionObject(
            collider, btBroadphaseProxy::DefaultFilter, btBroadphaseProxy::AllFilter);
        multiBody->getLink(i).m_collider = collider;
        colliders.push_back(collider);
    }

    built = true;
}

void ArticulatedArm::setJointTargetVelocity(int jointIndex, float velocity) {
    if (!multiBody || jointIndex < 0 || jointIndex >= static_cast<int>(motors.size())) return;
    if (motors[jointIndex] == nullptr) return;
    motors[jointIndex]->setVelocityTarget(velocity);
}

void ArticulatedArm::setJointTargetPosition(int jointIndex, float position, float kp) {
    if (!multiBody || jointIndex < 0 || jointIndex >= static_cast<int>(motors.size())) return;
    if (motors[jointIndex] == nullptr) return;
    motors[jointIndex]->setPositionTarget(position, kp);
}

void ArticulatedArm::reset() {
    if (!multiBody) return;
    for (int i = 0; i < multiBody->getNumLinks(); ++i) {
        multiBody->setJointPos(i, 0.0f);
        multiBody->setJointVel(i, 0.0f);
    }
    for (auto* m : motors) {
        if (m) m->setVelocityTarget(0.0f);
    }
    multiBody->setBasePos(basePos);
    multiBody->setBaseWorldTransform(btTransform(btQuaternion::getIdentity(), basePos));
    btAlignedObjectArray<btQuaternion> scratchQ;
    btAlignedObjectArray<btVector3> scratchM;
    multiBody->forwardKinematics(scratchQ, scratchM);
}

btTransform ArticulatedArm::getEndEffectorTransform() const {
    if (!multiBody || multiBody->getNumLinks() == 0) {
        return btTransform::getIdentity();
    }
    const int last = multiBody->getNumLinks() - 1;
    return multiBody->getLink(last).m_cachedWorldTransform;
}

std::pair<btVector3, btQuaternion> ArticulatedArm::getEndEffectorPose() const {
    btTransform t = getEndEffectorTransform();
    return { t.getOrigin(), t.getRotation() };
}

std::vector<float> ArticulatedArm::getJointPositions() const {
    std::vector<float> out;
    if (!multiBody) return out;
    for (int i = 0; i < multiBody->getNumLinks(); ++i) {
        out.push_back(multiBody->getJointPos(i));
    }
    return out;
}
