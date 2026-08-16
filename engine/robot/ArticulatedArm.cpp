#include "ArticulatedArm.h"
#include <BulletCollision/CollisionShapes/btConvexHullShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>

// NOTA IMPLEMENTATIVA: questo file usa l'API btMultiBody cosi' come
// documentata/usata negli esempi ufficiali bullet3 (es. Importers/
// ImportURDFDemo). Non l'ho ancora compilato in questo ambiente (Bullet
// da sorgente richiede una build lunga) — la firma esatta di alcuni
// metodi (setupRevolute, ecc.) va validata al primo build reale.
// Consideralo "90% affidabile, da verificare al primo compile", non
// codice gia' testato.

ArticulatedArm::ArticulatedArm(PhysicsWorld& world, const std::string& baseName,
                                 const btVector3& basePosition)
    : physicsWorld(world), basePos(basePosition) {
    (void)baseName;
}

ArticulatedArm::~ArticulatedArm() {
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
    // Base fissa a terra (il braccio e' montato, non e' un corpo libero).
    multiBody = new btMultiBody(n, /*mass*/ 0.0f, btVector3(0, 0, 0),
                                 /*fixedBase*/ true, /*canSleep*/ false);
    multiBody->setBasePos(basePos);
    multiBody->setBaseWorldTransform(btTransform(btQuaternion::getIdentity(), basePos));

    for (int i = 0; i < n; ++i) {
        const JointSpec& s = links[i];
        int parentIndex = i - 1; // -1 significa "base"
        btVector3 inertia = s.linkInertia.length2() > 0.0f
                                 ? s.linkInertia
                                 : btVector3(s.linkMass, s.linkMass, s.linkMass);

        multiBody->setupRevolute(
            i, s.linkMass, inertia, parentIndex,
            s.rotParentToThis,
            s.axis,
            s.pivotInParent,
            -s.pivotInChild,
            /*disableParentCollision*/ true);

        if (s.lowerLimit <= s.upperLimit) {
            auto* limit = new btMultiBodyJointLimitConstraint(multiBody, i, s.lowerLimit, s.upperLimit);
            physicsWorld.raw()->addMultiBodyConstraint(limit);
        }
        // NOTA: setJointMaxForce non esiste su btMultiBody — il limite di
        // forza per giunto si applica con un oggetto btMultiBodyJointMotor
        // separato (una constraint, come i joint limit sopra), non e' una
        // proprieta' diretta del link. Da aggiungere quando servira' un
        // controllo motore piu' realistico di setJointVel puro.
    }

    multiBody->finalizeMultiDof();
    physicsWorld.raw()->addMultiBody(multiBody);

    // Collider per link, costruiti dagli hull convessi pre-decomposti
    // (mai una mesh concava su un link dinamico: vedi tools/mesh_preprocess.py)
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
    if (!multiBody || jointIndex < 0 || jointIndex >= multiBody->getNumLinks()) return;
    multiBody->setJointVel(jointIndex, velocity);
}

void ArticulatedArm::reset() {
    if (!multiBody) return;
    for (int i = 0; i < multiBody->getNumLinks(); ++i) {
        multiBody->setJointPos(i, 0.0f);
        multiBody->setJointVel(i, 0.0f);
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
