#include "Parcel.h"

ParcelDims getParcelDims(ParcelSize size) {
    switch (size) {
        case ParcelSize::SMALL:  return { btVector3(0.10f, 0.075f, 0.05f), 0.8f };
        case ParcelSize::MEDIUM: return { btVector3(0.15f, 0.125f, 0.10f), 2.5f };
        case ParcelSize::LARGE:  return { btVector3(0.20f, 0.175f, 0.15f), 6.0f };
    }
    return { btVector3(0.15f, 0.125f, 0.10f), 2.5f };
}

Parcel::Parcel(btDiscreteDynamicsWorld* w, ParcelSize s, const btVector3& spawnPos)
    : world(w), size(s), dims(getParcelDims(s)) {
    shape = new btBoxShape(dims.halfExtents);
    btVector3 inertia(0, 0, 0);
    shape->calculateLocalInertia(dims.mass, inertia);

    btTransform t;
    t.setIdentity();
    t.setOrigin(spawnPos);
    auto* motionState = new btDefaultMotionState(t);
    btRigidBody::btRigidBodyConstructionInfo info(dims.mass, motionState, shape, inertia);
    info.m_friction = 0.7f; // pacchi impilati: attrito realistico, non di default (0.5)
    body = new btRigidBody(info);
    body->setActivationState(DISABLE_DEACTIVATION);
    world->addRigidBody(body);
}

Parcel::~Parcel() {
    world->removeRigidBody(body);
    delete body->getMotionState();
    delete body;
    delete shape;
}

void Parcel::resetPosition(const btVector3& pos) {
    btTransform t;
    t.setIdentity();
    t.setOrigin(pos);
    body->setWorldTransform(t);
    body->getMotionState()->setWorldTransform(t);
    body->setLinearVelocity(btVector3(0, 0, 0));
    body->setAngularVelocity(btVector3(0, 0, 0));
    body->activate(true);
}

btVector3 Parcel::getPosition() const {
    return body->getWorldTransform().getOrigin();
}
