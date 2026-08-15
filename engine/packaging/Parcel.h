#pragma once
#include <btBulletDynamicsCommon.h>

// Le 3 taglie non sono uno standard Amazon "ufficiale" univoco (non esiste:
// Amazon usa decine di formati carton diversi per fulfillment center) —
// sono taglie rappresentative S/M/L dichiarate esplicitamente come
// semplificazione di progetto. Dimensioni in metri, masse plausibili per
// pacco imballato (non il solo cartone).
enum class ParcelSize { SMALL, MEDIUM, LARGE };

struct ParcelDims {
    btVector3 halfExtents; // m
    float mass;            // kg
};

ParcelDims getParcelDims(ParcelSize size);

class Parcel {
public:
    Parcel(btDiscreteDynamicsWorld* world, ParcelSize size, const btVector3& spawnPos);
    ~Parcel();

    void resetPosition(const btVector3& pos);
    btVector3 getPosition() const;
    ParcelSize getSize() const { return size; }
    btVector3 getHalfExtents() const { return dims.halfExtents; }
    btRigidBody* getBody() const { return body; }

private:
    btDiscreteDynamicsWorld* world;
    btRigidBody* body = nullptr;
    btCollisionShape* shape = nullptr;
    ParcelSize size;
    ParcelDims dims;
};
