#include "CircularConveyor.h"
#include <cmath>

CircularConveyor::CircularConveyor(btVector3 c, float r, float y, int n, float w)
    : center(c), radius(r), heightY(y), slots(n), angularSpeed(w) {}

void CircularConveyor::advance(float dt) {
    phase += angularSpeed * dt;
    const float twoPi = 6.28318530718f;
    if (phase > twoPi) phase -= twoPi;
    if (phase < 0.0f) phase += twoPi;
}

btVector3 CircularConveyor::slotPosition(int slotIndex) const {
    const float twoPi = 6.28318530718f;
    float angle = phase + (twoPi * slotIndex) / static_cast<float>(slots);
    float x = center.x() + radius * std::cos(angle);
    float z = center.z() + radius * std::sin(angle);
    return btVector3(x, heightY, z);
}

void CircularConveyor::driveParcel(Parcel* parcel, int slotIndex) {
    parcel->resetPosition(slotPosition(slotIndex));
}
