#pragma once
#include <btBulletDynamicsCommon.h>
#include <vector>
#include "../packaging/Parcel.h"

// Nastro CIRCOLARE. Scelta di design deliberata: NON simuliamo attrito
// belt-pacco su una traiettoria curva (instabile: la direzione della
// velocita' tangenziale cambia ad ogni istante, con override discreto a
// 60Hz il pacco deriva verso l'esterno della curva). Invece: N "slot"
// cinematici equispaziati sul cerchio; un pacco non ancora afferrato viene
// posizionato direttamente nello slot che lo trasporta (stesso principio
// del precedente ConveyorBelt rettilineo — velocity/position override — esteso a una
// traiettoria curva dove e' la scelta piu' robusta, non un compromesso).
class CircularConveyor {
public:
    CircularConveyor(btVector3 center, float radius, float heightY,
                      int numSlots, float angularSpeedRadPerSec);

    // Avanza la fase del nastro. Da chiamare una volta per step, PRIMA
    // di stepSimulation, come nella versione precedente del nastro.
    void advance(float dt);

    // Posizione mondo dello slot i-esimo alla fase corrente.
    btVector3 slotPosition(int slotIndex) const;

    // Sposta il pacco nello slot assegnato (se non e' gia' stato afferrato
    // da un braccio — quello lo decide il chiamante, il conveyor non lo sa).
    void driveParcel(Parcel* parcel, int slotIndex);

    int numSlots() const { return slots; }
    float currentPhase() const { return phase; }

private:
    btVector3 center;
    float radius;
    float heightY;
    int slots;
    float angularSpeed;
    float phase = 0.0f; // radianti
};
