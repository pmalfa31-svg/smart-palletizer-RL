#pragma once
#include <btBulletDynamicsCommon.h>
#include <vector>

// Euristica di bin-packing 3D basata su heightmap (imparentata con
// "Deepest Bottom-Left" / Extreme-Point packing, letteratura classica sul
// 3D bin packing — non e' un algoritmo inventato ad hoc).
//
// Responsabilita' netta: questo planner decide SOLO la posizione (x,z) e
// l'orientamento migliori per il prossimo pacco, ottimizzando altezza
// risultante (spazio vuoto) e planarita' dell'appoggio (stabilita'). Il
// COME ci arriva il braccio (RL) e' un problema completamente separato:
// vedi la discussione di architettura nel README.
struct PlacementResult {
    float x, z;       // centro del pacco sul pallet (coordinate mondo)
    float restHeight;  // quota su cui il pacco andra' ad appoggiare
    bool rotated90;     // orientamento scelto
    float stabilityScore; // 0 (instabile, appoggio su spigoli) - 1 (perfettamente piano)
};

class PalletPlanner {
public:
    PalletPlanner(float palletWidth, float palletDepth, float cellSize,
                   btVector3 palletOrigin);

    // halfExtents del pacco (solo x,z contano per il footprint; y e' l'altezza).
    PlacementResult findPlacement(const btVector3& halfExtents) const;

    // Da chiamare DOPO che il braccio ha davvero posato il pacco nella
    // posizione suggerita, per aggiornare la heightmap.
    void commitPlacement(const PlacementResult& placement, const btVector3& halfExtents);

    void reset();

private:
    float width, depth, cellSize;
    btVector3 origin;
    int cols, rows;
    std::vector<float> heightmap; // row-major, altezza corrente per cella

    float maxHeightInFootprint(int gx, int gz, int fw, int fd) const;
    float heightVarianceInFootprint(int gx, int gz, int fw, int fd) const;
    void setHeightInFootprint(int gx, int gz, int fw, int fd, float h);
};
