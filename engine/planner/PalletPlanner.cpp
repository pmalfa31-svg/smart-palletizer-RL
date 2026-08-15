#include "PalletPlanner.h"
#include <cmath>
#include <limits>

PalletPlanner::PalletPlanner(float w, float d, float cs, btVector3 o)
    : width(w), depth(d), cellSize(cs), origin(o) {
    cols = static_cast<int>(std::ceil(width / cellSize));
    rows = static_cast<int>(std::ceil(depth / cellSize));
    heightmap.assign(static_cast<size_t>(cols) * rows, 0.0f);
}

void PalletPlanner::reset() {
    std::fill(heightmap.begin(), heightmap.end(), 0.0f);
}

float PalletPlanner::maxHeightInFootprint(int gx, int gz, int fw, int fd) const {
    float maxH = 0.0f;
    for (int z = gz; z < gz + fd; ++z)
        for (int x = gx; x < gx + fw; ++x)
            maxH = std::max(maxH, heightmap[static_cast<size_t>(z) * cols + x]);
    return maxH;
}

float PalletPlanner::heightVarianceInFootprint(int gx, int gz, int fw, int fd) const {
    float sum = 0.0f, sumSq = 0.0f;
    int n = fw * fd;
    for (int z = gz; z < gz + fd; ++z)
        for (int x = gx; x < gx + fw; ++x) {
            float h = heightmap[static_cast<size_t>(z) * cols + x];
            sum += h;
            sumSq += h * h;
        }
    float mean = sum / n;
    return (sumSq / n) - (mean * mean);
}

void PalletPlanner::setHeightInFootprint(int gx, int gz, int fw, int fd, float h) {
    for (int z = gz; z < gz + fd; ++z)
        for (int x = gx; x < gx + fw; ++x)
            heightmap[static_cast<size_t>(z) * cols + x] = h;
}

PlacementResult PalletPlanner::findPlacement(const btVector3& halfExtents) const {
    // Prova entrambi gli orientamenti (0 e 90 gradi), scorre ogni posizione
    // valida della griglia, e sceglie quella che minimizza:
    //   1) l'altezza risultante (riempie prima gli spazi bassi/vuoti)
    //   2) la varianza dell'appoggio (preferisce superfici piane -> stabile)
    PlacementResult best{};
    float bestScore = std::numeric_limits<float>::max();
    bool found = false;

    struct Orientation { float fw, fd; bool rotated; };
    Orientation orientations[2] = {
        {halfExtents.x() * 2.0f, halfExtents.z() * 2.0f, false},
        {halfExtents.z() * 2.0f, halfExtents.x() * 2.0f, true}
    };

    for (const auto& o : orientations) {
        int fw = std::max(1, static_cast<int>(std::ceil(o.fw / cellSize)));
        int fd = std::max(1, static_cast<int>(std::ceil(o.fd / cellSize)));
        if (fw > cols || fd > rows) continue;

        for (int gz = 0; gz <= rows - fd; ++gz) {
            for (int gx = 0; gx <= cols - fw; ++gx) {
                float restH = maxHeightInFootprint(gx, gz, fw, fd);
                float variance = heightVarianceInFootprint(gx, gz, fw, fd);
                // Peso: l'altezza domina (riempie prima i buchi), la
                // varianza e' un tie-breaker per la stabilita'.
                float score = restH * 10.0f + variance;
                if (score < bestScore) {
                    bestScore = score;
                    found = true;
                    best.x = origin.x() + (gx + fw / 2.0f) * cellSize;
                    best.z = origin.z() + (gz + fd / 2.0f) * cellSize;
                    best.restHeight = restH;
                    best.rotated90 = o.rotated;
                    best.stabilityScore = 1.0f / (1.0f + variance);
                }
            }
        }
    }

    if (!found) {
        // Nessuna posizione valida (pacco piu' grande del pallet): fallback
        // al centro, altezza 0 — il chiamante dovrebbe considerarlo un errore.
        best.x = origin.x() + width / 2.0f;
        best.z = origin.z() + depth / 2.0f;
        best.restHeight = 0.0f;
        best.rotated90 = false;
        best.stabilityScore = 0.0f;
    }
    return best;
}

void PalletPlanner::commitPlacement(const PlacementResult& p, const btVector3& halfExtents) {
    float fwF = p.rotated90 ? halfExtents.z() * 2.0f : halfExtents.x() * 2.0f;
    float fdF = p.rotated90 ? halfExtents.x() * 2.0f : halfExtents.z() * 2.0f;
    int fw = std::max(1, static_cast<int>(std::ceil(fwF / cellSize)));
    int fd = std::max(1, static_cast<int>(std::ceil(fdF / cellSize)));

    int gx = static_cast<int>(std::round((p.x - origin.x()) / cellSize - fw / 2.0f));
    int gz = static_cast<int>(std::round((p.z - origin.z()) / cellSize - fd / 2.0f));
    gx = std::max(0, std::min(gx, cols - fw));
    gz = std::max(0, std::min(gz, rows - fd));

    float newHeight = p.restHeight + halfExtents.y() * 2.0f;
    setHeightInFootprint(gx, gz, fw, fd, newHeight);
}
