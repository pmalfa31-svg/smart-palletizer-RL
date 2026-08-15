"""
Test sul PalletPlanner (l'unico pezzo gia' completo e testabile senza aver
finito l'integrazione URDF/MultiBody). Vanno eseguiti dopo la build:
    cmake --build build
    pytest tests/
"""
import pytest

palletizer_core = pytest.importorskip(
    "palletizer_core", reason="modulo non compilato: esegui prima la build CMake"
)


def test_first_parcel_placed_at_origin_corner():
    planner = palletizer_core.PalletPlanner(1.2, 0.8, 0.05, (1.5, 0.0, 1.5))
    result = planner.find_placement((0.15, 0.125, 0.10))
    assert result.rest_height == pytest.approx(0.0)


def test_second_parcel_avoids_stacking_on_first_when_space_free():
    planner = palletizer_core.PalletPlanner(1.2, 0.8, 0.05, (1.5, 0.0, 1.5))
    half = (0.15, 0.125, 0.10)
    first = planner.find_placement(half)
    planner.commit_placement(first, half)

    second = planner.find_placement(half)
    # Con spazio libero sul pallet, il planner deve preferire un'area
    # scoperta piuttosto che impilare subito sul primo pacco.
    assert second.rest_height == pytest.approx(0.0)
    assert (second.x, second.z) != (first.x, first.z)


def test_stacking_when_pallet_footprint_is_full():
    planner = palletizer_core.PalletPlanner(0.3, 0.25, 0.05, (0.0, 0.0, 0.0))
    half = (0.15, 0.125, 0.10)
    first = planner.find_placement(half)
    planner.commit_placement(first, half)

    second = planner.find_placement(half)
    # Pallet minuscolo, un solo pacco ci sta: il secondo DEVE impilarsi.
    assert second.rest_height > 0.0
