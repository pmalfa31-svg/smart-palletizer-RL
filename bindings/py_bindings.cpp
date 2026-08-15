#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "../engine/core/PhysicsWorld.h"
#include "../engine/packaging/Parcel.h"
#include "../engine/conveyor/CircularConveyor.h"
#include "../engine/planner/PalletPlanner.h"
// ArticulatedArm va esposto quando finalizziamo il caricamento URDF ->
// JointSpec lato Python (tools/urdf_parser.py + mesh_preprocess.py):
// serve prima decidere il formato dati di scambio (dict/dataclass -> JointSpec).

namespace py = pybind11;

// btVector3 non e' un tipo che pybind11 conosce di default: lo convertiamo
// da/verso una tupla Python (x, y, z), coerente con come lo usiamo nei
// test e nella config YAML (liste di 3 numeri).
namespace pybind11 { namespace detail {
template <> struct type_caster<btVector3> {
public:
    PYBIND11_TYPE_CASTER(btVector3, _("btVector3"));

    bool load(handle src, bool) {
        if (!py::isinstance<py::sequence>(src)) return false;
        auto seq = py::reinterpret_borrow<py::sequence>(src);
        if (seq.size() != 3) return false;
        value = btVector3(seq[0].cast<float>(), seq[1].cast<float>(), seq[2].cast<float>());
        return true;
    }

    static handle cast(const btVector3& src, return_value_policy, handle) {
        return py::make_tuple(src.x(), src.y(), src.z()).release();
    }
};
}} // namespace pybind11::detail

PYBIND11_MODULE(palletizer_core, m) {
    m.doc() = "Motore fisico C++ (Bullet MultiBody) del sistema di pallettizzazione";

    py::enum_<ParcelSize>(m, "ParcelSize")
        .value("SMALL", ParcelSize::SMALL)
        .value("MEDIUM", ParcelSize::MEDIUM)
        .value("LARGE", ParcelSize::LARGE);

    py::class_<PhysicsWorld>(m, "PhysicsWorld")
        .def(py::init<>())
        .def("step_simulation", &PhysicsWorld::stepSimulation,
             py::arg("dt"), py::arg("max_sub_steps") = 10)
        .def("add_ground_plane", &PhysicsWorld::addGroundPlane, py::arg("y") = 0.0f);

    py::class_<CircularConveyor>(m, "CircularConveyor")
        .def(py::init<btVector3, float, float, int, float>(),
             py::arg("center"), py::arg("radius"), py::arg("height_y"),
             py::arg("num_slots"), py::arg("angular_speed"))
        .def("advance", &CircularConveyor::advance)
        .def("slot_position", &CircularConveyor::slotPosition)
        .def("drive_parcel", &CircularConveyor::driveParcel)
        .def("num_slots", &CircularConveyor::numSlots);

    py::class_<PlacementResult>(m, "PlacementResult")
        .def_readwrite("x", &PlacementResult::x)
        .def_readwrite("z", &PlacementResult::z)
        .def_readwrite("rest_height", &PlacementResult::restHeight)
        .def_readwrite("rotated90", &PlacementResult::rotated90)
        .def_readwrite("stability_score", &PlacementResult::stabilityScore);

    py::class_<PalletPlanner>(m, "PalletPlanner")
        .def(py::init<float, float, float, btVector3>())
        .def("find_placement", &PalletPlanner::findPlacement)
        .def("commit_placement", &PalletPlanner::commitPlacement)
        .def("reset", &PalletPlanner::reset);

    // TODO prossima sessione: bind di Parcel e ArticulatedArm, e la classe
    // "PalletizerScene" che li compone tutti in un singolo step()/reset()
    // in stile Gym (equivalente della precedente classe monolitica AmbienteRobot, ma componendo
    // pezzi separati invece di un god-object).
}
