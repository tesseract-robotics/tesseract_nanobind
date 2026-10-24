// GIL audit contract fixture (scripts/audit_gil.py): one binding per way the real TUs release
// the GIL, and the cases that must not count as a release.
#include <nanobind/nanobind.h>
#include <nanobind/operators.h>

#include <vector>

namespace nb = nanobind;
using namespace nb::literals;

namespace fixture {
struct Planner {
    int solve(int request) const { return request; }
    int name() const { return 0; }
    bool operator==(const Planner&) const { return true; }
    bool operator!=(const Planner&) const { return false; }
};
struct Manager {
    int contactTest(int n) const { return n; }
};
}  // namespace fixture

namespace {
// Released in a function template the binding lambda calls (environment's check_trajectory).
template <typename Manager>
int check(const Manager& manager, int n) {
    nb::gil_scoped_release release;
    return manager.contactTest(n);
}

// Released in a free function bound by pointer (check_trajectory_segment).
int segment(const fixture::Manager& manager, int n) {
    nb::gil_scoped_release release;
    return manager.contactTest(n);
}

// Takes the GIL, never releases it: a callback wrapper (PyEventCallbackFn).
int with_gil() {
    nb::gil_scoped_acquire acquire;
    return 0;
}

// Binds a module function under the name it is given, with the call_guard (geometry's
// def_create_mesh_from_path): the release belongs to each name passed at a call site.
template <typename T>
void def_loader(nb::module_& m, const char* name, const char* doc) {
    m.def(name, [](int n) { return T(n); }, "n"_a, nb::call_guard<nb::gil_scoped_release>(), doc);
}

// The same helper shape without a release.
template <typename T>
void def_plain_loader(nb::module_& m, const char* name) {
    m.def(name, [](int n) { return T(n); }, "n"_a);
}

template <typename Class, typename... Extra>
void bind_equality(Class& cls, const Extra&... extra) {
    cls.def(nb::self == nb::self, extra...).def(nb::self != nb::self, extra...);
}
}  // namespace

NB_MODULE(_gil_fixture, m) {
    // A local lambda bound by name (kinematics' inv_kin_from_list).
    auto solve_twice = [](const fixture::Planner& self, int request) {
        nb::gil_scoped_release release;
        return self.solve(request) * 2;
    };

    auto planner = nb::class_<fixture::Planner>(m, "Planner")
        .def(nb::init<>())
        // call_guard on a member pointer.
        .def("solve", &fixture::Planner::solve, "request"_a, nb::call_guard<nb::gil_scoped_release>())
        // A getter: no release.
        .def("name", &fixture::Planner::name)
        // In-body release in the bound lambda.
        .def("solveInBody", [](const fixture::Planner& self, int request) {
            nb::gil_scoped_release release;
            return self.solve(request);
        }, "request"_a)
        .def("solveTwice", solve_twice, "request"_a)
        // An acquire is not a release.
        .def("callback", [](const fixture::Planner&) { return with_gil(); })
        // A release inside a local struct's method is that struct's, not this binding's.
        .def("localStruct", [](const fixture::Planner&) {
            struct Inner {
                int run() const {
                    nb::gil_scoped_release release;
                    return 1;
                }
            };
            return 0;
        });
    bind_equality(planner, nb::call_guard<nb::gil_scoped_release>());

    nb::class_<fixture::Manager>(m, "Manager")
        .def(nb::init<>())
        .def("contactTest", [](const fixture::Manager& self, int n) { return check(self, n); }, "n"_a);

    m.def("segment", &segment, "manager"_a, "n"_a);
    m.def("plain", [](int n) { return n; }, "n"_a);
    def_loader<int>(m, "loadInt", "An int.");
    def_loader<long>(m, "loadLong", "A long.");
    def_plain_loader<int>(m, "loadPlain");
}
