// GIL audit contract fixture (scripts/audit_gil.py): a part of the `fixture` module, its own TU
// linked beside gil_bindings.cpp (as common_logging.cpp beside common.cpp). Its releases are the
// module's. Also the template forms the real parts and modules use: a `.def` on a receiver the
// caller passes, as a parameter (motion_planners_simple's bind_instruction_info) or a member of
// one (motion_planners_descartes' bind_descartes), and a released `nb::init`.
#include <nanobind/nanobind.h>

namespace nb = nanobind;

namespace fixture_part {
struct Sink {
    explicit Sink(int n) : n(n) {}
    int flush(int m) const { return n + m; }
    int n;
};
struct InfoA {};
struct InfoB {};
template <typename T>
struct Box {
    T fill(T v) const { return v; }
};
}  // namespace fixture_part

namespace {
// A receiver parameter: named per call site.
template <typename Info>
void bind_info(nb::class_<Info>& cls) {
    cls.def("probe", [](const Info&, int n) { return n; }, nb::call_guard<nb::gil_scoped_release>());
}

// A receiver that is a member of a parameter, behind a chain of `.def` calls.
template <typename T>
struct BoxClasses {
    nb::class_<fixture_part::Box<T>> box;
};

template <typename T>
void bind_boxes(BoxClasses<T>& c) {
    c.box.def(nb::init<>())
        .def("fill", &fixture_part::Box<T>::fill, nb::call_guard<nb::gil_scoped_release>());
}
}  // namespace

void bind_part(nb::module_& m) {
    nb::class_<fixture_part::Sink>(m, "Sink")
        .def(nb::init<int>(), nb::call_guard<nb::gil_scoped_release>())
        .def("flush", &fixture_part::Sink::flush, nb::call_guard<nb::gil_scoped_release>());
    m.def("drain", [](int n) {
        nb::gil_scoped_release release;
        return n;
    });

    nb::class_<fixture_part::InfoA> info_a(m, "InfoA");
    bind_info<fixture_part::InfoA>(info_a);
    nb::class_<fixture_part::InfoB> info_b(m, "InfoB");
    bind_info(info_b);

    using D = BoxClasses<double>;
    D d{nb::class_<fixture_part::Box<double>>(m, "BoxD")};
    bind_boxes(d);
    using F = BoxClasses<float>;
    F f{nb::class_<fixture_part::Box<float>>(m, "BoxF")};
    bind_boxes(f);
}
