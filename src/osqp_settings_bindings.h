#pragma once

// The OSQP settings both OSQP-backed classes expose (trajopt_sqp.OSQPEigenSolver and
// motion_planners_trajopt_ifopt.TrajOptIfoptOSQPSolverProfile): one definition, so the two cannot
// drift. Each setter forwards to OsqpEigen::Settings; each getter reads the OSQPSettings it
// wraps. The defaults are whoever built the settings' (setDefaultOSQPSettings in trajopt); the
// getters report them, so no docstring repeats a value.

#include "tesseract_nb.h"
#include <OsqpEigen/Settings.hpp>

// `settings(self)` returns the OsqpEigen::Settings& of a bound `self`.
template <typename Class, typename Settings>
void bind_osqp_settings(Class& cls, Settings settings) {
    using T = typename Class::Type;
    const auto raw = [settings](const T& self) -> const OSQPSettings& {
        return *settings(const_cast<T&>(self)).getSettings();
    };
    cls.def("setPolish", [settings](T& self, bool v) { settings(self).setPolish(v); }, "polish"_a,
            "Polish the ADMM solution (OSQP `polishing`).")
        .def("getPolish", [raw](const T& self) { return raw(self).polishing != 0; })
        .def("setWarmStart", [settings](T& self, bool v) { settings(self).setWarmStart(v); }, "warm_start"_a,
             "Warm-start each QP solve from the previous solution (OSQP `warm_starting`).")
        .def("getWarmStart", [raw](const T& self) { return raw(self).warm_starting != 0; })
        .def("setAdaptiveRho", [settings](T& self, bool v) { settings(self).setAdaptiveRho(v); }, "adaptive_rho"_a,
             "Adapt the ADMM step size rho (OSQP `adaptive_rho`).")
        .def("getAdaptiveRho", [raw](const T& self) { return raw(self).adaptive_rho != 0; })
        .def("setAdaptiveRhoInterval", [settings](T& self, int v) { settings(self).setAdaptiveRhoInterval(v); },
             "interval"_a,
             "Adapt rho every `interval` iterations. OSQP's 0 adapts on wall-clock timing, which makes "
             "solutions differ run to run; a fixed interval (e.g. 25) makes them reproducible.")
        .def("getAdaptiveRhoInterval", [raw](const T& self) { return static_cast<int>(raw(self).adaptive_rho_interval); })
        .def("setMaxIteration", [settings](T& self, int v) { settings(self).setMaxIteration(v); }, "max_iter"_a,
             "Maximum ADMM iterations per QP solve.")
        .def("getMaxIteration", [raw](const T& self) { return static_cast<int>(raw(self).max_iter); })
        .def("setAbsoluteTolerance", [settings](T& self, double v) { settings(self).setAbsoluteTolerance(v); },
             "abs_tol"_a, "Absolute convergence tolerance (OSQP `eps_abs`).")
        .def("getAbsoluteTolerance", [raw](const T& self) { return static_cast<double>(raw(self).eps_abs); })
        .def("setRelativeTolerance", [settings](T& self, double v) { settings(self).setRelativeTolerance(v); },
             "rel_tol"_a, "Relative convergence tolerance (OSQP `eps_rel`).")
        .def("getRelativeTolerance", [raw](const T& self) { return static_cast<double>(raw(self).eps_rel); })
        .def("setVerbosity", [settings](T& self, bool v) { settings(self).setVerbosity(v); }, "verbose"_a,
             "Write OSQP's progress to the console.")
        .def("getVerbosity", [raw](const T& self) { return raw(self).verbose != 0; });
}
