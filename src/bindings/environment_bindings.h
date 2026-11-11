#pragma once

// The tesseract_environment module's translation units, one concern each; NB_MODULE(environment)
// in environment.cpp calls them in registration order.

#include "tesseract_nb.h"

// CommandType, Command and every concrete command class (environment_commands.cpp).
void bind_commands(nb::module_& m);
