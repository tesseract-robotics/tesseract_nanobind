// Audit contract fixture: a part of the `fixture` module, its own TU linked beside
// fixture_bindings.cpp (as environment_commands.cpp beside environment.cpp). It #includes
// shared.h, which the primary does not: the module then owns and audits the header.
#include <tesseract/fixture/shared.h>
