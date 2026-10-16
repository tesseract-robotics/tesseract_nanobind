// Audit contract fixture (#256): compiles alone, and so does clash_b.h; together they redefine
// `Clash`, so a synthetic TU of both fails while each header parses on its own.
#pragma once

namespace tesseract::clash
{
struct Clash
{
  int a;
};
}  // namespace tesseract::clash
