// Audit contract fixture (#256): under the nested prefix `tesseract/fixture/sub/`, so its
// longest-prefix owner audits it, never the `tesseract/fixture/` module.
#pragma once

namespace tesseract::fixture::sub
{
struct Nested
{
  int x;
};
}  // namespace tesseract::fixture::sub
