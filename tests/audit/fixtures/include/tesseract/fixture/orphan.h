// Audit contract fixture (E0): no binding includes it, so it is one `header` gap row.
#pragma once

namespace tesseract::fixture
{
struct Orphan
{
  int value = 0;
};

int orphanHelper(int n);
}  // namespace tesseract::fixture
