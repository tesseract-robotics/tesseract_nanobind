// Audit contract fixture (PE-C4): no binding includes it, and its only definitions
// specialise yaml-cpp's `YAML::convert`, a foreign namespace: no `header` row.
#pragma once

namespace tesseract::fixture
{
struct Plain;  // forward declaration only: not auditable
}  // namespace tesseract::fixture

namespace YAML
{
template <class T>
struct convert;  // yaml-cpp's primary template, declared here so the fixture needs no yaml-cpp

template <>
struct convert<tesseract::fixture::Plain>
{
  static bool decode(int node, tesseract::fixture::Plain& rhs);
};
}  // namespace YAML
