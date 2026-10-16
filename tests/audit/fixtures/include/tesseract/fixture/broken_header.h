// Audit contract fixture (#256): no binding includes it and it does not compile on its own
// (`undeclared_t`), so it is one `unparseable header` row and the other headers keep theirs.
#pragma once

namespace tesseract::fixture
{
undeclared_t brokenFactory();
}  // namespace tesseract::fixture
