// Audit contract fixture (#256): a binding of the `tesseract/fixture/` module that #includes a
// header under the nested `tesseract/fixture/sub/` prefix directly. The nested prefix's module
// owns that header, so the parent never audits it, even as a direct include.
#include <tesseract/fixture/fwd.h>
#include <tesseract/fixture/sub/nested.h>
