// Audit contract fixture TU: gadget.h is already pulled in by widget.h, and still audited (C1).
#include <tesseract/fixture/widget.h>
#include <tesseract/fixture/gadget.h>
// Third-party, included directly anyway: never audited (THIRD_PARTY_HEADERS).
#include <tesseract/fixture/vhacd/VHACD.h>
