// Audit contract fixture: a plugin anchor header as upstream writes one: concrete plugin
// factories and the library's anchor, nothing else.
#pragma once

#include <boost_plugin_loader/macros.h>
#include <tesseract/anchor/plugin_factory.h>

namespace tesseract::anchor
{
class AnchorSolver;

class AnchorSolverFactory : public InvKinFactory
{
public:
  int create() const override;
};

PLUGIN_ANCHOR_DECL(AnchorFactoriesAnchor)
}  // namespace tesseract::anchor
