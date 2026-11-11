// Audit contract fixture: a plugin anchor header that also declares API, which listing it in
// PLUGIN_ANCHOR_INCLUDES must not hide.
#pragma once

#include <boost_plugin_loader/macros.h>
#include <tesseract/anchor/plugin_factory.h>

namespace tesseract::anchor
{
class LeakyFactory : public InvKinFactory
{
public:
  int create() const override;
};

double leakyHelper(int value);

class Leaky
{
public:
  int value{ 0 };
};

class LeakyMixedFactory : public InvKinFactory, public Leaky
{
public:
  int create() const override;
};

PLUGIN_ANCHOR_DECL(LeakyFactoriesAnchor)
}  // namespace tesseract::anchor
