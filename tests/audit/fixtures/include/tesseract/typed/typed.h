// Audit contract fixture for overload matching by parameter type (#247).
#pragma once
#include <cstddef>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/uuid/uuid.hpp>

namespace tesseract::typed
{
struct Item
{
};
using Items = std::vector<Item>;  // bound as an opaque `Items` class (nb::bind_vector)

enum class Mode
{
  FAST,
  SAFE
};

class Seq
{
public:
  int& at(std::size_t i);  // const/non-const pair: one Python form covers both
  const int& at(std::size_t i) const;
  void push(const Item& item);  // `const T&`/`T&&` pair: one Python form covers both
  void push(Item&& item);
};

void pick(const std::string& name);                // bound
void pick(const std::vector<std::string>& names);  // same arity, unbound: a gap
void take(const Items& items);                     // bound under the typedef's opaque name
void take_all(const Items& items);                 // bound structurally, as Sequence[Item]
template <class T>
void put(const T& value);  // template parameter: no key, arity decides (a fallback)
void guess(int value);     // the stub annotates `object`: no key, arity decides (a fallback)

struct Command
{
  virtual ~Command() = default;
};
struct AddThing : Command
{
};
struct RemoveThing : AddThing  // ancestors are transitive
{
};
// Bound once per subclass, as Environment::applyCommand is: Python passes a derived class.
bool apply(std::shared_ptr<const Command> command);

struct Shape
{
};
struct Ball : Shape
{
};
// The stub takes the base: a Shape is no Ball, so this overload stays a gap.
void roll(const std::vector<std::shared_ptr<Ball>>& balls);

// One parameter per mapping rule; the stub spells each one as nanobind does.
void kinds(bool b,
           int i,
           std::size_t n,
           double x,
           std::string s,
           const char* c,
           const std::filesystem::path& p,
           const std::vector<std::string>& names,
           const std::unordered_map<std::string, double>& values,
           const std::map<std::string, Item>& items,
           const std::pair<std::string, std::string>& pair,
           std::shared_ptr<const Item> shared,
           std::optional<int> opt,
           const std::variant<std::string, Eigen::Isometry3d>& var,
           const Eigen::VectorXd& vec,
           const Eigen::Ref<const Eigen::VectorXd>& ref,
           const Eigen::VectorXi& ivec,
           const Eigen::Isometry3d& iso,
           Mode mode,
           std::function<void(int)> fn,
           const boost::uuids::uuid& id);
}  // namespace tesseract::typed
