// Audit contract fixture: every declaration exercises one matching rule.
#pragma once
#include <cstddef>
#include <cstdint>
#include <functional>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>  // third-party: never audited

#include <tesseract/fixture/gadget.h>  // first inclusion of gadget.h: the TU's own #include must still count (C1)

namespace tesseract::fixture
{
struct Base  // abstract: a non-const Base& is an in/out object, not an out-param (A3)
{
  virtual ~Base() = default;
  virtual void run() = 0;
};

struct Owner
{
};

struct Plain  // no declared constructor: implicit arity-0 __init__
{
  int x = 0;
};

class Widget
{
public:
  Widget();
  explicit Widget(int size);
  Widget(const Widget& other) = default;  // excluded: copy constructor
  ~Widget();                              // excluded: destructor

  int size() const;
  void resize(int n);                           // gap: no Python method
  bool operator==(const Widget& other) const;   // covered by __eq__
  Widget operator+(const Widget& other) const;  // gap: unmapped operator
  explicit operator bool() const;               // covered by __bool__ (A4)
  static void* operator new(std::size_t n);     // excluded: allocation operator (A4)
  Owner owner() const;                          // stub annotates it as a quoted C++ name
  [[deprecated]] void old();                    // excluded: deprecated

  template <class Archive>
  void serialize(Archive& ar);  // excluded: cereal hook

  int count = 0;  // field: covered by a property

private:
  int secret_ = 0;  // excluded: private
};

enum class Color
{
  RED,
  GREEN
};

// Unscoped: LEVEL_LOW is also a namespace-scope C++ name, yet a module constant
// aliasing Level.LEVEL_LOW is still a deviation (Note 1).
enum Level
{
  LEVEL_LOW,
  LEVEL_HIGH
};

struct Runner  // abstract and bound: no __init__ gap (I3); operator() is __call__ (I5)
{
  Runner() = default;
  virtual ~Runner() = default;
  virtual void go() = 0;
  virtual bool operator()(int n) const = 0;
};

struct FastRunner : Runner  // Python inherits go/__call__ from the bound Runner (I4)
{
  void go() override;
  bool operator()(int n) const override;
};

struct RemoteRunner : Runner  // stub base lives in another module's stub: it still resolves (I7)
{
  void go() override;
  bool operator()(int n) const override;
};

struct LostRunner : Runner  // stub base names a module with no stub: members stay gaps (I7)
{
  void go() override;
  bool operator()(int n) const override;
};

template <class Archive>
void serialize(Archive& ar, Owner& owner);  // excluded: free cereal hook (I2)

void flatten(std::vector<int>& out);  // void + one out-param: Python returns it directly (I6)

class Bag
{
public:
  std::size_t size() const;        // container-protocol: bound as __len__ (C14)
  int& operator[](std::size_t i);  // __getitem__; container-protocol: __setitem__ too (M18)
  const int* begin() const;        // iterator-pair: begin/end bound as __iter__ (C6)
  const int* end() const;
  template <typename T>
  Bag operator*(const T& factor) const;  // templated operator* bound as __mul__ (M13)
};

std::ostream& operator<<(std::ostream& os, const Bag& bag);  // stream-insertion: Bag.__str__ (M5)

class Record;
template <class Archive>
void serialize(Archive& ar, Record& obj);  // excluded: free cereal hook (I2)

class Record
{
public:
  Record();  // serialization-default-ctor: befriends serialize + another ctor, unbound (E2)
  explicit Record(int id);

private:
  int id_ = 0;
  template <class Archive>
  friend void ::tesseract::fixture::serialize(Archive& ar, Record& obj);
};

class Buffer  // mirrors BytesResource (#166)
{
public:
  Buffer(std::string url, std::vector<std::uint8_t> bytes, int parent = 0);  // bound: arity 2-3
  // byte-buffer pair: (const uint8_t*, size_t) is one Python `bytes`; the bound ctor covers it (#210)
  Buffer(std::string url, const std::uint8_t* bytes, std::size_t n, int parent = 0);
  // raw buffer: no Python positional form, so the arity-overlapping bound ctor cannot cover it (#166)
  Buffer(std::string url, const double* samples, int parent);
  template <class InputIt>
  Buffer(InputIt first, InputIt last);  // constructor template: keyed as __init__, not a method
  // a byte-buffer pair bound with a non-`bytes` Python parameter is not covered
  void write(const std::uint8_t* data, std::size_t n);
};

int scale(int value, double factor = 1.0);  // defaulted: arity 1-2
int scale(int value, double factor);        // redeclaration: must not add an overload
int area(int w);                            // Python adds an arity-2 overload: deviation
bool collect(std::vector<int>& out, Base& runner, int n);  // out-param: out only; arity 3 -> 2
void describe(std::stringstream& ss, int n);               // stringstream -> str; arity 2 -> 1
void fill(Eigen::Ref<Eigen::MatrixXd> out, int n);  // Eigen::Ref out-param, void: returned; arity 2 -> 1 (gh-213)
void shift(Eigen::Ref<Eigen::VectorXd> qs, int n);  // the same shape bound in place at arity 2: exact
double norm(const Eigen::Ref<const Eigen::MatrixXd>& m);  // const Ref: an input, not an out-param
double trace(Eigen::Ref<const Eigen::MatrixXd> m);        // by-value Ref of const: an input
}  // namespace tesseract::fixture

namespace std
{
template <>
struct hash<tesseract::fixture::Owner>  // std specialisation: not module API (I1)
{
  std::size_t operator()(const tesseract::fixture::Owner& owner) const;
};
}  // namespace std
