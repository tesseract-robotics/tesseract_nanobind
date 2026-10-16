// Audit contract fixture: members inherited from an unbound base (A).
#pragma once

namespace tesseract::inherit
{
struct Shape  // unbound base: one class row; its public members are inherited rows (A1)
{
  virtual ~Shape() = default;
  bool operator==(const Shape& rhs) const;
  bool operator!=(const Shape& rhs) const;
  int sides() const;
  double area = 0;
  // Filtered as in the base's own record: never inherited (A9).
  Shape& operator=(const Shape&) = default;
  void reset() = delete;
  [[deprecated]] int legacy() const;
  template <class A>
  void serialize(A& ar, unsigned version);

protected:
  bool equals(const Shape& other) const;  // not public: never inherited (A5)
};

struct Square final : Shape  // stub binds none of Shape's API: 4 inherited gaps (A1)
{
  Square();
};

struct Circle final : Shape  // own operator== hides Shape::operator== only (A2)
{
  Circle();
  bool operator==(const Circle& rhs) const;
};

struct Hexagon final : Shape  // stub binds all of it: covered, no rows (A3)
{
  Hexagon();
};

struct Hidden : private Shape  // private base: nothing inherited (A4)
{
  Hidden();
};

struct Exposer : Shape  // a using-declaration hides nothing: unbound area stays a gap (A10)
{
  using Shape::area;
};

struct Runner  // bound base
{
  virtual ~Runner() = default;
  virtual void go() = 0;
  int laps() const;
};

struct SlowRunner final : Runner  // laps() is not a row: the bound Runner audits it (A6)
{
  SlowRunner();
  void go() override;
};

struct Middle : Runner  // unbound intermediate
{
  int depth() const;
};

struct DeepRunner final : Middle  // depth() inherited from Middle, laps() through it (A7)
{
  DeepRunner();
  void go() override;
};

struct Relay final : Runner  // re-binds Runner's API on the derived class (A8)
{
  Relay();
  void go() override;
};

struct Remote  // bound in another extension's stub (A11)
{
  int ping() const;
};

struct Courier final : Remote  // ping() is not a row: the other module audits it (A11)
{
  Courier();
};
}  // namespace tesseract::inherit
