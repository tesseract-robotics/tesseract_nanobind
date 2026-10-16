// Audit contract fixture: inherited members (A) and stub base classes (T5).
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

struct Runner  // bound base
{
  virtual ~Runner() = default;
  virtual void go() = 0;
  int laps() const;
};

struct SlowRunner final : Runner  // stub omits the Runner base: T5 gap; laps() is not an A row (A6)
{
  SlowRunner();
  void go() override;
};

struct Middle : Runner  // unbound intermediate
{
  int depth() const;
};

struct DeepRunner final : Middle  // stub base Runner = nearest bound ancestor: no T5 row (T5-2); depth() inherited (A7)
{
  DeepRunner();
  void go() override;
};

struct Relay final : Runner  // re-binds Runner's API on the derived class (A8)
{
  Relay();
  void go() override;
};

struct Loner  // stub claims a base C++ does not have: T5 deviation (T5-3)
{
  Loner();
};
}  // namespace tesseract::inherit
