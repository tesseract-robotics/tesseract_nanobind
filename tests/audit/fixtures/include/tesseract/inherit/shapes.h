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

struct Tile final : Shape  // stub binds none of Shape's API: 4 inherited gaps (A1)
{
  Tile();
};

struct Disc final : Shape  // own operator== hides Shape::operator== only (A2)
{
  Disc();
  bool operator==(const Disc& rhs) const;
};

struct Hexagon final : Shape  // stub binds all of it: covered, no rows (A3)
{
  Hexagon();
};

struct Concealed : private Shape  // private base: nothing inherited (A4)
{
  Concealed();
};

struct Exposer : Shape  // a using-declaration hides nothing: unbound area stays a gap (A10)
{
  using Shape::area;
};

struct Sprinter  // bound base
{
  virtual ~Sprinter() = default;
  virtual void go() = 0;
  int laps() const;
};

struct SlowSprinter final : Sprinter  // laps() is not a row: the bound Sprinter audits it (A6)
{
  SlowSprinter();
  void go() override;
};

struct Middle : Sprinter  // unbound intermediate
{
  int depth() const;
};

struct DeepSprinter final : Middle  // depth() inherited from Middle, laps() through it (A7)
{
  DeepSprinter();
  void go() override;
};

struct Relay final : Sprinter  // re-binds Sprinter's API on the derived class (A8)
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
