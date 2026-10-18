// Audit contract fixture: type-erased Polys (#256 group 1).
#pragma once
#include <cstddef>
#include <functional>
#include <memory>
#include <typeindex>
#include <vector>

namespace tesseract::poly
{
class ThingInterface  // unbound type-erasure interface (ACCEPTED_SYMBOLS: type-erasure-interface)
{
public:
  virtual ~ThingInterface() = default;
  virtual std::unique_ptr<ThingInterface> clone() const = 0;
};

class Square final : public ThingInterface  // a Poly alternative, bound with the copy protocol
{
public:
  Square() = default;
  std::unique_ptr<ThingInterface> clone() const override;  // clone-as-copy: covered
};

class Circle final : public ThingInterface  // a Poly alternative, bound without it
{
public:
  Circle() = default;
  std::unique_ptr<ThingInterface> clone() const override;  // gap
};

class ThingPoly
{
public:
  ThingPoly() = default;
  ThingPoly(const ThingInterface& impl);  // covered: Python passes the derived Square
  std::type_index getType() const;        // type-index
  ThingInterface& getThing();             // type-erasure-interface (ACCEPTED_SYMBOLS)
  template <typename T>
  T& as();  // template-instance-name: asSquare
  template <typename T>
  bool is() const;  // no instance listed: gap

private:
  std::unique_ptr<ThingInterface> impl_;
};

bool isSquare(const ThingPoly& thing);  // free-function-as-method: bound as ThingPoly.isSquare
bool isRound(const ThingPoly& thing);   // bound nowhere: gap
bool operator==(const Square& a, const Square& b);  // free-function-as-method: Square.__eq__
bool operator!=(const Circle& a, const Circle& b);  // Circle binds no __ne__: gap

class Box
{
public:
  void put(const ThingPoly& thing);  // stub: put(ThingPoly), put(Square): poly-alternative-overloads
  ThingPoly& getThing();             // stub adds setThing: reference-setter
};

class Things  // forwards the std::vector<int> interface
{
public:
  std::vector<int>::iterator begin();
  std::vector<int>::iterator end();
  std::size_t size() const;
  void push_back(const int& x);
  int* data();                                  // vector-forwarding-no-python-form
  std::vector<int>::reverse_iterator rbegin();  // vector-forwarding-no-python-form
  std::vector<int>::reverse_iterator rend();    // vector-forwarding-no-python-form
  void swap(std::vector<int>& other) noexcept;  // vector-forwarding-no-python-form
  template <typename... Args>
  int& emplace_back(Args&&... args);  // vector-forwarding-no-python-form

private:
  std::vector<int> container_;
};

class View
{
public:
  View(std::vector<std::reference_wrapper<Square>> squares);  // reference-wrapper-view
  View(const Box& box);                                        // covered sibling
};

class Lonely
{
public:
  Lonely(std::vector<std::reference_wrapper<Square>> squares);  // no covered sibling: gap
  Lonely(int n);                                                // gap
};

class Shelf
{
public:
  void place(const ThingPoly& thing);                    // covered
  void place(const ThingPoly& thing, int row, int col);  // ACCEPTED_CPP_OVERLOADS (fixture)
};
}  // namespace tesseract::poly
