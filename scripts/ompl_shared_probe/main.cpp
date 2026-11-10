#include <iostream>
extern "C" void set_seed();
extern "C" unsigned read_seed();
int main() {
    set_seed();
    const auto seed = read_seed();
    std::cout << "Seed read from second DLL: " << seed << '\n';
    return seed == 1 ? 0 : 1;
}
