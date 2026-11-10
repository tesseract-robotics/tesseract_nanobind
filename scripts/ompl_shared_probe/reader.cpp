#include <ompl/util/RandomNumbers.h>
extern "C" unsigned read_seed() { return static_cast<unsigned>(ompl::RNG::getSeed()); }
