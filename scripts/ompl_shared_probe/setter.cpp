#include <ompl/util/RandomNumbers.h>
extern "C" void set_seed() { ompl::RNG::setSeed(1); }
