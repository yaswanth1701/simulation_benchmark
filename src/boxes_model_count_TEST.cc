#include <string.h>
#include "benchmark_boxes.hh"

using namespace gz;
using namespace benchmark;

const int g_models_min = 1;
const int g_models_max = 105;
const int g_models_step = 20;


INSTANTIATE_TEST_CASE_P(
    DartBoxes, BoxesTest,
    ::testing::Combine(::testing::Values("gz-physics-dartsim-plugin"),::testing::Values(5.0e-4),
                       ::testing::Range(g_models_min, g_models_max,g_models_step),
                       ::testing::Bool(), ::testing::Values(true)));

INSTANTIATE_TEST_SUITE_P(
    BulletBoxes, BoxesTest,
    ::testing::Combine(::testing::Values("gz-physics-bullet-plugin"), ::testing::Values(5.0e-4),
                       ::testing::Range(g_models_min, g_models_max,g_models_step),
                       ::testing::Bool(), ::testing::Values(true)));


INSTANTIATE_TEST_SUITE_P(
    BulletFeatherstoneBoxes, BoxesTest,
    ::testing::Combine(::testing::Values("gz-physics-bullet-featherstone-plugin"), ::testing::Values(5.0e-4),
                       ::testing::Range(g_models_min, g_models_max,g_models_step),
                       ::testing::Bool(), ::testing::Values(true)));

////////////////////////////////////////////////
int main(int argc, char **argv) 
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}