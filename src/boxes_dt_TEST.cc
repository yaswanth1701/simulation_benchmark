#include <string.h>
#include "benchmark_boxes.hh"

using namespace gz;
using namespace benchmark;

const double g_dt_min = 1e-4;
const double g_dt_max = 1.01e-3;
const double g_dt_step = 1.0e-4;


INSTANTIATE_TEST_CASE_P(
    DartBoxes, BoxesTest,
    ::testing::Combine(::testing::Values("gz-physics-dartsim-plugin"), ::testing::Range(g_dt_min, g_dt_max, g_dt_step),
                       ::testing::Values(1), ::testing::Values(true), ::testing::Bool()));

INSTANTIATE_TEST_SUITE_P(
    BulletBoxes, BoxesTest,
    ::testing::Combine(::testing::Values("gz-physics-bullet-plugin"), ::testing::Range(g_dt_min, g_dt_max, g_dt_step),
                       ::testing::Values(1), ::testing::Values(true), ::testing::Bool()));


INSTANTIATE_TEST_SUITE_P(
    BulletFeatherstoneBoxes, BoxesTest,
    ::testing::Combine(::testing::Values("gz-physics-bullet-featherstone-plugin"), ::testing::Range(g_dt_min, g_dt_max, g_dt_step),
                       ::testing::Values(1), ::testing::Values(true), ::testing::Bool()));

////////////////////////////////////////////////
int main(int argc, char **argv) 
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}