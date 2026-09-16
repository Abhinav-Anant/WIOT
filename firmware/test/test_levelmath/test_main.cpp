// Host-side tests for the math that decides "how full is the tank" and
// "is water flowing". Get these wrong and you either flood a 28,000 L sump
// or never detect the municipal supply at all.
//
//   pio test -e native

#include <unity.h>
#include "levelmath.h"

using namespace wiot;

// Real geometry: sensor 35cm above full water, 300cm above the floor.
static const uint16_t EMPTY_CM = 300;
static const uint16_t FULL_CM  = 35;

void test_percent_at_extremes() {
  // Far echo = empty tank. Near echo = full tank. An inverted mapping here
  // is the bug that keeps the pump running into an already-full tank.
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f,   levelPercent(EMPTY_CM, EMPTY_CM, FULL_CM));
  TEST_ASSERT_FLOAT_WITHIN(0.1f, 100.0f, levelPercent(FULL_CM,  EMPTY_CM, FULL_CM));
}

void test_percent_midpoint() {
  // span = 265cm; 167cm of air means 133cm of that span is water.
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 50.2f, levelPercent(167, EMPTY_CM, FULL_CM));
}

void test_percent_clamps() {
  TEST_ASSERT_EQUAL_FLOAT(0.0f,   levelPercent(500, EMPTY_CM, FULL_CM));  // below floor
  TEST_ASSERT_EQUAL_FLOAT(100.0f, levelPercent(10,  EMPTY_CM, FULL_CM));  // inside blind zone
}

void test_percent_survives_bad_config() {
  // Someone sets both distances the same over MQTT. Must not divide by zero.
  float p = levelPercent(100, 50, 50);
  TEST_ASSERT_TRUE(p >= 0.0f && p <= 100.0f);
}

void test_litres() {
  TEST_ASSERT_EQUAL_UINT32(0,     litresFromPercent(0.0f,   28317));
  TEST_ASSERT_EQUAL_UINT32(28317, litresFromPercent(100.0f, 28317));
  TEST_ASSERT_UINT32_WITHIN(5, 14158, litresFromPercent(50.0f, 28317));
}

void test_flow() {
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 60.0f, lpmFromPulses(135, 135.0f));
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f,  lpmFromPulses(0,   135.0f));
  // A zero K-factor pushed over MQTT must not produce inf and fake a flood of water.
  TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f,  lpmFromPulses(135, 0.0f));
}

void test_median_rejects_side_echo() {
  uint16_t s[5] = {100, 102, 101, 300, 99};   // 300 = echo off the sump wall
  TEST_ASSERT_EQUAL_UINT16(101, medianOf5(s));
}

void test_median_tolerates_two_failed_reads() {
  uint16_t s[5] = {0, 101, 0, 100, 102};      // 0 = no echo returned
  TEST_ASSERT_EQUAL_UINT16(100, medianOf5(s));
}

void test_median_reports_failure_when_most_reads_fail() {
  uint16_t s[5] = {0, 0, 0, 100, 102};
  TEST_ASSERT_EQUAL_UINT16(0, medianOf5(s));  // caller treats 0 as sensor failure
}

void test_valid_distance_rejects_blind_zone() {
  TEST_ASSERT_FALSE(validDistance(15,  20, 600));   // inside AJ-SR04M blind zone
  TEST_ASSERT_FALSE(validDistance(700, 20, 600));   // beyond range
  TEST_ASSERT_TRUE (validDistance(167, 20, 600));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_percent_at_extremes);
  RUN_TEST(test_percent_midpoint);
  RUN_TEST(test_percent_clamps);
  RUN_TEST(test_percent_survives_bad_config);
  RUN_TEST(test_litres);
  RUN_TEST(test_flow);
  RUN_TEST(test_median_rejects_side_echo);
  RUN_TEST(test_median_tolerates_two_failed_reads);
  RUN_TEST(test_median_reports_failure_when_most_reads_fail);
  RUN_TEST(test_valid_distance_rejects_blind_zone);
  return UNITY_END();
}
