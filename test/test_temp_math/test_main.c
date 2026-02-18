#include <math.h>
#include <stdint.h>
#include <unity.h>

#include "temp_math.h"

void setUp(void) {}
void tearDown(void) {}

void test_mlx_raw_to_celsius_zero_kelvin(void)
{
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -273.15f, mlx_raw_to_celsius(0));
}

void test_mlx_raw_to_celsius_body_temp(void)
{
    // 37.00C => raw = (37 + 273.15) / 0.02 = 15507.5, nearest integer = 15508
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 37.00f, mlx_raw_to_celsius(15508));
}

void test_mlx_raw_to_celsius_room_temp(void)
{
    // 25.00C => raw = (25 + 273.15) / 0.02 = 14907.5, nearest integer = 14908
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 25.00f, mlx_raw_to_celsius(14908));
}

int app_main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_mlx_raw_to_celsius_zero_kelvin);
    RUN_TEST(test_mlx_raw_to_celsius_body_temp);
    RUN_TEST(test_mlx_raw_to_celsius_room_temp);
    return UNITY_END();
}
