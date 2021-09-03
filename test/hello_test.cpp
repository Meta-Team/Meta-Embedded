//
// Created by liuzikai on 8/14/21.
//

#include <CppUTest/TestHarness.h>

TEST_GROUP(HelloTest) {};

TEST(HelloTest, BasicAssertions) {
    STRCMP_EQUAL("hello", "hello");
    CHECK_EQUAL(7 * 6, 42);
}