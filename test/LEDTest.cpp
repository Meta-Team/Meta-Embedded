//
// Created by liuzikai on 8/15/21.
//

#include <CppUTest/TestHarness.h>
#include <CppUTestExt/MockSupport.h>
#include "LED.h"
#include "hal.h"

TEST_GROUP(LED) {
};

#define STR_VALUE(arg)      #arg
#define FUNCTION_NAME(name) STR_VALUE(name)

static void testGPIOFunctionCall(const char *funcName, int port, int pad, void (*func)()) {
    mock().expectOneCall(funcName)
            .withIntParameter("port", port)
            .withIntParameter("pad", pad);
    func();
}

static void testGPIOFunctionCall(const char *funcName, int port, int pad, void (*func)(int), int i) {
    mock().expectOneCall(funcName)
            .withIntParameter("port", port)
            .withIntParameter("pad", pad);
    func(i);
}

TEST(LED, TurnOnRed) {
    testGPIOFunctionCall(FUNCTION_NAME(SET_LED_ON), LED_RED_GPIO_PORT, LED_RED_GPIO_PAD, LED::redOnX);
}

TEST(LED, TurnOffRed) {
    testGPIOFunctionCall(FUNCTION_NAME(SET_LED_OFF), LED_RED_GPIO_PORT, LED_RED_GPIO_PAD, LED::redOffX);
}

TEST(LED, ToggleRed) {
    testGPIOFunctionCall("palTogglePad", LED_RED_GPIO_PORT, LED_RED_GPIO_PAD, LED::redToggleX);
}

TEST(LED, TurnOnGreen) {
    testGPIOFunctionCall(FUNCTION_NAME(SET_LED_ON), LED_GREEN_GPIO_PORT, LED_GREEN_GPIO_PAD, LED::greenOnX);
}

TEST(LED, TurnOffGreen) {
    testGPIOFunctionCall(FUNCTION_NAME(SET_LED_OFF), LED_GREEN_GPIO_PORT, LED_GREEN_GPIO_PAD, LED::greenOffX);
}

TEST(LED, ToggleGreen) {
    testGPIOFunctionCall("palTogglePad", LED_GREEN_GPIO_PORT, LED_GREEN_GPIO_PAD, LED::greenToggleX);
}

TEST(LED, TurnOnNumber) {
    for (int i = NUMBERED_LED_MIN; i <= NUMBERED_LED_MAX; i++) {
        testGPIOFunctionCall(FUNCTION_NAME(SET_LED_ON), NUMBERED_LED_GPIO_PORT, NUMBERED_LED_GPIO_PAD(i),
                             LED::numberOnX, i);
    }
}

TEST(LED, TurnOffNumber) {
    for (int i = NUMBERED_LED_MIN; i <= NUMBERED_LED_MAX; i++) {
        testGPIOFunctionCall(FUNCTION_NAME(SET_LED_OFF), NUMBERED_LED_GPIO_PORT, NUMBERED_LED_GPIO_PAD(i),
                             LED::numberOffX, i);
    }
}

TEST(LED, ToggleNumber) {
    for (int i = NUMBERED_LED_MIN; i <= NUMBERED_LED_MAX; i++) {
        testGPIOFunctionCall("palTogglePad", NUMBERED_LED_GPIO_PORT, NUMBERED_LED_GPIO_PAD(i), LED::numberToggleX, i);
    }
}

TEST(LED, TurnOnOutOfBoundNumberHasNoEffect) {
    LED::numberOnX(NUMBERED_LED_MIN - 1);
    LED::numberOnX(NUMBERED_LED_MAX + 1);
    // Mock plug-in inserts expectation assertion automatically, same for below
}

TEST(LED, TurnOffOutOfBoundNumberHasNoEffect) {
    LED::numberOffX(NUMBERED_LED_MIN - 1);
    LED::numberOffX(NUMBERED_LED_MAX + 1);
}

TEST(LED, ToggleOutOfBoundNumberHasNoEffect) {
    LED::numberToggleX(NUMBERED_LED_MIN - 1);
    LED::numberToggleX(NUMBERED_LED_MAX + 1);
}

TEST(LED, TurnAllOff) {
    // We don't use mock().strictOrder();
    mock().expectOneCall(FUNCTION_NAME(SET_LED_OFF))
            .withIntParameter("port", LED_RED_GPIO_PORT)
            .withIntParameter("pad", LED_RED_GPIO_PAD);
    mock().expectOneCall(FUNCTION_NAME(SET_LED_OFF))
            .withIntParameter("port", LED_GREEN_GPIO_PORT)
            .withIntParameter("pad", LED_GREEN_GPIO_PAD);
    for (int i = NUMBERED_LED_MIN; i <= NUMBERED_LED_MAX; i++) {
        mock().expectOneCall(FUNCTION_NAME(SET_LED_OFF))
                .withIntParameter("port", NUMBERED_LED_GPIO_PORT)
                .withIntParameter("pad", NUMBERED_LED_GPIO_PAD(i));
    }
    LED::allOffX();
}