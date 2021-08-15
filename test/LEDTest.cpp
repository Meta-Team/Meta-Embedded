//
// Created by liuzikai on 8/15/21.
//

#include <gtest/gtest.h>
#include "LED.h"
#include "hal.h"

void expectGPIOImageChange(uint32_t port, uint32_t pad, uint32_t init) {
    EXPECT_EQ(gpioImage[port] ^ init, (1U << pad));
}

void testGPIOChangesCorrectly(uint32_t port, uint32_t pad, uint32_t init, const std::function<void()> &f) {
    gpioImage[port] = init;
    f();
    expectGPIOImageChange(port, pad, init);
}

void testGPIOToggleCorrectly(uint32_t port, uint32_t pad, uint32_t init, const std::function<void()> &f) {
    gpioImage[port] = init;
    f();
    expectGPIOImageChange(port, pad, init);
    f();
    EXPECT_EQ(gpioImage[port], init);
}

void testGPIOChangesCorrectly(uint32_t port, uint32_t pad, uint32_t init, const std::function<void(int)> &f, int i) {
    gpioImage[port] = init;
    f(i);
    expectGPIOImageChange(port, pad, init);
}

void testGPIOToggleCorrectly(uint32_t port, uint32_t pad, uint32_t init, const std::function<void(int)> &f, int i) {
    gpioImage[port] = init;
    f(i);
    expectGPIOImageChange(port, pad, init);
    f(i);
    EXPECT_EQ(gpioImage[port], init);
}

#define LED_ALL_ON_MASK (~LED_ALL_OFF_MASK)

TEST(LED, TurnOnRed) {
    testGPIOChangesCorrectly(LED_RED_GPIO_PORT, LED_RED_GPIO_PAD, LED_ALL_OFF_MASK, LED::redOnX);
}

TEST(LED, TurnOffRed) {
    testGPIOChangesCorrectly(LED_RED_GPIO_PORT, LED_RED_GPIO_PAD, LED_ALL_ON_MASK, LED::redOffX);
}

TEST(LED, ToggleRed) {
    testGPIOToggleCorrectly(LED_RED_GPIO_PORT, LED_RED_GPIO_PAD, LED_ALL_OFF_MASK, LED::redToggleX);
    testGPIOToggleCorrectly(LED_RED_GPIO_PORT, LED_RED_GPIO_PAD, LED_ALL_ON_MASK, LED::redToggleX);
}

TEST(LED, TurnOnGreen) {
    testGPIOChangesCorrectly(LED_GREEN_GPIO_PORT, LED_GREEN_GPIO_PAD, LED_ALL_OFF_MASK, LED::greenOnX);
}

TEST(LED, TurnOffGreen) {
    testGPIOChangesCorrectly(LED_GREEN_GPIO_PORT, LED_GREEN_GPIO_PAD, LED_ALL_ON_MASK, LED::greenOffX);
}

TEST(LED, ToggleGreen) {
    testGPIOToggleCorrectly(LED_GREEN_GPIO_PORT, LED_GREEN_GPIO_PAD, LED_ALL_OFF_MASK, LED::greenToggleX);
    testGPIOToggleCorrectly(LED_GREEN_GPIO_PORT, LED_GREEN_GPIO_PAD, LED_ALL_ON_MASK, LED::greenToggleX);
}

TEST(LED, TurnOnNumber) {
    for (int i = LED_NUMBERED_MIN; i <= LED_NUMBERED_MAX; i++) {
        testGPIOChangesCorrectly(LED_NUMBERED_GPIO_PORT, LED_NUMBERED_GPIO_PAD(i), LED_ALL_OFF_MASK, LED::numberOnX, i);
    }
}

TEST(LED, TurnOffNumber) {
    for (int i = LED_NUMBERED_MIN; i <= LED_NUMBERED_MAX; i++) {
        testGPIOChangesCorrectly(LED_NUMBERED_GPIO_PORT, LED_NUMBERED_GPIO_PAD(i), LED_ALL_ON_MASK, LED::numberOffX, i);
    }
}

TEST(LED, ToggleNumber) {
    for (int i = LED_NUMBERED_MIN; i <= LED_NUMBERED_MAX; i++) {
        testGPIOToggleCorrectly(LED_NUMBERED_GPIO_PORT, LED_NUMBERED_GPIO_PAD(i), LED_ALL_ON_MASK, LED::numberToggleX,
                                i);
    }
}

void testOutOfBoundNumberHasNoEffect(uint32_t init, const std::function<void(int)> &f, int i) {
    gpioImage[LED_NUMBERED_GPIO_PORT] = init;
    f(i);
    EXPECT_EQ(gpioImage[LED_NUMBERED_GPIO_PORT], init);
}

TEST(LED, TurnOnOutOfBoundNumberHasNoEffect) {
    testOutOfBoundNumberHasNoEffect(LED_ALL_OFF_MASK, LED::numberOnX, LED_NUMBERED_MIN - 1);
    testOutOfBoundNumberHasNoEffect(LED_ALL_OFF_MASK, LED::numberOnX, LED_NUMBERED_MAX + 1);
}

TEST(LED, TurnOffOutOfBoundNumberHasNoEffect) {
    testOutOfBoundNumberHasNoEffect(LED_ALL_ON_MASK, LED::numberOffX, LED_NUMBERED_MIN - 1);
    testOutOfBoundNumberHasNoEffect(LED_ALL_ON_MASK, LED::numberOffX, LED_NUMBERED_MAX + 1);
}

TEST(LED, ToggleOutOfBoundNumberHasNoEffect) {
    testOutOfBoundNumberHasNoEffect(LED_ALL_ON_MASK, LED::numberToggleX, LED_NUMBERED_MIN - 1);
    testOutOfBoundNumberHasNoEffect(LED_ALL_OFF_MASK, LED::numberToggleX, LED_NUMBERED_MAX + 1);
}

#define LED_OFF_STATE (!LED_ON_STATE)

TEST(LED, TurnAllOff) {
    gpioImage[LED_RED_GPIO_PORT] = gpioImage[LED_GREEN_GPIO_PORT] = gpioImage[LED_NUMBERED_GPIO_PORT] = LED_ALL_ON_MASK;
    LED::allOffX();
    EXPECT_EQ(gpioImage[LED_RED_GPIO_PORT] & (1U << LED_RED_GPIO_PAD), LED_OFF_STATE << LED_RED_GPIO_PAD);
    EXPECT_EQ(gpioImage[LED_GREEN_GPIO_PORT] & (1U << LED_GREEN_GPIO_PAD), LED_OFF_STATE << LED_GREEN_GPIO_PAD);
    for (int i = LED_NUMBERED_MIN; i <= LED_NUMBERED_MAX; i++) {
        EXPECT_EQ(gpioImage[LED_NUMBERED_GPIO_PORT] & (1U << LED_NUMBERED_GPIO_PAD(i)),
                  LED_OFF_STATE << LED_NUMBERED_GPIO_PAD(i));
    }
}