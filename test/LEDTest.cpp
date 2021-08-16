//
// Created by liuzikai on 8/15/21.
//

#include <gtest/gtest.h>
#include "LED.h"
#include "hal.h"

#if LED_ON_STATE == 1U
static constexpr PortImageType LED_ALL_OFF_MASK = 0;
static constexpr PortImageType LED_ALL_ON_MASK = -1;
#else
static constexpr PortImageType LED_ALL_OFF_MASK = -1;
static constexpr PortImageType LED_ALL_ON_MASK = 0;
#endif


void expectGPIOImageChange(unsigned port, unsigned pad, PortImageType init) {
    EXPECT_EQ(gpioPortImage[port] ^ init, (1U << pad));
}

void testGPIOChangesCorrectly(unsigned port, unsigned pad, PortImageType init, const std::function<void()> &f) {
    gpioPortImage[port] = init;
    f();
    expectGPIOImageChange(port, pad, init);
}

void testGPIOToggleCorrectly(unsigned port, unsigned pad, PortImageType init, const std::function<void()> &f) {
    gpioPortImage[port] = init;
    f();
    expectGPIOImageChange(port, pad, init);
    f();
    EXPECT_EQ(gpioPortImage[port], init);
}

void testGPIOChangesCorrectly(unsigned port, unsigned pad, PortImageType init, const std::function<void(int)> &f, int i) {
    gpioPortImage[port] = init;
    f(i);
    expectGPIOImageChange(port, pad, init);
}

void testGPIOToggleCorrectly(unsigned port, unsigned pad, PortImageType init, const std::function<void(int)> &f, int i) {
    gpioPortImage[port] = init;
    f(i);
    expectGPIOImageChange(port, pad, init);
    f(i);
    EXPECT_EQ(gpioPortImage[port], init);
}

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
    for (int i = NUMBERED_LED_MIN; i <= NUMBERED_LED_MAX; i++) {
        testGPIOChangesCorrectly(NUMBERED_LED_GPIO_PORT, NUMBERED_LED_GPIO_PAD(i), LED_ALL_OFF_MASK, LED::numberOnX, i);
    }
}

TEST(LED, TurnOffNumber) {
    for (int i = NUMBERED_LED_MIN; i <= NUMBERED_LED_MAX; i++) {
        testGPIOChangesCorrectly(NUMBERED_LED_GPIO_PORT, NUMBERED_LED_GPIO_PAD(i), LED_ALL_ON_MASK, LED::numberOffX, i);
    }
}

TEST(LED, ToggleNumber) {
    for (int i = NUMBERED_LED_MIN; i <= NUMBERED_LED_MAX; i++) {
        testGPIOToggleCorrectly(NUMBERED_LED_GPIO_PORT, NUMBERED_LED_GPIO_PAD(i), LED_ALL_ON_MASK, LED::numberToggleX,
                                i);
    }
}

void testOutOfBoundNumberHasNoEffect(PortImageType init, const std::function<void(int)> &f, int i) {
    gpioPortImage[NUMBERED_LED_GPIO_PORT] = init;
    f(i);
    EXPECT_EQ(gpioPortImage[NUMBERED_LED_GPIO_PORT], init);
}

TEST(LED, TurnOnOutOfBoundNumberHasNoEffect) {
    testOutOfBoundNumberHasNoEffect(LED_ALL_OFF_MASK, LED::numberOnX, NUMBERED_LED_MIN - 1);
    testOutOfBoundNumberHasNoEffect(LED_ALL_OFF_MASK, LED::numberOnX, NUMBERED_LED_MAX + 1);
}

TEST(LED, TurnOffOutOfBoundNumberHasNoEffect) {
    testOutOfBoundNumberHasNoEffect(LED_ALL_ON_MASK, LED::numberOffX, NUMBERED_LED_MIN - 1);
    testOutOfBoundNumberHasNoEffect(LED_ALL_ON_MASK, LED::numberOffX, NUMBERED_LED_MAX + 1);
}

TEST(LED, ToggleOutOfBoundNumberHasNoEffect) {
    testOutOfBoundNumberHasNoEffect(LED_ALL_ON_MASK, LED::numberToggleX, NUMBERED_LED_MIN - 1);
    testOutOfBoundNumberHasNoEffect(LED_ALL_OFF_MASK, LED::numberToggleX, NUMBERED_LED_MAX + 1);
}

#define LED_OFF_STATE (!LED_ON_STATE)

TEST(LED, TurnAllOff) {
    gpioPortImage[LED_RED_GPIO_PORT] = gpioPortImage[LED_GREEN_GPIO_PORT] = gpioPortImage[NUMBERED_LED_GPIO_PORT] = LED_ALL_ON_MASK;
    LED::allOffX();
    EXPECT_EQ(gpioPortImage[LED_RED_GPIO_PORT] & (1U << LED_RED_GPIO_PAD), LED_OFF_STATE << LED_RED_GPIO_PAD);
    EXPECT_EQ(gpioPortImage[LED_GREEN_GPIO_PORT] & (1U << LED_GREEN_GPIO_PAD), LED_OFF_STATE << LED_GREEN_GPIO_PAD);
    for (int i = NUMBERED_LED_MIN; i <= NUMBERED_LED_MAX; i++) {
        EXPECT_EQ(gpioPortImage[NUMBERED_LED_GPIO_PORT] & (1U << NUMBERED_LED_GPIO_PAD(i)),
                  LED_OFF_STATE << NUMBERED_LED_GPIO_PAD(i));
    }
}