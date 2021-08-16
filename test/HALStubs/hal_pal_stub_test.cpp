//
// Created by liuzikai on 8/15/21.
//

#include <gtest/gtest.h>
#include "hal_pal_stub.h"

TEST(HALPALStub, gpioImageClearAtSetup) {
    for (const auto &image : gpioPortImage) {
        EXPECT_EQ(image, 0);
    }
}

TEST(HALPALStub, palSetPadWorks) {
    gpioPortImage[GPIOA] = 0;
    palSetPad(GPIOA, 0);
    EXPECT_EQ(gpioPortImage[GPIOA], 0x0001);

    gpioPortImage[GPIOK] = 0x8888;
    palSetPad(GPIOK, 4);
    EXPECT_EQ(gpioPortImage[GPIOK], 0x8898);
}

TEST(HALPALStub, palClearPadWorks) {
    gpioPortImage[GPIOB] = 0xFFFF;
    palClearPad(GPIOB, 15);
    EXPECT_EQ(gpioPortImage[GPIOB], 0x7FFF);

    palClearPad(GPIOB, 0);
    EXPECT_EQ(gpioPortImage[GPIOB], 0x7FFE);
}

TEST(HALPALStub, palTogglePadWorks) {
    gpioPortImage[GPIOA] = 0;
    palTogglePad(GPIOA, 0);
    EXPECT_EQ(gpioPortImage[GPIOA], 0x0001);

    palTogglePad(GPIOA, 0);
    EXPECT_EQ(gpioPortImage[GPIOA], 0x0000);
}

class SharedTest :
        public testing::TestWithParam<std::function<void(unsigned, unsigned )>> {
};

TEST_P(SharedTest, CheckOutOfBound) {
    EXPECT_THROW(GetParam()(GPIO_COUNT, 0), std::range_error);
    EXPECT_THROW(GetParam()(-1, 0), std::range_error);
    EXPECT_THROW(GetParam()(GPIOA, 32), std::range_error);
    EXPECT_THROW(GetParam()(GPIOA, -1), std::range_error);
}

INSTANTIATE_TEST_SUITE_P(HALPALStub, SharedTest, testing::Values(palSetPad, palClearPad, palTogglePad));