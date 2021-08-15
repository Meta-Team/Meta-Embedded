//
// Created by liuzikai on 8/15/21.
//

#include <gtest/gtest.h>
#include "hal_pal_stub.h"

TEST(HALPALStub, gpioImageClearAtSetup) {
    for (const auto &image : gpioImage) {
        EXPECT_EQ(image, 0);
    }
}

TEST(HALPALStub, palSetPadWorks) {
    gpioImage[GPIOA] = 0;
    palSetPad(GPIOA, 0);
    EXPECT_EQ(gpioImage[GPIOA], 0x00000001);

    gpioImage[GPIOK] = 0x88888888;
    palSetPad(GPIOK, 4);
    EXPECT_EQ(gpioImage[GPIOK], 0x88888898);
}

TEST(HALPALStub, palClearPadWorks) {
    gpioImage[GPIOB] = 0xFFFFFFFF;
    palClearPad(GPIOB, 31);
    EXPECT_EQ(gpioImage[GPIOB], 0x7FFFFFFF);

    palClearPad(GPIOB, 0);
    EXPECT_EQ(gpioImage[GPIOB], 0x7FFFFFFE);
}

TEST(HALPALStub, palTogglePadWorks) {
    gpioImage[GPIOA] = 0;
    palTogglePad(GPIOA, 0);
    EXPECT_EQ(gpioImage[GPIOA], 0x00000001);

    palTogglePad(GPIOA, 0);
    EXPECT_EQ(gpioImage[GPIOA], 0x00000000);
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