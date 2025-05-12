#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include "easy_filters.h"  

constexpr float EPSILON = 1e-5f;
bool nearlyEqual(float a, float b, float epsilon = EPSILON) {
    return std::fabs(a - b) < epsilon;
}

TEST(LowpassFilterTest, CreateValidAlpha) {
    EXPECT_NE(xLowpassFilterCreate(0.5f), nullptr);
    EXPECT_NE(xLowpassFilterCreate(0.0f), nullptr);
    EXPECT_NE(xLowpassFilterCreate(1.0f), nullptr);
}

TEST(LowpassFilterTest, CreateInvalidAlpha) {
    EXPECT_EQ(xLowpassFilterCreate(-0.1f), nullptr);
    EXPECT_EQ(xLowpassFilterCreate(1.1f), nullptr);
    EXPECT_EQ(xLowpassFilterCreate(NAN), nullptr);
}

TEST(LowpassFilterTest, FirstSendReturnsNotFinish) {
    auto filter = xLowpassFilterCreate(0.5f);
    ASSERT_NE(filter, nullptr);

    EXPECT_EQ(xLowpassFilterSend(filter, 42.0f), FILTER_NOT_FINISH);
    EXPECT_TRUE(nearlyEqual(xLowpassFilterRecv(filter), 42.0f));
}

TEST(LowpassFilterTest, FilterWorksCorrectly) {
    auto filter = xLowpassFilterCreate(0.5f);
    ASSERT_NE(filter, nullptr);

    // First input
    EXPECT_EQ(xLowpassFilterSend(filter, 100.0f), FILTER_NOT_FINISH);
    EXPECT_TRUE(nearlyEqual(xLowpassFilterRecv(filter), 100.0f));

    // Second input
    EXPECT_EQ(xLowpassFilterSend(filter, 200.0f), FILTER_OK);
    float expected = 100.0f * 0.5f + 200.0f * 0.5f;
    EXPECT_TRUE(nearlyEqual(xLowpassFilterRecv(filter), expected));
}

TEST(LowpassFilterTest, NullPointerHandling) {
    EXPECT_EQ(xLowpassFilterSend(nullptr, 123.0f), FILTER_ERR_INVALID_PARAM);
    EXPECT_TRUE(std::isnan(xLowpassFilterRecv(nullptr)));
}

TEST(LowpassFilterTest, OutputNanBeforeFirstInput) {
    auto filter = xLowpassFilterCreate(0.3f);
    ASSERT_NE(filter, nullptr);
    EXPECT_TRUE(std::isnan(xLowpassFilterRecv(filter)));
}

TEST(LowpassFilterTest, InfInputHandling) {
    auto filter = xLowpassFilterCreate(0.6f);
    ASSERT_NE(filter, nullptr);

    // First send: INF
    EXPECT_EQ(xLowpassFilterSend(filter, std::numeric_limits<float>::infinity()), FILTER_NOT_FINISH);
    EXPECT_TRUE(std::isinf(xLowpassFilterRecv(filter)));

    // Next send: finite value
    EXPECT_EQ(xLowpassFilterSend(filter, 100.0f), FILTER_OK);
    EXPECT_TRUE(std::isinf(xLowpassFilterRecv(filter)));
}

TEST(LowpassFilterTest, NegativeInfInputHandling) {
    auto filter = xLowpassFilterCreate(0.6f);
    ASSERT_NE(filter, nullptr);

    xLowpassFilterSend(filter, -std::numeric_limits<float>::infinity());
    EXPECT_TRUE(std::isinf(xLowpassFilterRecv(filter)));

    xLowpassFilterSend(filter, 100.0f);
    EXPECT_TRUE(std::isinf(xLowpassFilterRecv(filter)));
}