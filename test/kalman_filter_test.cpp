#include <gtest/gtest.h>
#include <math.h>
#include "easy_filters.h"  // 包含卡尔曼滤波器的头文件

// 测试卡尔曼滤波器类
class KalmanFilterTest : public ::testing::Test {
protected:
    // 在每个测试之前初始化
    void SetUp() override {
        // 创建卡尔曼滤波器，初始化参数 Q=0.1, R=0.1, 初始估计值 initialEstimate=0.0f
        filter = xKalmanFilterCreate(0.1f, 0.1f, 0.0f);
    }

    // 在每个测试之后清理
    void TearDown() override {
        if (filter != NULL) {
            FILTER_PORT_FREE(filter);
        }
    }

    KalmanFilter_t filter;
};

// 测试卡尔曼滤波器创建
TEST_F(KalmanFilterTest, CreateKalmanFilter) {
    ASSERT_NE(filter, nullptr);  // 检查滤波器对象不为空
}

// 测试卡尔曼滤波器发送数据并接收数据
TEST_F(KalmanFilterTest, SendDataAndReceiveEstimate) {
    // 发送第一次数据
    ASSERT_EQ(xKalmanFilterSend(filter, 1.0f), FILTER_OK);

    // 发送第二次数据
    ASSERT_EQ(xKalmanFilterSend(filter, 2.0f), FILTER_OK);

    // 发送第三次数据
    ASSERT_EQ(xKalmanFilterSend(filter, 3.0f), FILTER_OK);
}

// 测试卡尔曼滤波器的多次更新
TEST_F(KalmanFilterTest, MultipleUpdates) {
    // 向滤波器发送多次数据
    for (int i = 1; i <= 10; ++i) {
        ASSERT_EQ(xKalmanFilterSend(filter, (float)i), FILTER_OK);
    }
}

// 测试卡尔曼滤波器无效输入（NULL 参数）
TEST_F(KalmanFilterTest, NullFilter) {
    ASSERT_EQ(xKalmanFilterSend(nullptr, 1.0f), FILTER_ERR_INVALID_PARAM);  // NULL 参数应该返回错误
    ASSERT_TRUE(std::isnan(xKalmanFilterRecv(nullptr)));  // NULL 参数应该返回 NAN
}

// 测试卡尔曼滤波器无效数据（NaN）
TEST_F(KalmanFilterTest, InvalidDataNaN) {
    ASSERT_EQ(xKalmanFilterSend(filter, NAN), FILTER_ERR_INVALID_PARAM);  // NaN 数据应该返回错误
}

// 测试卡尔曼滤波器创建无效的滤波器对象
TEST_F(KalmanFilterTest, InvalidCreateKalmanFilter) {
    // 创建一个无效的卡尔曼滤波器，Q 和 R 为负值
    KalmanFilter_t invalidFilter = xKalmanFilterCreate(-1.0f, -1.0f, 0.0f);
    ASSERT_EQ(invalidFilter, nullptr);  // 应该返回 NULL
}
