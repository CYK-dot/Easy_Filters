#include <gtest/gtest.h>
#include <math.h>

#include "easy_filters.h"  // 包含窗口滤波器的头文件

// 测试窗口滤波器类
class WindowFilterTest : public ::testing::Test {
protected:
    // 在每个测试之前初始化
    void SetUp() override {
        // 创建一个大小为 3 的窗口滤波器
        filter = xWindowFilterCreate(3);
    }

    // 在每个测试之后清理
    void TearDown() override {
        if (filter != NULL) {
            FILTER_PORT_FREE(filter);
        }
    }

    WindowFilter_t filter;
};

// 测试窗口滤波器创建
TEST_F(WindowFilterTest, CreateWindowFilter) {
    ASSERT_NE(filter, nullptr);  // 检查滤波器对象不为空
}

// 测试向滤波器中发送数据
TEST_F(WindowFilterTest, SendData) {
    ASSERT_EQ(xWindowFilterSend(filter, 1.0f), FILTER_OK);  // 发送数据 1.0f
    ASSERT_EQ(xWindowFilterSend(filter, 2.0f), FILTER_OK);  // 发送数据 2.0f
    ASSERT_EQ(xWindowFilterSend(filter, 3.0f), FILTER_OK);  // 发送数据 3.0f

    // 读取平均值，应该是 (1 + 2 + 3) / 3 = 2.0f
    ASSERT_FLOAT_EQ(xWindowFilterRecv(filter), 2.0f);
}

// 测试窗口满时的数据覆盖
TEST_F(WindowFilterTest, WindowFull) {
    xWindowFilterSend(filter, 1.0f);
    xWindowFilterSend(filter, 2.0f);
    xWindowFilterSend(filter, 3.0f);

    // 发送新数据，旧数据 1.0f 被覆盖
    ASSERT_EQ(xWindowFilterSend(filter, 4.0f), FILTER_OK);

    // 读取平均值，应该是 (2 + 3 + 4) / 3 = 3.0f
    ASSERT_FLOAT_EQ(xWindowFilterRecv(filter), 3.0f);
}

// 测试读取空窗口数据
TEST_F(WindowFilterTest, EmptyWindow) {
    // 窗口为空，调用 recv 应该返回 NAN
    ASSERT_TRUE(std::isnan(xWindowFilterRecv(filter)));
}

// 测试无效数据输入（NaN）
TEST_F(WindowFilterTest, InvalidDataNaN) {
    ASSERT_EQ(xWindowFilterSend(filter, NAN), FILTER_ERR_INVALID_PARAM);
}

// 测试无效数据输入（Inf）
TEST_F(WindowFilterTest, InvalidDataInf) {
    ASSERT_EQ(xWindowFilterSend(filter, INFINITY), FILTER_ERR_INVALID_PARAM);
}

// 测试数据超出边界条件
TEST_F(WindowFilterTest, InvalidWindowSize) {
    // 创建一个无效的窗口大小
    WindowFilter_t invalidFilter = xWindowFilterCreate(0);  // 无效窗口大小
    ASSERT_EQ(invalidFilter, nullptr);  // 应该返回 NULL
}
