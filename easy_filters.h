#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>

// ϵͳ���ýӿ�
#define FILTER_PORT_MALLOC            malloc
#define FILTER_PORT_FREE              free

// ������
#define FILTER_OK 0
#define FILTER_NOT_FINISH -1
#define FILTER_ERR_INVALID_PARAM -2

// һ�׵�ͨ�˲�����
typedef void* LowpassFilter_t;
LowpassFilter_t xLowpassFilterCreate(float alpha);
int xLowpassFilterSend(LowpassFilter_t filter,float dataToFilter);
float xLowpassFilterRecv(LowpassFilter_t filter);

// ��ֵ�����˲�����
typedef void* WindowFilter_t;
WindowFilter_t xWindowFilterCreate(uint16_t windowSize);
int xWindowFilterSend(WindowFilter_t filter,float dataToFilter);
float xWindowFilterRecv(WindowFilter_t filter);

// һ�׿������˲�����
typedef void* KalmanFilter_t;
KalmanFilter_t xKalmanFilterCreate(float Q,float R,float initEstimate);
int xKalmanFilterSend(KalmanFilter_t filter, float measuredValue) ;
float xKalmanFilterRecv(KalmanFilter_t filter) ;


