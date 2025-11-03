
#ifndef PUBLIC_H
#define PUBLIC_H

#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define bool int
#define true 1
#define false 0

#define RETURN_ERROR (-1)
#define RETURN_OK 0

// 最大wrap
#define MAX_WRAP 280
// 最大LPOS
#define MAX_LPOS 730994
// 最大IO数量
#define MAX_IO_NUM 10000
// 最小IO数量
#define MIN_IO_NUM 10

/* 磁头状态 */
typedef enum {
    HEAD_STATIC = 0,  // 静止
    HEAD_RW,          // 读写
    HEAD_STATUS_BUTT
} HEAD_STATUS;

/* 磁头位置与状态 */
typedef struct {
    uint32_t wrap;
    uint32_t lpos;
    HEAD_STATUS status;
} HeadInfo;

/* IO 结构体 */
typedef struct {
    uint32_t id;         // IO序号
    uint32_t wrap;       // 起始wrap
    uint32_t startLpos;  // 起始lpos
    uint32_t endLpos;    // 结束lpos
} IOUint;

/* IO 容器 */
typedef struct {
    uint32_t len;     // io数量
    IOUint *ioArray;  // io数组，访问方式ioArray[i]
} IOVector;

/* 输入参数结构体 */
typedef struct {
    HeadInfo headInfo;
    IOVector ioVec;
} InputParam;

/* 输出数据结构体 */
typedef struct {
    uint32_t len;        // io数量
    uint32_t *sequence;  // io序号的排列，访问方式sequence[i]
} OutputParam;

/* 关键指标 */
typedef struct {
    uint32_t ioCount;                 // IO请求数量
    double algorithmRunningDuration;  // 算法运行时长 毫秒
    long memoryUse;                   // 内存占用 KB   （程序外工具 统计）
    uint32_t addressingDuration;      // 寻址时长 毫秒
    uint32_t readDuration;            // 读时长 毫秒
    uint32_t tapeBeltWear;            // 带体磨损
    uint16_t lposPassTime[MAX_LPOS];  // 记录每个lpos的划过次数
    uint32_t tapeMotorWear;           // 电机磨损
    uint32_t errorIOCount;            // 错误的调度IO数量
} KeyMetrics;

/* 带体分段磨损 */
typedef struct{
    uint16_t segWear[MAX_LPOS];       // 每个lpos的磨损，即每个lpos的划过次数
}TapeBeltSegWearInfo;

/* 访问时间 */
typedef struct{
    uint32_t addressDuration;           // 寻址时间
    uint32_t readDuration;              // 读数据时间
}AccessTime;

/**
 * @brief 寻址时间计算
 * @param  start            磁头起始位置和状态
 * @param  target           目标位置和期望到达位置的状态
 * @return uint32_t         返回时间，单位毫秒
 */
uint32_t SeekTimeCalculate(const HeadInfo *start, const HeadInfo *target);

/**
 * @brief  带体磨损计算
 * @param  start            起始位置和状态
 * @param  target           目标位置和状态
 * @param  segWearInfo      带体的分段磨损信息，即记录磁头划过每个lpos的次数
 * @return uint32_t         返回磁头所经过的LPOS数量，划过一个LPOS计数1次
 */
uint32_t BeltWearTimes(const HeadInfo *start, const HeadInfo *target, TapeBeltSegWearInfo *segWearInfo);

/**
 * @brief  电机磨损计算
 * @param  start            起始位置和状态
 * @param  target           目标位置和状态
 * @return uint32_t         返回电机的启停次数
 */
uint32_t MotorWearTimes(const HeadInfo *start, const HeadInfo *target);

/**
 * @brief 计算读IO数据的时间
 * @param  startLpos        lpos范围
 * @return uint32_t          毫秒
 */
uint32_t ReadTimeCalculate(uint32_t lposRange);

/**
 * @brief  计算批量IO的总访问时间，寻址时间 + 读IO数据时间
 * @param  input            磁头和批量的IO信息
 * @param  output           排序后的IO序列
 * @param  accessTime       输出参数，访问时间结构体
 */
void TotalAccessTime(const InputParam *input, const OutputParam *output, AccessTime *accessTime);

/**
 * @brief  统计磁带带体的磨损次数，即，划过一个LPOS计数1次
 * @param  input            磁头和批量的IO信息
 * @param  output           排序后的IO序列
 * @param  segWearInfo      带体的分段磨损信息，即记录磁头划过每个lpos的次数
 * @return uint32_t         带体磨损次数
 */
uint32_t TotalTapeBeltWearTimes(const InputParam *input, const OutputParam *output, TapeBeltSegWearInfo *segWearInfo);

/**
 * @brief  统计电机磨损次数，即，启、停各计数1次，掉头计数2次，
 * @param  input            磁头和批量的IO信息
 * @param  output           排序后的IO序列
 * @return uint32_t         电机磨损次数
 */
uint32_t TotalMotorWearTimes(const InputParam *input, const OutputParam *output);
#ifdef __cplusplus
}
#endif

#endif  // PUBLICK_H
