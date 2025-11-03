/*
 * Copyright (c) Huawei Technologies Co., Ltd. 2024-2024. All rights reserved.
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/time.h>
#include "algorithm.h"
#include "public.h"
#include <pthread.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <sys/sysinfo.h>

#define MAX_PATH_LENGTH 256
#define LOCAL

int GetMissingNumbers(uint32_t N, uint32_t *A, uint32_t size)
{
    // 创建一个标记数组，用于标记 1 到 N 是否出现在 A[] 中
    uint32_t *flags = (uint32_t *)calloc(N + 1, sizeof(uint32_t));  // 初始化为 0

    // 标记数组 A[] 中出现的数字
    for (uint32_t i = 0; i < size; i++) {
        if (A[i] >= 1 && A[i] <= N) {
            flags[A[i]] = 1;
        }
    }

    // 统计未出现的数字
    uint32_t count = 0;
    for (int i = 1; i <= N; i++) {
        if (flags[i] == 0) {
            count++;
        }
    }

    // 释放标记数组的内存
    free(flags);

    return count;
}

/* 打印关键指标 */
void PrintMetrics(const KeyMetrics *metrics)
{
    printf("\nKey Metrics:\n");
    printf("\tioCount:\t\t\t %u\n", metrics->ioCount);
    printf("\talgorithmRunningDuration:\t %.3f (ms)\n", metrics->algorithmRunningDuration);
    printf("\tmemoryUse:\t\t\t %ld (KB)\n", metrics->memoryUse);
    printf("\taddressingDuration:\t\t %u (ms)\n", metrics->addressingDuration);
    printf("\treadDuration:\t\t\t %u (ms)\n", metrics->readDuration);
    printf("\ttapeBeltWear:\t\t\t %u\n", metrics->tapeBeltWear);
    printf("\ttapeMotorWear:\t\t\t %u\n", metrics->tapeMotorWear);
    printf("\terrorIOCount:\t\t\t %u\n", metrics->errorIOCount);
}

/* 将 KeyMetrics 结构体的内容保存到 TXT 文件 */
void SaveKeyMetricsToFile(const char *filename, const KeyMetrics *metrics)
{
    FILE *file = fopen(filename, "w");
    if (file == NULL) {
        perror("Error opening file");
        return;
    }

    fprintf(file, "/* 关键指标结构体 */\n");
    fprintf(file, "ioCount: %u \n", metrics->ioCount);
    fprintf(file, "algorithmRunningDuration(ms): %.2f \n", metrics->algorithmRunningDuration);
    fprintf(file, "memoryUse(ms): %lu \n", metrics->memoryUse);
    fprintf(file, "addressingDuration(ms): %u \n", metrics->addressingDuration);
    fprintf(file, "readDuration(ms): %u\n", metrics->readDuration);
    fprintf(file, "tapeBeltWear(times): %u\n", metrics->tapeBeltWear);

    // fprintf(file, "lposPassTime: ");
    // for (int i = 0; i < MAX_LPOS; i++) {
    //     fprintf(file, "%u", metrics->lposPassTime[i]);
    //     if (i < MAX_LPOS - 1) {
    //         fprintf(file, ", ");
    //     }
    // }
    // fprintf(file, "\n");

    fprintf(file, "tapeMotorWear(times): %u \n", metrics->tapeMotorWear);
    fprintf(file, "errorIOCount: %u \n", metrics->errorIOCount);

    fclose(file);
    printf("\n指标写入文件 %s\n", filename);
}

static bool CheckLposValid(uint32_t lpos)
{
    if (0 <= lpos && lpos < MAX_LPOS) {
        return true;
    }
    printf("Error! lpos = (%u) 不在合法范围[0-%u]内. 请修改！\n", lpos, MAX_LPOS - 1);
    return false;
}

static bool CheckWrapValid(uint32_t wrap)
{
    if (0 <= wrap && wrap < MAX_WRAP) {
        return true;
    }
    printf("Error! wrap = (%u) 不在合法范围[0-%u]内. 请修改！\n", wrap, MAX_WRAP - 1);
    return false;
}

static bool CheckHeadStatusValid(int32_t status)
{
    if ((HEAD_STATUS)status == HEAD_STATIC || (HEAD_STATUS)status == HEAD_RW) {
        return true;
    }
    printf("Error! 磁头状态 = (%u) 不在合法范围[%u , %u]. 请修改！\n", status, (int32_t)HEAD_STATIC, (int32_t)HEAD_RW);
    return false;
}

static bool CheckIONumValid(uint32_t ioNum)
{
    if ((MIN_IO_NUM <= ioNum) && (ioNum <= MAX_IO_NUM)) {
        return true;
    }
    printf("Error! IO数量 = (%u) 不在合法范围[%u-%u]内. 请修改！\n", ioNum, MIN_IO_NUM, MAX_IO_NUM);
    return false;
}

static bool CheckStartLposAndEndLposValid(uint32_t wrap, uint32_t startLpos, uint32_t endLpos)
{
    if (((wrap % 2 == 0) && (startLpos < endLpos)) || ((wrap % 2 == 1) && (startLpos > endLpos))) {
        return true;
    }
    printf("Error! wrap(%u) startLpos(%u) endLpos(%u). 请检查：wrap为偶数的 读写方向是BOT->EOT; "
           "wrap为奇数的读写方向是EOT->BOT\n",
        wrap,
        startLpos,
        endLpos);
    return false;
}
static bool CheckIOIndexsequence(const IOVector *ioVec)
{
    for (uint32_t i = 0; i < ioVec->len; i++) {
        if (ioVec->ioArray[i].id != i + 1) {
            printf("Error! 请检查输入的IO序列号是否是顺序的. io[%u]\n", i + 1);
            return false;
        }
    }
    return true;
}
/* 读取文件内容并解析 */
int parseFile(const char *filename, HeadInfo *headInfo, IOVector *ioVector)
{
    FILE *file = fopen(filename, "r");
    if (!file) {
        perror("Failed to open file");
        return RETURN_ERROR;
    }

    char line[256];
    int ioCount = 0;
    IOUint *ioArray = NULL;

    while (fgets(line, sizeof(line), file)) {
        if (strncmp(line, "[\"head", 6) == 0) {
            // 读取 head 信息
            if (fgets(line, sizeof(line), file)) {
                sscanf(line, "[%u,%u,%u]", &headInfo->wrap, &headInfo->lpos, (uint32_t *)&headInfo->status);
                if (CheckWrapValid(headInfo->wrap) == false || CheckLposValid(headInfo->lpos) == false ||
                    CheckHeadStatusValid(headInfo->status) == false) {
                    fclose(file);
                    return RETURN_ERROR;
                }
                printf("head info : %s\n", line);
            }
        } else if (strncmp(line, "[\"io count", 10) == 0) {
            // 读取 io count 信息
            if (fgets(line, sizeof(line), file)) {
                sscanf(line, "[%u]", &ioVector->len);
                if (CheckIONumValid(ioVector->len) == false) {
                    fclose(file);
                    return RETURN_ERROR;
                }
                ioArray = (IOUint *)malloc(ioVector->len * sizeof(IOUint));
                printf("io count = %u\n", ioVector->len);
            }
        } else if (strncmp(line, "[\"io", 4) == 0) {
            // printf("input io array:\n");
        } else if (line[0] == '[' && line[1] > '0' && line[1] <= '9') {
            IOUint io;
            sscanf(line, "[%u,%u,%u,%u]", &io.id, &io.wrap, &io.startLpos, &io.endLpos);

            if (CheckWrapValid(io.wrap) == false || CheckLposValid(io.endLpos) == false ||
                CheckLposValid(io.startLpos) == false) {
                fclose(file);
                return RETURN_ERROR;
            }
            if (CheckStartLposAndEndLposValid(io.wrap, io.startLpos, io.endLpos) == false) {
                fclose(file);
                return RETURN_ERROR;
            }

            ioArray[ioCount] = io;
            ioCount++;
            // printf("io [%u] : %s", ioCount, line);
        }
    }
    printf("\n\n");

    fclose(file);

    if (ioVector->len != ioCount) {
        printf("Error! 请检查测试用例文件[\"io count\"] 数值(%u)是否与实际io数量(%u)一致。 len != ioCount\n",
            ioVector->len,
            ioCount);
        return RETURN_ERROR;
    }
    if (ioVector->len > MAX_IO_NUM) {
        printf("Error! IO数量超限制10000。IO number(%u) should less than %u\n", ioVector->len, MAX_IO_NUM);
        return RETURN_ERROR;
    }
    ioVector->ioArray = ioArray;

    if(CheckIOIndexsequence(ioVector) == false){
        return RETURN_ERROR;
    }

    return RETURN_OK;
}

int main(int argc, char *argv[])
{
    printf("\n\nWelcome to HW project.\n\n");

    /* 输入dataset文件地址 */
    int opt;
    char *file = NULL;
    pthread_t thread;
    int ret;

    /* 使用 getopt 解析命令行参数 */
    while ((opt = getopt(argc, argv, "f:")) != -1) {
        switch (opt) {
            case 'f':
                file = optarg;
                break;
            default:
                fprintf(stderr, "Usage: %s -f filename. [example: ./main -f /heme/case_1.txt] \n", argv[0]);
                exit(EXIT_FAILURE);
        }
    }

    if (file == NULL) {
        fprintf(stderr, "Usage: %s -f filename. [example: ./main -f /heme/case_1.txt] \n", argv[0]);
        exit(EXIT_FAILURE);
    }

    printf("The file path is: %s\n", file);

    /* 获取输入参数 */
    InputParam *inputParam = (InputParam *)malloc(sizeof(InputParam));
    ret = parseFile(file, &inputParam->headInfo, &inputParam->ioVec);
    if (ret < 0) {
        printf("InputParam error\n");
        return RETURN_ERROR;
    }

    /* 定初始化输出参数 */
    OutputParam *output = (OutputParam *)malloc(sizeof(OutputParam));
    output->len = inputParam->ioVec.len;
    output->sequence = (uint32_t *)malloc(output->len * sizeof(uint32_t));

    /* 统计算法运行时间 */
    struct timeval start, end;
    gettimeofday(&start, NULL);

    /* 算法执行 */
    ret = AlgorithmRun(inputParam, output);

    gettimeofday(&end, NULL);  // 记录结束时间
    long seconds, useconds;    // 秒数和微秒数
    seconds = end.tv_sec - start.tv_sec;
    useconds = end.tv_usec - start.tv_usec;

    /* 统计指标 */
    KeyMetrics metrics = {0};

    /* IO 数量 */
    metrics.ioCount = inputParam->ioVec.len;
    /* 获取错误调度的IO数量 */
    metrics.errorIOCount = GetMissingNumbers(inputParam->ioVec.len, output->sequence, output->len);
    if (metrics.errorIOCount > 0) {
        /* 总微秒数 */
        metrics.algorithmRunningDuration = 0;
        /* 访问时间 */
        metrics.addressingDuration = 0;
        /* 带体磨损 */
        metrics.tapeBeltWear = 0;
        /* 电机磨损 */
        metrics.tapeMotorWear = 0;
    } else {
        /* 访问时间 */
        AccessTime accessTime = {0};
        // 计算时间





        TotalAccessTime(inputParam, output, &accessTime);
        metrics.addressingDuration = accessTime.addressDuration;
        metrics.readDuration = accessTime.readDuration;
        /* 带体磨损 */
        TapeBeltSegWearInfo segWearInfo = {0};
        metrics.tapeBeltWear = TotalTapeBeltWearTimes(inputParam, output, &segWearInfo);
        memcpy(&metrics.lposPassTime, segWearInfo.segWear, sizeof(segWearInfo.segWear));
        /* 电机磨损 */
        metrics.tapeMotorWear = TotalMotorWearTimes(inputParam, output);
    }

    /* 总毫秒数 */
    metrics.algorithmRunningDuration = ((seconds)*1000000 + useconds) / 1000.0;
    /* 内存占用 */
    metrics.memoryUse = 0;


    PrintMetrics(&metrics);
    /* 保存指标数据到文件 */
    SaveKeyMetricsToFile("./metrics.txt", &metrics);


    /***基线**************************************/
    // 计算baseline读时延迟
    IOScheduleAlgorithmBaseline(inputParam, output, true);
    /* 访问时间 */
    AccessTime accessTime = {0};
    TotalAccessTime(inputParam, output, &accessTime);
    double baseline_address = accessTime.addressDuration;
      /* 带体磨损 */
    TapeBeltSegWearInfo segWearInfo = {0};
    double baseline_tapeBeltWear = TotalTapeBeltWearTimes(inputParam, output, &segWearInfo);

    /* 电机磨损 */
    double baseline_tapeMotorWear = TotalMotorWearTimes(inputParam, output);

    /************************************************/

    /*输出评分函数*/
    // 初赛调度算法加分 =（基线读时延 - 排序后读时延）* 10 + 调度用时加分 =（20s – IO排序时间）* 10
    // 保证不超时，不超内存，调度正确

    double real_address = metrics.addressingDuration;
    double real_tapeBeltWear = metrics.tapeBeltWear;
    double real_tapeMotorWear = metrics.tapeMotorWear;



    #ifdef LOCAL
        printf("基线的情况\n");
        printf("寻道时间 : %lf, 带体磨损 : %lf, 电机磨损 : %lf\n", baseline_address, baseline_tapeBeltWear, baseline_tapeMotorWear);
        printf("实际调度的情况\n");
        printf("寻道时间 : %lf, 带体磨损 : %lf, 电机磨损 : %lf\n", real_address, real_tapeBeltWear, real_tapeMotorWear);
        // double score = (baselineRes - imporveRes) * 10 + (20 - metrics.algorithmRunningDuration / 1000) * 10;
        // printf("score : %lf\n", score);
    #endif

    // printf("\n\nOutput sequence: [");
    // for (uint32_t i = 0; i < output->len; i++) {
    //     printf("%u, ", output->sequence[i]);
    // }
    // printf("]\n\n\n");

    free(inputParam->ioVec.ioArray);
    free(inputParam);
    free(output->sequence);
    free(output);

    return RETURN_OK;
}