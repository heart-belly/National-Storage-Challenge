
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "algorithm.h"

#ifndef DEBUG
#define DEBUG
#endif
#define DEBUG1
/**
  * @brief  随机数生成器
 **/
uint32_t x = 123456789;
uint32_t y = 362436069;
uint32_t z = 521288629;


// 标记正常加速以及正常减速再加速需要的最小时间间隔
int SE = 8225;
int SE0 = 5941;


int PdMaxPos, rdMinPos;
int SameDircetTerval;
int addRoute, turnRoute, rturnRoute;
int turnInterval, rturnInterval;

// 标记数组(主要为了暴力搜索使用)
bool st[10000 + 10];
uint32_t ioCount;
double best = 0;
HeadInfo info1;
HeadInfo info2;
TapeBeltSegWearInfo segWearInfo = {0};
uint32_t readDurationSum, beltSum, MotorSum;
uint32_t s = 0;
// 禁忌表
uint16_t* TabuList[2000];//禁忌表

// 记时
struct timeval1 start, end;

// 定义可能的状态选择（三种延时的占比可能）
double backup[3] = {0.3, 0.5, 0.2}; // 备份（绝大部分是顺序io)
double hdd[3] = {0.5, 0.3, 0.2}; // 高性能(io随机给出)

// 保存基线的情况
double baselineAD, baselineBelt, baselineMotor;

// 针对io数量为50的做法（减少启发的计算开销）
uint32_t threeCost[1000 + 10][1000 + 10][3]; // 0 : 寻道开销， 1 ： 磁带磨损开销; 2 : 电机磨损开销
uint32_t headToFirst[1000 + 10][3];



int cmp2(const void *a, const void *b);

void tanxindiaodu(const InputParam *input, OutputParam *output);

inline uint32_t randN() {
		x ^= x << 16;
		x ^= x >> 5;
		x ^= x << 1;
		unsigned int t = x;
		x = y; y = z; z = t ^ x ^ y;
		return z;		
};

inline int nextN(int x) {return randN() % x;} // [0, x]
inline int nextNN(int a, int b) {return a + (randN() % (b - a));} // [a, b)
inline double nextN_double() {return (randN() + 0.5) * (1.0 / 4294967296.0);}


inline void preDealData(const InputParam *input) {

}

/**
 * @brief  算法接口
 * @param  input            输入参数
 * @param  output           输出参数
 * @return int32_t          返回成功或者失败，RETURN_OK 或 RETURN_ERROR
 */
inline int32_t IOScheduleAlgorithm(const InputParam *input, OutputParam *output)
{
    int32_t ret;

    /* 算法示例：先入先出算法 */
    output->len = input->ioVec.len;
    for (uint32_t i = 0; i < output->len; i++) {
        output->sequence[i] = input->ioVec.ioArray[i].id;
    }



    return RETURN_OK;
}

/**
 * @brief  scan(默认从左到右)
 * @param  input            输入参数
 * @param  output           输出参数
 * @return int32_t          返回成功或者失败，RETURN_OK 或 RETURN_ERROR
 */
inline uint32_t IOScheduleAlgorithmScan(const InputParam *input, OutputParam *output)
{
    int32_t ret;

    /* 算法示例：scan算法 */
    output->len = input->ioVec.len;
    for (uint32_t i = 0; i < output->len; i++) {
        output->sequence[i] = input->ioVec.ioArray[i].id;
    }
    // 获得当前的磁头位置
    uint32_t lpos = input->headInfo.lpos;
    // 遍历得到从左到右的io调度数量以及从右到左的io调度数据
    int leftIo = 0, rightIo = 0;
    for (uint32_t i = 0; i < output->len; i++) {
        if (input->ioVec.ioArray[i].startLpos <= input->ioVec.ioArray[i].endLpos) {
            leftIo ++;
        } else {
            rightIo ++;
        }
    }

    // 创建小顶堆
    Heap* minHeap = createHeap(10 + leftIo);
    // 创建小顶堆的备份数组(从左到右)
    int* minTemp = (int *)malloc((leftIo + 10) * sizeof(int));
    int minTop = 0;
    // 创建大顶堆
    Heap* maxHeap = createHeap(10 + rightIo);
    // 创建大顶堆的备份数组（从右到左）
    int* maxTemp = (int *)malloc((10 + rightIo) * sizeof(int));
    int maxTop = 0;

    // 在备份数组中保存io的索引
    for (uint32_t i = 0; i < output->len; i++) {
        if (input->ioVec.ioArray[i].startLpos <= input->ioVec.ioArray[i].endLpos) {
            minTemp[minTop ++] = i;
        } else {
            maxTemp[maxTop ++ ] = i;
        }
    }
    // 磁头开始的时候从左到右走
    uint32_t curLpos = lpos;

    for (int i = 0; i < minTop; i ++ ) {
        int index = minTemp[i];
        if (input->ioVec.ioArray[index].startLpos >= curLpos) {
            // 放入小顶堆
            IoInfo info  = {input->ioVec.ioArray[index].id, input->ioVec.ioArray[index].startLpos, input->ioVec.ioArray[index].endLpos};
            insertMin(minHeap, info);
        }
    }
    // 创建答案数组以及bool数组表示有无调度
    int* ans = (int *)malloc((output->len) * sizeof(int));
    int ansTop = 0;
    _Bool *st = (_Bool *)malloc((output->len + 10) * sizeof(_Bool)); // 索引为io的id
    for (int i = 1; i <= output->len; ++ i) st[i] = 0;
    // 第一次遍历(队列不为空)
    while (minHeap->size > 0) {
        // 得到队头元素并删除
        IoInfo io = extractMin(minHeap);
        int id = io.id, start = io.startPos, end = io.endPos;
        if (start >= curLpos) {
            ans[ansTop ++ ] = id;
            st[id] = 1;
            curLpos = end;
        }
    }

    // 遍历到wrap的尾巴位置
    _Bool flag = 0; // false : 从右到左; true : 从左到右
    while (ansTop < output->len) {
        if (flag) {
            // 将备份数组中的io调度全部放入小顶堆中
            int cnt = 0;
            for (int i = 0; i < minTop; i ++ ) {
                // 放入小顶堆
                int index = minTemp[i];
                IoInfo info  = {input->ioVec.ioArray[index].id, input->ioVec.ioArray[index].startLpos, input->ioVec.ioArray[index].endLpos};
                if (!st[input->ioVec.ioArray[index].id]) {
                    insertMin(minHeap, info);  
                    cnt ++; 
                }             
            }

            minTop = 0;
            if (cnt > 0) {
                // 队列中有元素（取出队头）
                IoInfo io = extractMin(minHeap);
                int id = io.id, start = io.startPos, end = io.endPos;
                ans[ansTop ++ ] = id;
                st[id] = 1;
                curLpos = end;

                while (minHeap->size > 0) {
                    IoInfo io = extractMin(minHeap);
                    int id = io.id, start = io.startPos, end = io.endPos;
                    if (start >= curLpos) {
                        ans[ansTop ++ ] = id;
                        st[id] = 1;
                        curLpos = end;
                    } else {
                        minTemp[minTop ++] = id - 1;

                    }   
                }

            }

            flag = 0;
        } else {
            int cnt = 0;
            for (int i = 0; i < maxTop; i ++ ) {
                // 放入大顶堆
                int index = maxTemp[i];
                IoInfo info  = {input->ioVec.ioArray[index].id, input->ioVec.ioArray[index].startLpos, input->ioVec.ioArray[index].endLpos};
                if (!st[input->ioVec.ioArray[index].id]) {
                    insertMax(maxHeap, info);  
                    cnt ++; 
                }             
            }

            maxTop = 0;
            if (cnt > 0) {
                // 队列中有元素（取出队头）
                IoInfo io = extractMax(maxHeap);
                int id = io.id, start = io.startPos, end = io.endPos;
                ans[ansTop ++ ] = id;
                st[id] = 1;
                curLpos = end;


                while (maxHeap->size > 0) {
                    IoInfo io = extractMax(maxHeap);
                    int id = io.id, start = io.startPos, end = io.endPos;
                    if (start <= curLpos) {
                        ans[ansTop ++ ] = id;
                        st[id] = 1;
                        curLpos = end;
                    } else {
                        maxTemp[maxTop ++] = id - 1;
                    }   
                }
            }
            flag = 1;
        }
    }


    // 将答案写进output
    for (int i = 0; i < output->len; i ++ ) {
        output->sequence[i] = ans[i];
    }


    free(minHeap);
    free(minTemp);
    free(maxHeap);
    free(maxTemp);
    free(st);
    free(ans);



    return RETURN_OK;
}
/**
 * @brief  scan（从右到左）
 * @param  input            输入参数
 * @param  output           输出参数
 * @return int32_t          返回成功或者失败，RETURN_OK 或 RETURN_ERROR
 */
inline uint32_t IOScheduleAlgorithmScanRightToLeft(const InputParam *input, OutputParam *output)
{
    int32_t ret;

    /* 算法示例：scan算法 */
    output->len = input->ioVec.len;
    for (uint32_t i = 0; i < output->len; i++) {
        output->sequence[i] = input->ioVec.ioArray[i].id;
    }
    // 获得当前的磁头位置
    uint32_t lpos = input->headInfo.lpos;
    // 遍历得到从左到右的io调度数量以及从右到左的io调度数据
    int leftIo = 0, rightIo = 0;
    for (uint32_t i = 0; i < output->len; i++) {
        if (input->ioVec.ioArray[i].startLpos <= input->ioVec.ioArray[i].endLpos) {
            leftIo ++;
        } else {
            rightIo ++;
        }
    }


    // 创建小顶堆
    Heap* minHeap = createHeap(10 + leftIo);
    // 创建小顶堆的备份数组(从左到右)
    int* minTemp = (int *)malloc((leftIo + 10) * sizeof(int));
    int minTop = 0;
    // 创建大顶堆
    Heap* maxHeap = createHeap(10 + rightIo);
    // 创建大顶堆的备份数组（从右到左）
    int* maxTemp = (int *)malloc((10 + rightIo) * sizeof(int));
    int maxTop = 0;
    // 在备份数组中保存io的索引
    for (uint32_t i = 0; i < output->len; i++) {
        if (input->ioVec.ioArray[i].startLpos <= input->ioVec.ioArray[i].endLpos) {
            minTemp[minTop ++] = i;
        } else {
            maxTemp[maxTop ++ ] = i;
        }
    }
    // 磁头开始的时候从右到左走
    uint32_t curLpos = lpos;

    for (int i = 0; i < maxTop; i ++ ) {
        int index = maxTemp[i];
        if (input->ioVec.ioArray[index].startLpos <= curLpos) {
            // 放入小顶堆
            IoInfo info  = {input->ioVec.ioArray[index].id, input->ioVec.ioArray[index].startLpos, input->ioVec.ioArray[index].endLpos};
            insertMax(maxHeap, info);
        }
    }
    // 创建答案数组以及bool数组表示有无调度
    int* ans = (int *)malloc((output->len) * sizeof(int));
    int ansTop = 0;
    _Bool *st = (_Bool *)malloc((output->len + 10) * sizeof(_Bool)); // 索引为io的id
    // 第一次遍历(队列不为空)
    for (int i = 1; i <= output->len; ++ i) st[i] = 0;
    while (maxHeap->size > 0) {
        // 得到队头元素并删除
        IoInfo io = extractMax(maxHeap);
        int id = io.id, start = io.startPos, end = io.endPos;
        if (start <= curLpos) {
            ans[ansTop ++ ] = id;
            st[id] = 1;
            curLpos = end;
        }
    }
    // 遍历到wrap的尾巴位置
    _Bool flag = 0; // false : 从左到右; true : 从右到左
    while (ansTop < output->len) {
        if (flag) {
            int cnt = 0;
            for (int i = 0; i < maxTop; i ++ ) {
                // 放入小顶堆
                int index = maxTemp[i];
                IoInfo info  = {input->ioVec.ioArray[index].id, input->ioVec.ioArray[index].startLpos, input->ioVec.ioArray[index].endLpos};
                int id = input->ioVec.ioArray[index].id;
                if (!st[input->ioVec.ioArray[index].id]) {
                    insertMax(maxHeap, info);  
                    cnt ++; 
                }             
            }
            // printf("aaaaaaaaaaaaaaaaaa%d\n", cnt);
            maxTop = 0;
            if (cnt > 0) {
                // 队列中有元素（取出队头）
                IoInfo io = extractMax(maxHeap);
                int id = io.id, start = io.startPos, end = io.endPos;
                ans[ansTop ++ ] = id;
                st[id] = 1;
                curLpos = end;

                while (maxHeap->size > 0) {
                    IoInfo io = extractMax(maxHeap);
                    int id = io.id, start = io.startPos, end = io.endPos;
                    if (start <= curLpos) {
                        ans[ansTop ++ ] = id;
                        st[id] = 1;
                        curLpos = end;
                    } else {
                        maxTemp[maxTop ++] = id - 1;

                    }   
                }

            }

            flag = 0;
        } else {
            int cnt = 0;
            for (int i = 0; i < minTop; i ++ ) {
                // 放入大顶堆
                int index = minTemp[i];
                IoInfo info  = {input->ioVec.ioArray[index].id, input->ioVec.ioArray[index].startLpos, input->ioVec.ioArray[index].endLpos};
                if (!st[input->ioVec.ioArray[index].id]) {
                    insertMin(minHeap, info);  
                    cnt ++; 
                }             
            }
            // printf("bbbbbbbbbbbbbbb%d\n", cnt);
            // printf("bbbbbbbbbbbbbbb%d\n", ansTop);
            minTop = 0;
            if (cnt > 0) {
                // 队列中有元素（取出队头）
                IoInfo io = extractMin(minHeap);
                int id = io.id, start = io.startPos, end = io.endPos;
                ans[ansTop ++ ] = id;
                st[id] = 1;
                curLpos = end;


                while (minHeap->size > 0) {
                    IoInfo io = extractMin(minHeap);
                    int id = io.id, start = io.startPos, end = io.endPos;
                    if (start >= curLpos) {
                        ans[ansTop ++ ] = id;
                        st[id] = 1;
                        curLpos = end;
                    } else {
                        minTemp[minTop ++] = id - 1;
                    }   
                }
            }
            flag = 1;
        }
        // printf("-----------------------------------------");
        // for (int i = 1; i <= output->len; ++ i) {
        //     printf("%d\n", st[i]);
        // }
        // printf("-----------------------------------------");
    }


    // 将答案写进output
    for (int i = 0; i < output->len; i ++ ) {
        output->sequence[i] = ans[i];
    }


    free(minHeap);
    free(minTemp);
    free(maxHeap);
    free(maxTemp);
    free(st);
    free(ans);


    return RETURN_OK;
}

/**
 * @brief  baseline(最近邻算法)
 * @param  input            输入参数
 * @param  output           输出参数
 * @return int32_t          返回成功或者失败，RETURN_OK 或 RETURN_ERROR
 */
inline uint32_t IOScheduleAlgorithmNearest(const InputParam *input, OutputParam *output)
{
    int32_t ret;

    /* 算法示例：Nearest算法 */
    output->len = input->ioVec.len;
    for (uint32_t i = 0; i < output->len; i++) {
        output->sequence[i] = input->ioVec.ioArray[i].id;
    }

    // 根据start排序
 // 遍历得到从左到右的io调度数量以及从右到左的io调度数据
    int leftIo = 0, rightIo = 0;
    IoInfo *info = (IoInfo *)malloc((output->len) * sizeof(IoInfo));
    for (int i = 0; i < output->len; i++) {
        if (input->ioVec.ioArray[i].startLpos <= input->ioVec.ioArray[i].endLpos) {
            leftIo ++;
        } else {
            rightIo ++;
        }

        // info[i] = {input->ioVec.ioArray[i].id ,input->ioVec.ioArray[i].startLpos,input->ioVec.ioArray[i].endLpos};
        info[i].id = input->ioVec.ioArray[i].id, info[i].startPos = input->ioVec.ioArray[i].startLpos, info[i].endPos = input->ioVec.ioArray[i].endLpos;
    }
    qsort(info, output->len, sizeof(IoInfo),cmp);   

    for (int i = 0; i < output->len; i ++ ) {
        output->sequence[i] = info[i].id;
    }



    return RETURN_OK;
}

inline uint32_t IOScheduleAlgorithmBaseline(const InputParam *input, OutputParam *output, bool flag)
{
    int32_t ret;

    /* 算法示例：Nearest算法 */
    output->len = input->ioVec.len;
    for (uint32_t i = 0; i < output->len; i++) {
        output->sequence[i] = input->ioVec.ioArray[i].id;
    }

    // 根据start排序
 // 遍历得到从左到右的io调度数量以及从右到左的io调度数据
    int leftIo = 0, rightIo = 0;
    IoInfo *infoLeft = (IoInfo *)malloc((output->len) * sizeof(IoInfo));
    IoInfo *infoRight = (IoInfo *)malloc((output->len) * sizeof(IoInfo));
    for (int i = 0; i < output->len; i++) {
        if (input->ioVec.ioArray[i].startLpos <= input->ioVec.ioArray[i].endLpos) {
            infoLeft[leftIo].id = input->ioVec.ioArray[i].id, infoLeft[leftIo].startPos = input->ioVec.ioArray[i].startLpos,
            infoLeft[leftIo].endPos = input->ioVec.ioArray[i].endLpos;
            leftIo ++;
        } else {
            infoRight[rightIo].id = input->ioVec.ioArray[i].id, infoRight[rightIo].startPos = input->ioVec.ioArray[i].startLpos,
            infoRight[rightIo].endPos = input->ioVec.ioArray[i].endLpos;
            rightIo ++;
        }
    }
    if (leftIo > 0) qsort(infoLeft, leftIo, sizeof(IoInfo), cmp);  
    if (rightIo > 0) qsort(infoRight, rightIo, sizeof(IoInfo), cmpp); 

    int index = 0;


    if (flag) {
        for (int i = 0; i < leftIo; ++ i) {
            output->sequence[index ++] = infoLeft[i].id;
        }
        for (int i = 0; i < rightIo; i ++ ) {
            output->sequence[index ++] = infoRight[i].id;
        }        
    } else {
        for (int i = 0; i < rightIo; i ++ ) {
            output->sequence[index ++] = infoRight[i].id;
        } 

        for (int i = 0; i < leftIo; ++ i) {
            output->sequence[index ++] = infoLeft[i].id;
        }  
    }


    free(infoLeft);
    free(infoRight);

    return RETURN_OK;
}


/**
 * @brief  算法运行的主入口
 * @param  input            输入参数
 * @param  output           输出参数
 * @return uint32_t          返回成功或者失败，RETURN_OK 或 RETURN_ERROR
 */
inline int32_t AlgorithmRun(const InputParam *input, OutputParam *output)
{
    gettimeofday(&start, NULL);
    // 运行基线算法
    saveBaseline(input, output);
    double fBaseline = 0.0;
    double bestScore = fBaseline;

    int *answer = (int *)malloc((output->len) * sizeof(int));
    // 保存结果
    for (int i = 0; i < output->len; ++ i) answer[i] = output->sequence[i];

    ioCount = input->ioVec.len;
    int32_t ret;
    if (ioCount <= 10) {
        // Bruteforce(input, output);
        IOScheduleAlgorithmBaseline(input, output, false);
        double rBaseline = getBackupScore(input, output);
        if (rBaseline > bestScore) {
            bestScore = rBaseline;
            for (int i = 0; i < output->len; ++ i) answer[i] = output->sequence[i];
        }
        IOScheduleAlgorithmScan(input, output);
        double ScanLR = getBackupScore(input, output);
        if (ScanLR > bestScore) {
            bestScore = ScanLR;
            for (int i = 0; i < output->len; ++ i) answer[i] = output->sequence[i];
        }
        IOScheduleAlgorithmScanRightToLeft(input, output);
        double ScanRL = getBackupScore(input, output);
        if (ScanRL > bestScore) {
            bestScore = ScanRL;
            for (int i = 0; i < output->len; ++ i) answer[i] = output->sequence[i];
        }

        #ifdef DEBUG
            printf("正向基线:%lf\n", fBaseline);
            printf("反向基线: %lf\n", rBaseline);
            printf("从左到右扫描: %lf\n", ScanLR);
            printf("从右到左扫描: %lf\n", ScanRL);
        #endif

        
    } else if (ioCount <= 2000){

        IOScheduleAlgorithmBaseline(input, output, false);

        double rBaselineScore = getBackupScore(input, output);
        double rBaselineScorex = getHddScore(input, output);

        if (rBaselineScore > bestScore) {
            bestScore = rBaselineScore;
            for (int i = 0; i < output->len; ++ i) answer[i] = output->sequence[i];
        }
        // printf("ssssssssssss\n");
        IOScheduleAlgorithmScan(input, output);
        // printf("ssssssssssss\n");
        double scanLRScore = getBackupScore(input, output);
        double scanLRScorex = getHddScore(input, output);

        if (scanLRScore > bestScore) {
            bestScore = scanLRScore;
            for (int i = 0; i < output->len; ++ i) answer[i] = output->sequence[i];
        }

        IOScheduleAlgorithmScanRightToLeft(input, output);

        double scanRLScore = getBackupScore(input, output);
        double scanRLScorex = getHddScore(input, output);

        if (scanRLScore > bestScore) {
            bestScore = scanRLScore;
            for (int i = 0; i < output->len; ++ i) answer[i] = output->sequence[i];
        }
        tanxindiaodu(input, output);
        double nearestScore = getBackupScore(input, output);
        double nearestScorex = getHddScore(input, output);


        if (nearestScore > bestScore) {
            bestScore = nearestScore;
            for (int i = 0; i < output->len; ++ i) answer[i] = output->sequence[i];
        }
        #ifdef DEBUG
            printf("********backup**********\n");
            printf("rbaselineScore: %lf\n", rBaselineScore);
            printf("scanLRScore: %lf\n", scanLRScore);
            printf("scanRLScore: %lf\n", scanRLScore);
            printf("nearestScore: %lf\n", nearestScore);
            printf("********hdd**********\n");
            printf("rbaselineScore: %lf\n", rBaselineScorex);
            printf("scanLRScore: %lf\n", scanLRScorex);
            printf("scanRLScore: %lf\n", scanRLScorex);
            printf("nearestScore: %lf\n", nearestScorex);
        #endif 


    } else {
        tanxindiaodu(input, output);
        double nearestScore = getBackupScore(input, output);
        double nearestScorex = getHddScore(input, output);


        if (nearestScore > bestScore) {
            bestScore = nearestScore;
            for (int i = 0; i < output->len; ++ i) answer[i] = output->sequence[i];
        }
    }

    for (int i = 0; i < output->len; ++ i) output->sequence[i] = answer[i];


    // /******************************************************/

    // info1.status = 1, info1.lpos = 50000, info1.wrap = 0;
    // info2.status = 1, info2.lpos = 51000, info2.wrap = 1;
    // for (int i = 0; i < 10; ++ i) {
    //     info2.lpos = 50000 + i;
    //     printf("info2.pos %d\n",  info2.lpos);
    //     printf("really : %d\n", BeltWearTimes(&info1, &info2, &segWearInfo));
    //     printf("sub: %d\n", abs(info1.lpos - info2.lpos));
    //     printf("really - sub: %d\n", BeltWearTimes(&info1, &info2, &segWearInfo) - abs(info1.lpos - info2.lpos));
    //     printf("************************************************\n");
    // }



    if (ioCount > 10 && ioCount <= 1000 && (baselineMotor >= 9 || bestScore == 0)) {
        // 由于系统提供的磁带磨损计算开销太大，经过我的实践发现，在符合一些情况下，加数的时间是固定的
        info1.status = 1, info1.lpos = 30000, info1.wrap = 0;
        info2.status = 1, info2.lpos = 20000, info2.wrap = 0;
        // 二分正常加速需要的最小时间间隔()
        addRoute = BeltWearTimes(&info1, &info2, &segWearInfo) - 10000;
        int l = MAX_LPOS - addRoute, r = MAX_LPOS;
        // 寻找第一个不满足的位置
        while (l < r) {
            int mid = (l + r) / 2;
            if (check1(mid)) {
                l = mid + 1;
            } else {
                r = mid;
            }
        }
        PdMaxPos = l - 1;
        rdMinPos = 0 + (MAX_LPOS - PdMaxPos);
        // printf("XXXXXXXXXXXXXX%d\n", PdMaxPos);

        /***************************************************************/
        info1.status = 1, info1.lpos = 20000, info1.wrap = 0;
        info2.status = 1, info2.lpos = 10000, info2.wrap = 0;
        // 二分正常加速需要的最小时间间隔()
        l = 0, r = l + addRoute;
        // 寻找第一个不满足的位置
        while (l < r) {
            int mid = (l + r) / 2;
            if (check1_1(mid)) {
                r = mid;
            } else {
                l = mid + 1;
            }
        }
        SameDircetTerval = l + 100;
        /*************************掉头的最小时间间隔*************************************/
        info1.status = 1, info1.lpos = 30000, info1.wrap = 0;
        info2.status = 1, info2.lpos = 20000, info2.wrap = 1;
        turnRoute = BeltWearTimes(&info1, &info2, &segWearInfo) - 10000;
        l = 20000, r = 30000;
        while (l < r) {
            int mid = (l + r) / 2;
            if (check2(mid)) {
                l = mid + 1;
            } else {
                r = mid;
            }
        }
        turnInterval = (30000 - l) + 100;
        // printf("ssssssssssssssssssssss%d\n", turnInterval);
        /*****************************************************************************/
        info1.status = 1, info1.lpos = 20000, info1.wrap = 0;
        info2.status = 1, info2.lpos = 21000, info2.wrap = 1;
        rturnRoute = BeltWearTimes(&info1, &info2, &segWearInfo) - 1000;
        l = MAX_LPOS - 10000, r = MAX_LPOS;
        while (l < r) {
            int mid = (l + r) / 2;
            if (check3(mid)) {
                l = mid + 1;
            } else {
                r = mid;
            }
        }
        rturnInterval = (MAX_LPOS - l) + 10;
        // 计算SE 和 SE0
        info1.status = 1, info1.lpos = 20000, info1.wrap = 0;
        info2.status = 1, info2.lpos = 10000, info2.wrap = 2;
        SE = abs(10000 - BeltWearTimes(&info1, &info2, &segWearInfo));
        // printf("SE : %d\n", SE);
        info1.status = 0, info1.lpos = 20000, info1.wrap = 0;
        SE0 = abs(10000 - BeltWearTimes(&info1, &info2, &segWearInfo));
        // printf("SE0 : %d\n", SE0);


        struct timeval1 e1;
        gettimeofday(&e1, NULL);  // 记录结束时间
        // 统计两个io之间（寻道时间，磁带磨损，电机磨损）
        int canUseHillClimb = 1;
        for (int i = 0; i < output->len; ++ i) {
            for (int j = 0; j < output->len; ++ j) {
                if (i == j) continue;
                info1.status = 1, info1.lpos = input->ioVec.ioArray[i].endLpos, info1.wrap = input->ioVec.ioArray[i].wrap;
                info2.status = 1, info2.lpos = input->ioVec.ioArray[j].startLpos, info2.wrap = input->ioVec.ioArray[j].wrap;
                // 寻道时间
                threeCost[i + 1][j + 1][0] = SeekTimeCalculate(&info1, &info2);
                // 带体磨损
                // // TapeBeltSegWearInfo segWearInfo = {0};
                threeCost[i + 1][j + 1][1] = getBeltWear(&info1, &info2);
                // printf("really: %d\n", threeCost[i + 1][j + 1][1]);
                // printf("my caculate: %d\n", getBeltWear(&info1, &info2));
                // if (abs(getBeltWear(&info1, &info2) - threeCost[i + 1][j + 1][1]) > 2) {
                //     printf("%d %d\n", info1.wrap, info1.lpos);
                //     printf("%d %d\n", info2.wrap, info2.lpos);
                //     printf("really: %d\n", threeCost[i + 1][j + 1][1]);
                //     printf("my caculate: %d\n", getBeltWear(&info1, &info2));
                // }
                // // oldPartBelt += getBeltWear(&info1, &info2);
                // // 电机磨损
                threeCost[i + 1][j + 1][2]=  MotorWearTimes(&info1, &info2);
            }
            gettimeofday(&end, NULL);  // 记录结束时间
            long seconds, useconds;    // 秒数和微秒数
            seconds = end.tv_sec - e1.tv_sec;
            useconds = end.tv_usec - e1.tv_usec;
            uint32_t costTime = ((seconds)*1000000 + useconds) / 1000000.0;
            if (costTime > 15) {
                canUseHillClimb = 0;
                break;
            }
        }
        // 计算磁头初始位置到每个io的开销
        for (int i = 0; i < output->len; ++ i) {
                info1.status = 0, info1.lpos = input->headInfo.lpos, info1.wrap = input->headInfo.wrap;
                info2.status = 1, info2.lpos = input->ioVec.ioArray[i].startLpos, info2.wrap = input->ioVec.ioArray[i].wrap;  
                headToFirst[i + 1][0] = SeekTimeCalculate(&info1, &info2);
                headToFirst[i + 1][1] = getBeltWear(&info1, &info2);
                headToFirst[i + 1][2] = MotorWearTimes(&info1, &info2);
        }

        gettimeofday(&end, NULL);  // 记录结束时间
        long seconds, useconds;    // 秒数和微秒数
        seconds = end.tv_sec - e1.tv_sec;
        useconds = end.tv_usec - e1.tv_usec;
        uint32_t costTime = ((seconds)*1000000 + useconds) / 1000000.0;

        printf("预处理的时间开销:%d\n", costTime);


        if (canUseHillClimb == 1) {
            hillClimbInsert(input, output);
        }
    
    }



    #ifdef DEBUG1
        double backup = getBackupScore(input, output);
        double hdd = getHddScore(input, output);

        printf("bakcup: %lf\n", backup);
        printf("hdd: %lf\n", hdd);
    #endif



    free(answer);

    return RETURN_OK;
}

inline int cmp2(const void *a, const void *b) {
    return (int) (((*(unsigned long long *) a) >> 32) - ((*(unsigned long long *) b) >> 32));
}


void tanxindiaodu(const InputParam *input, OutputParam *output) {
    static HeadInfo pointInInfo[MAX_IO_NUM + 1];
    static HeadInfo pointOutInfo[MAX_IO_NUM + 1];
    pointInInfo[0] = input->headInfo;
    pointOutInfo[0] = input->headInfo;

    static int bingcha[MAX_IO_NUM + 1];

    static int nearTable[MAX_IO_NUM + 1][2];
    static int in[MAX_IO_NUM + 1];
    static int out[MAX_IO_NUM + 1];
    int n = (int) (1 + input->ioVec.len);
    nearTable[0][0] = -1;
    nearTable[0][1] = -1;
    bingcha[0] = 0;
    for (int i = 1; i < n; i++) {
        bingcha[i] = i;
        pointInInfo[i].status = HEAD_RW;
        pointInInfo[i].wrap = input->ioVec.ioArray[i - 1].wrap;
        pointInInfo[i].lpos = input->ioVec.ioArray[i - 1].startLpos;
        pointOutInfo[i].status = HEAD_RW;
        pointOutInfo[i].wrap = input->ioVec.ioArray[i - 1].wrap;
        pointOutInfo[i].lpos = input->ioVec.ioArray[i - 1].endLpos;
        nearTable[i][0] = -1;
        nearTable[i][1] = -1;
    }



    //todo in,out分别排序，out去二分查找in的位置，求前n个

    int totalTime = 0;
    for (int i = 0; i < n; i++) {
        in[i] = i;
        out[i] = i;
    }
    int originN = n;

    const int k = 500;

    while (n != 1) {
        static unsigned long long inSort[MAX_IO_NUM + 1];
        static unsigned long long outSort[MAX_IO_NUM + 1];
        for (int i = 0; i < n; i++) {
            int index = in[i];
            int left = (int) pointInInfo[index].lpos;
            inSort[i] = ((unsigned long long) left << 32) + i;
        }
        for (int i = 0; i < n; i++) {
            int index = out[i];
            int right = (int) pointOutInfo[index].lpos;
            outSort[i] = ((unsigned long long) right << 32) + i;
        }
        qsort(inSort, n, sizeof(unsigned long long), cmp2);
        qsort(outSort, n, sizeof(unsigned long long), cmp2);

        static int sortDisIn[MAX_IO_NUM + 1][2];
        static int sortDisOut[MAX_IO_NUM + 1][2];
        for (int i = 0; i < n; i++) {
            sortDisIn[i][0] = (int) (inSort[i] >> 32);
            sortDisIn[i][1] = (int) (inSort[i] & 0xFFFFFFFF);
            sortDisOut[i][0] = (int) (outSort[i] >> 32);
            sortDisOut[i][1] = (int) (outSort[i] & 0xFFFFFFFF);
        }

        //对每一个end，去找候选的n个中最近的
        int leftPos = 0, rightPos = min(k, n - 1);//,一般情况下包含同一块不能到0，只能0出发
        static int indexNearest[MAX_IO_NUM + 1][2];
        static unsigned long long indexSortNearest[MAX_IO_NUM + 1];
        for (int i = 0; i < n; i++) {
            int dis = sortDisOut[i][0];
            int index1 = sortDisOut[i][1];
            int leftDis = abs(dis - sortDisIn[leftPos][0]);
            while (rightPos + 1 < n && abs(dis - sortDisIn[rightPos + 1][0]) < leftDis) {
                leftPos++;
                rightPos++;
                leftDis = abs(dis - sortDisIn[leftPos][0]);
            }

            indexNearest[index1][0] = 0x3f3f3f3f;
            int realIndex1 = out[index1];
            for (int j = leftPos; j <= rightPos; j++) {
                int index2 = sortDisIn[j][1];
                int realIndex2 = in[index2];
                if (index1 == index2 || realIndex2 == 0) {
                    //同一块,不可以往0插入
                    continue;
                }

                int realTime = (int)((SeekTimeCalculate(&pointOutInfo[realIndex1], &pointInInfo[realIndex2])));

                // double readTime = (SeekTimeCalculate(&pointOutInfo[realIndex1], &pointInInfo[realIndex2]));
                // printf("%lf %lf %lf\n", readTime, belt, motor);

                
                // double x = (hdd[0] / baselineAD * (readTime)  + hdd[1] * belt / baselineBelt + hdd[2] / baselineMotor * motor);
                // // printf("%lf\n", x);
                // // // // printf("%lf\n", realTime * 1e10);
                // x = belt * 1e6;
                // // printf("%lf\n", x);
                // realTime = x;
                // // printf("%d\n", realTime);


                if (realTime < indexNearest[index1][0]) {
                    //只取最近的
                    indexNearest[index1][0] = realTime;
                    indexNearest[index1][1] = index2;

                }
            }
            indexSortNearest[i] = ((unsigned long long) indexNearest[index1][0] << 32) + index1;
        }
        qsort(indexSortNearest, n, sizeof(unsigned long long), cmp2);

        static int sortDisTotal[MAX_IO_NUM + 1][2];
        for (int i = 0; i < n; i++) {
            sortDisTotal[i][0] = (int) (indexSortNearest[i] >> 32);
            sortDisTotal[i][1] = (int) (indexSortNearest[i] & 0xFFFFFFFF);
        }

        //合并，近的开始
        int newN = 0;
        static int newIn[MAX_IO_NUM + 1];
        static int newOut[MAX_IO_NUM + 1];
        static int endStart[MAX_IO_NUM + 1];
        static int startEnd[MAX_IO_NUM + 1];
        for (int i = 0; i < n; i++) {
            endStart[out[i]] = in[i];
            startEnd[in[i]] = out[i];
        }
        //

        for (int i = 0; i < n; i++) {
            int dis = sortDisTotal[i][0];
            int index1 = sortDisTotal[i][1];
            int index2 = indexNearest[sortDisTotal[i][1]][1];
            //todo index2没被连接
            int realIndex1 = out[index1];
            int realIndex2 = in[index2];
            int fa1 = bingcha[realIndex1];
            int fa2 = bingcha[realIndex2];
            while (bingcha[fa1] != fa1) {
                bingcha[fa1] = bingcha[bingcha[fa1]];
                fa1 = bingcha[fa1];
            }
            while (bingcha[fa2] != fa2) {
                bingcha[fa2] = bingcha[bingcha[fa2]];
                fa2 = bingcha[fa2];
            }
            if (fa1 != fa2 && nearTable[realIndex2][0] == -1 && nearTable[realIndex1][1] == -1) {
                //没被连接
                bingcha[fa2] = fa1;
                nearTable[realIndex2][0] = realIndex1;
                nearTable[realIndex1][1] = realIndex2;
                totalTime += dis;
                //两块合并了
                newIn[newN] = endStart[realIndex1];
                newOut[newN] = startEnd[realIndex2];
                endStart[newOut[newN]] = newIn[newN];
                startEnd[newIn[newN]] = newOut[newN];
            } else {
                newIn[newN] = in[index1];
                newOut[newN] = out[index1];
            }
            newN++;
        }
        n = 0;
        for (int i = 0; i < newN; i++) {
            if (nearTable[newIn[i]][0] == -1 && nearTable[newOut[i]][1] == -1) {
                in[n] = newIn[i];
                out[n] = newOut[i];
                n++;
            }
        }
        int realN = 0;
        for (int i = 0; i < originN; i++) {
            if (nearTable[i][0] == -1) {
                realN++;
            }
        }

        //printf("n:%d realN:%d\n", n, realN);
    }

    //合并完进行输出


    //printf("totalTime:%d\n", totalTime);
    //更新了
    //更新in out，重开,
    //按照链表恢复数据
    int cur = 0;
    for (int i = 0; i < output->len; i++) {
        output->sequence[i] = input->ioVec.ioArray[nearTable[cur][1] - 1].id;
        cur = nearTable[cur][1];
    }
    // double d = energy(input, output);
    // AccessTime accessTime = {0};
    // TotalAccessTime(input, output, &accessTime);
    // printf("%lf", d);
    // printf("%lf", accessTime.readDuration);


}


// 初始化堆
inline Heap* createHeap(int capacity) {
    Heap* heap = (Heap*)malloc(sizeof(Heap));
    heap->data = (IoInfo *)malloc(capacity * sizeof(IoInfo));
    heap->size = 0;
    heap->capacity = capacity;
    return heap;
}
// 交换两个元素
inline void swap(IoInfo *a, IoInfo *b) {
    IoInfo temp = *a;
    *a = *b;
    *b = temp;
}

// 获取父节点的索引
inline int parent(int i) {
    return (i - 1) / 2;
}

// 获取左子节点的索引
inline int leftChild(int i) {
    return 2 * i + 1;
}

// 获取右子节点的索引
inline int rightChild(int i) {
    return 2 * i + 2;
}

// 插入元素到堆中
inline void insertMax(Heap* heap, IoInfo value) {
    if (heap->size == heap->capacity) {
        printf("Heap is full!\\n");
        return;
    }
    heap->size++;
    int i = heap->size - 1;
    heap->data[i] = value;

    // 调整堆
    while (i != 0 && heap->data[parent(i)].startPos < heap->data[i].startPos) {
        swap(&heap->data[i], &heap->data[parent(i)]);
        i = parent(i);
    }
}

// 插入元素到堆中
inline void insertMin(Heap* heap, IoInfo value) {
    if (heap->size == heap->capacity) {
        printf("Heap is full!\\n");
        return;
    }
    heap->size ++;
    int i = heap->size - 1;
    heap->data[i] = value;

    // 调整堆
    while (i != 0 && heap->data[parent(i)].startPos > heap->data[i].startPos) {
        swap(&heap->data[i], &heap->data[parent(i)]);
        i = parent(i);
    }
}

// 调整堆
inline void minHeapify(Heap* heap, int i) {
    int smallest = i;
    int left = leftChild(i);
    int right = rightChild(i);

    if (left < heap->size && heap->data[left].startPos < heap->data[smallest].startPos) {
        smallest = left;
    }
    if (right < heap->size && heap->data[right].startPos < heap->data[smallest].startPos) {
        smallest = right;
    }
    if (smallest != i) {
        swap(&heap->data[i], &heap->data[smallest]);
        minHeapify(heap, smallest);
    }
}

// 删除堆顶元素
inline IoInfo extractMin(Heap* heap) {
    if (heap->size <= 0) {
        IoInfo ret = {-1, -1, -1};
        return  ret;
    }
    if (heap->size == 1) {
        heap->size--;
        return heap->data[0];
    }

    IoInfo root = heap->data[0];
    heap->data[0] = heap->data[heap->size - 1];
    heap->size--;
    minHeapify(heap, 0);

    return root;
}


// 调整堆
inline void maxHeapify(Heap* heap, int i) {
    int largest = i;
    int left = leftChild(i);
    int right = rightChild(i);

    if (left < heap->size && heap->data[left].startPos > heap->data[largest].startPos) {
        largest = left;
    }
    if (right < heap->size && heap->data[right].startPos > heap->data[largest].startPos) {
        largest = right;
    }
    if (largest != i) {
        swap(&heap->data[i], &heap->data[largest]);
        maxHeapify(heap, largest);
    }
}

// 删除堆顶元素
inline IoInfo extractMax(Heap* heap) {
    if (heap->size <= 0) {
        IoInfo ret = {-1, -1, -1};
        return ret;
    }
    if (heap->size == 1) {
        heap->size--;
        return heap->data[0];
    }

    IoInfo root = heap->data[0];
    heap->data[0] = heap->data[heap->size - 1];
    heap->size--;
    maxHeapify(heap, 0);

    return root;
}


inline int cmp(const void *a,const void *b) {
	return (*(IoInfo*)a).startPos - (*(IoInfo*)b).startPos;
}
inline int cmpp(const void *a,const void *b) {
    return - ((*(IoInfo*)a).startPos - (*(IoInfo*)b).startPos);
}
/**暴力寻找最优解***/
/**暴力寻找最优解***/
inline void Bruteforce(const InputParam *input, OutputParam *output) {
    int length = input->ioVec.len;
    /*开辟内存旧序列以及新序列*/
    uint16_t *bestSeries = (uint16_t *) malloc(sizeof (uint16_t) * length);
    uint16_t *oldSeries = (uint16_t *) malloc(sizeof (uint16_t) * length);

    // 初始解
    IOScheduleAlgorithmScan(input, output);
    // 得到初始解的分数（只考虑时间指标） (只计算寻道时间,因为读写时间是一样的)
    best = energySeekPath(input, output);

    for (int i = 0; i < length; ++ i) {
        bestSeries[i] = oldSeries[i] = output->sequence[i];
    }

    Bruteforce1(input, output, 0, bestSeries, oldSeries);

    // 替换最好的结果
    for (int i = 0; i < ioCount; ++ i) {
        output->sequence[i] = bestSeries[i];
    }

    



}

inline void Bruteforce1(const InputParam *input, OutputParam *output, int coveredCount, uint16_t *bestSeries, uint16_t *oldSeries) {
    if (coveredCount == ioCount) {
        for (int j = 0; j < ioCount; ++ j) {
                output->sequence[j] = oldSeries[j];
        }
        if (energySeekPath(input, output) < best) {
            for (int j = 0; j < ioCount; ++ j) {
                bestSeries[j] = oldSeries[j];
            }
            best = energySeekPath(input, output);
        }

        return;

    }

    for (int i = 1; i <= ioCount; ++ i) {
        if (st[i]) continue;

        st[i] = true;

        oldSeries[coveredCount] = i;

        if (coveredCount == 0) {
            info1.status = 0, info1.lpos = input->headInfo.lpos, info1.wrap = input->headInfo.wrap;
            info2.status = 1, info2.lpos = input->ioVec.ioArray[oldSeries[0] - 1].startLpos, info2.wrap = input->ioVec.ioArray[oldSeries[0] - 1].wrap;

            s += SeekTimeCalculate(&info1, &info2);
        } else {
            int id1 = oldSeries[coveredCount - 1] - 1, id2 = oldSeries[coveredCount] - 1;
            info1.status = 1, info1.lpos = input->ioVec.ioArray[id1].endLpos, info1.wrap = input->ioVec.ioArray[id1].wrap;
            info2.status = 1, info2.lpos = input->ioVec.ioArray[id2].startLpos, info2.wrap = input->ioVec.ioArray[id2].wrap;

            s += SeekTimeCalculate(&info1, &info2);
        }

        if (s < best) {
            Bruteforce1(input, output, coveredCount + 1, bestSeries, oldSeries);
        }

        if (coveredCount == 0) {
            info1.status = 0, info1.lpos = input->headInfo.lpos, info1.wrap = input->headInfo.wrap;
            info2.status = 1, info2.lpos = input->ioVec.ioArray[oldSeries[0] - 1].startLpos, info2.wrap = input->ioVec.ioArray[oldSeries[0] - 1].wrap;

            s -= SeekTimeCalculate(&info1, &info2);
        } else {
            int id1 = oldSeries[coveredCount - 1] - 1, id2 = oldSeries[coveredCount] - 1;
            info1.status = 1, info1.lpos = input->ioVec.ioArray[id1].endLpos, info1.wrap = input->ioVec.ioArray[id1].wrap;
            info2.status = 1, info2.lpos = input->ioVec.ioArray[id2].startLpos, info2.wrap = input->ioVec.ioArray[id2].wrap;

            s -= SeekTimeCalculate(&info1, &info2);
        }

        st[i] = false;
    }
}



/***************模拟退火相关函数*****************************/
inline void SA1(const InputParam *input, OutputParam *output) {
    /* 定初始化输出参数 */
    // OutputParam *outputTemp = (OutputParam *)malloc(sizeof(OutputParam));
    // outputTemp->len = input->ioVec.len;
    // outputTemp->sequence = (uint32_t *)malloc(outputTemp->len * sizeof(uint32_t));
    int length = input->ioVec.len;
    // 模拟退火参数设置
    double startT = 0.08; // 初始温度
    int lk =  4; // 每个温度下的迭代次数
    double delta = 0.96; // 温度下降缩减因子
    double endT = 0.000000001; // 结束温度
    int limit = 10000; // 概率选择上限
    int maxiter = 1000000; // 最大迭代次数



    /*开辟内存旧序列以及新序列*/
    uint16_t *bestSeries = (uint16_t *) malloc(sizeof (uint16_t) * length);
    uint16_t *oldSeries = (uint16_t *) malloc(sizeof (uint16_t) * length);
    uint16_t *newSeries = (uint16_t *) malloc(sizeof (uint16_t) * length);
    // 初始解
    // initSolution2(input, output);
    for (int i = 0; i < length; ++ i) {
        bestSeries[i] = oldSeries[i] = output->sequence[i];
    }

    // 得到初始解的分数（只考虑时间指标）
    double bestResult = getHddScore(input, output);
    double curReult = bestResult;
    int stuck = 0;
    int cnt = 0;
    while (true) {
        for (int i = 0; i < lk; ++ i ) {
            stuck ++;
            cnt ++;
            // neighborsSwap(oldSeries, newSeries, length);
            // neighborsSwap(oldSeries, newSeries, length);
           neighborsInsertPart(oldSeries, newSeries, length);
            // 改变output
            for (int j = 0; j < length; ++ j) {
                output->sequence[j] = newSeries[j];
            }
            
            double newResult = getHddScore(input, output);
            // printf("sss\n");
            double change = newResult - curReult;

            // #ifdef DEBUG
            //     printf("curScore: %lf\n", newResult );
            // #endif
            if (change >= 0) {
                curReult = newResult;
                memcpy(oldSeries, newSeries, length * sizeof (uint16_t));
                // #ifdef DEBUG
                //     printf("curScore: %lf\n", curReult );
                // #endif
                if (curReult > bestResult) {
                    stuck = 0;
                    bestResult = curReult;
                    memcpy(bestSeries, oldSeries, length * sizeof (uint16_t));
                    #ifdef DEBUG
                        printf("bestScore: %lf\n", bestResult );
                    #endif
                }
            }
        }

        startT = startT * delta;  

        gettimeofday(&end, NULL);  // 记录结束时间
        long seconds, useconds;    // 秒数和微秒数
        seconds = end.tv_sec - start.tv_sec;
        useconds = end.tv_usec - start.tv_usec;
        uint32_t costTime = ((seconds)*1000000 + useconds) / 1000000.0;
        if (costTime > 18) break;
        
    }

    // 将迭代后的最优结果返回给output
    for (int i = 0; i < length; ++ i) {
        output->sequence[i] = bestSeries[i];
    }

    printf("迭代次数: %d\n", cnt);





    // 释放内存
    free(bestSeries);
    free(oldSeries);    
    free(newSeries);


}

inline void SA2(const InputParam *input, OutputParam *output) {
    /* 定初始化输出参数 */
    // OutputParam *outputTemp = (OutputParam *)malloc(sizeof(OutputParam));
    // outputTemp->len = input->ioVec.len;
    // outputTemp->sequence = (uint32_t *)malloc(outputTemp->len * sizeof(uint32_t));
    int length = input->ioVec.len;
    // 模拟退火参数设置
    double startT = 800; // 初始温度
    int lk = min(50, length); // 每个温度下的迭代次数
    double delta = 0.97; // 温度下降缩减因子
    double endT = 0.1; // 结束温度
    int limit = 10000; // 概率选择上限
    int maxiter = 1000000; // 最大迭代次数




    /*开辟内存旧序列以及新序列*/
    uint16_t *bestSeries = (uint16_t *) malloc(sizeof (uint16_t) * length);
    uint16_t *oldSeries = (uint16_t *) malloc(sizeof (uint16_t) * length);
    uint16_t *newSeries = (uint16_t *) malloc(sizeof (uint16_t) * length);
    // 初始解
    // initSolution2(input, output);
    for (int i = 0; i < length; ++ i) {
        bestSeries[i] = oldSeries[i] = output->sequence[i];
    }
    // 得到初始解的分数（只考虑时间指标）
    double bestResult = energy(input, output);
    double curReult = bestResult;

    int stuck = 0;
    while (true) {
        for (int i = 0; i < lk; ++ i ) {
            stuck ++;
            // 得到领域解
            neighborsSwap(oldSeries, newSeries, length);
            // 改变output
            for (int j = 0; j < length; ++ j) {
                output->sequence[j] = newSeries[j];
            }

            double newResult = energy(input, output);

            double change = newResult - curReult;

            if (change < 0 || stuck >= 8000) {
                stuck = 0;
                curReult = newResult;
                memcpy(oldSeries, newSeries, length * sizeof (uint16_t));
                if (curReult < bestResult) {
                    bestResult = curReult;
                    memcpy(bestSeries, oldSeries, length * sizeof (uint16_t));
                    #ifdef DEBUG
                        printf("bestScore: %lf\n", bestResult );
                    #endif
                }
            }
        }

        startT = startT * delta;  
        
        gettimeofday(&end, NULL);  // 记录结束时间
        long seconds, useconds;    // 秒数和微秒数
        seconds = end.tv_sec - start.tv_sec;
        useconds = end.tv_usec - start.tv_usec;
        uint32_t costTime = ((seconds)*1000000 + useconds) / 1000000.0;
        if (costTime > 118) break;
    }

    // 将迭代后的最优结果返回给output
    for (int i = 0; i < length; ++ i) {
        output->sequence[i] = bestSeries[i];
    }





    // 释放内存
    free(bestSeries);
    free(oldSeries);    
    free(newSeries);


}
inline void SA3(const InputParam *input, OutputParam *output) {
    /* 定初始化输出参数 */
    // OutputParam *outputTemp = (OutputParam *)malloc(sizeof(OutputParam));
    // outputTemp->len = input->ioVec.len;
    // outputTemp->sequence = (uint32_t *)malloc(outputTemp->len * sizeof(uint32_t));
    int length = input->ioVec.len;
    // 模拟退火参数设置
    double startT = 8000; // 初始温度
    int lk = min(50, length * 100); // 每个温度下的迭代次数
    double delta = 0.97; // 温度下降缩减因子
    double endT = 0.1; // 结束温度
    int limit = 10000; // 概率选择上限
    int maxiter = 1000000; // 最大迭代次数




    /*开辟内存旧序列以及新序列*/
    uint16_t *bestSeries = (uint16_t *) malloc(sizeof (uint16_t) * length);
    uint16_t *oldSeries = (uint16_t *) malloc(sizeof (uint16_t) * length);
    uint16_t *newSeries = (uint16_t *) malloc(sizeof (uint16_t) * length);
    // 初始解
    // initSolution2(input, output);
    for (int i = 0; i < length; ++ i) {
        bestSeries[i] = oldSeries[i] = output->sequence[i];
    }
    // 得到初始解的分数（只考虑时间指标）
    double bestResult = energy(input, output);
    double curReult = bestResult;
    
    for (int it = 0; it < maxiter; it ++ ) {
        for (int i = 0; i < lk; ++ i ) {
            // 得到领域解
            neighbors2Opt(oldSeries, newSeries, length);
            // 改变output
            for (int j = 0; j < length; ++ j) {
                output->sequence[j] = newSeries[j];
            }

            double newResult = energy(input, output);

            double change = newResult - curReult;

            if (change < 0 || exp((-1.0 * change) / startT) >  nextN_double()) {
                curReult = newResult;
                memcpy(oldSeries, newSeries, length * sizeof (uint16_t));
                if (curReult < bestResult) {
                    bestResult = curReult;
                    memcpy(bestSeries, oldSeries, length * sizeof (uint16_t));
                    #ifdef DEBUG
                        printf("bestScore: %lf\n", bestResult );
                    #endif

                }
            }
        }
        startT = startT * delta;  
    }


    // 将迭代后的最优结果返回给output
    for (int i = 0; i < length; ++ i) {
        output->sequence[i] = bestSeries[i];
    }





    // 释放内存
    free(bestSeries);
    free(oldSeries);    
    free(newSeries);


}
inline void initSolution1(const InputParam *input, OutputParam *output) {
	/********scan-algorithm*****************/
    IOScheduleAlgorithmScan(input, output);

}
inline void initSolution2(const InputParam *input, OutputParam *output) {
	/********scan-algorithm*****************/
    IOScheduleAlgorithmNearest( input, output);

}
// 计算在某个状态下的能量（寻道 + 读写）
inline double energy(const InputParam *input, const OutputParam *output) {
	/************caculate- time****************************/
	// 计算函数：一个io请求读写完毕-到另一个io请求读写完毕的读写时间为距离
	// 距离 == 寻道时间 + 目标io请求的读写时间
 	AccessTime accessTime = {0};
	TotalAccessTime(input, output, &accessTime);

      /* 带体磨损 */
    TapeBeltSegWearInfo segWearInfo = {0};
    double tapeBeltWear = TotalTapeBeltWearTimes(input, output, &segWearInfo);

    /* 电机磨损 */
    double tapeMotorWear = TotalMotorWearTimes(input, output);


	return accessTime.addressDuration +  tapeBeltWear + tapeMotorWear;
}
// 计算在某个状态下的能量（寻道）
inline uint32_t energySeekPath(const InputParam *input, const OutputParam *output) {
	/************caculate- time****************************/
	// 计算函数：一个io请求读写完毕-到另一个io请求读写完毕的读写时间为距离
	// 距离 == 寻道时间 + 目标io请求的读写时间
 	AccessTime accessTime = {0};
	TotalAccessTime(input, output, &accessTime);

	return accessTime.addressDuration;
}


// 计算最终分数
double getBackupScore(const InputParam *input, const OutputParam *output) {
 	AccessTime accessTime = {0};
	TotalAccessTime(input, output, &accessTime);

      /* 带体磨损 */
    TapeBeltSegWearInfo segWearInfo = {0};
    double tapeBeltWear = TotalTapeBeltWearTimes(input, output, &segWearInfo);

    /* 电机磨损 */
    double tapeMotorWear = TotalMotorWearTimes(input, output);


    return backup[0] * (baselineAD - 1.0 * accessTime.addressDuration - 1.0 * accessTime.readDuration ) / baselineAD 
    + backup[1] * (baselineBelt - 1.0 * tapeBeltWear) / baselineBelt +
    backup[2] * (baselineMotor - 1.0 * tapeMotorWear) / baselineMotor;


}
double getHddScore(const InputParam *input, const OutputParam *output) {

 	AccessTime accessTime = {0};
	TotalAccessTime(input, output, &accessTime);

      /* 带体磨损 */
    TapeBeltSegWearInfo segWearInfo = {0};
    double tapeBeltWear = TotalTapeBeltWearTimes(input, output, &segWearInfo);

    /* 电机磨损 */
    double tapeMotorWear = TotalMotorWearTimes(input, output);


    return hdd[0] * (baselineAD - 1.0 * accessTime.addressDuration - 1.0 * accessTime.readDuration) / baselineAD 
    + hdd[1] * (baselineBelt - 1.0 * tapeBeltWear) / baselineBelt +
    hdd[2] * (baselineMotor - 1.0 * tapeMotorWear) / baselineMotor;
}


double saveBaseline(const InputParam *input, const OutputParam *output) {
    // 
    IOScheduleAlgorithmBaseline(input, output, true);
    // IOScheduleAlgorithmScan(input, output);
    AccessTime accessTime = {0};
	TotalAccessTime(input, output, &accessTime);

      /* 带体磨损 */
    TapeBeltSegWearInfo segWearInfo = {0};
    double tapeBeltWear = TotalTapeBeltWearTimes(input, output, &segWearInfo);

    /* 电机磨损 */
    double tapeMotorWear = TotalMotorWearTimes(input, output);


    baselineAD = accessTime.addressDuration + accessTime.readDuration;
    baselineBelt = tapeBeltWear;
    baselineMotor = tapeMotorWear;

}

















// 领域算子1
inline void neighborsSwap(uint16_t *oldSeries, uint16_t *newSeries, int length) {
    int l = nextN(length);
    int r = nextN(length);

    while (l == r) {
        r = nextN(length);
    }


    int temp = oldSeries[l];
    oldSeries[l] = oldSeries[r];
    oldSeries[r] = temp;
    for (int i = 0; i < length; ++ i) {
        newSeries[i] = oldSeries[i];
    }
    temp = oldSeries[l];
    oldSeries[l] = oldSeries[r];
    oldSeries[r] = temp;
}
// 领域算子2
inline void neighbors2Opt(uint16_t *oldSeries, uint16_t *newSeries, int length) {
    int l = nextNN(0, length);
    int r = nextNN(0, length);

    while (l == r) {
        r = nextNN(0, length);
    }

    if (l > r) {
        int temp = l;
        l = r;
        r = temp;
    }

    // [0, l - 1]
    int cnt = 0;
    for (int i = 0; i <= l - 1; ++ i ) {
        newSeries[cnt ++ ] = oldSeries[i];
    }
    // [l, r] -> [r, l]
    for (int i = r; i >= l; -- i ) {
        newSeries[cnt ++] = oldSeries[i];
    }
    // [r + 1, lenth - 1]
    for (int i = r + 1; i < length; ++ i) {
        newSeries[cnt ++] = oldSeries[i];
    }

}

inline void neighborsSwapChange(uint16_t *oldSeries,  int length, int* swapa, int* swapb) {
    int l = nextN(length);
    int r = nextN(length);

    while (l == r || l == 0 || l == length - 1 || r == 0 || r == length - 1) {
        r = nextN(length);
        l = nextN(length);
    }


    if (l > r) {
        int temp = l;
        l = r;
        r = temp;
    }
    int temp = oldSeries[l];
    oldSeries[l] = oldSeries[r];
    oldSeries[r] = temp;
    (*swapa) = l, (*swapb) = r;

}

// 领域算子3
inline void neighborsInsert(uint16_t *oldSeries, uint16_t *newSeries, int length) {
    int l = nextN(length);
    int r = nextN(length);

    while (l == r) {
        r = nextN(length);
    }

    if (l > r) {
        int temp = l;
        l = r;
        r = temp;
    }
    // 插入之前部分
    for (int i = 0; i < l; i ++ ) {
        newSeries[i] = oldSeries[i];
    }
    // [l, r) 向后移动
    int temp = oldSeries[r];
    for (int i = r - 1; i >= l; i -- ) {
        newSeries[i + 1] = oldSeries[i];
    }
    newSeries[l] = temp;
    // 插入后面部分
    for (int i = r + 1; i < length; i ++ ) {
        newSeries[i] = oldSeries[i];
    }

}

// 领域算子3
inline void neighborsInsertPart(uint16_t *oldSeries, uint16_t *newSeries, int length) {

    // 选择需要移动的部分
    int l = nextN(length);
    int r = nextN(length);

    while (l == r || abs(r - l + 1) >= length || abs(l - r + 1) >= length) {
        r = nextN(length);
    }

    if (l > r) {
        int temp = l;
        l = r;
        r = temp;
    }

    // 选择插入的位置随机选择
    int flag = 0, insertPos = -1;
    while (insertPos == -1) {
        int randomPos = -1;
        if (flag == 0 && l >= 0) {
            if (l > 0) {
                randomPos = nextNN(0, l);
            }
            flag = 1;
        } else if (flag == 1 && r < length){
            if (r < length - 1) {
                randomPos = nextNN(r + 1, length);    
            }
            flag = 0;
        }

        if (randomPos != -1) {
            insertPos = randomPos;
        }
        // printf("%d% d %d\n", insertPos, l, r);
    }
    // l - r 插入到pos位置
    if (insertPos < l) {
        // 向前插入
        for (int i = 0; i < insertPos; ++ i) newSeries[i] = oldSeries[i];
        for (int i = insertPos, len = 0; len < r - l + 1; ++ len, ++ i) {
            newSeries[i] = oldSeries[l + len];
        }
        // insertPos ---- l - 1
        for (int i = insertPos + r - l + 1, len = 0; len < l - 1 - insertPos + 1; ++ len, ++ i) {
            newSeries[i] = oldSeries[insertPos + len];
        }
        // r + 1 - length - 1
        for (int i = r + 1; i < length; ++ i) {
            newSeries[i] = oldSeries[i];
        }
    } else {
        // for (int i = 0; i < length; ++ i) {
        //     newSeries[i] = oldSeries[i];
        // }

        for (int i = 0; i <= l - 1; ++ i) newSeries[i] = oldSeries[i];
        for (int i = l, len = 0; len < insertPos - (r + 1) + 1; ++ len, ++ i) {
            newSeries[i] = oldSeries[r + 1 + len];
        }
        for (int i = l + insertPos - (r + 1) + 1, len = 0; len < r - l + 1; ++ len, ++ i) {
            newSeries[i] = oldSeries[l + len];
        }
        for (int i = insertPos + 1; i < length; ++ i) {
            newSeries[i] = oldSeries[i];
        }
    }


}


// 领域算子3
inline void neighborsInsertPartChange(int *insertPos, int *insertL, int *insertR, int length) {
    // 选择需要移动的部分
    int l = nextN(length);
    int r = nextN(length);

    while (l == r || abs(r - l + 1) >= length || abs(l - r + 1) >= length) {
        r = nextN(length);
    }

    if (l > r) {
        int temp = l;
        l = r;
        r = temp;
    }

    // 选择插入的位置随机选择
    int flag = 0, _insertPos = -1;
    while (_insertPos == -1) {
        int randomPos = -1;
        if (flag == 0 && l >= 0) {
            if (l > 0) {
                randomPos = nextNN(0, l);
            }
            flag = 1;
        } else if (flag == 1 && r < length){
            if (r < length - 1) {
                randomPos = nextNN(r + 1, length);    
            }
            flag = 0;
        }

        if (randomPos != -1) {
            _insertPos = randomPos;
        }
        // printf("%d% d %d\n", insertPos, l, r);
    }

    // 得到插入的位置以及待插入的序列

    (*insertPos) = _insertPos, (*insertL) = l, (*insertR) = r;

}

inline void updateInsertPartChange(uint16_t *oldSeries, uint16_t *newSeries, int insertPos, int insertL, int insertR, int length) {
    int l = insertL, r = insertR;
    // l - r 插入到pos位置
    if (insertPos < l) {
        // 向前插入
        for (int i = 0; i < insertPos; ++ i) newSeries[i] = oldSeries[i];
        for (int i = insertPos, len = 0; len < r - l + 1; ++ len, ++ i) {
            newSeries[i] = oldSeries[l + len];
        }
        // insertPos ---- l - 1
        for (int i = insertPos + r - l + 1, len = 0; len < l - 1 - insertPos + 1; ++ len, ++ i) {
            newSeries[i] = oldSeries[insertPos + len];
        }
        // r + 1 - length - 1
        for (int i = r + 1; i < length; ++ i) {
            newSeries[i] = oldSeries[i];
        }
    } else {
        // for (int i = 0; i < length; ++ i) {
        //     newSeries[i] = oldSeries[i];
        // }

        for (int i = 0; i <= l - 1; ++ i) newSeries[i] = oldSeries[i];
        for (int i = l, len = 0; len < insertPos - (r + 1) + 1; ++ len, ++ i) {
            newSeries[i] = oldSeries[r + 1 + len];
        }
        for (int i = l + insertPos - (r + 1) + 1, len = 0; len < r - l + 1; ++ len, ++ i) {
            newSeries[i] = oldSeries[l + len];
        }
        for (int i = insertPos + 1; i < length; ++ i) {
            newSeries[i] = oldSeries[i];
        }
    }

}























// 领域算子3
inline void neighborsInsertChange(uint16_t *oldSeries, int length, int *insertL, int *insertR) {
    int l = nextN(length);
    int r = nextN(length);

    while (l == r || l == 0 || l == length - 1 || r == 0 || r == length - 1) {
        r = nextN(length);
        l = nextN(length);
    }

    if (l > r) {
        int temp = l;
        l = r;
        r = temp;
    }

    (*insertL) = l, (*insertR) = r;

}

// 领域算子4(作为扰动算子)
inline void neighborsDisturbance1(uint16_t *oldSeries, uint16_t *newSeries, int length) {
    int l1 = nextN(length);
    int r1 = nextN(length);
    int l2 = nextN(length);
    int r2 = nextN(length);

    while (l1 == l2 || l1 == r1 || l1 == r2 || r1 == l2 || r1 == r2 || l2 == r2) {
            r1 = nextN(length);
            l2 = nextN(length);
            r2 = nextN(length);
    }

    int sort[4] = {l1, r1, l2, r2};
    for (int i = 0; i < 3; i ++ ) {
        for (int j = 0; j < 3 - i; j ++ ) {
            if (sort[j] > sort[j + 1]) {
                int temp = sort[j];
                sort[j] = sort[j + 1];
                sort[j + 1] = temp;
            }
        }
    }

    l1 = sort[0], r1 = sort[1], l2 = sort[2], r2 = sort[3];
    int cnt = 0;
    for (int i = 0; i < l1; i ++ ) {
        newSeries[cnt ++] = oldSeries[i];
    }
    for (int i = l2; i <= r2; i ++ ) {
        newSeries[cnt ++ ] = oldSeries[i];
    }
    for (int i = r1 + 1; i < l2; i ++ ) {
        newSeries[cnt ++] = oldSeries[i];
    }
    for (int i = l1; i <= r1; i ++ ) {
        newSeries[cnt ++] = oldSeries[i];
    }

    for (int i = r2 + 1; i < length; i ++ ) {
        newSeries[cnt ++] = oldSeries[i];
    }

}

// 领域算子5(作为扰动算子)
inline void neighborsDisturbance2(uint16_t *oldSeries, uint16_t *newSeries, int length) {
    int l1 = nextN(length);
    int l2 = nextN(length);
    int l3 = nextN(length);


    while (l1 == l2 || l1 == l3 || l2 == l3) {
            l2 = nextN(length);
            l3 = nextN(length);
    }

    int sort[3] = {l1, l2, l3 };
    for (int i = 0; i < 2; i ++ ) {
        for (int j = 0; j < 2 - i; j ++ ) {
            if (sort[j] > sort[j + 1]) {
                int temp = sort[j];
                sort[j] = sort[j + 1];
                sort[j + 1] = temp;
            }
        }
    }

    l1 = sort[0], l2 = sort[1], l3 = sort[2];
    int cnt = 0;
    for (int i = 0; i < l1; i ++ ) {
        newSeries[cnt ++] = oldSeries[i];
    }
    for (int i = l3; i < length; i ++ ) {
        newSeries[cnt ++] = oldSeries[i];  
    }
    for (int i =l2; i < l3; i ++ ) {
        newSeries[cnt ++] = oldSeries[i];  
    }
    for (int i = l1; i < l2; i ++ ) {
        newSeries[cnt ++] = oldSeries[i];  
    }
}

/***************爬山算法及变种*****************************/
inline void GVNS(const InputParam *input, OutputParam *output, int length) {
  /* 定初始化输出参数 */
    int maxiter = 10000; // 最大迭代次数
    int lmax = 3, kmax = 3; // 领域算子和扰动算子
    int neigNumber = 100;



    /*开辟内存旧序列以及新序列*/
    uint16_t *bestSeries = (uint16_t *) malloc(sizeof (uint16_t) * length);
    uint16_t *oldSeries = (uint16_t *) malloc(sizeof (uint16_t) * length);
    uint16_t *newSeries = (uint16_t *) malloc(sizeof (uint16_t) * length);
    uint16_t *newSeriesTemp = (uint16_t *) malloc(sizeof (uint16_t) * length);
    uint16_t *shakeSeries = (uint16_t *) malloc(sizeof (uint16_t) * length);
    // 初始解
    // initSolution2(input, output);
    for (int i = 0; i < length; ++ i) {
        bestSeries[i] = oldSeries[i] = output->sequence[i];
    }
    // 得到初始解的分数（只考虑时间指标）
    double bestResult = getBackupScore(input, output);


    int stuck = 0;
    for (int it = 0; it < maxiter; it ++ ) {
        stuck ++;
        int l = 0,k = 0;
        while (k < kmax) {
            // 扰动
            switch(k) {
                case 0: neighborsSwap(oldSeries, shakeSeries, length);break;
                case 1: neighborsInsert(oldSeries, shakeSeries, length);break;
                case 2: neighborsDisturbance1(oldSeries, shakeSeries, length);break;
            }

            while (l < lmax) {
                double curResult = -1;
                for (int i = 0; i < neigNumber; i ++ ) {
                    switch(l) {
                        case 0: neighbors2Opt(shakeSeries, newSeries, length);break;
                        case 1: neighbors2Opt(shakeSeries, newSeries, length);break;
                        case 2: neighbors2Opt(shakeSeries, newSeries, length);break;
                    }

                    // 改变output
                    for (int j = 0; j < length; ++ j) {
                        output->sequence[j] = newSeries[j];
                    }

                    double newResult = getBackupScore(input, output);
                    if (curResult == -1 || curResult > newResult) {
                        curResult = newResult;
                        memcpy(newSeriesTemp, newSeries, length * sizeof (uint16_t));
                    }

                    
                }
                // 改变output
                for (int j = 0; j < length; ++ j) {
                    output->sequence[j] = shakeSeries[j];
                }

                double shakeCost =  getBackupScore(input, output);
                // 改变output
                for (int j = 0; j < length; ++ j) {
                    output->sequence[j] = newSeriesTemp[j];
                }
                double curCost =  getBackupScore(input, output);

                if (curCost > shakeCost) {
                    shakeCost = curCost;
                    memcpy(shakeSeries, newSeriesTemp, length * sizeof (uint16_t));
                    l = 0;
                } else {
                    l = l + 1;
                }

            }
            // 改变output
            for (int j = 0; j < length; ++ j) {
                output->sequence[j] = shakeSeries[j];
            }

            double shakeCost =  getBackupScore(input, output);
            // 最优选择
            if (shakeCost > bestResult) {
                stuck = 0;
                bestResult = shakeCost;
                #ifdef DEBUG
                    printf("bestScore: %lf\n", bestResult );
                #endif

                memcpy(bestSeries, shakeSeries, length * sizeof (uint16_t));
            }
            // 改变output
            for (int j = 0; j < length; ++ j) {
                output->sequence[j] = oldSeries[j];
            }
            // 比较扰动解与当前解的选择
            double curCost = getBackupScore(input, output);
            if (shakeCost > curCost) {
                curCost = shakeCost;
                memcpy(oldSeries, shakeSeries, length * sizeof (uint16_t));
                k = 0;
            } else {
                k = k + 1;
            }


        }
        gettimeofday(&end, NULL);  // 记录结束时间
        long seconds, useconds;    // 秒数和微秒数
        seconds = end.tv_sec - start.tv_sec;
        useconds = end.tv_usec - start.tv_usec;
        uint32_t costTime = ((seconds)*1000000 + useconds) / 1000000.0;
        if (costTime > 10) break;

    }


    // 将迭代后的最优结果返回给output
    for (int i = 0; i < length; ++ i) {
        output->sequence[i] = bestSeries[i];
    }





    // 释放内存
    free(bestSeries);
    free(oldSeries);    
    free(newSeries);
    free(newSeriesTemp);
    free(shakeSeries);

}


inline uint32_t getBeltWear(HeadInfo *info1, HeadInfo *info2) {
    int direct1 = (*info1).wrap % 2 == 0 ? 1 : -1;
    int direct2 = (*info2).wrap % 2 == 0 ? 1 : -1;


    int extra = ((*info1).status == 1 ? addRoute : SE0);

    if ((*info1).status == 1) {
        // 1 : 正向; 2 : 反向
        int lpos1 = (*info1).lpos, lpos2 = (*info2).lpos;
        if (direct1 == direct2) {
            if (direct1 == 1) {
                if (lpos2 >= lpos1) {
                    return abs(lpos1 - lpos2) + ((*info1).wrap != (*info2).wrap ? 1 : 0);
                } else {
                    if (lpos1 <= PdMaxPos && lpos2 >= SameDircetTerval) {
                        return abs(lpos1 - lpos2) + addRoute;
                    }
                    else if (lpos2 >= SameDircetTerval) {
                        return abs(lpos1 - lpos2) + addRoute - (lpos1 - PdMaxPos) * 2;

                    } else if (lpos1 <= PdMaxPos){
                        return abs(lpos1 - lpos2) + addRoute - (SameDircetTerval - lpos2) * 2;
                    }else {
                        return abs(lpos1 - lpos2) + (MAX_LPOS - lpos1) * 2 + 2 * (lpos2);
                    }
                }
            } else {
                if (lpos2 <= lpos1) {
                    return abs(lpos1 - lpos2) + ((*info1).wrap != (*info2).wrap ? 1 : 0);
                } else {
                    if (lpos1 >= rdMinPos && lpos2 <= MAX_LPOS -  SameDircetTerval) {
                        return abs(lpos1 - lpos2) + addRoute;
                    } else if (lpos2 <= MAX_LPOS -  SameDircetTerval) {
                        return abs(lpos1 - lpos2) + addRoute - (rdMinPos - lpos1) * 2;
                    } else if (lpos1 >= rdMinPos) {
                        return abs(lpos1 - lpos2) + addRoute - (lpos2 - (MAX_LPOS - SameDircetTerval)) * 2;
                    }else {
                        return abs(lpos1 - lpos2) + (lpos1) * 2 + 2 * (MAX_LPOS - lpos2);
                    }
                }
            }
        } else {
            if (direct1 == 1) {
                if (lpos1 >= lpos2) {
                    int sub = lpos1 - lpos2;
                    if (lpos1 <= PdMaxPos) {
                        if (sub > turnInterval) {
                            return sub + turnRoute;
                        } else {
                            return sub + turnRoute + (turnInterval - sub) * 2;
                        }
                    } else {
                        if (sub > turnInterval) {
                            return sub + turnRoute - (lpos1 - PdMaxPos) * 2;
                        } else {
                            return sub + turnRoute - (lpos1 - PdMaxPos) * 2 + (turnInterval - sub) * 2;
                        }                 
                    }
                } else {
                    int sub = lpos2 - lpos1;
                    if (lpos1 < PdMaxPos) {
                        if (lpos2 >= MAX_LPOS - rturnInterval) {
                            return sub + rturnRoute - (lpos2 + rturnInterval - MAX_LPOS) * 2;
                        } else {
                            return sub + rturnRoute;
                        }
                    }

                    if (lpos1 <= MAX_LPOS - rturnInterval && lpos2 <= MAX_LPOS - rturnInterval) {
                        return abs(lpos1 - lpos2) + rturnRoute;
                    } 
                }
            } else {
                if (lpos1 <= lpos2) {
                    int sub = lpos2 - lpos1;
                    if (lpos1 >= rdMinPos) {
                        if (sub > turnInterval) {
                            return sub + turnRoute;
                        } else {
                            return sub + turnRoute + (turnInterval - sub) * 2;
                        }                       
                    } else {
                        if (sub > turnInterval) {
                            return sub + turnRoute - (rdMinPos - lpos1) * 2;
                        } else {
                            return sub + turnRoute - (rdMinPos - lpos1) * 2 + (turnInterval - sub) * 2;
                        }
                    }

                }else {
                    int sub = lpos1 - lpos2;
                    if (lpos1 > rdMinPos) {
                        if (lpos2 < rturnInterval) {
                            return sub + rturnRoute - (rturnInterval - lpos2) * 2;
                        } else {
                             return sub + rturnRoute;
                        }
                    }
                    if (lpos1 >= rturnInterval && lpos2 >= rturnInterval) {
                        return abs(lpos1 - lpos2) + rturnRoute;
                    }
                }
            }
        }

    } else {
        // 1 : 正向; 2 : 反向

        int lpos1 = (*info1).lpos, lpos2 = (*info2).lpos;
        if (direct1 == direct2) {
            if (direct1 == 1) {
                if (lpos2 >= lpos1) {
                    if ((lpos2 - lpos1) >= addRoute) {
                        return abs(lpos1 - lpos2) + ((*info1).wrap != (*info2).wrap ? 1 : 0);
                    }
                } else {
                    if (lpos1 <= PdMaxPos) {
                        return abs(lpos1 - lpos2) + extra;
                    }
                }
            } else {
                if (lpos2 <= lpos1) {
                    if ( (lpos1 - lpos2) >= addRoute) {
                        return abs(lpos1 - lpos2) + ((*info1).wrap != (*info2).wrap ? 1 : 0);
                    }
                } else {
                    if (lpos1 >= rdMinPos) {
                        return abs(lpos1 - lpos2) + extra;
                    }
                }
            }
        }
    }


    return BeltWearTimes(info1, info2, &segWearInfo);

}


inline void hillClimbSwap(const InputParam *input, OutputParam *output) {
    /* 定初始化输出参数 */

    int length = input->ioVec.len;

    int cnt = 0;



    /*开辟内存旧序列以及新序列*/
    uint16_t *bestSeries = (uint16_t *) malloc(sizeof (uint16_t) * length);
    uint16_t *oldSeries = (uint16_t *) malloc(sizeof (uint16_t) * length);
    // 初始解
    // initSolution2(input, output);
    for (int i = 0; i < length; ++ i) {
        bestSeries[i] = oldSeries[i] = output->sequence[i];
    }
    // 得到初始解的分数（只考虑时间指标）
    double bestResult = getBackupScore(input, output);
    double curReult = bestResult;
    while (true) {
 
        cnt ++;
        /**
         * 使用swapChange算子的时候减少不必要的计算开销
        */
        int swapa = -1, swapb = -1;
        neighborsSwapChange(oldSeries, length, &swapa, &swapb);

        int temp = oldSeries[swapa];
        oldSeries[swapa] = oldSeries[swapb];
        oldSeries[swapb] = temp;
        // 计算改变的时间
        double oldPartDuration = 0.0, oldPartBelt = 0.0, oldPartMotor = 0.0;
        
        int id1 = oldSeries[swapa - 1] - 1, id2 = oldSeries[swapa] - 1;
        // info1.status = 1, info1.lpos = input->ioVec.ioArray[id1].endLpos, info1.wrap = input->ioVec.ioArray[id1].wrap;
        // info2.status = 1, info2.lpos = input->ioVec.ioArray[id2].startLpos, info2.wrap = input->ioVec.ioArray[id2].wrap;

        // 寻道时间
        oldPartDuration += threeCost[id1 + 1][id2 + 1][0];
        // 带体磨损
        // TapeBeltSegWearInfo segWearInfo = {0};
        oldPartBelt += threeCost[id1 + 1][id2 + 1][1];
        // oldPartBelt += getBeltWear(&info1, &info2);
        // 电机磨损
        oldPartMotor +=  threeCost[id1 + 1][id2 + 1][2];

        /********************************************************************/
        id1 = oldSeries[swapa] - 1, id2 = oldSeries[swapa + 1] - 1;

        // 寻道时间
        oldPartDuration += threeCost[id1 + 1][id2 + 1][0];
        // 带体磨损
        // TapeBeltSegWearInfo segWearInfo = {0};
        oldPartBelt += threeCost[id1 + 1][id2 + 1][1];
        // oldPartBelt += getBeltWear(&info1, &info2);
        // 电机磨损
        oldPartMotor +=  threeCost[id1 + 1][id2 + 1][2];
        /************************************************************************/

        id1 = oldSeries[swapb - 1] - 1, id2 = oldSeries[swapb] - 1;
  
            // 寻道时间
        oldPartDuration += threeCost[id1 + 1][id2 + 1][0];
        // 带体磨损
        // TapeBeltSegWearInfo segWearInfo = {0};
        oldPartBelt += threeCost[id1 + 1][id2 + 1][1];
        // oldPartBelt += getBeltWear(&info1, &info2);
        // 电机磨损
        oldPartMotor +=  threeCost[id1 + 1][id2 + 1][2];
        /*********************************************************************/

        id1 = oldSeries[swapb] - 1, id2 = oldSeries[swapb + 1] - 1;
            // 寻道时间
        oldPartDuration += threeCost[id1 + 1][id2 + 1][0];
        // 带体磨损
        // TapeBeltSegWearInfo segWearInfo = {0};
        oldPartBelt += threeCost[id1 + 1][id2 + 1][1];
        // oldPartBelt += getBeltWear(&info1, &info2);
        // 电机磨损
        oldPartMotor +=  threeCost[id1 + 1][id2 + 1][2];




        temp = oldSeries[swapa];
        oldSeries[swapa] = oldSeries[swapb];
        oldSeries[swapb] = temp;


        double newPartDuration = 0.0, newPartBelt = 0.0, newPartMotor = 0.0;
        /****************************************************************/
        id1 = oldSeries[swapa - 1] - 1, id2 = oldSeries[swapa] - 1;
        // 寻道时间
        newPartDuration += threeCost[id1 + 1][id2 + 1][0];
        // 带体磨损
        // TapeBeltSegWearInfo segWearInfo = {0};
        newPartBelt += threeCost[id1 + 1][id2 + 1][1];
        // oldPartBelt += getBeltWear(&info1, &info2);
        // 电机磨损
        newPartMotor +=  threeCost[id1 + 1][id2 + 1][2];

        id1 = oldSeries[swapa] - 1, id2 = oldSeries[swapa + 1] - 1;
        /*************************************************************/
        // 寻道时间
        newPartDuration += threeCost[id1 + 1][id2 + 1][0];
        // 带体磨损
        // TapeBeltSegWearInfo segWearInfo = {0};
        newPartBelt += threeCost[id1 + 1][id2 + 1][1];
        // oldPartBelt += getBeltWear(&info1, &info2);
        // 电机磨损
        newPartMotor +=  threeCost[id1 + 1][id2 + 1][2];


        /***************************************************************/
        id1 = oldSeries[swapb - 1] - 1, id2 = oldSeries[swapb] - 1;
        // 寻道时间
        newPartDuration += threeCost[id1 + 1][id2 + 1][0];
        // 带体磨损
        // TapeBeltSegWearInfo segWearInfo = {0};
        newPartBelt += threeCost[id1 + 1][id2 + 1][1];
        // oldPartBelt += getBeltWear(&info1, &info2);
        // 电机磨损
        newPartMotor +=  threeCost[id1 + 1][id2 + 1][2];
        /**************************************************************/
        
        id1 = oldSeries[swapb] - 1, id2 = oldSeries[swapb + 1] - 1;
        /****************************************************************/
        // 寻道时间
        newPartDuration += threeCost[id1 + 1][id2 + 1][0];
        // 带体磨损
        // TapeBeltSegWearInfo segWearInfo = {0};
        newPartBelt += threeCost[id1 + 1][id2 + 1][1];
        // oldPartBelt += getBeltWear(&info1, &info2);
        // 电机磨损
        newPartMotor +=  threeCost[id1 + 1][id2 + 1][2];

        /********************************************************************/
        double change = backup[0] / baselineAD * (oldPartDuration - newPartDuration) +
        backup[1] / baselineBelt * (oldPartBelt - newPartBelt ) + backup[2] / baselineMotor * (oldPartMotor - newPartMotor);


        if (change > 0) {
            curReult = curReult + change;
            if (curReult > bestResult) {
                bestResult = curReult;
                #ifdef DEBUG
                    printf("bestScore: %lf\n", bestResult );
                #endif
                memcpy(bestSeries, oldSeries, length * sizeof (uint16_t));
                
            }
        } else {
            // 不满足则复原
            int temp = oldSeries[swapa];
            oldSeries[swapa] = oldSeries[swapb];
            oldSeries[swapb] = temp;
        }



        gettimeofday(&end, NULL);  // 记录结束时间
        long seconds, useconds;    // 秒数和微秒数
        seconds = end.tv_sec - start.tv_sec;
        useconds = end.tv_usec - start.tv_usec;
        uint32_t costTime = ((seconds)*1000000 + useconds) / 1000000.0;
        if (costTime > 18) break;
        
    }
    printf("迭代次数: %d\n", cnt);
    // 将迭代后的最优结果返回给output
    for (int i = 0; i < length; ++ i) {
        output->sequence[i] = bestSeries[i];
    }


    // 释放内存
    free(bestSeries);
    free(oldSeries);   


}
inline void hillClimbInsert(const InputParam *input, OutputParam *output) {
 /* 定初始化输出参数 */

    int length = input->ioVec.len;

    int cnt = 0;

    int stuck = (length == 1000 ? 1000 : 3000);

    struct timeval1 e1;
    gettimeofday(&e1, NULL);  // 记录结束时间


    /*开辟内存旧序列以及新序列*/
    uint16_t *bestSeries = (uint16_t *) malloc(sizeof (uint16_t) * length);
    uint16_t *oldSeries = (uint16_t *) malloc(sizeof (uint16_t) * length);
    uint16_t *newSeries = (uint16_t *) malloc(sizeof (uint16_t) * length);
    // 初始解
    // initSolution2(input, output);
    for (int i = 0; i < length; ++ i) {
        bestSeries[i] = oldSeries[i] = output->sequence[i];
    }
    double temp[3];
    // 得到初始解的分数（只考虑时间指标）
    double bestResult = getHddScore(input, output);
    memcpy(temp, hdd, sizeof hdd);
    // double bestResult = getBackupScore(input, output);
    // memcpy(temp, backup, sizeof backup);
    // printf("bestResult:%lf\n", bestResult);
    double curReult = bestResult;

    while (true) {

        cnt ++;
        /**
        * 使用swapChange算子的时候减少不必要的计算开销
        */
        int insertPos = -1, left = -1, right = -1;
        neighborsInsertPartChange(&insertPos, &left,  &right, length);

        // 计算改变的时间
        double oldPartDuration = 0.0, oldPartBelt = 0.0, oldPartMotor = 0.0;
        double newPartDuration = 0.0, newPartBelt = 0.0, newPartMotor = 0.0;
        
        if (insertPos < left) {
            // 得到旧情况未插入前开销
            // 插入点前一个点与插入点之间的开销
            /*****************************计算旧序列的所有改变的开销**********************************************/
            int insertPosId = oldSeries[insertPos];
            int leftId = oldSeries[left];
            int beforeLeftId = oldSeries[left - 1];
            int rightId = oldSeries[right];
            if (insertPos == 0) {
                oldPartDuration += headToFirst[insertPosId][0];
                oldPartBelt += headToFirst[insertPosId][1];
                oldPartMotor += headToFirst[insertPosId][2];
            } else {
                int beforeInsertPosId = oldSeries[insertPos - 1];
                oldPartDuration += threeCost[beforeInsertPosId][insertPosId][0];
                oldPartBelt +=threeCost[beforeInsertPosId][insertPosId][1];
                // oldPartBelt += threeCost[beforeInsertPosId][insertPosId][1];
                oldPartMotor += threeCost[beforeInsertPosId][insertPosId][2];
            }
            // 待插入序列的l与l-1之间的开销
            oldPartDuration += threeCost[beforeLeftId][leftId][0];
            oldPartBelt += threeCost[beforeLeftId][leftId][1];
            // oldPartBelt += threeCost[beforeLeftId][leftId][1];
            oldPartMotor += threeCost[beforeLeftId][leftId][2]; 

            // 待插入序列r与r+1之间的开销
            if (right == length - 1) {}
            else {
                int afterRightId = oldSeries[right + 1];
                oldPartDuration += threeCost[rightId][afterRightId][0];
                oldPartBelt += threeCost[rightId][afterRightId][1];
                // oldPartBelt += threeCost[rightId][afterRightId][1];
                oldPartMotor += threeCost[rightId][afterRightId][2];    
            }


            /*****************************计算新序列的所有改变的开销**********************************************/

            // 插入点之前的点到l的开销
            if (insertPos == 0) {
                newPartDuration += headToFirst[leftId][0];
                newPartBelt += headToFirst[leftId][1];
                newPartMotor += headToFirst[leftId][2];  
            } else {
                int beforeInsertPosId = oldSeries[insertPos - 1];
                newPartDuration += threeCost[beforeInsertPosId][leftId][0];
                newPartBelt += threeCost[beforeInsertPosId][leftId][1];
                // newPartBelt += threeCost[beforeInsertPosId][leftId][1];
                newPartMotor += threeCost[beforeInsertPosId][leftId][2];                
            }

            // 插入点的右侧到插入点的开销
            newPartDuration += threeCost[rightId][insertPosId][0];
            newPartBelt += threeCost[rightId][insertPosId][1];
            // newPartBelt += threeCost[rightId][insertPosId][1];
            newPartMotor += threeCost[rightId][insertPosId][2];  

            // l前一个点到r后一点的开销
            if (right == length - 1) {}
            else {
                int afterRightId = oldSeries[right + 1];
                newPartDuration += threeCost[beforeLeftId][afterRightId][0];
                newPartBelt += threeCost[beforeLeftId][afterRightId][1];
                // newPartBelt += threeCost[beforeLeftId][afterRightId][1];
                newPartMotor += threeCost[beforeLeftId][afterRightId][2];   
            }


        } else {

        }

        /********************************************************************/
        double change = temp[0] / baselineAD * (oldPartDuration - newPartDuration) +
        temp[1] / baselineBelt * (oldPartBelt - newPartBelt ) +  temp[2] / baselineMotor * (oldPartMotor - newPartMotor);

        if (change > 0) {

            gettimeofday(&e1, NULL);

            curReult = curReult + change;
            updateInsertPartChange(oldSeries, newSeries, insertPos, left, right, length);
            memcpy(oldSeries, newSeries, length * sizeof (uint16_t));
            bestResult = curReult;
            #ifdef DEBUG
                printf("bestScore: %lf\n", bestResult );
            #endif

                
                
        
        }




        gettimeofday(&end, NULL);  // 记录结束时间
        long tseconds, ttuseconds;
        long seconds, useconds;    // 秒数和微秒数

        tseconds = end.tv_sec - e1.tv_sec;
        ttuseconds = end.tv_usec - e1.tv_usec;

        long mseconds = (tseconds)*1000 + ttuseconds / 1000;

        seconds = end.tv_sec - start.tv_sec;
        useconds = end.tv_usec - start.tv_usec;
        uint32_t costTime = ((seconds)*1000000 + useconds) / 1000000.0;
        if (costTime > 18 || mseconds >= stuck) break;
        
    }
    #ifdef DEBUG    
        printf("迭代次数: %d\n", cnt);
    #endif

    memcpy(bestSeries, oldSeries, length * sizeof (uint16_t));
    // 将迭代后的最优结果返回给output
    for (int i = 0; i < length; ++ i) {
        output->sequence[i] = bestSeries[i];
    }


    // 释放内存
    free(bestSeries);
    free(oldSeries);   
    free(newSeries);   
}
/***************禁忌搜索算法*****************************/
inline void TS(const InputParam *input, OutputParam *output, int length, int op) {

    // 参数设置
    int maxiter = 1000;
    int neighbor = 400;
    int tabuLen = 50;




    /*开辟内存旧序列以及新序列*/
    uint16_t *gSeries = (uint16_t *) malloc(sizeof (uint16_t) * length);
    uint16_t *bestSeries = (uint16_t *) malloc(sizeof (uint16_t) * length);
    uint16_t *localSeries = (uint16_t *) malloc(sizeof (uint16_t) * length);
    uint16_t *tempSeries = (uint16_t *) malloc(sizeof (uint16_t) * length);


    // // 开辟禁忌表内存
    // uint16_t** TabuList;//禁忌表
    for (int i = 0; i < tabuLen; i ++ ) {
        TabuList[i] = (uint16_t *) malloc(sizeof (uint16_t) * length);
    }

    for (int i = 0; i < length; ++ i) {
        bestSeries[i] = gSeries[i] = output->sequence[i];
    }
    // 得到初始解的分数（只考虑时间指标）
    double bestResult = getBackupScore(input, output);
    double localResult = -1;
    double tempResult = -1;

    for (int i = 0; i < maxiter; ++ i) {
        localResult = 1e9;
        for (int j = 0; j < neighbor; ++ j) {
            switch (op) {
            case 0 : neighborsInsert(gSeries, tempSeries, length); break;
            case 1 : neighborsSwap(gSeries, tempSeries, length); break;
            case 2 : neighbors2Opt(gSeries, tempSeries, length); break;
            }
            /**
            * 使用swapChange算子的时候减少不必要的计算开销
            */
       

            
            if (!inTabuList(tempSeries, tabuLen)) {
                // 替换output
                for (int j1 = 0; j1 < length; j1 ++ ) {
                    output->sequence[j1] = tempSeries[j1];
                }
                tempResult = getBackupScore(input, output);
                if (tempResult > localResult) {
                    memcpy(localSeries, tempSeries, length * sizeof (uint16_t));
                    localResult = tempResult;
                }
            }
        }

        if (localResult > bestResult) {
            #ifdef DEBUG
                printf("bestScore: %lf\n", bestResult);
            #endif DEBUG
            bestResult = localResult;
            memcpy(bestSeries, localSeries, length * sizeof (uint16_t));
        }

        memcpy(gSeries, localSeries, sizeof (uint16_t) * length);
        pushTabuList(localSeries, tabuLen);// 加入禁忌表

        gettimeofday(&end, NULL);  // 记录结束时间
        long seconds, useconds;    // 秒数和微秒数
        seconds = end.tv_sec - start.tv_sec;
        useconds = end.tv_usec - start.tv_usec;
        uint32_t costTime = ((seconds)*1000000 + useconds) / 1000000.0;
        if (costTime > 18) break;
    }

    // 将迭代后的最优结果返回给output
    for (int i = 0; i < length; ++ i) {
        output->sequence[i] = bestSeries[i];
    }



    // 释放内存
    free(bestSeries);
    free(localSeries); 
    free(tempSeries); 
    free(gSeries);

}

// 判读编码是否在禁忌表中
inline bool inTabuList(uint16_t* series, int tabuLen){
	int i;
	int flag = 0;
	for(i = 0; i < tabuLen; i ++ ){
		flag = 0;
		for(int j = 0; j < ioCount; j ++ ){
			if(series[j] != TabuList[i][j]){
				flag = 1;
				break;
			}
		}
		if(flag == 0)	
			break;
	}
	return !(i == tabuLen);
}

// 加入禁忌表
inline void pushTabuList(uint16_t* arr, int tabuLen){
	// 删除队列第一个编码
	for(int i = 0; i < tabuLen - 1; i ++ ){
		for(int j = 0; j < ioCount; j ++ ){
			TabuList[i][j] = TabuList[i + 1][j];
		}
	}
	// 加入队列尾部
	for(int k = 0; k < ioCount; k ++ ){
		TabuList[tabuLen - 1][k] = arr[k];
	}
}

// 二分1
inline int check1(int mid) {
    info1.status = 1, info1.lpos = mid, info1.wrap = 0;
    info2.status = 1, info2.lpos = 20000, info2.wrap = 0;
    int sub = abs(BeltWearTimes(&info1, &info2, &segWearInfo) - (mid - 20000));
    // printf("%d\n", sub);
    sub = abs(sub - addRoute);
    if(sub < 3) return true;
    return false;
}
inline int check1_1(int mid) {
    info1.status = 1, info1.lpos = 20000, info1.wrap = 0;
    info2.status = 1, info2.lpos = mid, info2.wrap = 0;
    int sub = abs(BeltWearTimes(&info1, &info2, &segWearInfo) - (20000 - mid));
    // printf("%d\n", sub);
    sub = abs(sub - addRoute);
    if(sub < 3) return true;
    return false;
}
inline int check2(int mid) {
    info1.status = 1, info1.lpos = 30000, info1.wrap = 0;
    info2.status = 1, info2.lpos = mid, info2.wrap = 1;
    int sub = abs(BeltWearTimes(&info1, &info2, &segWearInfo) - (30000 - mid));
    // printf("%d\n", sub);
    sub = abs(sub - turnRoute);
    if(sub < 3) return true;
    return false;
}
inline int check3(int mid) {
    info1.status = 1, info1.lpos = 30000, info1.wrap = 0;
    info2.status = 1, info2.lpos = mid, info2.wrap = 1;
    int sub = abs(BeltWearTimes(&info1, &info2, &segWearInfo) - (mid - 30000));
    // printf("%d\n", sub);
    sub = abs(sub - rturnRoute);
    if(sub < 3) return true;
    return false;
}