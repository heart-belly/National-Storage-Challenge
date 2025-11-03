
#ifndef ALGORITHM_H
#define ALGORITHM_H

#include "../public.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_HEAP_SIZE 10010
#define min(x, y) x > y ? y : x;
#define max(x, y) x > y ? x : y;

typedef struct {
    int id;
    int startPos;
    int endPos;
} IoInfo;

typedef struct {
    IoInfo *data;
    int size;
    int capacity;
} Heap;

struct timeval1 {
    long tv_sec;   /* seconds */
    long tv_usec;  /* microseconds */
};

int cmp(const void *a,const void *b);
int cmpp(const void *a,const void *b);
void swap(IoInfo *a, IoInfo *b); // 交换
Heap* createHeap(int capacity); // 创建堆
int parent(int i); // 获取父节点
int leftChild(int i);
int rightChild(int i);
void insertMax(Heap* heap, IoInfo value);
void insertMin(Heap* heap, IoInfo value);
void minHeapify(Heap* heap, int i);
IoInfo extractMin(Heap* heap);
void maxHeapify(Heap* heap, int i);
IoInfo extractMax(Heap* heap) ;
void freeHeap(Heap* heap);
// 初始解函数
int32_t IOScheduleAlgorithm(const InputParam *input, OutputParam *output);
uint32_t IOScheduleAlgorithmScan(const InputParam *input, OutputParam *output);
uint32_t IOScheduleAlgorithmScanRightToLeft(const InputParam *input, OutputParam *output);
uint32_t IOScheduleAlgorithmNearest(const InputParam *input, OutputParam *output);
uint32_t IOScheduleAlgorithmBaseline(const InputParam *input, OutputParam *output, bool flag);
int32_t AlgorithmRun(const InputParam *input, OutputParam *output);
/*
* 随机种子
*/
int nextN(int x);
int nextNN(int a, int b);
double nextN_double();
uint32_t randN();
/*
* 初始化方案
*/
void preDealData(); // 预处理数据（计算两个io请求之间的寻道时间）
void initSolution1();
void initSolution2();
/*
* 暴力遍历寻找解析解
*/
void Bruteforce(const InputParam *input, OutputParam *output);
void Bruteforce1(const InputParam *input, OutputParam *output, int coveredCount, uint16_t *bestSeries, uint16_t *oldSeries);
/*
* 计算寻路时间以及读写时间
*/
double energy(const InputParam *input, const OutputParam *output);
uint32_t energySeekPath(const InputParam *input, const OutputParam *output);
double getBackupScore(const InputParam *input, const OutputParam *output);
double getHddScore(const InputParam *input, const OutputParam *output);
double saveBaseline(const InputParam *input, const OutputParam *output);
/*
* 模拟退火以及变种
*/
void SA1(const InputParam *input, OutputParam *output);
void SA2(const InputParam *input, OutputParam *output);
void SA3(const InputParam *input, OutputParam *output);

/*
* 爬山算法以及变种(在数据量大的时候使用)
*/
void GVNS(const InputParam *input, OutputParam *output, int length);
void hillClimbSwap(const InputParam *input, OutputParam *output);
void hillClimbInsert(const InputParam *input, OutputParam *output);
/*
* 禁忌搜索
*/
void TS(const InputParam *input, OutputParam *output, int length, int op);
bool inTabuList(uint16_t* series, int tabuLen);
void pushTabuList(uint16_t* arr, int tabuLen);
/*
* 领域算子
*/
void neighborsSwap(uint16_t *oldSeries, uint16_t *newSeries, int length);
void neighbors2Opt(uint16_t *oldSeries, uint16_t *newSeries, int length);
void neighborsInsert(uint16_t *oldSeries, uint16_t *newSeries, int length);
void neighborsInsertPart(uint16_t *oldSeries, uint16_t *newSeries, int length);
void neighborsDisturbance1(uint16_t *oldSeries, uint16_t *newSeries, int length);
void neighborsDisturbance2(uint16_t *oldSeries, uint16_t *newSeries, int length);

/**
 *  减少无效遍历
*/
void neighborsInsertChange(uint16_t *oldSeries, int length, int *insertL, int *insertR);
void neighborsSwapChange(uint16_t *oldSeries,  int length, int* swapa, int* swapb);
void neighborsInsertPartChange(int *insertPos, int *insertL, int *insertR, int length);
uint32_t getBeltWear(HeadInfo *info1, HeadInfo *info2);
void updateInsertPartChange(uint16_t *oldSeries, uint16_t *newSeries, int insertPos, int insertL, int insertR, int length);


/*
*/
int check1(int mid);
int check1_1(int mid);
int check2(int mid);
int check3(int mid);
#ifdef __cplusplus
}
#endif

#endif  // ALGORITHM_H