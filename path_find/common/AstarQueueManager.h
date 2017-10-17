#ifndef __ASTAR_QUEUE_MANAGER_H__
#define __ASTAR_QUEUE_MANAGER_H__
#include <stddef.h>
#include "AstarQueue.h"

//最远距离
#define MAX_INDEX 2048   //==MAP_GRID_XMAX + MAP_GRID_YMAX
struct AstarQueueManager
{
    AstarQueue queues[MAX_INDEX];
    int minIndex;
    int maxIndex;
    int size;
};


inline void AstarQueueManagerInit(AstarQueueManager * manager)
{
    manager->minIndex = MAX_INDEX;
    manager->maxIndex = 0;
    manager->size = 0;
}

inline void AstarQueueManagerClear(AstarQueueManager * manager)
{
    for (int i = manager->minIndex; i <= manager->maxIndex; ++i)
    {
        AstarQueueClear(&manager->queues[i]);
    }
    manager->minIndex = MAX_INDEX;
    manager->maxIndex = 0;
    manager->size = 0;
}

inline int AstarQueueManagerAdd(AstarQueueManager * manager, const ASTAR_DATA * data)
{
    //  超出范围
    if (data->_evaluate_value >= MAX_INDEX || data->_evaluate_value < 0)
    {
        return -1;
    }
    int index = data->_evaluate_value;
    //  指定的队列已经满
    if (manager->queues[index].size >= MAX_ELEMENT_NUM)
        return -1;
    AstarQueueAdd(&manager->queues[index], data);
    if (index < manager->minIndex)
        manager->minIndex = index;
    if (index > manager->maxIndex)
        manager->maxIndex = index;
    ++manager->size;
    return 0;
}

inline ASTAR_DATA * AstarQueueManagerRemoveMin(AstarQueueManager * manager)
{
    ASTAR_DATA * rData = NULL;
    for (int i=manager->minIndex; i <= manager->maxIndex; ++i)
    {
        if (manager->queues[i].size > 0)
        {
            --manager->size;
            manager->minIndex = i;
            rData =  AstarQueueRemoveTop(&manager->queues[i]);
            return rData;
        }
    }
    return rData;
}

//统计cpu周期
typedef unsigned long long u64;
#define rdtscll_64(val) do {\
          unsigned int __a,__d; \
          __asm__ __volatile__("rdtsc" : "=a" (__a), "=d" (__d)); \
          (val) = (((unsigned long long)__d)<<32) | (__a); \
          } while(0);

#define rdtscll rdtscll_64


#endif //__ASTAR_QUEUE_MANAGER_H__

