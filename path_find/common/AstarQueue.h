#ifndef _ASTAR_QUEUE_H
#define _ASTAR_QUEUE_H
#include <stddef.h>

#define MAX_ELEMENT_NUM 1024 //== max(MAP_GRID_XMAX, MAP_GRID_YMAX)

//  以下是A*算法(队列)所算的结构
typedef struct astar_data
{
    short _x;
    short _y;
    short _evaluate_value;
    bool _is_diag_front;
} ASTAR_DATA;

struct AstarQueue
{
    ASTAR_DATA elements[MAX_ELEMENT_NUM];
    short headIndex;
    short tailIndex;
    short size;
};

inline int AstarQueueClear(AstarQueue * queue)
{
    queue->size = 0;
    return 0;
}

inline int AstarQueueAdd(AstarQueue * queue, const ASTAR_DATA * element)
{
    if (queue->size == 0)
    {
        queue->headIndex = queue->tailIndex = 0;
    }
    else
    {
        ++queue->tailIndex;
        if (queue->tailIndex >= MAX_ELEMENT_NUM)
        {
            --queue->tailIndex;
            return -1;
        }
    }
    queue->elements[queue->tailIndex] = *element;
    ++queue->size;
    return 0;
}

inline ASTAR_DATA * AstarQueueRemoveTop(AstarQueue * queue)
{
    if (queue->size <= 0)
        return NULL;
    else
    {
        --queue->size;
        ++queue->headIndex;
        return &queue->elements[queue->headIndex - 1];
    }
}

#endif //_ASTAR_QUEUE_H

