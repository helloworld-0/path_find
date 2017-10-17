#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "AstarQueueManager.h"
#include "MapManager.h"

//  以下为公用结构
static int mark_grids[MAP_GRID_XMAX][MAP_GRID_YMAX];		//	标记某点是否已被访问
int path_finding_times = 0;//如果mark_grids[x][y]==path_finding_times代表这次已经访问过了

const int  MAX_DIRE_NUM = 3;
const int E[MAX_DIRE_NUM + 1][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};


MAP_GRID last_grid[MAP_GRID_XMAX][MAP_GRID_YMAX];		//	某个格子的上一步位置
static float g_value[MAP_GRID_XMAX][MAP_GRID_YMAX];		//	g值
const float sqrt_two = 1.4f;
AstarQueueManager manager;//临时数据,存放openset

void init_path_find_times()
{
    memset(mark_grids, 0, sizeof(mark_grids));
    path_finding_times = 1;
}



int find_path_astar(const MAP_GRID *start, const MAP_GRID *end)
{
    int start_x = start->x, start_y = start->y;    
    int end_x = end->x, end_y = end->y;

    //起点既终点
    if ((start_x == end_x) && (start_y == end_y))
    {
        return 0;
    }

    last_grid[start_x][start_y].x = last_grid[start_x][start_y].y = -1;
    ++path_finding_times;
    if (path_finding_times < 0) init_path_find_times();
    
    mark_grids[start_x][start_y] = path_finding_times;
    g_value[start_x][start_y] = 0;
    ASTAR_DATA data;
    data._x = start_x;
    data._y = start_y;
    data._evaluate_value =  abs(end_x - start_x) + abs(end_y - start_y);
    AstarQueueManagerClear(&manager);
    AstarQueueManagerAdd(&manager, &data);
 
    mark_grids[end_x][end_y] = 0;
    
    while (true)
    {
        if (manager.size <= 0)
            break;
        
        ASTAR_DATA * tData = AstarQueueManagerRemoveMin(&manager);
        if (tData == NULL) break;

        int curr_x = tData->_x;
        int curr_y = tData->_y;

        //检查水平垂直方向
        for (int j  =MAX_DIRE_NUM; j >= 0; j--)
        {
            int next_x = curr_x + E[j][0];
            int next_y = curr_y + E[j][1];
            if ((next_x >= 0) && (next_y >= 0) && (next_x <= MAP_GRID_XMAX) && (next_y <= MAP_GRID_YMAX))
            { 
                if (is_grid_blocked(next_x, next_y)) continue;
                
                if (mark_grids[next_x][next_y] != path_finding_times)
                {
                    last_grid[next_x][next_y].x = curr_x;
                    last_grid[next_x][next_y].y = curr_y;
                    mark_grids[next_x][next_y] = path_finding_times;
                    g_value[next_x][next_y] = g_value[curr_x][curr_y] + 1;

                    if (next_x == end_x && next_y == end_y) return 0;
                    data._x = next_x;
                    data._y = next_y;
                    data._evaluate_value =  g_value[next_x][next_y] + abs(end_x - next_x) + abs(end_y - next_y);
                    AstarQueueManagerAdd(&manager, &data);
                }
            }
        }
    }

    if (mark_grids[end_x][end_y] == path_finding_times) return 0;
    return -1;
}

int main(int argc, char* argv[])
{
    if (argc <= 5)
    {
        printf("usage: %s start_x start_y end_x(max:%d) end_y(max:%d) count\n", argv[0], MAP_GRID_XMAX, MAP_GRID_YMAX);
        return -1;
    }
    MAP_GRID start, end;
    start.x = atoi(argv[1]);
    start.y = atoi(argv[2]);
    end.x = atoi(argv[3]);
    end.y = atoi(argv[4]);
    int count = atoi(argv[5]);
    u64 start_circle = 0, end_circle = 0;
    rdtscll(start_circle);
    for (int i = 0; i < count; ++i)
    {
        find_path_astar(&start, &end);
    }
    rdtscll(end_circle);
    printf("cost cpu circle %llu\n", end_circle-start_circle);
    
    int ret = find_path_astar(&start, &end);
    if (ret != 0) 
    {
        printf("can not reach dst. ret = %d\n", ret);
        return ret;
    }

    int grid_num = 0;
    MAP_GRID tmp[1000];
    MAP_GRID tmp_last_grid = end;//last_grid[end.x][end.y]
    
    printf("result:");
    while(tmp_last_grid.x >= 0 && tmp_last_grid.y >= 0)
    {
        tmp[grid_num++] = tmp_last_grid;
        tmp_last_grid =  last_grid[tmp_last_grid.x][tmp_last_grid.y];

    }
    
    for (int i = grid_num; i > 0; --i)
    {

        printf("(%d, %d)", tmp[i-1].x, tmp[i-1].y);
    }
    return 0;
}

