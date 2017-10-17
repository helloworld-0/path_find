#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "AstarQueueManager.h"
#include "MapManager.h"

//  以下为公用结构
int path_finding_times = 0;
MAP_GRID parent_grid[MAP_GRID_XMAX][MAP_GRID_YMAX];		//	某个格子的父节点坐标
//	标记某点是否已关闭，如果mark_grids[x][y]==path_finding_times代表这次已经访问过了
static int mark_grids[MAP_GRID_XMAX][MAP_GRID_YMAX];		

//存放openset,时间复杂度o(1)
AstarQueueManager manager;

void init_path_find_times()
{
    memset(mark_grids, 0, sizeof(mark_grids));
    path_finding_times = 1;
}

bool is_grid_valid(int x, int y)
{
    bool rx = x >= 0 && x < MAP_GRID_XMAX;
    bool ry = y >= 0 &&y < MAP_GRID_YMAX;

    return rx && ry;
}

bool is_grid_valid(MAP_GRID* pos)
{
    return is_grid_valid(pos->x, pos->y);
}

bool jsp_is_parented(MAP_GRID* pos)
{
    if (!is_grid_valid(pos)) return false;
    return mark_grids[pos->x][pos->y] == path_finding_times;
}

int jsp_add_parant( MAP_GRID *point,  MAP_GRID *parent)
{
    if (!is_grid_valid(point) || !is_grid_valid(parent))return -1;
    
    parent_grid[point->x][point->y] = *parent;	
    mark_grids[point->x][point->y] = path_finding_times;		//	标记某点是否已被访问
    return 0;
}

int jsp_add_jump_point( MAP_GRID *point,  MAP_GRID *parent, MAP_GRID *end, bool is_only_diag_front = false)
{
    if (jsp_is_parented(point)) return 0;
    
    jsp_add_parant(point, parent);
    ASTAR_DATA data;
    data._x = point->x;
    data._y = point->y;
    data._evaluate_value =  abs(end->x - point->x) + abs(end->y - point->y);
    data._is_diag_front = is_only_diag_front;
    AstarQueueManagerAdd(&manager, &data);
    return 0;
}

bool has_force_neighbor(MAP_GRID* pos, char xdir, char ydir)
{
    if (ydir == 0)
    {
        if (is_grid_blocked(pos->x, pos->y + 1) && !is_grid_blocked(pos->x + xdir, pos->y + 1)) return true;
        if (is_grid_blocked(pos->x, pos->y - 1) && !is_grid_blocked(pos->x + xdir, pos->y - 1)) return true;
    }
    else if (xdir == 0)
    {
        if (is_grid_blocked(pos->x + 1, pos->y) && !is_grid_blocked(pos->x + 1, pos->y+ydir)) return true;
        if (is_grid_blocked(pos->x - 1, pos->y) && !is_grid_blocked(pos->x - 1, pos->y+ydir)) return true;
    }
    else
    {
        if (is_grid_blocked(pos->x , pos->y - ydir) 
            && !is_grid_blocked(pos->x + xdir, pos->y)
            && !is_grid_blocked(pos->x + xdir, pos->y-ydir)) return true;
        
        if (is_grid_blocked(pos->x - xdir, pos->y) 
            && !is_grid_blocked(pos->x, pos->y + ydir)
            && !is_grid_blocked(pos->x - xdir, pos->y+ydir)) return true;
    }

    return false;
}

//返回值代表是否添加了一个跳点
static bool jsp_horizontal_jump(MAP_GRID *start, char xdir, MAP_GRID *end)
{
    MAP_GRID next(start->x + xdir, start->y);
    while(true)
    {
        if (is_grid_blocked(&next)) return false;
        if (jsp_is_parented(&next)) return false;
        if ((next == *end) || has_force_neighbor(&next, xdir, 0))
        {
            jsp_add_jump_point(&next, start, end);
            return true;
        }
        next.x += xdir;
    }
    return false;
}

static bool jsp_vertical_jump(MAP_GRID *start, char ydir, MAP_GRID *end)
{
    MAP_GRID next(start->x, start->y + ydir);
    while(true)
    {
        if (is_grid_blocked(&next)) return false;
        if (jsp_is_parented(&next)) return false;
        if ((next == *end) || has_force_neighbor(&next, 0, ydir))
        {
            jsp_add_jump_point(&next, start, end);
            return true;
        }
        next.y += ydir;
    }
    return false;
}

static bool jsp_diagonal_jump(MAP_GRID *start, char xdir, char ydir, MAP_GRID *end)
{
    MAP_GRID next(start->x + xdir, start->y + ydir);
    if (is_grid_blocked(start->x +xdir, start->y) && is_grid_blocked(start->x, start->y + ydir)) return false;
    
    while(true)
    {
        if (is_grid_blocked(&next)) return false;
        if (jsp_is_parented(&next)) return false;
        if ((next == *end) || has_force_neighbor(&next, xdir, ydir))
        {
            jsp_add_jump_point(&next, start, end);
            return true;
        }
        bool has_jump_point = jsp_horizontal_jump(&next, xdir, end) || jsp_vertical_jump(&next, ydir, end);
        if (has_jump_point)
        {
            jsp_add_jump_point(&next, start, end, true);
            return true;
        }
        next.x += xdir;
        next.y += ydir;
    }

    return false;
}



int find_path_jsp(MAP_GRID *start, MAP_GRID *end)
{
    int start_x = start->x, start_y = start->y;    
    int end_x = end->x, end_y = end->y;

    //起点既终点
    if ((start_x == end_x) && (start_y == end_y))
    {
        return 0;
    }

    parent_grid[start_x][start_y].x = parent_grid[start_x][start_y].y = -1;
    ++path_finding_times;
    if (path_finding_times < 0) init_path_find_times();
        
    AstarQueueManagerClear(&manager);
    //AstarQueueManagerAdd(&manager, &data);
    jsp_add_parant(start, start);

    //起点向8个方向探测
    jsp_horizontal_jump(start, 1, end);
    jsp_horizontal_jump(start, -1, end);
    jsp_vertical_jump(start, 1, end);
    jsp_vertical_jump(start, -1, end);
    jsp_diagonal_jump(start, 1, 1, end);
    jsp_diagonal_jump(start, 1, -1, end);
    jsp_diagonal_jump(start, -1, 1, end);
    jsp_diagonal_jump(start, -1, -1, end);

    while (true)
    {
        if (manager.size <= 0)
            break;
        
        ASTAR_DATA * tData = AstarQueueManagerRemoveMin(&manager);
        if (tData == NULL) return -2;

        MAP_GRID jump_point(tData->_x,  tData->_y);
        if (!jsp_is_parented(&jump_point)) return -3;
        
        MAP_GRID *parent = &parent_grid[tData->_x][tData->_y];
        //水平方向扩展
        if (jump_point.y == parent->y)
        {
            int xdir = (jump_point.x - parent->x);
            xdir = xdir/abs(xdir);
            jsp_horizontal_jump(&jump_point, xdir, end);
            jsp_vertical_jump(&jump_point, 1, end);
            jsp_vertical_jump(&jump_point, -1, end);
            jsp_diagonal_jump(&jump_point, xdir, 1, end);
            jsp_diagonal_jump(&jump_point, xdir, -1, end);
        }
        //垂直方向扩展
        else if (jump_point.x == parent->x)
        {
            int ydir = (jump_point.y - parent->y);
            ydir = ydir/abs(ydir);
            jsp_vertical_jump(&jump_point, ydir, end);
            jsp_horizontal_jump(&jump_point, 1, end);
            jsp_horizontal_jump(&jump_point, -1, end);
            jsp_diagonal_jump(&jump_point, 1, ydir, end);
            jsp_diagonal_jump(&jump_point, -1, ydir, end);
        }
        //对角线扩展
        else
        {
            int xdir = (jump_point.x - parent->x);
            xdir = xdir/abs(xdir);
            
            int ydir = (jump_point.y - parent->y);
            ydir = ydir/abs(ydir);
            jsp_diagonal_jump(&jump_point, xdir, ydir, end);
            if (!tData->_is_diag_front)
            {
                jsp_horizontal_jump(&jump_point, xdir, end);
                jsp_vertical_jump(&jump_point, ydir, end);
                jsp_diagonal_jump(&jump_point, -xdir, ydir, end);
                jsp_diagonal_jump(&jump_point, xdir, -ydir, end);
                jsp_diagonal_jump(&jump_point, xdir, ydir, end);
            }
            
        }
        
        if (jsp_is_parented(end)) return 0;
    }

    
    return -1;
}

int main(int argc, char* argv[])
{
    if (argc <= 5)
    {
        printf("usage: %s start_x start_y end_x(max:%d) end_y(max:%d) [count]\n", argv[0], MAP_GRID_XMAX, MAP_GRID_YMAX);
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
        find_path_jsp(&start, &end);
    }
    rdtscll(end_circle);
    printf("cost cpu circle %llu\n", end_circle-start_circle);
    
    int ret = find_path_jsp(&start, &end);
    if (ret != 0) 
    {
        printf("can not reach dst. ret = %d\n", ret);
        return ret;
    }
    
    
    int last_x = parent_grid[end.x][end.y].x;
    int last_y = parent_grid[end.x][end.y].y;

    printf("reserved result:(%d, %d)", end.x, end.y);
    while(last_x != start.x || last_y != start.y)
    {
        printf("(%d, %d)", last_x, last_y);
        MAP_GRID *pre_grid = &parent_grid[last_x][last_y];
        last_x = pre_grid->x;
        last_y = pre_grid->y;
    }
    printf("(%d, %d)\n", start.x, start.y);
    return 0;
}

