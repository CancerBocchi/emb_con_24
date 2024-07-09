#ifndef __A_START_H__
#define __A_START_H__

#include <stdio.h>
#include <math.h>
#include <stdbool.h>

#define ROW 10
#define COL 10
#define INF 1e9
#define MAX_NODES ROW * COL

typedef struct {
    int x, y;
} Point;

//g为从起点移动到此的代价，h为从此移动到终点的代价
typedef struct {
    Point point;
    int f, g, h;
} Node;

typedef struct {
    Node nodes[MAX_NODES];
    int size;
} PriorityQueue;

extern Point return_node[25];

int find_node(Point start,Point goal);
void a_star_search(int grid[ROW][COL], Point start, Point goal);

#endif // !A_START_H__
