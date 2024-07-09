#include "a_star.h"

int directions[4][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
Point came_from[ROW][COL];
//运行节点，，找到需要转向的点，在使用该运行节点完成车辆运行
Point return_node[25];


void swap(Node *a, Node *b) {
    Node temp = *a;
    *a = *b;
    *b = temp;
}

void push(PriorityQueue *pq, Node node) {
    pq->nodes[pq->size++] = node;
    int i = pq->size - 1;
    while (i && pq->nodes[i].f < pq->nodes[(i - 1) / 2].f) {
        swap(&pq->nodes[i], &pq->nodes[(i - 1) / 2]);
        i = (i - 1) / 2;
    }
}

Node pop(PriorityQueue *pq) {
    Node root = pq->nodes[0];
    pq->nodes[0] = pq->nodes[--pq->size];
    int i = 0;
    while (2 * i + 1 < pq->size) {
        int left = 2 * i + 1;
        int right = 2 * i + 2;
        int smallest = left;
        if (right < pq->size && pq->nodes[right].f < pq->nodes[left].f) {
            smallest = right;
        }
        if (pq->nodes[i].f < pq->nodes[smallest].f) break;
        swap(&pq->nodes[i], &pq->nodes[smallest]);
        i = smallest;
    }
    return root;
}

bool is_valid(int grid[ROW][COL], int x, int y) {
    return (x >= 0) && (x < ROW) && (y >= 0) && (y < COL) && (grid[x][y] == 0);
}

int heuristic(Point a, Point b) {
    return abs(a.x - b.x) + abs(a.y - b.y);
}

void a_star_search(int grid[ROW][COL], Point start, Point goal) {
    PriorityQueue open_set = { .size = 0 };
    bool closed_set[ROW][COL] = {false};
    int g_score[ROW][COL];

    for (int i = 0; i < ROW; i++) {
        for (int j = 0; j < COL; j++) {
            g_score[i][j] = INF;
        }
    }

    g_score[start.x][start.y] = 0;
    Node start_node = {start, heuristic(start, goal), 0, heuristic(start, goal)};
    push(&open_set, start_node);

    while (open_set.size > 0) {
        Node current_node = pop(&open_set);
        Point current = current_node.point;

        if (current.x == goal.x && current.y == goal.y) {
            printf("Path found: ");
            while (!(current.x == start.x && current.y == start.y)) {
                printf("(%d, %d) <- ", current.x, current.y);
                current = came_from[current.x][current.y];
            }
            printf("(%d, %d)\n", start.x, start.y);
            return;
        }

        closed_set[current.x][current.y] = true;

        for (int i = 0; i < 4; i++) {
            int new_x = current.x + directions[i][0];
            int new_y = current.y + directions[i][1];

            if (!is_valid(grid, new_x, new_y) || closed_set[new_x][new_y]) {
                continue;
            }

            int tentative_g_score = g_score[current.x][current.y] + 1;

            if (tentative_g_score < g_score[new_x][new_y]) {
                came_from[new_x][new_y] = current;
                g_score[new_x][new_y] = tentative_g_score;
                int f_score = tentative_g_score + heuristic((Point){new_x, new_y}, goal);
                Node neighbor = {(Point){new_x, new_y}, f_score, tentative_g_score, heuristic((Point){new_x, new_y}, goal)};
                push(&open_set, neighbor);
            }
        }
    }

    printf("No path found.\n");
}

/**
 * @brief 根据 return node 中的列表寻找转折点
 * 
 * @param start 开始点
 * @param goal 终止点
 * 
 * @retval 返回一个节点数量
 */
int find_node(Point start,Point goal){
    Point current = goal;
    int i = 1;
    int x_change_flag = 0;
    int y_change_flag = 0;
    while (!(current.x == start.x && current.y == start.y)) {

        if(x_change_flag == 0 && y_change_flag == 0) {
            if (current.x == came_from[current.x][current.y].x)
                y_change_flag = 1;
            else if (current.y == came_from[current.x][current.y].y)
                x_change_flag = 1;
        }
        else{
            //这一次变化x
            if (current.y == came_from[current.x][current.y].y){
                //上一次没变化 x
                if(!x_change_flag) {
                    return_node[i] = current;
                    i++;
                    y_change_flag = 0;
                    x_change_flag = 1;
                }
            }
                //这一次变化y
            else if (current.x == came_from[current.x][current.y].x)
                //上一次没变化 y
                if(!y_change_flag){
                    return_node[i] = current;
                    i++;
                    y_change_flag = 1;
                    x_change_flag = 0;
                }
        }

        current = came_from[current.x][current.y];
    }
    return_node[0] = goal;
    return_node[i] = start;
    return i+1;
}