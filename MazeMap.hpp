// =============================================================================
//  MazeMap.hpp  -  Bit-packed 9x9 maze map (walls + visited cells)
//  MTRN3100 Micromouse - for assignment 4.3 Autonomous Mapping
//
//  AI DISCLOSURE (assignment 5.1): written with assistance of a generative AI
//  (Claude); logic reviewed by the team.
// =============================================================================
//  HOW THE MAZE IS REPRESENTED
//  A 9x9 grid of cells. Walls live BETWEEN cells, so we store them as two
//  separate grids (this way a wall shared by two cells is stored only once):
//
//    vWalls: vertical walls   -> 9 rows x 10 boundaries (left edge .. right edge)
//    hWalls: horizontal walls -> 10 boundaries x 9 cols (top edge .. bottom edge)
//
//  Directions use the same convention as the rest of the code:
//    N = towards row-1,  S = towards row+1,  W = towards col-1,  E = towards col+1
// =============================================================================
#pragma once

#include <Arduino.h>

#define MAZE_ROWS 9
#define MAZE_COLS 9

// Wall directions
#define WALL_N 0
#define WALL_E 1
#define WALL_S 2
#define WALL_W 3

namespace mtrn3100 {

class MazeMap {
public:
    MazeMap() { clear(); }

    // Wipe the map (no walls known, nothing visited).
    void clear() {
        for (uint8_t i = 0; i < sizeof(vWalls); i++) vWalls[i] = 0;
        for (uint8_t i = 0; i < sizeof(hWalls); i++) hWalls[i] = 0;
        for (uint8_t i = 0; i < sizeof(visited); i++) visited[i] = 0;
    }

    // ---- WALLS -------------------------------------------------------------
    // Record that cell (row,col) has a wall on the given side.
    void setWall(int8_t row, int8_t col, uint8_t dir) {
        if (!inRange(row, col)) return;
        switch (dir) {
            case WALL_N: setBit(hWalls, row * MAZE_COLS + col); break;             // boundary above
            case WALL_S: setBit(hWalls, (row + 1) * MAZE_COLS + col); break;       // boundary below
            case WALL_W: setBit(vWalls, row * (MAZE_COLS + 1) + col); break;       // boundary left
            case WALL_E: setBit(vWalls, row * (MAZE_COLS + 1) + col + 1); break;   // boundary right
        }
    }

    bool hasWall(int8_t row, int8_t col, uint8_t dir) const {
        if (!inRange(row, col)) return true;         // outside the maze = treat as wall
        switch (dir) {
            case WALL_N: return getBit(hWalls, row * MAZE_COLS + col);
            case WALL_S: return getBit(hWalls, (row + 1) * MAZE_COLS + col);
            case WALL_W: return getBit(vWalls, row * (MAZE_COLS + 1) + col);
            case WALL_E: return getBit(vWalls, row * (MAZE_COLS + 1) + col + 1);
        }
        return false;
    }

    // Convenience: record the three walls the robot can see from a cell, given
    // which way it is currently facing. front/left/right come from the lidars.
    // facing: 0=N, 1=E, 2=S, 3=W
    void setWallsFromSensors(int8_t row, int8_t col, uint8_t facing,
                             bool front, bool left, bool right) {
        if (front) setWall(row, col, facing);
        if (right) setWall(row, col, (facing + 1) & 3);   // 90 deg clockwise
        if (left)  setWall(row, col, (facing + 3) & 3);   // 90 deg anticlockwise
    }

    // ---- VISITED CELLS -----------------------------------------------------
    void visit(int8_t row, int8_t col) {
        if (inRange(row, col)) setBit(visited, row * MAZE_COLS + col);
    }

    bool isVisited(int8_t row, int8_t col) const {
        return inRange(row, col) && getBit(visited, row * MAZE_COLS + col);
    }

    // How many cells have been visited (used for the % score).
    uint8_t visitedCount() const {
        uint8_t n = 0;
        for (uint8_t i = 0; i < MAZE_ROWS * MAZE_COLS; i++)
            if (getBit(visited, i)) n++;
        return n;
    }

    // Completion percentage required by the assignment (visited / 81 * 100).
    uint8_t percentComplete() const {
        return (uint16_t)visitedCount() * 100 / (MAZE_ROWS * MAZE_COLS);
    }

private:
    static bool inRange(int8_t r, int8_t c) {
        return r >= 0 && r < MAZE_ROWS && c >= 0 && c < MAZE_COLS;
    }
    static void setBit(uint8_t *arr, uint16_t i)       { arr[i >> 3] |=  (1 << (i & 7)); }
    static bool getBit(const uint8_t *arr, uint16_t i) { return arr[i >> 3] & (1 << (i & 7)); }

    // 9 rows x 10 vertical boundaries  = 90 bits -> 12 bytes
    uint8_t vWalls[(MAZE_ROWS * (MAZE_COLS + 1) + 7) / 8];
    // 10 horizontal boundaries x 9 cols = 90 bits -> 12 bytes
    uint8_t hWalls[((MAZE_ROWS + 1) * MAZE_COLS + 7) / 8];
    // 81 cells                          = 81 bits -> 11 bytes
    uint8_t visited[(MAZE_ROWS * MAZE_COLS + 7) / 8];
};

}  // namespace mtrn3100
