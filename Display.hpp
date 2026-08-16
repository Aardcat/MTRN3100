// =============================================================================
//  Display.hpp  -  OLED maze visualisation (assignment 4.3 requirement)
//  MTRN3100 Micromouse
//
//  AI DISCLOSURE (assignment 5.1): written with assistance of a generative AI
//  (Claude); logic reviewed by the team.
// =============================================================================
//
//  SETUP:
//    1. Arduino IDE -> Tools -> Manage Libraries -> install "U8g2" (olikraus).
//    2. OLED is I2C: SDA=A4, SCL=A5 (shares the bus with the IMU + lidars).
//       Typical address 0x3C - no clash with lidars (0x52/54/56) or MPU (0x68).
//    3. If your screen shows nothing, try the SH1106 constructor instead
//       (some 128x64 modules use that controller) - see ALTERNATIVE below.
// =============================================================================
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>
// #include <U8g2lib.h>
#include "MazeMap.hpp"

namespace mtrn3100 {

class Display {
public:
    // Page-buffer (low RAM) constructor for a 128x64 SSD1306 OLED over I2C.
    // ALTERNATIVE if your module is an SH1106:
    //   U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
    Display() : u8g2(U8G2_R0, U8X8_PIN_NONE) {}

    // ---- FONTS (change these two lines to resize the text) -----------------
    //  SMALL_FONT: labels + the visited/total line (full alphabet, "_tr").
    //  BIG_FONT  : the % number only. Digits-only ("_tn") to save flash.
    //  Want it EVEN BIGGER? swap BIG_FONT for u8g2_font_fub20_tn (~1.2 kB) and
    //  change the "digits * 10" spacing in drawStats() to digits * 14.
    //  Want it smaller/cheaper? use u8g2_font_7x13B_tn and digits * 7.
    #define SMALL_FONT u8g2_font_5x8_tr
    #define BIG_FONT   u8g2_font_10x20_tn

    void begin() {
        u8g2.begin();
        u8g2.setFont(SMALL_FONT);
    }

    // Draw the whole screen: maze on the left, stats on the right.
    // robotRow/robotCol = where the robot is now (pass -1 to hide the marker).
    void showMaze(const MazeMap &map, int8_t robotRow = -1, int8_t robotCol = -1) {
        u8g2.firstPage();
        do {
            drawMaze(map, robotRow, robotCol);
            drawStats(map);
        } while (u8g2.nextPage());        // page loop = the low-RAM rendering
    }

    // Simple text message (handy for "MAPPING", "SOLVING", errors, etc.)
    void showMessage(const char *line1, const char *line2 = nullptr) {
        u8g2.firstPage();
        do {
            u8g2.setFont(SMALL_FONT);      // make sure we are not still on BIG_FONT
            u8g2.drawStr(2, 12, line1);
            if (line2) u8g2.drawStr(2, 26, line2);
        } while (u8g2.nextPage());
    }

private:
    // --- layout constants (128x64 screen) ---
    static const uint8_t CELL = 6;    // pixels per maze cell
    static const uint8_t OX   = 1;    // maze origin x
    static const uint8_t OY   = 1;    // maze origin y
    // 9 cells * 6 px = 54 px square, leaving x=56..127 for text.

    void drawMaze(const MazeMap &map, int8_t robotRow, int8_t robotCol) {
        for (int8_t r = 0; r < MAZE_ROWS; r++) {
            for (int8_t c = 0; c < MAZE_COLS; c++) {
                uint8_t x = OX + c * CELL;
                uint8_t y = OY + r * CELL;

                // visited cells get a dot in the middle so you can see coverage
                if (map.isVisited(r, c)) u8g2.drawPixel(x + CELL / 2, y + CELL / 2);

                // draw only the N and W walls of each cell, plus the outer
                // S/E edges - this avoids drawing every shared wall twice.
                if (map.hasWall(r, c, WALL_N)) u8g2.drawHLine(x, y, CELL);
                if (map.hasWall(r, c, WALL_W)) u8g2.drawVLine(x, y, CELL);
                if (r == MAZE_ROWS - 1 && map.hasWall(r, c, WALL_S))
                    u8g2.drawHLine(x, y + CELL, CELL);
                if (c == MAZE_COLS - 1 && map.hasWall(r, c, WALL_E))
                    u8g2.drawVLine(x + CELL, y, CELL);
            }
        }

        // robot marker: a small filled box in its current cell
        if (robotRow >= 0 && robotCol >= 0) {
            u8g2.drawBox(OX + robotCol * CELL + 2, OY + robotRow * CELL + 2, 3, 3);
        }
    }

    void drawStats(const MazeMap &map) {
        const uint8_t TX = 58;                      // text column x

        // NOTE (memory): we use u8g2.print() rather than snprintf(). snprintf
        // drags the whole vfprintf formatter into flash (~1.5 kB) which we
        // cannot afford.
        //
        // FONT SIZES: the big "%" number uses a DIGITS-ONLY font (the "_tn"
        // suffix instead of "_tr"). A _tn font contains only 0-9 and a few
        // symbols, so it costs ~600 bytes of flash instead of ~2.5 kB for the
        // full alphabet. The '%' sign is not in a _tn font, so it is drawn
        // with the small font right after the number.
        u8g2.drawStr(TX, 7, "MAPPED");

        // ---- big % number (the score the assignment asks for) --------------
        u8g2.setFont(BIG_FONT);
        u8g2.setCursor(TX, 30);                     // baseline of the big text
        u8g2.print(map.percentComplete());
        uint8_t digits = (map.percentComplete() >= 100) ? 3
                       : (map.percentComplete() >= 10)  ? 2 : 1;
        u8g2.setFont(SMALL_FONT);                   // back to small for the '%'
        u8g2.drawStr(TX + digits * 10 + 1, 30, "%");

        // ---- "visited/total" so the marker can see the raw count -----------
        u8g2.setCursor(TX, 44);
        u8g2.print(map.visitedCount());
        u8g2.print('/');
        u8g2.print((uint8_t)(MAZE_ROWS * MAZE_COLS));
    }

    U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2;
};

}  // namespace mtrn3100
