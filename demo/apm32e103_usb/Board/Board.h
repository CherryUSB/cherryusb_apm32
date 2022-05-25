/*!
 * @file        Board.h
 *
 * @brief       Header file for Board
 *
 * @version     V1.0.0
 *
 * @date        2021-07-26
 *
 */
#ifndef BOARD_H
#define BOARD_H

#if defined (APM32E103_MINI)
#include "Board_APM32E103_MINI.h"

#else
#error "Please select first the APM32  board to be used (in board.h)"
#endif

#endif

