/*!
 * @file        Board.c
 *
 * @brief       This file provides firmware functions to manage Leds and push-buttons 
 *
 * @version     V1.0.0
 *
 * @date        2021-07-26
 *
 */

#include "Board.h"

#if defined (APM32E103_MINI)
#include "Board_APM32E103_MINI.c"

#else
#error "Please select first the APM32 board to be used (in board.c)"
#endif


