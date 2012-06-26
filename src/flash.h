//------------------------------------------------------------------------------
// Copyright (c) 2011 by Silicon Laboratories Inc.  All rights reserved.
// The program contained in this listing is proprietary to Silicon Laboratories,
// headquartered in Austin, Texas, U.S.A. and is subject to worldwide copyright
// protection, including protection under the United States Copyright Act of 1976
// as an unpublished work, pursuant to Section 104 and Section 408 of Title XVII
// of the United States code.  Unauthorized copying, adaptation, distribution,
// use, or display is prohibited by this law.
// 
// Silicon Laboratories provides this software solely and exclusively 
// for use on Silicon Laboratories' microcontroller products.
// 
// This software is provided "as is".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
// OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
// SILICON LABORATORIES SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, 
// INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
//------------------------------------------------------------------------------
/// @file myUart0.h

#ifndef __FLASH_H__
#define __FLASH_H__

#include <stdbool.h>

void flash_erase_page(unsigned int addr);
int flash_write_data(unsigned char* buf,unsigned int addr, unsigned int num);
void flash_initialize(void);
#endif
//-eof--------------------------------------------------------------------------
