/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 by Hector Cura Jr.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#ifndef _timer_util_h_
#define _timer_util_h_

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 *  Convenience values for some common timings.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define TICKS_IN_100_NS    8
#define TICKS_IN_250_NS   20
#define TICKS_IN_500_NS   40 
#define TICKS_IN_300_NS   24
#define TICKS_IN_600_NS   48
#define TICKS_IN_750_NS   60
#define TICKS_IN_900_NS   72 
#define TICKS_IN_1000_NS  80
#define TICKS_IN_1300_NS 104
#define TICKS_IN_2000_NS 160
#define TICKS_IN_3000_NS 240
#define TICKS_IN_3450_NS 276
#define TICKS_IN_4000_NS 320
#define TICKS_IN_4700_NS 376
#define TICKS_IN_5000_NS 400
 
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 *  Following method returns the current value of special
 *  CCOUNT register. This register counts number of processor 
 *  ticks and overflows every 53.7 seconds.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
inline uint32_t getCCOUNT() 
{
    uint32_t ccount_val;
    __asm__ volatile ("esync; rsr %0, ccount" : "=r"(ccount_val));
    return ccount_val;    
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
 *  Following method introduces an approximate delay in ticks.
 *  The ticks are based on cpu clock speed. Default for ESP8266EX
 *  is 80MhZ. This means each tick executes in ~12.5ns.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
inline void delay(volatile uint32_t delay_time_in_cpu_ticks) 
{
    uint32_t start = getCCOUNT();
    while ((getCCOUNT()-start) < delay_time_in_cpu_ticks);
}
#endif // _timer_util_h_

