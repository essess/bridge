/**
 * Copyright (c) 2024 Sean Stasiak. All rights reserved.
 * Developed by: Sean Stasiak <sean@atypeng.com>
 * Refer to license terms in license.txt; In the absence of such a file,
 * contact me at the above email address and I can provide you with one.
 */

#ifndef   _main_h_
#define   _main_h_

#ifdef __cplusplus
extern "C"
{
#endif

enum _taskpri_t {                /*!  global task priorities                      */
  TASKPRI_HIGHEST = 255,         /**< reserved for main() during startup          */
  /* ... */
  TASKPRI_HTBUS   = 200,
  TASKPRI_ATEBUS  = 199,
  /* ... */
  TASKPRI_PD16OUT = 129,
  TASKPRI_IOEOUT  = 128,
  /* ... */
  TASKPRI_TEST     = 3,
  TASKPRI_STATUS   = 2,          /**< periodic IOE/PD16 status msg generation     */
  TASKPRI_BUTLED   = 1,          /**< button debounce and led chirp tasks         */
  TASKPRI_IDLE     = 0,
  TASKPRI_LOWEST   = 0           /**< reserved for main() post startup (idle)     */
};

enum {
  IRQPRI_HIGHEST = 0,
  IRQPRI_0       = IRQPRI_HIGHEST, /**<         !OUTSIDE OF CTL VISIBILITY!         */
  IRQPRI_1       = 1,              /**<         !OUTSIDE OF CTL VISIBILITY!         */
  IRQPRI_2       = 2,              /**<         !OUTSIDE OF CTL VISIBILITY!         */
  IRQPRI_3       = 3,              /**<         !OUTSIDE OF CTL VISIBILITY!         */
  IRQPRI_4       = 4,              /**<         !OUTSIDE OF CTL VISIBILITY!         */
  IRQPRI_5       = 5,              /**<         !OUTSIDE OF CTL VISIBILITY!         */
  IRQPRI_6       = 6,              /**<         !OUTSIDE OF CTL VISIBILITY!         */      
  IRQPRI_7       = 7,              /**<         !OUTSIDE OF CTL VISIBILITY!         */

  IRQPRI_SYSTICK = 8,              /**< highest usable priority within rtos context */
  IRQPRI_FDCAN   = 9,
  IRQPRI_10      = 10,
  IRQPRI_11      = 11,
  IRQPRI_12      = 12,
  IRQPRI_13      = 13,
  IRQPRI_BUTTON  = 14,
  IRQPRI_PENDSV  = 15,
  IRQPRI_LOWEST  = 15,             /**< lowest usable priority within rtos context  */
};

#ifdef __cplusplus
}
#endif

#endif /* _main_h_ */