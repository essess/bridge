/**
 * Copyright (c) 2024 Sean Stasiak. All rights reserved.
 * Developed by: Sean Stasiak <sean@atypeng.com>
 * Refer to license terms in license.txt; In the absence of such a file,
 * contact me at the above email address and I can provide you with one.
 */

#include <assert.h>
#include "stm32g4xx.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_gpio.h"
#include "stm32g4xx_ll_rcc.h"
#include "can.h"

 /** ---------------------------------------------------------------------------
 * @internal
 * @brief constants
 */

enum _constants_t {
  FLS_NBR = 28,                          /* Max. Filter List Standard Number */
  FLE_NBR = 8,                           /* Max. Filter List Extended Number */
  RF0_NBR = 3,                           /* RX FIFO 0 Elements Number        */
  RF1_NBR = 3,                           /* RX FIFO 1 Elements Number        */
  TEF_NBR = 3,                           /* TX Event FIFO Elements Number    */
  TFQ_NBR = 3,                           /* TX FIFO/Queue Elements Number    */

  FLS_SIZE =  1*4,                   /* Filter Standard Element Size in bytes */
  FLE_SIZE =  2*4,                   /* Filter Extended Element Size in bytes */
  RFQ_SIZE = 18*4,                   /* RX FIFO 0 Elements Size in bytes      */
  TEF_SIZE =  2*4,                   /* TX Event FIFO Elements Size in bytes  */
  TFQ_SIZE = 18*4,                   /* TX FIFO/Queue Elements Size in bytes  */

  FLSSA = 0,                            /* Filter List Standard Start Address */
  FLESA = FLSSA + (FLS_NBR * FLS_SIZE), /* Filter List Extended Start Address */
  RF0SA = FLESA + (FLE_NBR * FLE_SIZE), /* Rx FIFO 0 Start Address            */
  RF1SA = RF0SA + (RF0_NBR * RFQ_SIZE), /* Rx FIFO 1 Start Address            */
  TEFSA = RF1SA + (RF1_NBR * RFQ_SIZE), /* Tx Event FIFO Start Address        */
  TFQSA = TEFSA + (TEF_NBR * TEF_SIZE), /* Tx FIFO/Queue Start Address        */
  SRAMCAN_SIZE = TFQSA + (TFQ_NBR * TFQ_SIZE), /* Message RAM size            */

  SRAMCAN1_BASE = 0x4000A400,
  SRAMCAN2_BASE = 0x4000A750,
  SRAMCAN3_BASE = 0x4000AAA0
};

/** ---------------------------------------------------------------------------
 * @internal
 * @brief protos/fwdrefs/data
 */
typedef union _rxfe_t {
  uint32_t R[RFQ_SIZE/sizeof(uint32_t)];
} rxfe_t;

typedef union _txbe_t {
  uint32_t T[TFQ_SIZE/sizeof(uint32_t)];
} txbe_t;

struct _can_t {
  FDCAN_GlobalTypeDef *r;
  rxfe_t *rf0e[3];
  rxfe_t *rf1e[3];
  txbe_t *txbe[3];
};

static can_t can[] = {
  [CANID_FDCAN1] = { .r = FDCAN1,
                     .rf0e = { [0] = (rxfe_t*)(SRAMCAN1_BASE+RF0SA+(0*RFQ_SIZE)),
                               [1] = (rxfe_t*)(SRAMCAN1_BASE+RF0SA+(1*RFQ_SIZE)),
                               [2] = (rxfe_t*)(SRAMCAN1_BASE+RF0SA+(2*RFQ_SIZE)) },
                     .rf1e = { [0] = (rxfe_t*)(SRAMCAN1_BASE+RF1SA+(0*RFQ_SIZE)),
                               [1] = (rxfe_t*)(SRAMCAN1_BASE+RF1SA+(1*RFQ_SIZE)),
                               [2] = (rxfe_t*)(SRAMCAN1_BASE+RF1SA+(2*RFQ_SIZE)) },
                     .txbe = { [0] = (txbe_t*)(SRAMCAN1_BASE+TFQSA+(0*TFQ_SIZE)),
                               [1] = (txbe_t*)(SRAMCAN1_BASE+TFQSA+(1*TFQ_SIZE)),
                               [2] = (txbe_t*)(SRAMCAN1_BASE+TFQSA+(2*TFQ_SIZE)) }
                   },
  [CANID_FDCAN2] = { .r = FDCAN2,
                     .rf0e = { [0] = (rxfe_t*)(SRAMCAN2_BASE+RF0SA+(0*RFQ_SIZE)),
                               [1] = (rxfe_t*)(SRAMCAN2_BASE+RF0SA+(1*RFQ_SIZE)),
                               [2] = (rxfe_t*)(SRAMCAN2_BASE+RF0SA+(2*RFQ_SIZE)) },
                     .rf1e = { [0] = (rxfe_t*)(SRAMCAN2_BASE+RF1SA+(0*RFQ_SIZE)),
                               [1] = (rxfe_t*)(SRAMCAN2_BASE+RF1SA+(1*RFQ_SIZE)),
                               [2] = (rxfe_t*)(SRAMCAN2_BASE+RF1SA+(2*RFQ_SIZE)) },
                     .txbe = { [0] = (txbe_t*)(SRAMCAN2_BASE+TFQSA+(0*TFQ_SIZE)),
                               [1] = (txbe_t*)(SRAMCAN2_BASE+TFQSA+(1*TFQ_SIZE)),
                               [2] = (txbe_t*)(SRAMCAN2_BASE+TFQSA+(2*TFQ_SIZE)) }
                   },
  [CANID_FDCAN3] = { .r = FDCAN3,
                     .rf0e = { [0] = (rxfe_t*)(SRAMCAN3_BASE+RF0SA+(0*RFQ_SIZE)),
                               [1] = (rxfe_t*)(SRAMCAN3_BASE+RF0SA+(1*RFQ_SIZE)),
                               [2] = (rxfe_t*)(SRAMCAN3_BASE+RF0SA+(2*RFQ_SIZE)) },
                     .rf1e = { [0] = (rxfe_t*)(SRAMCAN3_BASE+RF1SA+(0*RFQ_SIZE)),
                               [1] = (rxfe_t*)(SRAMCAN3_BASE+RF1SA+(1*RFQ_SIZE)),
                               [2] = (rxfe_t*)(SRAMCAN3_BASE+RF1SA+(2*RFQ_SIZE)) },
                     .txbe = { [0] = (txbe_t*)(SRAMCAN3_BASE+TFQSA+(0*TFQ_SIZE)),
                               [1] = (txbe_t*)(SRAMCAN3_BASE+TFQSA+(1*TFQ_SIZE)),
                               [2] = (txbe_t*)(SRAMCAN3_BASE+TFQSA+(2*TFQ_SIZE)) }
                   }
};

/*! assert helper */
static inline int
  is_can(can_t *c)
{
  return((c == &can[CANID_FDCAN1]) || (c == &can[CANID_FDCAN2]) || (c == &can[CANID_FDCAN3]));
}

/** -------------------------------------------------------------------------
 * @public
 * @brief take controller out of init and resync to bus
 */
void
  can_start(can_t *can)
{
  assert(is_can(can));
  CLEAR_BIT(can->r->CCCR, FDCAN_CCCR_INIT);
}

/** -------------------------------------------------------------------------
 * @public
 * @brief initialize can clock source and port i/o's
 *
 *  Default config is:
 *   Traditional CAN2.0 framing at 1MBit
 *   RX all 11bit & 29bit IDs to a 3 element deep RXFIFO0
 *   3 element deep TX FIFO
 *
 *   Assumes PCLK1 = 170MHz
 *   You do your own filtering. This microcontroller
 *   has more horsepower than an old 486. Re-evaluate
 *   in the future.
 * @note NOT THREADSAFE
 */
can_t *can_init(canid_t id)
{
  static unsigned init;
  if(!init) {
    can_init_cb();
    init = ~0;
  }

  if((id == CANID_FDCAN1) && can_init_byid_cb(CANID_FDCAN1)) {
    /* further internal config here */
    return &can[CANID_FDCAN1];
  }
  else
  if((id == CANID_FDCAN2) && can_init_byid_cb(CANID_FDCAN2)) {
    /* further internal config here */
    return &can[CANID_FDCAN2];
  }
  else
  if((id == CANID_FDCAN3) && can_init_byid_cb(CANID_FDCAN3)) {
    /* further internal config here */
    return &can[CANID_FDCAN3];
  }
  return 0;
}

/** -------------------------------------------------------------------------
 * @internal
 * @brief global can initialization; called from the same context
 *        as where can_init() was called
 */
__WEAK void can_init_cb(void)
{
  FDCAN_CONFIG->CKDIV = 0;  /**< PDIV := 0000b (/1)   */
                            /*   170MHz/PDIV = 170MHz */
  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_FDCAN);
  LL_RCC_SetFDCANClockSource(LL_RCC_FDCAN_CLKSOURCE_PLL);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_FDCAN);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_FDCAN);

  uint32_t *const pmem = ((uint32_t*)SRAMCAN_BASE);
  for(unsigned i = 0; i<((1024*3)/sizeof(*pmem)) ;i++) { pmem[i] = 0; }
}


 /** -------------------------------------------------------------------------
 * @internal
 * @brief per-port can initialization; called from the same context
 *        as where can_init() was called
 * @retval int non-zero on success
 */
__WEAK int can_init_byid_cb(canid_t id)
{
  return 0;
}

/** -------------------------------------------------------------------------
 * @public
 * @brief non-blocking poll for an available frame
 *        returns non-zero on success
 */
int can_in_rxf0(can_t *can, canframe_t *frame)
{
  assert(is_can(can));
  assert(frame);

  unsigned const rxf0s = can->r->RXF0S;
  unsigned const f0fl  = (rxf0s & FDCAN_RXF0S_F0FL);
  if(f0fl) {
    unsigned const idx = (rxf0s & FDCAN_RXF0S_F0GI) >> FDCAN_RXF0S_F0GI_Pos;
    assert(idx < 3);

    union {
      struct {
        unsigned id     :29;  /* LSB */
        unsigned rtr    :1;
        unsigned xtd    :1;
        unsigned esi    :1;   /* MSB */
      };
      unsigned uint;
    } R0 = { .uint = can->rf0e[idx]->R[0] };
    frame->extended = R0.xtd;
    frame->id = frame->extended? R0.id : (R0.id >> 18);

    union {
      struct {
        unsigned rxts   :16;  /* LSB */
        unsigned dlc    :4;
        unsigned brs    :1;
        unsigned fdf    :1;
        unsigned        :2;
        unsigned fidx   :7;
        unsigned anmf   :1;   /* MSB */
      };
      unsigned uint;
    } R1 = { .uint = can->rf0e[idx]->R[1] };
    frame->dlc = R1.dlc;

    frame->data.as_uint32[0] = can->rf0e[idx]->R[2];
    frame->data.as_uint32[1] = can->rf0e[idx]->R[3];

    can->r->RXF0A = idx;
  }

  return f0fl;
}

/** -------------------------------------------------------------------------
 * @public
 * @brief any room in the tx fifo?
 *        returns non-zero if room available
 */
int can_peek(can_t *can)
{
  assert(is_can(can));

  unsigned const txfqs = can->r->TXFQS;
  unsigned const tfqf_n = !(txfqs & FDCAN_TXFQS_TFQF);

  return tfqf_n;
}

/** -------------------------------------------------------------------------
 * @public
 * @brief non-blocking send of supplied frame
 *        returns non-zero on success
 */
int can_out(can_t *can, canframe_t *frame)
{
  assert(is_can(can));
  assert(frame);

  unsigned const txfqs = can->r->TXFQS;
  unsigned const tfqf_n = !(txfqs & FDCAN_TXFQS_TFQF);
  if(tfqf_n) {
    unsigned const idx = (txfqs & FDCAN_TXFQS_TFQPI) >> FDCAN_TXFQS_TFQPI_Pos;
    assert(idx < 3);

    union {
      struct {
        unsigned id     :29;  /* LSB */
        unsigned rtr    :1;
        unsigned xtd    :1;
        unsigned esi    :1;   /* MSB */
      };
      unsigned uint;
    } T0 = { .uint = 0 };
    T0.xtd = frame->extended;
    T0.id  = frame->id;
    if(!T0.xtd) { T0.id <<= 18; };
    can->txbe[idx]->T[0] = T0.uint;

    union {
      struct {
        unsigned        :16;  /* LSB */
        unsigned dlc    :4;
        unsigned brs    :1;
        unsigned fdf    :1;
        unsigned        :1;
        unsigned efc    :1;
        unsigned mm     :8;   /* MSB */
      };
      unsigned uint;
    } T1 = { .uint = 0 };
    assert(frame->dlc < 9);
    T1.dlc = frame->dlc;
    can->txbe[idx]->T[1] = T1.uint;

    can->txbe[idx]->T[2] = frame->data.as_uint32[0];
    can->txbe[idx]->T[3] = frame->data.as_uint32[1];

    can->r->TXBAR = (1 << idx);
  }

  return tfqf_n;
}