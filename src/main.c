/**
 * Copyright (c) 2024 Sean Stasiak. All rights reserved.
 * Developed by: Sean Stasiak <sean@atypeng.com>
 * Refer to license terms in license.txt; In the absence of such a file,
 * contact me at the above email address and I can provide you with one.
 *
 * Built upon: https://github.com/WeActStudio/WeActStudio.STM32G474CoreBoard
 * with a STM32G474CEU6 onboard (170Mhz, 128KB RAM, 512KB ROM)
 *
 * See also: https://github.com/STMicroelectronics/stm32g4xx-hal-driver
 *           https://htmlpreview.github.io/?https://github.com/STMicroelectronics/STM32CubeG4/blob/master/Release_Notes.html
 *
 * IN USE:
 *    PC.6  - BLUE LED
 *    PC.13 - USER BUTTON
 *    PB.8  - BOOT0 BUTTON
 *    PA.13 - SWDIO
 *    PA.14 - SWCLK
 *    PB.3  - FDCAN3_RX (ATE BUS)
 *    PB.4  - FDCAN3_TX (ATE BUS)
 *    PB.5  - FDCAN2_RX (HALTECH BUS)
 *    PB.6  - FDCAN2_TX (HALTECH BUS)
 *
 */

#include <stdint.h>
#include <assert.h>
#include <ctl_api.h>

#include "main.h"
#include "debugio.h"                /**< do not use within IRQ contexts! */
#include "stm32g4xx.h"                // get to point where these can be del'd
#include "stm32g4xx_ll_gpio.h"        // get to point where these can be del'd
#include "stm32g4xx_ll_bus.h"         // get to point where these can be del'd
#include "stm32g4xx_ll_system.h"      // get to point where these can be del'd
#include "stm32g4xx_ll_exti.h"        // get to point where these can be del'd
#include "stm32g4xx_ll_rcc.h"         // get to point where these can be del'd

#include "stm32g4xx_hal.h"

/** ---------------------------------------------------------------------------
 * @internal
 * @brief constants
 */
typedef enum _known_haltech_rxids_t {
  HTID_ECU_3E7   = 0x3E7,   /**< GenSens1<->4 from ecu         - 20Hz */
  HTID_ECU_3E8   = 0x3E8,   /**< GenSens5<->8 from ecu         - 20Hz */
  HTID_ECU_3E9   = 0x3E9,   /**< GenSens9/10 from ecu          - 20Hz */
  HTID_ECU_6F7   = 0x6F7,   /**< GenOut1<->20 from ecu         - 10Hz */

  HTID_PD16A_6D0 = 0x6D0,   /**< HCO25/HCO8/HBO command        - 20Hz max, 333Hz min */
  HTID_PD16A_6D1 = 0x6D1,   /**< HCO25/HCO8/HBO/SPI/AVI config - 2Hz max,  50Hz min  */
  HTID_PD16A_6D2 = 0x6D2,   /**< ECR/TERM config               - 2Hz max,  2Hz min - 1Hz actual */

  HTID_PD16B_6D8 = 0x6D8,   /**< HCO25/HCO8/HBO command        - 20Hz max, 333Hz min */
  HTID_PD16B_6D9 = 0x6D9,   /**< HCO25/HCO8/HBO/SPI/AVI config - 2Hz max,  50Hz min  */
  HTID_PD16B_6DA = 0x6DA,   /**< ECR/TERM config               - 2Hz max,  2Hz min - 1Hz actual */

  HTID_PD16C_6E0 = 0x6E0,   /**< HCO25/HCO8/HBO command        - 20Hz max, 333Hz min */
  HTID_PD16C_6E1 = 0x6E1,   /**< HCO25/HCO8/HBO/SPI/AVI config - 2Hz max,  50Hz min  */
  HTID_PD16C_6E2 = 0x6E2,   /**< ECR/TERM config               - 2Hz max,  2Hz min - 1Hz actual */

  HTID_PD16D_6E8 = 0x6E8,   /**< HCO25/HCO8/HBO command        - 20Hz max, 333Hz min */
  HTID_PD16D_6E9 = 0x6E9,   /**< HCO25/HCO8/HBO/SPI/AVI config - 2Hz max,  50Hz min  */
  HTID_PD16D_6EA = 0x6EA,   /**< ECR/TERM config               - 2Hz max,  2Hz min - 1Hz actual */

  HTID_PD16R5_700 = 0x700,  /**< same as "HCO25/HCO8/HBO command" but wrt internal R5 PD16 */

} known_haltech_rxids_t;

/**
 * [ 4bits ][   6bits   ][    6bits     ]
 * [  0000 ][ module_id ][   code_id    ]
 */

#define RESERVED_MODULE         ( 0)    /**< NOT available for use */
#define MISC_MODULE             ( 1)
#define IOA_MODULE              ( 2)
#define IOB_MODULE              ( 3)
#define PD16A_MODULE            ( 4)
#define PD16B_MODULE            ( 5)
#define PD16C_MODULE            ( 6)
#define PD16D_MODULE            ( 7)

#define _CODE( mod_, code_ )    ( ((mod_&0x3ful) << 6) | (code_&0x3ful) )

#define MISC_CODE( code_ )      ( _CODE( MISC_MODULE,  code_ ) )
#define IOA_CODE( code_ )       ( _CODE( IOA_MODULE,   code_ ) )
#define IOB_CODE( code_ )       ( _CODE( IOB_MODULE,   code_ ) )
#define PD16A_CODE( code_ )     ( _CODE( PD16A_MODULE, code_ ) )
#define PD16B_CODE( code_ )     ( _CODE( PD16B_MODULE, code_ ) )
#define PD16C_CODE( code_ )     ( _CODE( PD16C_MODULE, code_ ) )
#define PD16D_CODE( code_ )     ( _CODE( PD16D_MODULE, code_ ) )

#define GET_MODULE( code_ )     ( (code_ & (0x3fu<<6)) >> 6 )
#define GET_CODE( code_ )       ( code_ & 0x3fu )

typedef enum _errcode_t { /*! error codes */
  CODE_NONE    = 0,

  CODE_TXQFULL           = MISC_CODE( 0),
  CODE_TXFIFO            = MISC_CODE( 1),
  CODE_UNKMUXID          = MISC_CODE( 2),
  CODE_DEBOUNCE_UNK0     = MISC_CODE( 3),
  CODE_DEBOUNCE_UNK1     = MISC_CODE( 4),
  CODE_DEBOUNCE_UNK2     = MISC_CODE( 5),
  CODE_TXTIMEOUT         = MISC_CODE( 6),
  CODE_FRAMELOST         = MISC_CODE( 7),
  CODE_RXFIFO0           = MISC_CODE( 8),
  CODE_RXFIFO1           = MISC_CODE( 9),
  CODE_TXEFIFO           = MISC_CODE(10),
  CODE_RXFIFO0_ALLOC     = MISC_CODE(11),
  CODE_RXQFULL           = MISC_CODE(12),
  CODE_TXBUF_ABT         = MISC_CODE(13),
  CODE_ERR_PASSIVE       = MISC_CODE(14),
  CODE_ERR_WARNING       = MISC_CODE(15),
  CODE_BUS_OFF           = MISC_CODE(16),
  CODE_GENSENS_LOCK_FAIL = MISC_CODE(17),
  CODE_GENOUT_LOCK_FAIL  = MISC_CODE(18),
  CODE_UNKNOWN_ECUFRAME  = MISC_CODE(19),

  CODE_IOA_SENDSTATUS  = IOA_CODE( 0),
  CODE_IOA_ALLOCSTATUS = IOA_CODE( 1),
  CODE_IOA_SENDAVI14   = IOA_CODE( 2),
  CODE_IOA_SENDDPI12   = IOA_CODE( 3),
  CODE_IOA_SENDDPI34   = IOA_CODE( 4),

  CODE_IOB_SENDSTATUS  = IOB_CODE( 0),
  CODE_IOB_ALLOCSTATUS = IOB_CODE( 1),
  CODE_IOB_SENDAVI14   = IOA_CODE( 2),
  CODE_IOB_SENDDPI12   = IOA_CODE( 3),
  CODE_IOB_SENDDPI34   = IOA_CODE( 4),

  CODE_PD16A_SENDSTATUS  = PD16A_CODE( 0),
  CODE_PD16A_ALLOCSTATUS = PD16A_CODE( 1),
  CODE_PD16A_SENDAVI1    = PD16A_CODE( 2),
  CODE_PD16A_SENDAVI2    = PD16A_CODE( 3),
  CODE_PD16A_SENDAVI3    = PD16A_CODE( 4),
  CODE_PD16A_SENDAVI4    = PD16A_CODE( 5),
  CODE_PD16A_SENDSPI1    = PD16A_CODE( 6),
  CODE_PD16A_SENDSPI2    = PD16A_CODE( 7),
  CODE_PD16A_SENDSPI3    = PD16A_CODE( 8),
  CODE_PD16A_SENDSPI4    = PD16A_CODE( 9),
  CODE_PD16A_SENDHCO251  = PD16A_CODE(10),
  CODE_PD16A_SENDHCO252  = PD16A_CODE(11),
  CODE_PD16A_SENDHCO253  = PD16A_CODE(12),
  CODE_PD16A_SENDHCO254  = PD16A_CODE(13),
  CODE_PD16A_SENDHCO81   = PD16A_CODE(14),
  CODE_PD16A_SENDHCO82   = PD16A_CODE(15),
  CODE_PD16A_SENDHCO83   = PD16A_CODE(16),
  CODE_PD16A_SENDHCO84   = PD16A_CODE(17),
  CODE_PD16A_SENDHCO85   = PD16A_CODE(18),
  CODE_PD16A_SENDHCO86   = PD16A_CODE(19),
  CODE_PD16A_SENDHCO87   = PD16A_CODE(20),
  CODE_PD16A_SENDHCO88   = PD16A_CODE(21),
  CODE_PD16A_SENDHCO89   = PD16A_CODE(22),
  CODE_PD16A_SENDHCO810  = PD16A_CODE(23),
  CODE_PD16A_SENDHBO1    = PD16A_CODE(24),
  CODE_PD16A_SENDHBO2    = PD16A_CODE(25),
  CODE_PD16A_SENDDIAG0   = PD16A_CODE(26),
  CODE_PD16A_SENDDIAG1   = PD16A_CODE(27),
  CODE_PD16A_SENDDIAG2   = PD16A_CODE(28),
  CODE_PD16A_SENDDIAG3   = PD16A_CODE(29),
  CODE_PD16A_SENDDIAG4   = PD16A_CODE(30),
  CODE_PD16A_SENDDIAG5   = PD16A_CODE(31),

  CODE_PD16B_SENDSTATUS  = PD16B_CODE( 0),
  CODE_PD16B_ALLOCSTATUS = PD16B_CODE( 1),

  CODE_PD16C_SENDSTATUS  = PD16C_CODE( 0),
  CODE_PD16C_ALLOCSTATUS = PD16C_CODE( 1),

  CODE_PD16D_SENDSTATUS  = PD16D_CODE( 0),
  CODE_PD16D_ALLOCSTATUS = PD16D_CODE( 1),

  CODE_UNKNOWN = ~0ul
} errcode_t;
CTL_MESSAGE_QUEUE_t codes;

typedef enum _evt_t { /*! event set e, e_ioa, ... (uint32_t) */

  /*> BEGIN e <*/
  EVT_LED_CHIRP = (1<<24),

  EVT_HTBUS_RXQ_NOTEMPTY = (1<<9),
  EVT_HTBUS_TXQWD        = (1<<8),

  EVT_BOOT0_BUTUP        = (1<<7),
  EVT_BOOT0_BUTDOWN      = (1<<6),
  EVT_BOOT0_EDGE_BUTUP   = (1<<5),
  EVT_BOOT0_EDGE_BUTDOWN = (1<<4),

  EVT_USER_BUTUP        = (1<<3),
  EVT_USER_BUTDOWN      = (1<<2),
  EVT_USER_EDGE_BUTUP   = (1<<1),
  EVT_USER_EDGE_BUTDOWN = (1<<0),
  /*> END   e <*/

  /*> BEGIN e_ioe <*/
  EVT_IOE_DPI4 = (1<<7),
  EVT_IOE_DPI3 = (1<<6),
  EVT_IOE_DPI2 = (1<<5),
  EVT_IOE_DPI1 = (1<<4),
  EVT_IOE_AVI4 = (1<<3),
  EVT_IOE_AVI3 = (1<<2),
  EVT_IOE_AVI2 = (1<<1),
  EVT_IOE_AVI1 = (1<<0),
  /*> END   e_ioe <*/

  /*> BEGIN e_pd16 <*/
  EVT_PD16_DIAG4   = (1<<28),
  EVT_PD16_DIAG3   = (1<<27),
  EVT_PD16_DIAG2   = (1<<26),
  EVT_PD16_DIAG1   = (1<<25),
  EVT_PD16_DIAG0   = (1<<24),
  EVT_PD16_HBO_2   = (1<<23),
  EVT_PD16_HBO_1   = (1<<22),
  EVT_PD16_HCO8_10 = (1<<21),
  EVT_PD16_HCO8_9  = (1<<20),
  EVT_PD16_HCO8_8  = (1<<19),
  EVT_PD16_HCO8_7  = (1<<18),
  EVT_PD16_HCO8_6  = (1<<17),
  EVT_PD16_HCO8_5  = (1<<16),
  EVT_PD16_HCO8_4  = (1<<15),
  EVT_PD16_HCO8_3  = (1<<14),
  EVT_PD16_HCO8_2  = (1<<13),
  EVT_PD16_HCO8_1  = (1<<12),
  EVT_PD16_HCO25_4 = (1<<11),
  EVT_PD16_HCO25_3 = (1<<10),
  EVT_PD16_HCO25_2 = (1<<9),
  EVT_PD16_HCO25_1 = (1<<8),
  EVT_PD16_SPI4    = (1<<7),
  EVT_PD16_SPI3    = (1<<6),
  EVT_PD16_SPI2    = (1<<5),
  EVT_PD16_SPI1    = (1<<4),
  EVT_PD16_AVI4    = (1<<3),
  EVT_PD16_AVI3    = (1<<2),
  EVT_PD16_AVI2    = (1<<1),
  EVT_PD16_AVI1    = (1<<0),
  /*> END   e_pd16 <*/

  /*> BEGIN e_ecu_upd0 <*/
  EVT_ECU_GENSENS10 = (1<<29),
  EVT_ECU_GENSENS9  = (1<<28),
  EVT_ECU_GENSENS8  = (1<<27),
  EVT_ECU_GENSENS7  = (1<<26),
  EVT_ECU_GENSENS6  = (1<<25),
  EVT_ECU_GENSENS5  = (1<<24),
  EVT_ECU_GENSENS4  = (1<<23),
  EVT_ECU_GENSENS3  = (1<<22),
  EVT_ECU_GENSENS2  = (1<<21),
  EVT_ECU_GENSENS1  = (1<<20),
  EVT_ECU_GENOUT20  = (1<<19),
  EVT_ECU_GENOUT19  = (1<<18),
  EVT_ECU_GENOUT18  = (1<<17),
  EVT_ECU_GENOUT17  = (1<<16),
  EVT_ECU_GENOUT16  = (1<<15),
  EVT_ECU_GENOUT15  = (1<<14),
  EVT_ECU_GENOUT14  = (1<<13),
  EVT_ECU_GENOUT13  = (1<<12),
  EVT_ECU_GENOUT12  = (1<<11),
  EVT_ECU_GENOUT11  = (1<<10),
  EVT_ECU_GENOUT10  = (1<<9),
  EVT_ECU_GENOUT9   = (1<<8),
  EVT_ECU_GENOUT8   = (1<<7),
  EVT_ECU_GENOUT7   = (1<<6),
  EVT_ECU_GENOUT6   = (1<<5),
  EVT_ECU_GENOUT5   = (1<<4),
  EVT_ECU_GENOUT4   = (1<<3),
  EVT_ECU_GENOUT3   = (1<<2),
  EVT_ECU_GENOUT2   = (1<<1),
  EVT_ECU_GENOUT1   = (1<<0),
  /*> END   e_ecu_upd0 <*/

  EVT_NONE = 0<<0
} evt_t;

/** ---------------------------------------------------------------------------
 * @internal
 * @brief protos/fwdrefs/data
 */
static CTL_EVENT_SET_t e;             /*< top-level events  */
static CTL_EVENT_SET_t e_ioa, e_iob;  /*< IOE events -TEMP  */
static CTL_EVENT_SET_t e_pd16a;       /*< PD16 events -TEMP */
static CTL_EVENT_SET_t e_ecu_upd0;    /*< ECU update events */


static __NO_RETURN void task_test(void *);
static __NO_RETURN void task_led_chirp(void *);
static __NO_RETURN void task_but_debounce(void *);

static __NO_RETURN void task_ioe_status_out(void *);

static __NO_RETURN void task_ioa_avi14_out(void *);
static __NO_RETURN void task_ioa_dpi12_out(void *);
static __NO_RETURN void task_ioa_dpi34_out(void *);

static __NO_RETURN void task_iob_avi14_out(void *);
static __NO_RETURN void task_iob_dpi12_out(void *);
static __NO_RETURN void task_iob_dpi34_out(void *);

static __NO_RETURN void task_pd16_status_out(void *);

static __NO_RETURN void task_pd16a_avispi_out(void *);
static __NO_RETURN void task_pd16a_hco8hco25hbo_out(void *);
static __NO_RETURN void task_pd16a_diag_out(void *);

static __NO_RETURN void task_pd16b_avispi_out(void *);
static __NO_RETURN void task_pd16b_hco8hco25hbo_out(void *);
static __NO_RETURN void task_pd16b_diag_out(void *);

static __NO_RETURN void task_pd16c_avispi_out(void *);
static __NO_RETURN void task_pd16c_hco8hco25hbo_out(void *);
static __NO_RETURN void task_pd16c_diag_out(void *);

static __NO_RETURN void task_pd16d_avispi_out(void *);
static __NO_RETURN void task_pd16d_hco8hco25hbo_out(void *);
static __NO_RETURN void task_pd16d_diag_out(void *);

static __NO_RETURN void task_htbus(void *);

typedef struct _debounce_info_t { /*! use with task_but_debounce() */
  CTL_TIME_t to;
  CTL_EVENT_SET_t *pe;
  CTL_EVENT_SET_t rising;
  CTL_EVENT_SET_t falling;
  CTL_EVENT_SET_t up;
  CTL_EVENT_SET_t down;
} debounce_info_t;

typedef struct _canframe_t {
  union {
    FDCAN_RxHeaderTypeDef rxhdr;
    FDCAN_TxHeaderTypeDef txhdr;
  };
  union {
    uint8_t  as_uint8[64];
    uint32_t as_uint32[64/sizeof(uint32_t)];
  } data;
} canframe_t;

CTL_MESSAGE_QUEUE_t txq;
#define TXQ_COUNT (16)
void *txq_mem[TXQ_COUNT];

CTL_MESSAGE_QUEUE_t rxq;
#define RXQ_COUNT (16)
void *rxq_mem[RXQ_COUNT];

CTL_MEMORY_AREA_t frames;
#define FRAME_WORDSIZE  ((sizeof(canframe_t)/sizeof(unsigned))+1)
#define FRAME_COUNT     (TXQ_COUNT+RXQ_COUNT)
unsigned frame_mem[FRAME_WORDSIZE*FRAME_COUNT];

static unsigned htbus_send(canframe_t *);

typedef union _genout_t { /*! GenOut1 through GenOut20 state */
  struct {                /*   COMPILER DEPENDENT ORDERING   */
    uint32_t     :8;
    uint32_t _17 :1;
    uint32_t _18 :1;
    uint32_t _19 :1;
    uint32_t _20 :1;
    uint8_t      :4;
    uint32_t _9  :1;
    uint32_t _10 :1;
    uint32_t _11 :1;
    uint32_t _12 :1;
    uint32_t _13 :1;
    uint32_t _14 :1;
    uint32_t _15 :1;
    uint32_t _16 :1;
    uint32_t _1  :1;
    uint32_t _2  :1;
    uint32_t _3  :1;
    uint32_t _4  :1;
    uint32_t _5  :1;
    uint32_t _6  :1;
    uint32_t _7  :1;
    uint32_t _8  :1;
  } bit;
  uint32_t as_uint32;
} genout_t;
static genout_t genout;
static CTL_MUTEX_t mtx_genout; /**< R-M-W, otherwise read .as_uint32 is atomic */

typedef union _gensens_raw_t {
    uint16_t raw;
    uint8_t  as_unint8[2];  /**< endian swap helper */
} gensens_raw_t;

typedef struct _gensens_t { /*! GenSens1 through GenSens10 raw values     */
  gensens_raw_t _1;         /*  consumer responsible for appropriate conv */
  gensens_raw_t _2;
  gensens_raw_t _3;
  gensens_raw_t _4;
  gensens_raw_t _5;
  gensens_raw_t _6;
  gensens_raw_t _7;
  gensens_raw_t _8;
  gensens_raw_t _9;
  gensens_raw_t _10;
} gensens_t;
static gensens_t gensens;
static CTL_MUTEX_t mtx_gensens;

/** -------------------------------------------------------------------------
 * @internal
 * @brief The STM32Cube HAL typically uses current time/tick value for busy-waiting
 *        be careful/cognizant of where this is called. May need to do a yield for a
 *        millisec here in order to not block up everything else.
 *
 */
uint32_t
  HAL_GetTick(void)
{
  assert(SysTick->CTRL & (SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk));
  static_assert(sizeof(uint32_t) >= sizeof(CTL_TIME_t));
  return (uint32_t)ctl_get_current_time();
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief on-board led control
 */
__STATIC_FORCEINLINE void
  led_off(void) {
  LL_GPIO_ResetOutputPin(GPIOC, LL_GPIO_PIN_6);
  __DSB();
}
__STATIC_FORCEINLINE void
  led_on(void) {
  LL_GPIO_SetOutputPin(GPIOC, LL_GPIO_PIN_6);
  __DSB();
}

__STATIC_FORCEINLINE void
  led_chirp(void)
{
  ctl_events_pulse(&e, EVT_LED_CHIRP);
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief init led i/o's
 */
static void
  ledio_init(void)
{
  ErrorStatus e = ERROR;
  LL_GPIO_InitTypeDef pin;
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  led_off();

  LL_GPIO_StructInit(&pin);
  pin.Mode       = LL_GPIO_MODE_OUTPUT;
  pin.Pin        = LL_GPIO_PIN_6;
  pin.Pull       = LL_GPIO_PULL_NO;
  pin.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  e = LL_GPIO_Init(GPIOC, &pin);
  assert(e == SUCCESS);
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief poll current button state
 */
__STATIC_FORCEINLINE unsigned
  user_button_active(void) {  /**< active HIGH */
  return LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_13);
}

__STATIC_FORCEINLINE unsigned
  boot0_button_active(void) { /**< active HIGH */
  return LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_8);
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief init button i/o's
 */
static void
  buttonio_init(void)
{
  ErrorStatus e = ERROR;
  LL_GPIO_InitTypeDef pin;

  /* PC.13 - USER BUTTON */
  LL_GPIO_StructInit(&pin);
  pin.Mode = LL_GPIO_MODE_INPUT;
  pin.Pin  = LL_GPIO_PIN_13;
  pin.Pull = LL_GPIO_PULL_DOWN;
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOC);
  e = LL_GPIO_Init(GPIOC, &pin);
  assert(e == SUCCESS);

  /* PB.8  - BOOT0 BUTTON */
  LL_GPIO_StructInit(&pin);
  pin.Mode = LL_GPIO_MODE_INPUT;
  pin.Pin  = LL_GPIO_PIN_8;
  pin.Pull = LL_GPIO_PULL_NO;
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
  e = LL_GPIO_Init(GPIOB, &pin);
  assert(e == SUCCESS);

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);

  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTB, LL_SYSCFG_EXTI_LINE8);
  NVIC_DisableIRQ(EXTI9_5_IRQn);
  NVIC_SetPriority(EXTI9_5_IRQn, IRQPRI_BUTTON);

  LL_SYSCFG_SetEXTISource(LL_SYSCFG_EXTI_PORTC, LL_SYSCFG_EXTI_LINE13);
  NVIC_DisableIRQ(EXTI15_10_IRQn);
  NVIC_SetPriority(EXTI15_10_IRQn, IRQPRI_BUTTON);

  LL_EXTI_InitTypeDef exti;
  LL_EXTI_StructInit(&exti);
  exti.Line_0_31   = LL_EXTI_LINE_8 | LL_EXTI_LINE_13;
  exti.LineCommand = ENABLE;
  exti.Mode        = LL_EXTI_MODE_IT;
  exti.Trigger     = LL_EXTI_TRIGGER_RISING_FALLING;
  e = LL_EXTI_Init(&exti);
  assert(e == SUCCESS);

  NVIC_EnableIRQ(EXTI15_10_IRQn);
  NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief init debug i/o
 *        PA.13 - SWDIO
 *        PA.14 - SWCLK
 */
static void
  swdio_init(void)
{
  /* enable clk to GPIOA to make it lockable later */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
}

/** -------------------------------------------------------------------------
 * @internal
 * @brief lock i/o's from further change
 */
static void
  lock_io(void)
{
  LL_GPIO_LockPin(GPIOA, LL_GPIO_PIN_13 | LL_GPIO_PIN_14);
  LL_GPIO_LockPin(GPIOB, LL_GPIO_PIN_8 | LL_GPIO_PIN_3 | LL_GPIO_PIN_4 |
                         LL_GPIO_PIN_5 | LL_GPIO_PIN_6 );
  LL_GPIO_LockPin(GPIOC, LL_GPIO_PIN_6 | LL_GPIO_PIN_13);
}


/** ---------------------------------------------------------------------------
 * @internal
 * @brief IRQ Handler for boot0 button
 */
void EXTI9_5_IRQHandler(void)
{
  ctl_enter_isr();
  if(LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_8)) {
    ctl_events_pulse(&e, boot0_button_active()?EVT_BOOT0_EDGE_BUTDOWN:EVT_BOOT0_EDGE_BUTUP);
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_8);
  }
  ctl_exit_isr();
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief IRQ Handler for user button
 */
void EXTI15_10_IRQHandler(void)
{
  ctl_enter_isr();
  if(LL_EXTI_ReadFlag_0_31(LL_EXTI_LINE_13)) {
    ctl_events_pulse(&e, user_button_active()?EVT_USER_EDGE_BUTDOWN:EVT_USER_EDGE_BUTUP);
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_13);
  }
  ctl_exit_isr();
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief configure and kickoff CTL timebase
 * @note  timerFn is unused since SysTick_Handler() brute forces the call to
 *        ctl_increment_tick_from_isr() which uses the ctl_time_increment
 *        determined here during configuration.
 */
void ctl_start_timer(CTL_ISR_FN_t timerFn)
{
  assert(timerFn == ctl_increment_tick_from_isr);
  assert(SystemCoreClock == 170000000);
  enum { ONE_MS = 170000000/8/1000 };             /**< SystemCoreClock/SYSTICKDIVIDER/Hz   */
  SysTick->LOAD = ONE_MS-1;
  ctl_time_increment = (SysTick->LOAD+1)/ONE_MS;  /**< ctl_increment_tick_from_isr() param */
//ctl_timeslice_period = ctl_time_increment;      /**< round robining of tasks w/same pri  */
  SysTick->VAL  = 0;
  SysTick->CTRL = SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
  NVIC_SetPriority(PendSV_IRQn, IRQPRI_PENDSV);   /**< must be lowest CTL aware priority   */
  NVIC_SetPriority(SysTick_IRQn, IRQPRI_SYSTICK); /**< must be highest CTL aware priority  */
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief IRQ Handler
 */
void SysTick_Handler(void)
{
  ctl_enter_isr();
  assert(SystemCoreClock == 170000000);
  enum { ONE_MS = 170000000/8/1000 };  /**< SystemCoreClock/SYSTICKDIVIDER/Hz */
  assert(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk);
  assert(ctl_time_increment == (SysTick->LOAD+1)/ONE_MS);
  ctl_increment_tick_from_isr();
  ctl_exit_isr();
}

/** -------------------------------------------------------------------------
 * @internal
 * @brief uh oh
 */
void ctl_handle_error(CTL_ERROR_CODE_t e)
{
  switch(e) {
    case CTL_ERROR_NO_TASKS_TO_RUN:
      debug_puts("CTL_ERROR_NO_TASKS_TO_RUN"); break;
    case CTL_UNSUPPORTED_CALL_FROM_ISR:
      debug_puts("CTL_UNSUPPORTED_CALL_FROM_ISR"); break;
    case CTL_MUTEX_UNLOCK_CALL_ERROR:
      debug_puts("CTL_MUTEX_UNLOCK_CALL_ERROR"); break;
    case CTL_UNSPECIFIED_ERROR:
      debug_puts("CTL_UNSPECIFIED_ERROR"); break;
    case CTL_STACK_OVERFLOW:
      debug_puts("CTL_STACK_OVERFLOW"); break;
    default:
      debug_puts("UNKNOWN CTL_ERROR_CODE_t");
  };
  debug_break();
}

/** -------------------------------------------------------------------------
 * @internal
 * @brief grab raw values for GenSens1 through GenSens4 from ecu
 *
 */
static void
  decode_3E7(canframe_t * const pframe)
{
  assert(pframe);
  assert(pframe->rxhdr.DataLength >= 8);

  enum { BASE_MS = 50 };   /**< 20Hz base emit rate */
  if(ctl_mutex_lock(&mtx_gensens, CTL_TIMEOUT_DELAY, BASE_MS/2)) {
    gensens_raw_t update;
    CTL_EVENT_SET_t pulse = EVT_NONE; /**< foreach updated value, pulse associated event */

    update.as_unint8[1] = pframe->data.as_uint8[0];   /**< GenSens1 */
    update.as_unint8[0] = pframe->data.as_uint8[1];
    pulse |= (update.raw != gensens._1.raw)?EVT_ECU_GENSENS1:EVT_NONE;
    gensens._1.raw = update.raw;

    update.as_unint8[1] = pframe->data.as_uint8[2];   /**< GenSens2 */
    update.as_unint8[0] = pframe->data.as_uint8[3];
    pulse |= (update.raw != gensens._2.raw)?EVT_ECU_GENSENS2:EVT_NONE;
    gensens._2.raw = update.raw;

    update.as_unint8[1] = pframe->data.as_uint8[4];   /**< GenSens3 */
    update.as_unint8[0] = pframe->data.as_uint8[5];
    pulse |= (update.raw != gensens._3.raw)?EVT_ECU_GENSENS3:EVT_NONE;
    gensens._3.raw = update.raw;

    update.as_unint8[1] = pframe->data.as_uint8[6];   /**< GenSens4 */
    update.as_unint8[0] = pframe->data.as_uint8[7];
    pulse |= (update.raw != gensens._4.raw)?EVT_ECU_GENSENS4:EVT_NONE;
    gensens._4.raw = update.raw;

    ctl_events_pulse(&e_ecu_upd0, pulse);
    ctl_mutex_unlock(&mtx_gensens);
  } else { ctl_message_queue_post_nb(&codes, (void*)CODE_GENSENS_LOCK_FAIL); }

}

/** -------------------------------------------------------------------------
 * @internal
 * @brief grab raw values for GenSens5 through GenSens8 from ecu
 *
 */
static void
  decode_3E8(canframe_t * const pframe)
{
  assert(pframe);
  assert(pframe->rxhdr.DataLength >= 8);

  enum { BASE_MS = 50 };   /**< 20Hz base emit rate */
  if(ctl_mutex_lock(&mtx_gensens, CTL_TIMEOUT_DELAY, BASE_MS/2)) {
    gensens_raw_t update;
    CTL_EVENT_SET_t pulse = EVT_NONE; /**< foreach updated value, pulse associated event */

    update.as_unint8[1] = pframe->data.as_uint8[0];   /**< GenSens5 */
    update.as_unint8[0] = pframe->data.as_uint8[1];
    pulse |= (update.raw != gensens._5.raw)?EVT_ECU_GENSENS5:EVT_NONE;
    gensens._5.raw = update.raw;

    update.as_unint8[1] = pframe->data.as_uint8[2];   /**< GenSens6 */
    update.as_unint8[0] = pframe->data.as_uint8[3];
    pulse |= (update.raw != gensens._6.raw)?EVT_ECU_GENSENS6:EVT_NONE;
    gensens._6.raw = update.raw;

    update.as_unint8[1] = pframe->data.as_uint8[4];   /**< GenSens7 */
    update.as_unint8[0] = pframe->data.as_uint8[5];
    pulse |= (update.raw != gensens._7.raw)?EVT_ECU_GENSENS7:EVT_NONE;
    gensens._7.raw = update.raw;

    update.as_unint8[1] = pframe->data.as_uint8[6];   /**< GenSens8 */
    update.as_unint8[0] = pframe->data.as_uint8[7];
    pulse |= (update.raw != gensens._8.raw)?EVT_ECU_GENSENS8:EVT_NONE;
    gensens._8.raw = update.raw;

    ctl_events_pulse(&e_ecu_upd0, pulse);
    ctl_mutex_unlock(&mtx_gensens);
  } else { ctl_message_queue_post_nb(&codes, (void*)CODE_GENSENS_LOCK_FAIL); }

}

/** -------------------------------------------------------------------------
 * @internal
 * @brief grab raw values for GenSens9/10 from ecu,
 *        ignore rest until desired in the future
 *
 */
static void
  decode_3E9(canframe_t * const pframe)
{
  assert(pframe);
  assert(pframe->rxhdr.DataLength >= 4);

  enum { BASE_MS = 50 };   /**< 20Hz base emit rate */
  if(ctl_mutex_lock(&mtx_gensens, CTL_TIMEOUT_DELAY, BASE_MS/2)) {
    gensens_raw_t update;
    CTL_EVENT_SET_t pulse = EVT_NONE; /**< foreach updated value, pulse associated event */

    update.as_unint8[1] = pframe->data.as_uint8[0];   /**< GenSens9 */
    update.as_unint8[0] = pframe->data.as_uint8[1];
    pulse |= (update.raw != gensens._9.raw)?EVT_ECU_GENSENS9:EVT_NONE;
    gensens._9.raw = update.raw;

    update.as_unint8[1] = pframe->data.as_uint8[2];   /**< GenSens10 */
    update.as_unint8[0] = pframe->data.as_uint8[3];
    pulse |= (update.raw != gensens._10.raw)?EVT_ECU_GENSENS10:EVT_NONE;
    gensens._10.raw = update.raw;

    ctl_events_pulse(&e_ecu_upd0, pulse);
    ctl_mutex_unlock(&mtx_gensens);
  } else { ctl_message_queue_post_nb(&codes, (void*)CODE_GENSENS_LOCK_FAIL); }
}

/** -------------------------------------------------------------------------
 * @internal
 * @brief grab bit states for GenOut1 through GenOut20 from ecu,
 *        ignore rest until desired in the future
 *
 */
static void
  decode_6F7(canframe_t * const pframe)
{
  assert(pframe);
  assert(pframe->rxhdr.DataLength >= 4);

  enum { BASE_MS = 100 };   /**< 10Hz base emit rate */
  genout_t const update = { .as_uint32 = pframe->data.as_uint32[0] };

  if(ctl_mutex_lock(&mtx_genout, CTL_TIMEOUT_DELAY, BASE_MS/2)) {
    CTL_EVENT_SET_t pulse = EVT_NONE; /**< foreach updated bit, pulse associated event */
    pulse |= (update.bit._1  != genout.bit._1 )?EVT_ECU_GENOUT1: EVT_NONE;
    pulse |= (update.bit._2  != genout.bit._2 )?EVT_ECU_GENOUT2: EVT_NONE;
    pulse |= (update.bit._3  != genout.bit._3 )?EVT_ECU_GENOUT3: EVT_NONE;
    pulse |= (update.bit._4  != genout.bit._4 )?EVT_ECU_GENOUT4: EVT_NONE;
    pulse |= (update.bit._5  != genout.bit._5 )?EVT_ECU_GENOUT5: EVT_NONE;
    pulse |= (update.bit._6  != genout.bit._6 )?EVT_ECU_GENOUT6: EVT_NONE;
    pulse |= (update.bit._7  != genout.bit._7 )?EVT_ECU_GENOUT7: EVT_NONE;
    pulse |= (update.bit._8  != genout.bit._8 )?EVT_ECU_GENOUT8: EVT_NONE;
    pulse |= (update.bit._9  != genout.bit._9 )?EVT_ECU_GENOUT9: EVT_NONE;
    pulse |= (update.bit._10 != genout.bit._10)?EVT_ECU_GENOUT10:EVT_NONE;
    pulse |= (update.bit._11 != genout.bit._11)?EVT_ECU_GENOUT11:EVT_NONE;
    pulse |= (update.bit._12 != genout.bit._12)?EVT_ECU_GENOUT12:EVT_NONE;
    pulse |= (update.bit._13 != genout.bit._13)?EVT_ECU_GENOUT13:EVT_NONE;
    pulse |= (update.bit._14 != genout.bit._14)?EVT_ECU_GENOUT14:EVT_NONE;
    pulse |= (update.bit._15 != genout.bit._15)?EVT_ECU_GENOUT15:EVT_NONE;
    pulse |= (update.bit._16 != genout.bit._16)?EVT_ECU_GENOUT16:EVT_NONE;
    pulse |= (update.bit._17 != genout.bit._17)?EVT_ECU_GENOUT17:EVT_NONE;
    pulse |= (update.bit._18 != genout.bit._18)?EVT_ECU_GENOUT18:EVT_NONE;
    pulse |= (update.bit._19 != genout.bit._19)?EVT_ECU_GENOUT19:EVT_NONE;
    pulse |= (update.bit._20 != genout.bit._20)?EVT_ECU_GENOUT20:EVT_NONE;
    genout.as_uint32 = update.as_uint32;  /**< update internal state */
    ctl_events_pulse(&e_ecu_upd0, pulse);
    ctl_mutex_unlock(&mtx_genout);
  } else { ctl_message_queue_post_nb(&codes, (void*)CODE_GENOUT_LOCK_FAIL); }

}

/** -------------------------------------------------------------------------
 * @internal
 * @brief decode the internal R5 PD16 data frame(s)
 *        (same as the BASE+0 frame of PD16A/B/C/D)
 *
 */
static void
  decode_700(canframe_t * const pframe)
{
  assert(pframe);

  enum {
    HCO25_01 = (0<<5)|(0<<0),   /**< 5Hz, 10ms spacing (4x) */
    HCO25_02 = (0<<5)|(1<<0),
    HCO25_03 = (0<<5)|(2<<0),
    HCO25_04 = (0<<5)|(3<<0),

    HCO8_01  = (1<<5)|(0<<0),   /**< 5Hz, 10ms spacing (12x) */
    HCO8_02  = (1<<5)|(1<<0),
    HCO8_03  = (1<<5)|(2<<0),
    HCO8_04  = (1<<5)|(3<<0),
    HCO8_05  = (1<<5)|(4<<0),
    HCO8_06  = (1<<5)|(5<<0),
    HCO8_07  = (1<<5)|(6<<0),
    HCO8_08  = (1<<5)|(7<<0),
    HCO8_09  = (1<<5)|(8<<0),
    HCO8_10  = (1<<5)|(9<<0),
    HCO8_11  = (1<<5)|(10<<0),
    HCO8_12  = (1<<5)|(11<<0),

    HBO_01   = (2<<5)|(0<<0),   /**< 5Hz, 10ms spacing (4x) */
    HBO_02   = (2<<5)|(1<<0),
    HBO_03   = (2<<5)|(2<<0),
    HBO_04   = (2<<5)|(3<<0),
  };                            /* = 20 frames * 5 -> 100 frames/sec */

  switch(pframe->data.as_uint8[0]) {
    case HCO25_01:
    case HCO25_02:
    case HCO25_03:
    case HCO25_04:
    case HCO8_01:
    case HCO8_02:
    case HCO8_03:
    case HCO8_04:
    case HCO8_05:
    case HCO8_06:
    case HCO8_07:
    case HCO8_08:
    case HCO8_09:
    case HCO8_10:
    case HCO8_11:
    case HCO8_12:
    case HBO_01:
    case HBO_02:
    case HBO_03:
    case HBO_04:
      break;
    default:
      (void)ctl_message_queue_post_nb(&codes, (void*)CODE_UNKMUXID);
      return;
  }
}

/** -------------------------------------------------------------------------
 * @internal
 * @brief route/dump rx'd messages from the haltech bus
 *
 */
static void
  filter(canframe_t * const pframe)
{
  assert(pframe);
  assert(pframe->rxhdr.IdType != FDCAN_EXTENDED_ID);
  assert(pframe->rxhdr.FDFormat == FDCAN_FRAME_CLASSIC);
  switch(pframe->rxhdr.Identifier) {
    case HTID_ECU_3E7:
      decode_3E7(pframe);
      break;
    case HTID_ECU_3E8:
      decode_3E8(pframe);
      break;
    case HTID_ECU_3E9:
      decode_3E9(pframe);
      break;
    case HTID_ECU_6F7:
      decode_6F7(pframe);
      break;
    case HTID_PD16R5_700:
      decode_700(pframe);
      break;
    default:
      /* ctl_message_queue_post_nb(&codes, (void*)CODE_UNKNOWN_ECUFRAME); */
  }
}

/** -------------------------------------------------------------------------
 * @internal
 * @brief sine table lookup
 *        0-999 -> 0-100000
 *        easy test data needed?:  lookup(ctl_get_current_time()%1000);
 */
static unsigned const
  lookup(unsigned const idx)
{
  static const unsigned __ALIGNED(sizeof(unsigned)) __attribute__((section(".rodata")))
    table[1000] = {
      0x0c350, 0x0c48a, 0x0c5c4, 0x0c6fe, 0x0c839, 0x0c973, 0x0caad, 0x0cbe6, 0x0cd20, 0x0ce5a,
      0x0cf94, 0x0d0cd, 0x0d206, 0x0d340, 0x0d479, 0x0d5b1, 0x0d6ea, 0x0d823, 0x0d95b, 0x0da93,
      0x0dbcb, 0x0dd02, 0x0de3a, 0x0df71, 0x0e0a7, 0x0e1de, 0x0e314, 0x0e44a, 0x0e57f, 0x0e6b4,
      0x0e7e9, 0x0e91d, 0x0ea51, 0x0eb85, 0x0ecb8, 0x0edeb, 0x0ef1e, 0x0f04f, 0x0f181, 0x0f2b2,
      0x0f3e2, 0x0f513, 0x0f642, 0x0f771, 0x0f8a0, 0x0f9ce, 0x0fafb, 0x0fc28, 0x0fd54, 0x0fe80,
      0xffab,  0x100d5, 0x101ff, 0x10328, 0x10451, 0x10579, 0x106a0, 0x107c7, 0x108ed, 0x10a12,
      0x10b36, 0x10c5a, 0x10d7d, 0x10e9f, 0x10fc1, 0x110e1, 0x11201, 0x11320, 0x1143f, 0x1155c,
      0x11679, 0x11795, 0x118b0, 0x119ca, 0x11ae3, 0x11bfc, 0x11d13, 0x11e2a, 0x11f3f, 0x12054,
      0x12168, 0x1227b, 0x1238c, 0x1249d, 0x125ad, 0x126bc, 0x127ca, 0x128d7, 0x129e3, 0x12aee,
      0x12bf7, 0x12d00, 0x12e08, 0x12f0e, 0x13014, 0x13118, 0x1321b, 0x1331e, 0x1341f, 0x1351f,
      0x1361d, 0x1371b, 0x13817, 0x13912, 0x13a0d, 0x13b05, 0x13bfd, 0x13cf3, 0x13de9, 0x13edd,
      0x13fcf, 0x140c1, 0x141b1, 0x142a0, 0x1438d, 0x1447a, 0x14565, 0x1464e, 0x14737, 0x1481e,
      0x14903, 0x149e8, 0x14acb, 0x14bac, 0x14c8c, 0x14d6b, 0x14e49, 0x14f25, 0x14fff, 0x150d9,
      0x151b0, 0x15287, 0x1535c, 0x1542f, 0x15501, 0x155d2, 0x156a1, 0x1576e, 0x1583a, 0x15905,
      0x159ce, 0x15a95, 0x15b5b, 0x15c20, 0x15ce2, 0x15da4, 0x15e64, 0x15f22, 0x15fde, 0x16099,
      0x16153, 0x1620b, 0x162c1, 0x16376, 0x16429, 0x164da, 0x1658a, 0x16638, 0x166e4, 0x1678f,
      0x16838, 0x168e0, 0x16986, 0x16a2a, 0x16acc, 0x16b6d, 0x16c0c, 0x16caa, 0x16d45, 0x16ddf,
      0x16e77, 0x16f0e, 0x16fa3, 0x17036, 0x170c7, 0x17156, 0x171e4, 0x17270, 0x172fa, 0x17383,
      0x17409, 0x1748e, 0x17511, 0x17593, 0x17612, 0x17690, 0x1770c, 0x17786, 0x177fe, 0x17874,
      0x178e9, 0x1795c, 0x179cc, 0x17a3b, 0x17aa9, 0x17b14, 0x17b7e, 0x17be5, 0x17c4b, 0x17caf,
      0x17d11, 0x17d71, 0x17dcf, 0x17e2c, 0x17e86, 0x17edf, 0x17f35, 0x17f8a, 0x17fdd, 0x1802e,
      0x1807d, 0x180ca, 0x18116, 0x1815f, 0x181a6, 0x181ec, 0x1822f, 0x18271, 0x182b1, 0x182ef,
      0x1832a, 0x18364, 0x1839c, 0x183d2, 0x18406, 0x18438, 0x18469, 0x18497, 0x184c3, 0x184ed,
      0x18516, 0x1853c, 0x18561, 0x18583, 0x185a4, 0x185c2, 0x185df, 0x185f9, 0x18612, 0x18629,
      0x1863d, 0x18650, 0x18661, 0x18670, 0x1867c, 0x18687, 0x18690, 0x18697, 0x1869c, 0x1869f,
      0x186a0, 0x1869f, 0x1869c, 0x18697, 0x18690, 0x18687, 0x1867c, 0x18670, 0x18661, 0x18650,
      0x1863d, 0x18629, 0x18612, 0x185f9, 0x185df, 0x185c2, 0x185a4, 0x18583, 0x18561, 0x1853c,
      0x18516, 0x184ed, 0x184c3, 0x18497, 0x18469, 0x18438, 0x18406, 0x183d2, 0x1839c, 0x18364,
      0x1832a, 0x182ef, 0x182b1, 0x18271, 0x1822f, 0x181ec, 0x181a6, 0x1815f, 0x18116, 0x180ca,
      0x1807d, 0x1802e, 0x17fdd, 0x17f8a, 0x17f35, 0x17edf, 0x17e86, 0x17e2c, 0x17dcf, 0x17d71,
      0x17d11, 0x17caf, 0x17c4b, 0x17be5, 0x17b7e, 0x17b14, 0x17aa9, 0x17a3b, 0x179cc, 0x1795c,
      0x178e9, 0x17874, 0x177fe, 0x17786, 0x1770c, 0x17690, 0x17612, 0x17593, 0x17511, 0x1748e,
      0x17409, 0x17383, 0x172fa, 0x17270, 0x171e4, 0x17156, 0x170c7, 0x17036, 0x16fa3, 0x16f0e,
      0x16e77, 0x16ddf, 0x16d45, 0x16caa, 0x16c0c, 0x16b6d, 0x16acc, 0x16a2a, 0x16986, 0x168e0,
      0x16838, 0x1678f, 0x166e4, 0x16638, 0x1658a, 0x164da, 0x16429, 0x16376, 0x162c1, 0x1620b,
      0x16153, 0x16099, 0x15fde, 0x15f22, 0x15e64, 0x15da4, 0x15ce2, 0x15c20, 0x15b5b, 0x15a95,
      0x159ce, 0x15905, 0x1583a, 0x1576e, 0x156a1, 0x155d2, 0x15501, 0x1542f, 0x1535c, 0x15287,
      0x151b0, 0x150d9, 0x14fff, 0x14f25, 0x14e49, 0x14d6b, 0x14c8c, 0x14bac, 0x14acb, 0x149e8,
      0x14903, 0x1481e, 0x14737, 0x1464e, 0x14565, 0x1447a, 0x1438d, 0x142a0, 0x141b1, 0x140c1,
      0x13fcf, 0x13edd, 0x13de9, 0x13cf3, 0x13bfd, 0x13b05, 0x13a0d, 0x13912, 0x13817, 0x1371b,
      0x1361d, 0x1351f, 0x1341f, 0x1331e, 0x1321b, 0x13118, 0x13014, 0x12f0e, 0x12e08, 0x12d00,
      0x12bf7, 0x12aee, 0x129e3, 0x128d7, 0x127ca, 0x126bc, 0x125ad, 0x1249d, 0x1238c, 0x1227b,
      0x12168, 0x12054, 0x11f3f, 0x11e2a, 0x11d13, 0x11bfc, 0x11ae3, 0x119ca, 0x118b0, 0x11795,
      0x11679, 0x1155c, 0x1143f, 0x11320, 0x11201, 0x110e1, 0x10fc1, 0x10e9f, 0x10d7d, 0x10c5a,
      0x10b36, 0x10a12, 0x108ed, 0x107c7, 0x106a0, 0x10579, 0x10451, 0x10328, 0x101ff, 0x100d5,
      0x0ffab, 0x0fe80, 0x0fd54, 0x0fc28, 0x0fafb, 0x0f9ce, 0x0f8a0, 0x0f771, 0x0f642, 0x0f513,
      0x0f3e2, 0x0f2b2, 0x0f181, 0x0f04f, 0x0ef1e, 0x0edeb, 0x0ecb8, 0x0eb85, 0x0ea51, 0x0e91d,
      0x0e7e9, 0x0e6b4, 0x0e57f, 0x0e44a, 0x0e314, 0x0e1de, 0x0e0a7, 0x0df71, 0x0de3a, 0x0dd02,
      0x0dbcb, 0x0da93, 0x0d95b, 0x0d823, 0x0d6ea, 0x0d5b1, 0x0d479, 0x0d340, 0x0d206, 0x0d0cd,
      0x0cf94, 0x0ce5a, 0x0cd20, 0x0cbe6, 0x0caad, 0x0c973, 0x0c839, 0x0c6fe, 0x0c5c4, 0x0c48a,
      0x0c350, 0x0c216, 0x0c0dc, 0x0bfa2, 0x0be67, 0x0bd2d, 0x0bbf3, 0x0baba, 0x0b980, 0x0b846,
      0x0b70c, 0x0b5d3, 0x0b49a, 0x0b360, 0x0b227, 0x0b0ef, 0x0afb6, 0x0ae7d, 0x0ad45, 0x0ac0d,
      0x0aad5, 0x0a99e, 0x0a866, 0x0a72f, 0x0a5f9, 0x0a4c2, 0x0a38c, 0x0a256, 0x0a121, 0x09fec,
      0x09eb7, 0x09d83, 0x09c4f, 0x09b1b, 0x099e8, 0x098b5, 0x09782, 0x09651, 0x0951f, 0x093ee,
      0x092be, 0x0918d, 0x0905e, 0x08f2f, 0x08e00, 0x08cd2, 0x08ba5, 0x08a78, 0x0894c, 0x08820,
      0x086f5, 0x085cb, 0x084a1, 0x08378, 0x0824f, 0x08127, 0x08000, 0x07ed9, 0x07db3, 0x07c8e,
      0x07b6a, 0x07a46, 0x07923, 0x07801, 0x076df, 0x075bf, 0x0749f, 0x07380, 0x07261, 0x07144,
      0x07027, 0x06f0b, 0x06df0, 0x06cd6, 0x06bbd, 0x06aa4, 0x0698d, 0x06876, 0x06761, 0x0664c,
      0x06538, 0x06425, 0x06314, 0x06203, 0x060f3, 0x05fe4, 0x05ed6, 0x05dc9, 0x05cbd, 0x05bb2,
      0x05aa9, 0x059a0, 0x05898, 0x05792, 0x0568c, 0x05588, 0x05485, 0x05382, 0x05281, 0x05181,
      0x05083, 0x04f85, 0x04e89, 0x04d8e, 0x04c93, 0x04b9b, 0x04aa3, 0x049ad, 0x048b7, 0x047c3,
      0x046d1, 0x045df, 0x044ef, 0x04400, 0x04313, 0x04226, 0x0413b, 0x04052, 0x03f69, 0x03e82,
      0x03d9d, 0x03cb8, 0x03bd5, 0x03af4, 0x03a14, 0x03935, 0x03857, 0x0377b, 0x036a1, 0x035c7,
      0x034f0, 0x03419, 0x03344, 0x03271, 0x0319f, 0x030ce, 0x02fff, 0x02f32, 0x02e66, 0x02d9b,
      0x02cd2, 0x02c0b, 0x02b45, 0x02a80, 0x029be, 0x028fc, 0x0283c, 0x0277e, 0x026c2, 0x02607,
      0x0254d, 0x02495, 0x023df, 0x0232a, 0x02277, 0x021c6, 0x02116, 0x02068, 0x01fbc, 0x01f11,
      0x01e68, 0x01dc0, 0x01d1a, 0x01c76, 0x01bd4, 0x01b33, 0x01a94, 0x019f6, 0x0195b, 0x018c1,
      0x01829, 0x01792, 0x016fd, 0x0166a, 0x015d9, 0x0154a, 0x014bc, 0x01430, 0x013a6, 0x0131d,
      0x01297, 0x01212, 0x0118f, 0x0110d, 0x0108e, 0x01010, 0x00f94, 0x00f1a, 0x00ea2, 0x00e2c,
      0x00db7, 0x00d44, 0x00cd4, 0x00c65, 0x00bf7, 0x00b8c, 0x00b22, 0x00abb, 0x00a55, 0x009f1,
      0x0098f, 0x0092f, 0x008d1, 0x00874, 0x0081a, 0x007c1, 0x0076b, 0x00716, 0x006c3, 0x00672,
      0x00623, 0x005d6, 0x0058a, 0x00541, 0x004fa, 0x004b4, 0x00471, 0x0042f, 0x003ef, 0x003b1,
      0x00376, 0x0033c, 0x00304, 0x002ce, 0x0029a, 0x00268, 0x00237, 0x00209, 0x001dd, 0x001b3,
      0x0018a, 0x00164, 0x0013f, 0x0011d, 0x000fc, 0x000de, 0x000c1, 0x000a7, 0x0008e, 0x00077,
      0x00063, 0x00050, 0x0003f, 0x00030, 0x00024, 0x00019, 0x00010, 0x00009, 0x00004, 0x00001,
      0x00000, 0x00001, 0x00004, 0x00009, 0x00010, 0x00019, 0x00024, 0x00030, 0x0003f, 0x00050,
      0x00063, 0x00077, 0x0008e, 0x000a7, 0x000c1, 0x000de, 0x000fc, 0x0011d, 0x0013f, 0x00164,
      0x0018a, 0x001b3, 0x001dd, 0x00209, 0x00237, 0x00268, 0x0029a, 0x002ce, 0x00304, 0x0033c,
      0x00376, 0x003b1, 0x003ef, 0x0042f, 0x00471, 0x004b4, 0x004fa, 0x00541, 0x0058a, 0x005d6,
      0x00623, 0x00672, 0x006c3, 0x00716, 0x0076b, 0x007c1, 0x0081a, 0x00874, 0x008d1, 0x0092f,
      0x0098f, 0x009f1, 0x00a55, 0x00abb, 0x00b22, 0x00b8c, 0x00bf7, 0x00c65, 0x00cd4, 0x00d44,
      0x00db7, 0x00e2c, 0x00ea2, 0x00f1a, 0x00f94, 0x01010, 0x0108e, 0x0110d, 0x0118f, 0x01212,
      0x01297, 0x0131d, 0x013a6, 0x01430, 0x014bc, 0x0154a, 0x015d9, 0x0166a, 0x016fd, 0x01792,
      0x01829, 0x018c1, 0x0195b, 0x019f6, 0x01a94, 0x01b33, 0x01bd4, 0x01c76, 0x01d1a, 0x01dc0,
      0x01e68, 0x01f11, 0x01fbc, 0x02068, 0x02116, 0x021c6, 0x02277, 0x0232a, 0x023df, 0x02495,
      0x0254d, 0x02607, 0x026c2, 0x0277e, 0x0283c, 0x028fc, 0x029be, 0x02a80, 0x02b45, 0x02c0b,
      0x02cd2, 0x02d9b, 0x02e66, 0x02f32, 0x02fff, 0x030ce, 0x0319f, 0x03271, 0x03344, 0x03419,
      0x034f0, 0x035c7, 0x036a1, 0x0377b, 0x03857, 0x03935, 0x03a14, 0x03af4, 0x03bd5, 0x03cb8,
      0x03d9d, 0x03e82, 0x03f69, 0x04052, 0x0413b, 0x04226, 0x04313, 0x04400, 0x044ef, 0x045df,
      0x046d1, 0x047c3, 0x048b7, 0x049ad, 0x04aa3, 0x04b9b, 0x04c93, 0x04d8e, 0x04e89, 0x04f85,
      0x05083, 0x05181, 0x05281, 0x05382, 0x05485, 0x05588, 0x0568c, 0x05792, 0x05898, 0x059a0,
      0x05aa9, 0x05bb2, 0x05cbd, 0x05dc9, 0x05ed6, 0x05fe4, 0x060f3, 0x06203, 0x06314, 0x06425,
      0x06538, 0x0664c, 0x06761, 0x06876, 0x0698d, 0x06aa4, 0x06bbd, 0x06cd6, 0x06df0, 0x06f0b,
      0x07027, 0x07144, 0x07261, 0x07380, 0x0749f, 0x075bf, 0x076df, 0x07801, 0x07923, 0x07a46,
      0x07b6a, 0x07c8e, 0x07db3, 0x07ed9, 0x08000, 0x08127, 0x0824f, 0x08378, 0x084a1, 0x085cb,
      0x086f5, 0x08820, 0x0894c, 0x08a78, 0x08ba5, 0x08cd2, 0x08e00, 0x08f2f, 0x0905e, 0x0918d,
      0x092be, 0x093ee, 0x0951f, 0x09651, 0x09782, 0x098b5, 0x099e8, 0x09b1b, 0x09c4f, 0x09d83,
      0x09eb7, 0x09fec, 0x0a121, 0x0a256, 0x0a38c, 0x0a4c2, 0x0a5f9, 0x0a72f, 0x0a866, 0x0a99e,
      0x0aad5, 0x0ac0d, 0x0ad45, 0x0ae7d, 0x0afb6, 0x0b0ef, 0x0b227, 0x0b360, 0x0b49a, 0x0b5d3,
      0x0b70c, 0x0b846, 0x0b980, 0x0baba, 0x0bbf3, 0x0bd2d, 0x0be67, 0x0bfa2, 0x0c0dc, 0x0c216
    };

  assert(idx < (sizeof(table)/sizeof(table[0])));
  return table[idx];
}

/** -------------------------------------------------------------------------
 * @public
 * @brief entry point
 */

volatile unsigned sine;

__NO_RETURN int
  main(void)
{
  ctl_start_timer(ctl_increment_tick_from_isr);           /**< HAL depends on ticks */
  static CTL_TASK_t self;
  ctl_task_init(&self, TASKPRI_HIGHEST, "idle");

  swdio_init();
  ledio_init();
  buttonio_init();

  LL_APB1_GRP1_ForceReset(LL_APB1_GRP1_PERIPH_FDCAN);     /**  base FDCAN config    */
  LL_RCC_SetFDCANClockSource(LL_RCC_FDCAN_CLKSOURCE_PLL);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_FDCAN);
  LL_APB1_GRP1_ReleaseReset(LL_APB1_GRP1_PERIPH_FDCAN);
  FDCAN_CONFIG->CKDIV = 0;                                /**< PDIV := 0000b (/1)   */
                                                          /*   170MHz/PDIV = 170MHz */

  /* init all global events/mutexes/mem/queues/etc before kicking off tasks */
  ctl_events_init(&e,          EVT_NONE);
  ctl_events_init(&e_ioa,      EVT_NONE);
  ctl_events_init(&e_iob,      EVT_NONE);
  ctl_events_init(&e_pd16a,    EVT_NONE);
  ctl_events_init(&e_ecu_upd0, EVT_NONE);

  ctl_mutex_init(&mtx_genout);
  ctl_mutex_init(&mtx_gensens);

  ctl_memory_area_init(&frames, frame_mem, FRAME_WORDSIZE, FRAME_COUNT);
  ctl_message_queue_init(&txq, txq_mem, TXQ_COUNT);
  ctl_message_queue_init(&rxq, rxq_mem, RXQ_COUNT);

  static void *codes_mem[64];
  ctl_message_queue_init(&codes, codes_mem, 64);

  { /* boot0 button debounce */
    static debounce_info_t di = {
      .pe = &e,
      .to = 15,
      .rising  = EVT_BOOT0_EDGE_BUTUP,
      .falling = EVT_BOOT0_EDGE_BUTDOWN,
      .up   = EVT_BOOT0_BUTUP,
      .down = EVT_BOOT0_BUTDOWN
    };
    static CTL_TASK_t task;
    static unsigned stack[CTL_CPU_STATE_WORD_SIZE+4];
    ctl_task_run(&task, TASKPRI_BUTLED, &task_but_debounce, &di, "but_boot0_debounce", sizeof(stack)/sizeof(unsigned), stack, 0);
  }
  { /* user button debounce */
    static debounce_info_t di = {
      .pe = &e,
      .to = 15,
      .rising  = EVT_USER_EDGE_BUTUP,
      .falling = EVT_USER_EDGE_BUTDOWN,
      .up   = EVT_USER_BUTUP,
      .down = EVT_USER_BUTDOWN
    };
    static CTL_TASK_t task;
    static unsigned stack[CTL_CPU_STATE_WORD_SIZE+4];
    ctl_task_run(&task, TASKPRI_BUTLED, &task_but_debounce, &di, "but_user_debounce", sizeof(stack)/sizeof(unsigned), stack, 0);
  }
  { /* LED control */
    static CTL_TASK_t task;
    static unsigned stack[CTL_CPU_STATE_WORD_SIZE+4];
    ctl_task_run(&task, TASKPRI_BUTLED, &task_led_chirp, (void*)100, "led_chirp", sizeof(stack)/sizeof(unsigned), stack, 0);
  }
  { /* htbus management */
    static CTL_TASK_t task;
    static unsigned stack[CTL_CPU_STATE_WORD_SIZE+4];
    ctl_task_run(&task, TASKPRI_HTBUS, &task_htbus, (void*)1000, "htbus", sizeof(stack)/sizeof(unsigned), stack, 0);
  }
  { /* IOE status generation */
    static CTL_TASK_t task;
    static unsigned stack[CTL_CPU_STATE_WORD_SIZE+4];
    ctl_task_run(&task, TASKPRI_STATUS, &task_ioe_status_out, 0, "ioe_status_out", sizeof(stack)/sizeof(unsigned), stack, 0);
  }
  { /* IOE-A AVI1/2/3/4 message out */
    static CTL_TASK_t task;
    static unsigned stack[CTL_CPU_STATE_WORD_SIZE+32];
    ctl_task_run(&task, TASKPRI_IOEOUT, &task_ioa_avi14_out, 0, "ioa_avi14_out", sizeof(stack)/sizeof(unsigned), stack, 0);
  }
  { /* IOE-A DPI1/2 message out */
    static CTL_TASK_t task;
    static unsigned stack[CTL_CPU_STATE_WORD_SIZE+32];
    ctl_task_run(&task, TASKPRI_IOEOUT, &task_ioa_dpi12_out, 0, "ioa_dpi12_out", sizeof(stack)/sizeof(unsigned), stack, 0);
  }
  { /* IOE-A DPI3/4 message out */
    static CTL_TASK_t task;
    static unsigned stack[CTL_CPU_STATE_WORD_SIZE+32];
    ctl_task_run(&task, TASKPRI_IOEOUT, &task_ioa_dpi34_out, 0, "ioa_dpi34_out", sizeof(stack)/sizeof(unsigned), stack, 0);
  }
  { /* IOE-B AVI1/2/3/4 message out */
    static CTL_TASK_t task;
    static unsigned stack[CTL_CPU_STATE_WORD_SIZE+32];
    ctl_task_run(&task, TASKPRI_IOEOUT, &task_iob_avi14_out, 0, "iob_avi14_out", sizeof(stack)/sizeof(unsigned), stack, 0);
  }
  { /* IOE-B DPI1/2 message out */
    static CTL_TASK_t task;
    static unsigned stack[CTL_CPU_STATE_WORD_SIZE+32];
    ctl_task_run(&task, TASKPRI_IOEOUT, &task_iob_dpi12_out, 0, "iob_dpi12_out", sizeof(stack)/sizeof(unsigned), stack, 0);
  }
  { /* IOE-B DPI3/4 message out */
    static CTL_TASK_t task;
    static unsigned stack[CTL_CPU_STATE_WORD_SIZE+32];
    ctl_task_run(&task, TASKPRI_IOEOUT, &task_iob_dpi34_out, 0, "iob_dpi34_out", sizeof(stack)/sizeof(unsigned), stack, 0);
  }
  { /* PD16 status generation */
    static CTL_TASK_t task;
    static unsigned stack[CTL_CPU_STATE_WORD_SIZE+4];
    ctl_task_run(&task, TASKPRI_STATUS, &task_pd16_status_out, 0, "pd16_status_out", sizeof(stack)/sizeof(unsigned), stack, 0);
  }
  { /* PD16A AVI1/2/3/4-SPI1/2/3/4 out */
    static CTL_TASK_t task;
    static unsigned stack[CTL_CPU_STATE_WORD_SIZE+32];
    ctl_task_run(&task, TASKPRI_PD16OUT, &task_pd16a_avispi_out, 0, "pd16a_avispi_out", sizeof(stack)/sizeof(unsigned), stack, 0);
  }
  { /* PD16A HCO25/HCO8/HBO out */
    static CTL_TASK_t task;
    static unsigned stack[CTL_CPU_STATE_WORD_SIZE+32];
    ctl_task_run(&task, TASKPRI_PD16OUT, &task_pd16a_hco8hco25hbo_out, 0, "pd16a_hco8hco25hbo_out", sizeof(stack)/sizeof(unsigned), stack, 0);
  }
  { /* PD16A DIAG out */
    static CTL_TASK_t task;
    static unsigned stack[CTL_CPU_STATE_WORD_SIZE+32];
    ctl_task_run(&task, TASKPRI_PD16OUT, &task_pd16a_diag_out, 0, "pd16a_diag_out", sizeof(stack)/sizeof(unsigned), stack, 0);
  }

  //{ /* for testing whatever you want */
  //  static CTL_TASK_t task;
  //  static unsigned stack[CTL_CPU_STATE_WORD_SIZE+256];
  //  ctl_task_run(&task, TASKPRI_TEST, &task_test, 0, "task_test", sizeof(stack)/sizeof(unsigned), stack, 0);
  //}

  (void)ctl_task_set_priority(&self, TASKPRI_IDLE);
  lock_io();
  while(~0) { /**< IDLE */
    sine = lookup((ctl_get_current_time()/15)%1000); /* 1/15 Hz */
    errcode_t e;
    if(ctl_message_queue_receive_nb(&codes, (void**)&e)) {
      assert(e != CODE_NONE);
      debug_printf("MODULE: %d, CODE: %d\r\n", GET_MODULE(e), GET_CODE(e)); /* dump codes until something better is made */
    }
    #ifdef NDEBUG
    __WFI();
    #endif
  }
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief testing task - for whatever purpose
 */
static __NO_RETURN void
  task_test(void *arg)
{
  (void)arg;
  while(~0) {
    CTL_TIME_t const t0 = ctl_get_current_time();
    unsigned t;
    /* burst 8 'out of band' packets and make sure that the PCAN can pick them up correctly */
    /* this tests that the whole tx queue mechanism works as intended -> watch scope        */
    canframe_t *pframe;

    pframe = (canframe_t*)ctl_memory_area_allocate(&frames); assert(pframe);
    pframe->txhdr.Identifier = 0x100;
    pframe->txhdr.IdType = FDCAN_STANDARD_ID;
    pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
    pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
    pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
    pframe->txhdr.DataLength = 4;
    t = SysTick->VAL;
    pframe->data.as_uint8[3] = t >> 0;
    pframe->data.as_uint8[2] = t >> 8;
    pframe->data.as_uint8[1] = t >> 16;
    pframe->data.as_uint8[0] = 0x10;
    if(!htbus_send(pframe))
      ctl_memory_area_free(&frames, (unsigned*)pframe);

    pframe = (canframe_t*)ctl_memory_area_allocate(&frames); assert(pframe);
    pframe->txhdr.Identifier = 0x101;
    pframe->txhdr.IdType = FDCAN_STANDARD_ID;
    pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
    pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
    pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
    pframe->txhdr.DataLength = 4;
    t = SysTick->VAL;
    pframe->data.as_uint8[3] = t >> 0;
    pframe->data.as_uint8[2] = t >> 8;
    pframe->data.as_uint8[1] = t >> 16;
    pframe->data.as_uint8[0] = 0x20;
    if(!htbus_send(pframe))
      ctl_memory_area_free(&frames, (unsigned*)pframe);

    pframe = (canframe_t*)ctl_memory_area_allocate(&frames); assert(pframe);
    pframe->txhdr.Identifier = 0x102;
    pframe->txhdr.IdType = FDCAN_STANDARD_ID;
    pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
    pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
    pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
    pframe->txhdr.DataLength = 4;
    t = SysTick->VAL;
    pframe->data.as_uint8[3] = t >> 0;
    pframe->data.as_uint8[2] = t >> 8;
    pframe->data.as_uint8[1] = t >> 16;
    pframe->data.as_uint8[0] = 0x30;
    if(!htbus_send(pframe))
      ctl_memory_area_free(&frames, (unsigned*)pframe);

    pframe = (canframe_t*)ctl_memory_area_allocate(&frames); assert(pframe);
    pframe->txhdr.Identifier = 0x103;
    pframe->txhdr.IdType = FDCAN_STANDARD_ID;
    pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
    pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
    pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
    pframe->txhdr.DataLength = 4;
    t = SysTick->VAL;
    pframe->data.as_uint8[3] = t >> 0;
    pframe->data.as_uint8[2] = t >> 8;
    pframe->data.as_uint8[1] = t >> 16;
    pframe->data.as_uint8[0] = 0x40;
    if(!htbus_send(pframe))
      ctl_memory_area_free(&frames, (unsigned*)pframe);

    pframe = (canframe_t*)ctl_memory_area_allocate(&frames); assert(pframe);
    pframe->txhdr.Identifier = 0x104;
    pframe->txhdr.IdType = FDCAN_STANDARD_ID;
    pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
    pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
    pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
    pframe->txhdr.DataLength = 4;
    t = SysTick->VAL;
    pframe->data.as_uint8[3] = t >> 0;
    pframe->data.as_uint8[2] = t >> 8;
    pframe->data.as_uint8[1] = t >> 16;
    pframe->data.as_uint8[0] = 0x50;
    if(!htbus_send(pframe))
      ctl_memory_area_free(&frames, (unsigned*)pframe);

    pframe = (canframe_t*)ctl_memory_area_allocate(&frames); assert(pframe);
    pframe->txhdr.Identifier = 0x105;
    pframe->txhdr.IdType = FDCAN_STANDARD_ID;
    pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
    pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
    pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
    pframe->txhdr.DataLength = 4;
    t = SysTick->VAL;
    pframe->data.as_uint8[3] = t >> 0;
    pframe->data.as_uint8[2] = t >> 8;
    pframe->data.as_uint8[1] = t >> 16;
    pframe->data.as_uint8[0] = 0x60;
    if(!htbus_send(pframe))
      ctl_memory_area_free(&frames, (unsigned*)pframe);

    pframe = (canframe_t*)ctl_memory_area_allocate(&frames); assert(pframe);
    pframe->txhdr.Identifier = 0x106;
    pframe->txhdr.IdType = FDCAN_STANDARD_ID;
    pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
    pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
    pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
    pframe->txhdr.DataLength = 4;
    t = SysTick->VAL;
    pframe->data.as_uint8[3] = t >> 0;
    pframe->data.as_uint8[2] = t >> 8;
    pframe->data.as_uint8[1] = t >> 16;
    pframe->data.as_uint8[0] = 0x70;
    if(!htbus_send(pframe))
      ctl_memory_area_free(&frames, (unsigned*)pframe);

    pframe = (canframe_t*)ctl_memory_area_allocate(&frames); assert(pframe);
    pframe->txhdr.Identifier = 0x107;
    pframe->txhdr.IdType = FDCAN_STANDARD_ID;
    pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
    pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
    pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
    pframe->txhdr.DataLength = 4;
    t = SysTick->VAL;
    pframe->data.as_uint8[3] = t >> 0;
    pframe->data.as_uint8[2] = t >> 8;
    pframe->data.as_uint8[1] = t >> 16;
    pframe->data.as_uint8[0] = 0x80;
    if(!htbus_send(pframe))
      ctl_memory_area_free(&frames, (unsigned*)pframe);

    ctl_timeout_wait(t0+500);
  }
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief anyone who wants to 'chirp the LED' can signal EVT_LED_CHIRP to do so
 *        use ctl_events_pulse(&e, EVT_LED_CHIRP);
 */
static __NO_RETURN void
  task_led_chirp(void *arg)
{
  CTL_TIME_t const to = (CTL_TIME_t)arg; assert(to);
  evt_t evt = EVT_NONE;
  while(~0) {
    CTL_TIME_t const t = ctl_get_current_time();
    (evt == EVT_NONE)?led_off():led_on();
    evt = ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS, &e, EVT_LED_CHIRP,
                          CTL_TIMEOUT_ABSOLUTE, t+to);
  }
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief generic button debounce task
 */
static __NO_RETURN void
  task_but_debounce(void *arg)
{
  debounce_info_t * const pdi = arg; assert(pdi);
  typedef enum _state_t {
    DEBOUNCE_DOWN,
    DEBOUNCE_UP,
    IDLE
  } state_t;

  state_t s = IDLE;
  while(~0) {
    switch(s) {
      case DEBOUNCE_DOWN:
      case DEBOUNCE_UP: {    /**< wait for timeout, but restart on edge */
        evt_t const evt =
          ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS, pdi->pe,
                          pdi->rising | pdi->falling, CTL_TIMEOUT_DELAY, pdi->to);
               if(evt & pdi->falling) { s = DEBOUNCE_DOWN; } /*< restart   */
          else if(evt & pdi->rising)  { s = DEBOUNCE_UP; }   /*< restart   */
          else if(evt == EVT_NONE) {                         /*< debounced */
            ctl_events_pulse(pdi->pe, (s==DEBOUNCE_UP)?pdi->up:pdi->down);
            s = IDLE;
          } else {
            (void)ctl_message_queue_post_nb(&codes, (void*)CODE_DEBOUNCE_UNK0);
            ctl_task_die();
          }
        } break;
      case IDLE: { /**< block indefinitely until an edge arrives */
        evt_t const evt =
          ctl_events_wait_uc(CTL_EVENT_WAIT_ANY_EVENTS, pdi->pe,
                             pdi->rising | pdi->falling);
               if(evt & pdi->falling) { s = DEBOUNCE_DOWN; } /*< start */
          else if(evt & pdi->rising)  { s = DEBOUNCE_UP; }   /*< start */
          else {
            (void)ctl_message_queue_post_nb(&codes, (void*)CODE_DEBOUNCE_UNK1);
            ctl_task_die();
          }
        } break;
      default:
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_DEBOUNCE_UNK2);
        ctl_task_die(); /*> TODO: signal error <*/
    }
  }
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief handle IOEA/B status (0x02C6/7) emit
 */
static __NO_RETURN void
  task_ioe_status_out(void *arg)
{
  enum { ID_IOEA_STATUS = 0x2C6,
         ID_IOEB_STATUS = 0x2C7,
         BASE_MS        = 100,   /* 10Hz base emit rate                */
         PAUSE_MS       = 5 };   /* forced pause between status frames */
  static_assert((PAUSE_MS*2) < BASE_MS);

  (void)arg;
  while(~0) {
    CTL_TIME_t const t0 = ctl_get_current_time();

    canframe_t *pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_IOEA_STATUS;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_5;
      pframe->data.as_uint8[0] = (1<<4);
      pframe->data.as_uint8[1] = (2<<2)|(2<<0); /* 2.22.22 */
      pframe->data.as_uint8[2] = 22;
      pframe->data.as_uint8[3] = 22;
      pframe->data.as_uint8[4] = 222; /**< 0: Release, 1-100: Alpha <#>, */
                                      /*   101-200: Beta <100-#>,        */
                                      /*   201-255: Experimental <200-#> */
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_IOA_SENDSTATUS);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*1));
    } else { (void)ctl_message_queue_post_nb(&codes, (void*)CODE_IOA_ALLOCSTATUS); }

    pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_IOEB_STATUS;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_5;
      pframe->data.as_uint8[0] = (1<<4);
      pframe->data.as_uint8[1] = (2<<2)|(2<<0);
      pframe->data.as_uint8[2] = 22;
      pframe->data.as_uint8[3] = 22;
      pframe->data.as_uint8[4] = 222;
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_IOB_SENDSTATUS);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*2));
    } else { (void)ctl_message_queue_post_nb(&codes, (void*)CODE_IOB_ALLOCSTATUS); }

    ctl_timeout_wait(t0+BASE_MS); /**< stay on track - unknown if anything */
  }                               /*   above has had to block              */
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief handle IOEA AVI1/2/3/4 0x02C0 emit per spec
 */
static __NO_RETURN void
  task_ioa_avi14_out(void *arg)
{
  enum { ID_IOA_AVI14 = 0x2C0,
         BASE_MS      = 50,   /* 20Hz base emit rate               */
         LOCKOUT_MS   = 20 }; /* forced pause between AVI14 frames */

  (void)arg;
  while(~0) {
    CTL_TIME_t const t0 = ctl_get_current_time();
    /*> TODO: EMIT AVI14 0x02C0 <*/
    unsigned const v = (4095*sine)/100000; assert(v <= 4095);
    canframe_t *pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_IOA_AVI14;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_8;
      pframe->data.as_uint8[0] = v>>8;
      pframe->data.as_uint8[1] = v;
      pframe->data.as_uint8[2] = v>>8;
      pframe->data.as_uint8[3] = v;
      pframe->data.as_uint8[4] = v>>8;
      pframe->data.as_uint8[5] = v;
      pframe->data.as_uint8[6] = v>>8;
      pframe->data.as_uint8[7] = v;
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_IOA_SENDAVI14);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+LOCKOUT_MS);
    }
    CTL_EVENT_SET_t const evt =
      ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS, &e_ioa,
                      EVT_IOE_AVI1 | EVT_IOE_AVI2 | EVT_IOE_AVI3 | EVT_IOE_AVI4,
                      CTL_TIMEOUT_ABSOLUTE, t0+BASE_MS); /**< stay on track */
    (void)evt;   /**< knowing what unblocked us does not matter, time to tx */
  }
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief handle IOEA DPI1/2 0x02C2 emit per spec
 */
static __NO_RETURN void
  task_ioa_dpi12_out(void *arg)
{
  enum { ID_IOA_DPI12 = 0x2C2,
         BASE_MS      = 50,   /* 20Hz base emit rate               */
         LOCKOUT_MS   = 20 }; /* forced pause between DPI12 frames */

  (void)arg;
  while(~0) {
    unsigned const d = (1000*sine)/100000; assert(d <= 1000);
    uint64_t const p = (131071-1)*(uint64_t)sine/100000+1; assert(p <= 131071); assert(p > 0);
    CTL_TIME_t const t0 = ctl_get_current_time();
    /*> TODO: EMIT DPI12 0x02C2 <*/
    canframe_t *pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_IOA_DPI12;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_8;
      pframe->data.as_uint8[0] = (d>>2);
      pframe->data.as_uint8[1] = (d<<6)|((p>>16)&0b1);
      pframe->data.as_uint8[2] = p>>8;
      pframe->data.as_uint8[3] = p;
      pframe->data.as_uint8[4] = d>>2;
      pframe->data.as_uint8[5] = (d<<6)|((p>>16)&0b1);
      pframe->data.as_uint8[6] = p>>8;
      pframe->data.as_uint8[7] = p;
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_IOA_SENDDPI12);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+LOCKOUT_MS);
    }
    CTL_EVENT_SET_t const evt =
      ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS, &e_ioa,
                      EVT_IOE_DPI1 | EVT_IOE_DPI2,
                      CTL_TIMEOUT_ABSOLUTE, t0+BASE_MS); /**< stay on track */
    (void)evt;   /**< knowing what unblocked us does not matter, time to tx */
  }
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief handle IOEA DPI3/4 0x02C4 emit per spec
 */
static __NO_RETURN void
  task_ioa_dpi34_out(void *arg)
{
  enum { ID_IOA_DPI34 = 0x2C4,
         BASE_MS      = 50,   /* 20Hz base emit rate               */
         LOCKOUT_MS   = 20 }; /* forced pause between DPI12 frames */

  (void)arg;
  while(~0) {
    unsigned const d = (1000*sine)/100000; assert(d <= 1000);
    uint64_t const p = (131071-1)*(uint64_t)sine/100000+1; assert(p <= 131071); assert(p > 0);
    CTL_TIME_t const t0 = ctl_get_current_time();
    /*> TODO: EMIT DPI34 0x02C4 <*/
    canframe_t *pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_IOA_DPI34;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_8;
      pframe->data.as_uint8[0] = (d>>2);
      pframe->data.as_uint8[1] = (d<<6)|((p>>16)&0b1);
      pframe->data.as_uint8[2] = p>>8;
      pframe->data.as_uint8[3] = p;
      pframe->data.as_uint8[4] = d>>2;
      pframe->data.as_uint8[5] = (d<<6)|((p>>16)&0b1);
      pframe->data.as_uint8[6] = p>>8;
      pframe->data.as_uint8[7] = p;
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_IOA_SENDDPI34);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+LOCKOUT_MS);
    }
    CTL_EVENT_SET_t const evt =
      ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS, &e_ioa,
                      EVT_IOE_DPI3 | EVT_IOE_DPI4,
                      CTL_TIMEOUT_ABSOLUTE, t0+BASE_MS); /**< stay on track */
    (void)evt;   /**< knowing what unblocked us does not matter, time to tx */
  }
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief handle IOEB AVI1/2/3/4 0x02C1 emit per spec
 */
static __NO_RETURN void
  task_iob_avi14_out(void *arg)
{
  enum { ID_IOB_AVI14 = 0x2C1,
         BASE_MS      = 50,   /* 20Hz base emit rate               */
         LOCKOUT_MS   = 20 }; /* forced pause between AVI14 frames */

  (void)arg;
  while(~0) {
    CTL_TIME_t const t0 = ctl_get_current_time();
    /*> TODO: EMIT AVI14 0x02C1 <*/
    unsigned const v = (4095*sine)/100000; assert(v <= 4095);
    canframe_t *pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_IOB_AVI14;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_8;
      pframe->data.as_uint8[0] = v>>8;
      pframe->data.as_uint8[1] = v;
      pframe->data.as_uint8[2] = v>>8;
      pframe->data.as_uint8[3] = v;
      pframe->data.as_uint8[4] = v>>8;
      pframe->data.as_uint8[5] = v;
      pframe->data.as_uint8[6] = v>>8;
      pframe->data.as_uint8[7] = v;
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_IOB_SENDAVI14);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+LOCKOUT_MS);
    }
    CTL_EVENT_SET_t const evt =
      ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS, &e_iob,
                      EVT_IOE_AVI1 | EVT_IOE_AVI2 | EVT_IOE_AVI3 | EVT_IOE_AVI4,
                      CTL_TIMEOUT_ABSOLUTE, t0+BASE_MS); /**< stay on track */
    (void)evt;   /**< knowing what unblocked us does not matter, time to tx */
  }
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief handle IOEB DPI1/2 0x02C3 emit per spec
 */
static __NO_RETURN void
  task_iob_dpi12_out(void *arg)
{
  enum { ID_IOB_DPI12 = 0x2C3,
         BASE_MS      = 50,   /* 20Hz base emit rate               */
         LOCKOUT_MS   = 20 }; /* forced pause between DPI12 frames */

  (void)arg;
  while(~0) {
    unsigned const d = (1000*sine)/100000; assert(d <= 1000);
    uint64_t const p = (131071-1)*(uint64_t)sine/100000+1; assert(p <= 131071); assert(p > 0);
    CTL_TIME_t const t0 = ctl_get_current_time();
    /*> TODO: EMIT DPI12 0x02C3 <*/
    canframe_t *pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_IOB_DPI12;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_8;
      pframe->data.as_uint8[0] = (d>>2);
      pframe->data.as_uint8[1] = (d<<6)|((p>>16)&0b1);
      pframe->data.as_uint8[2] = p>>8;
      pframe->data.as_uint8[3] = p;
      pframe->data.as_uint8[4] = d>>2;
      pframe->data.as_uint8[5] = (d<<6)|((p>>16)&0b1);
      pframe->data.as_uint8[6] = p>>8;
      pframe->data.as_uint8[7] = p;
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_IOB_SENDDPI12);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+LOCKOUT_MS);
    }
    CTL_EVENT_SET_t const evt =
      ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS, &e_iob,
                      EVT_IOE_DPI1 | EVT_IOE_DPI2,
                      CTL_TIMEOUT_ABSOLUTE, t0+BASE_MS); /**< stay on track */
    (void)evt;   /**< knowing what unblocked us does not matter, time to tx */
  }
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief handle IOEB DPI3/4 0x02C5 emit per spec
 */
static __NO_RETURN void
  task_iob_dpi34_out(void *arg)
{
  enum { ID_IOB_DPI34 = 0x2C5,
         BASE_MS      = 50,   /* 20Hz base emit rate               */
         LOCKOUT_MS   = 20 }; /* forced pause between DPI12 frames */

  (void)arg;
  while(~0) {
    unsigned const d = (1000*sine)/100000; assert(d <= 1000);
    uint64_t const p = (131071-1)*(uint64_t)sine/100000+1; assert(p <= 131071); assert(p > 0);
    CTL_TIME_t const t0 = ctl_get_current_time();
    /*> TODO: EMIT DPI34 0x02C5 <*/
    canframe_t *pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_IOB_DPI34;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_8;
      pframe->data.as_uint8[0] = (d>>2);
      pframe->data.as_uint8[1] = (d<<6)|((p>>16)&0b1);
      pframe->data.as_uint8[2] = p>>8;
      pframe->data.as_uint8[3] = p;
      pframe->data.as_uint8[4] = d>>2;
      pframe->data.as_uint8[5] = (d<<6)|((p>>16)&0b1);
      pframe->data.as_uint8[6] = p>>8;
      pframe->data.as_uint8[7] = p;
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_IOB_SENDDPI34);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+LOCKOUT_MS);
    }
    CTL_EVENT_SET_t const evt =
      ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS, &e_iob,
                      EVT_IOE_DPI3 | EVT_IOE_DPI4,
                      CTL_TIMEOUT_ABSOLUTE, t0+BASE_MS); /**< stay on track */
    (void)evt;   /**< knowing what unblocked us does not matter, time to tx */
  }
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief handle PD16A/B/C/D status (0x06D5/DD/E5/E8) emit
 *
 */
static __NO_RETURN void
  task_pd16_status_out(void *arg)
{
  enum { ID_PD16A_STATUS = 0x6D5,
         ID_PD16B_STATUS = 0x6DD,
         ID_PD16C_STATUS = 0x6E5,
         ID_PD16D_STATUS = 0x6ED,
         BASE_MS         = 100,   /* 10Hz base emit rate                */
         PAUSE_MS        = 5 };   /* forced pause between status frames */
  static_assert((PAUSE_MS*4) < BASE_MS);

  (void)arg;
  while(~0) {
    CTL_TIME_t const t0 = ctl_get_current_time();

    canframe_t *pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16A_STATUS;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_5;
      pframe->data.as_uint8[0] = (1<<4);
      pframe->data.as_uint8[1] = (2<<2)|(2<<0);
      pframe->data.as_uint8[2] = 22;
      pframe->data.as_uint8[3] = 22;
      pframe->data.as_uint8[4] = 222;
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_SENDSTATUS);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*1));
    } else { (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_ALLOCSTATUS); }

    pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16B_STATUS;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_5;
      pframe->data.as_uint8[0] = (1<<4);
      pframe->data.as_uint8[1] = (2<<2)|(2<<0);
      pframe->data.as_uint8[2] = 22;
      pframe->data.as_uint8[3] = 22;
      pframe->data.as_uint8[4] = 222;
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16B_SENDSTATUS);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*2));
    } else { (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16B_ALLOCSTATUS); }

    pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16C_STATUS;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_5;
      pframe->data.as_uint8[0] = (1<<4);
      pframe->data.as_uint8[1] = (2<<2)|(2<<0);
      pframe->data.as_uint8[2] = 22;
      pframe->data.as_uint8[3] = 22;
      pframe->data.as_uint8[4] = 222;
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16C_SENDSTATUS);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*3));
    } else { (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16C_ALLOCSTATUS); }

    pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16D_STATUS;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_5;
      pframe->data.as_uint8[0] = (1<<4);
      pframe->data.as_uint8[1] = (2<<2)|(2<<0);
      pframe->data.as_uint8[2] = 22;
      pframe->data.as_uint8[3] = 22;
      pframe->data.as_uint8[4] = 222;
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16D_SENDSTATUS);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*4));
    } else { (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16D_ALLOCSTATUS); }

    ctl_timeout_wait(t0+BASE_MS); /**< stay on track - unknown if anything */
  }                               /*   above has had to block              */
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief handle PD16A AVI1/2/3/4-SPI1/2/3/4 0x06D3 emit per spec
 */
static __NO_RETURN void
  task_pd16a_avispi_out(void *arg)
{
  enum { ID_PD16_AVISPI = 0x6D3,
         BASE_MS    = 50,   /* 20Hz base emit rate             */
         PAUSE_MS   = 5 };  /* forced pause between mux frames */
  static_assert((PAUSE_MS*8) < BASE_MS);

  (void)arg;
  while(~0) {
    unsigned const v = (5000*sine)/100000;
    unsigned const d = (1000*sine)/100000;
    CTL_TIME_t const t0 = ctl_get_current_time();

    /*> TODO: EMIT AVI1 0x06D3 <*/
    canframe_t *pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16_AVISPI;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_4;
      pframe->data.as_uint8[0] = (4<<5)|(0);   /**< AVI1   */
      pframe->data.as_uint8[1] = (v>2500)?1:0; /**< on/off */
      pframe->data.as_uint8[2] = v>>8;         /**< v      */
      pframe->data.as_uint8[3] = v;
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_SENDAVI1);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*1));
    }
    /*> TODO: EMIT AVI2 0x06D3 <*/
    pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16_AVISPI;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_4;
      pframe->data.as_uint8[0] = (4<<5)|(1);   /**< AVI2   */
      pframe->data.as_uint8[1] = (v>2500)?1:0; /**< on/off */
      pframe->data.as_uint8[2] = v>>8;         /**< v      */
      pframe->data.as_uint8[3] = v;
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_SENDAVI2);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*2));
    }
    /*> TODO: EMIT AVI3 0x06D3 <*/
    pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16_AVISPI;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_4;
      pframe->data.as_uint8[0] = (4<<5)|(2);   /**< AVI3   */
      pframe->data.as_uint8[1] = (v>2500)?1:0; /**< on/off */
      pframe->data.as_uint8[2] = v>>8;         /**< v      */
      pframe->data.as_uint8[3] = v;
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_SENDAVI3);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*3));
    }
    /*> TODO: EMIT AVI4 0x06D3 <*/
    pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16_AVISPI;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_4;
      pframe->data.as_uint8[0] = (4<<5)|(3);   /**< AVI4   */
      pframe->data.as_uint8[1] = (v>2500)?1:0; /**< on/off */
      pframe->data.as_uint8[2] = v>>8;         /**< v      */
      pframe->data.as_uint8[3] = v;
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_SENDAVI4);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*4));
    }
    /*> TODO: EMIT SPI1 0x06D3 <*/
    pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16_AVISPI;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_8;
      pframe->data.as_uint8[0] = (3<<5)|(0);   /**< SPI1   */
      pframe->data.as_uint8[1] = (v>2500)?1:0; /**< on/off */
      pframe->data.as_uint8[2] = v>>8;         /**< v      */
      pframe->data.as_uint8[3] = v;
      pframe->data.as_uint8[4] = d>>8;         /**< duty   */
      pframe->data.as_uint8[5] = d;
      pframe->data.as_uint8[6] = d>>8;         /**< freq   */
      pframe->data.as_uint8[7] = d;
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_SENDSPI1);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*5));
    }
    /*> TODO: EMIT SPI2 0x06D3 <*/
    pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16_AVISPI;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_8;
      pframe->data.as_uint8[0] = (3<<5)|(1);   /**< SPI2   */
      pframe->data.as_uint8[1] = (v>2500)?1:0; /**< on/off */
      pframe->data.as_uint8[2] = v>>8;         /**< v      */
      pframe->data.as_uint8[3] = v;
      pframe->data.as_uint8[4] = d>>8;         /**< duty   */
      pframe->data.as_uint8[5] = d;
      pframe->data.as_uint8[6] = d>>8;         /**< freq   */
      pframe->data.as_uint8[7] = d;
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_SENDSPI2);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*6));
    }
    /*> TODO: EMIT SPI3 0x06D3 <*/
    pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16_AVISPI;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_8;
      pframe->data.as_uint8[0] = (3<<5)|(2);   /**< SPI3   */
      pframe->data.as_uint8[1] = (v>2500)?1:0; /**< on/off */
      pframe->data.as_uint8[2] = v>>8;         /**< v      */
      pframe->data.as_uint8[3] = v;
      pframe->data.as_uint8[4] = d>>8;         /**< duty   */
      pframe->data.as_uint8[5] = d;
      pframe->data.as_uint8[6] = d>>8;         /**< freq   */
      pframe->data.as_uint8[7] = d;
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_SENDSPI3);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*7));
    }
    /*> TODO: EMIT SPI4 0x06D3 <*/
    pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16_AVISPI;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_8;
      pframe->data.as_uint8[0] = (3<<5)|(3);   /**< SPI4   */
      pframe->data.as_uint8[1] = (v>2500)?1:0; /**< on/off */
      pframe->data.as_uint8[2] = v>>8;         /**< v      */
      pframe->data.as_uint8[3] = v;
      pframe->data.as_uint8[4] = d>>8;         /**< duty   */
      pframe->data.as_uint8[5] = d;
      pframe->data.as_uint8[6] = d>>8;         /**< freq   */
      pframe->data.as_uint8[7] = d;
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_SENDSPI4);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*8));
    }

    CTL_EVENT_SET_t const evt =
      ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS , &e_pd16a,
                      EVT_PD16_AVI1 | EVT_PD16_AVI2 | EVT_PD16_AVI3 | EVT_PD16_AVI4 |
                      EVT_PD16_SPI1 | EVT_PD16_SPI2 | EVT_PD16_SPI3 | EVT_PD16_SPI4,
                      CTL_TIMEOUT_ABSOLUTE, t0+BASE_MS);
    (void)evt;  /**< knowing what unblocked us does not matter, time to tx */
  }
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief handle PD16A HCO25/HCO8/HBO 0x06D4 emit per spec
 */
static __NO_RETURN void
  task_pd16a_hco8hco25hbo_out(void *arg)
{
  enum { ID_PD16_HCOHBO = 0x6D4,
         BASE_MS  = 200,  /* 5Hz base emit rate              */
         PAUSE_MS = 5 };  /* forced pause between mux frames */
  static_assert((PAUSE_MS*16) < BASE_MS);

  (void)arg;
  while(~0) {
    unsigned const l = (100*sine)/100000;
    unsigned const v = (16000*sine)/100000;
    uint64_t const i = (50000*(uint64_t)sine)/100000;
    CTL_TIME_t const t0 = ctl_get_current_time();

    /*> TODO: EMIT HCO25/HCO8/HBO 0x06D4 <*/
    canframe_t *pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16_HCOHBO;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_8;
      pframe->data.as_uint8[0] = (0<<5)|(0);  /**< HCO25-1     */
      pframe->data.as_uint8[1] = l;           /**< load        */
      pframe->data.as_uint8[2] = v>>8;        /**< v           */
      pframe->data.as_uint8[3] = v;
      pframe->data.as_uint8[4] = i>>8;        /**< low-side i  */
      pframe->data.as_uint8[5] = i;
      pframe->data.as_uint8[6] = i>>8;        /**< high-side i */
      pframe->data.as_uint8[7] = i;
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_SENDHCO251);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*1));
    }
    /*> TODO: EMIT HCO25/HCO8/HBO 0x06D4 <*/
    pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16_HCOHBO;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_8;
      pframe->data.as_uint8[0] = (0<<5)|(1);  /**< HCO25-2     */
      pframe->data.as_uint8[1] = l;           /**< load        */
      pframe->data.as_uint8[2] = v>>8;        /**< v           */
      pframe->data.as_uint8[3] = v;
      pframe->data.as_uint8[4] = i>>8;        /**< low-side i  */
      pframe->data.as_uint8[5] = i;
      pframe->data.as_uint8[6] = i>>8;        /**< high-side i */
      pframe->data.as_uint8[7] = i;
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_SENDHCO252);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*2));
    }
    /*> TODO: EMIT HCO25/HCO8/HBO 0x06D4 <*/
    pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16_HCOHBO;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_8;
      pframe->data.as_uint8[0] = (0<<5)|(2);  /**< HCO25-3     */
      pframe->data.as_uint8[1] = l;           /**< load        */
      pframe->data.as_uint8[2] = v>>8;        /**< v           */
      pframe->data.as_uint8[3] = v;
      pframe->data.as_uint8[4] = i>>8;        /**< low-side i  */
      pframe->data.as_uint8[5] = i;
      pframe->data.as_uint8[6] = i>>8;        /**< high-side i */
      pframe->data.as_uint8[7] = i;
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_SENDHCO253);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*3));
    }
    /*> TODO: EMIT HCO25/HCO8/HBO 0x06D4 <*/
    pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16_HCOHBO;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_8;
      pframe->data.as_uint8[0] = (0<<5)|(3);  /**< HCO25-4     */
      pframe->data.as_uint8[1] = l;           /**< load        */
      pframe->data.as_uint8[2] = v>>8;        /**< v           */
      pframe->data.as_uint8[3] = v;
      pframe->data.as_uint8[4] = i>>8;        /**< low-side i  */
      pframe->data.as_uint8[5] = i;
      pframe->data.as_uint8[6] = i>>8;        /**< high-side i */
      pframe->data.as_uint8[7] = i;
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_SENDHCO254);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*4));
    }
    /*> TODO: EMIT HCO25/HCO8/HBO 0x06D4 <*/
    pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16_HCOHBO;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_7;
      pframe->data.as_uint8[0] = (1<<5)|(0);  /**< HCO8-1 */
      pframe->data.as_uint8[1] = (4<<3)|(i>8000?2:0); /**< retry/status */
      pframe->data.as_uint8[2] = v>>8;        /**< v      */
      pframe->data.as_uint8[3] = v;
      pframe->data.as_uint8[4] = i>>8;        /**< i      */
      pframe->data.as_uint8[5] = i;
      pframe->data.as_uint8[6] = l;           /**< load   */
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_SENDHCO81);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*5));
    }
    /*> TODO: EMIT HCO25/HCO8/HBO 0x06D4 <*/
    pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16_HCOHBO;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_7;
      pframe->data.as_uint8[0] = (1<<5)|(1);  /**< HCO8-2 */
      pframe->data.as_uint8[1] = (4<<3)|(i>8000?2:0); /**< retry/status */
      pframe->data.as_uint8[2] = v>>8;        /**< v      */
      pframe->data.as_uint8[3] = v;
      pframe->data.as_uint8[4] = i>>8;        /**< i      */
      pframe->data.as_uint8[5] = i;
      pframe->data.as_uint8[6] = l;           /**< load   */
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_SENDHCO82);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*6));
    }
    /*> TODO: EMIT HCO25/HCO8/HBO 0x06D4 <*/
    pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16_HCOHBO;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_7;
      pframe->data.as_uint8[0] = (1<<5)|(2);  /**< HCO8-3 */
      pframe->data.as_uint8[1] = (4<<3)|(i>8000?2:0); /**< retry/status */
      pframe->data.as_uint8[2] = v>>8;        /**< v      */
      pframe->data.as_uint8[3] = v;
      pframe->data.as_uint8[4] = i>>8;        /**< i      */
      pframe->data.as_uint8[5] = i;
      pframe->data.as_uint8[6] = l;           /**< load   */
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_SENDHCO83);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*7));
    }
    /*> TODO: EMIT HCO25/HCO8/HBO 0x06D4 <*/
    pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16_HCOHBO;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_7;
      pframe->data.as_uint8[0] = (1<<5)|(3);  /**< HCO8-4 */
      pframe->data.as_uint8[1] = (4<<3)|(i>8000?2:0); /**< retry/status */
      pframe->data.as_uint8[2] = v>>8;        /**< v      */
      pframe->data.as_uint8[3] = v;
      pframe->data.as_uint8[4] = i>>8;        /**< i      */
      pframe->data.as_uint8[5] = i;
      pframe->data.as_uint8[6] = l;           /**< load   */
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_SENDHCO84);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*8));
    }
    /*> TODO: EMIT HCO25/HCO8/HBO 0x06D4 <*/
    pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16_HCOHBO;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_7;
      pframe->data.as_uint8[0] = (1<<5)|(4);  /**< HCO8-5 */
      pframe->data.as_uint8[1] = (4<<3)|(i>8000?2:0); /**< retry/status */
      pframe->data.as_uint8[2] = v>>8;        /**< v      */
      pframe->data.as_uint8[3] = v;
      pframe->data.as_uint8[4] = i>>8;        /**< i      */
      pframe->data.as_uint8[5] = i;
      pframe->data.as_uint8[6] = l;           /**< load   */
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_SENDHCO85);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*9));
    }
    /*> TODO: EMIT HCO25/HCO8/HBO 0x06D4 <*/
    pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16_HCOHBO;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_7;
      pframe->data.as_uint8[0] = (1<<5)|(5);  /**< HCO8-6 */
      pframe->data.as_uint8[1] = (4<<3)|(i>8000?2:0); /**< retry/status */
      pframe->data.as_uint8[2] = v>>8;        /**< v      */
      pframe->data.as_uint8[3] = v;
      pframe->data.as_uint8[4] = i>>8;        /**< i      */
      pframe->data.as_uint8[5] = i;
      pframe->data.as_uint8[6] = l;           /**< load   */
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_SENDHCO86);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*10));
    }
    /*> TODO: EMIT HCO25/HCO8/HBO 0x06D4 <*/
    pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16_HCOHBO;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_7;
      pframe->data.as_uint8[0] = (1<<5)|(6);  /**< HCO8-7 */
      pframe->data.as_uint8[1] = (4<<3)|(i>8000?2:0); /**< retry/status */
      pframe->data.as_uint8[2] = v>>8;        /**< v      */
      pframe->data.as_uint8[3] = v;
      pframe->data.as_uint8[4] = i>>8;        /**< i      */
      pframe->data.as_uint8[5] = i;
      pframe->data.as_uint8[6] = l;           /**< load   */
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_SENDHCO87);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*11));
    }
    /*> TODO: EMIT HCO25/HCO8/HBO 0x06D4 <*/
    pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16_HCOHBO;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_7;
      pframe->data.as_uint8[0] = (1<<5)|(7);  /**< HCO8-8 */
      pframe->data.as_uint8[1] = (4<<3)|(i>8000?2:0); /**< retry/status */
      pframe->data.as_uint8[2] = v>>8;        /**< v      */
      pframe->data.as_uint8[3] = v;
      pframe->data.as_uint8[4] = i>>8;        /**< i      */
      pframe->data.as_uint8[5] = i;
      pframe->data.as_uint8[6] = l;           /**< load   */
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_SENDHCO88);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*12));
    }
    /*> TODO: EMIT HCO25/HCO8/HBO 0x06D4 <*/
    pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16_HCOHBO;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_7;
      pframe->data.as_uint8[0] = (1<<5)|(8);  /**< HCO8-9 */
      pframe->data.as_uint8[1] = (4<<3)|(i>8000?2:0); /**< retry/status */
      pframe->data.as_uint8[2] = v>>8;        /**< v      */
      pframe->data.as_uint8[3] = v;
      pframe->data.as_uint8[4] = i>>8;        /**< i      */
      pframe->data.as_uint8[5] = i;
      pframe->data.as_uint8[6] = l;           /**< load   */
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_SENDHCO89);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*13));
    }
    /*> TODO: EMIT HCO25/HCO8/HBO 0x06D4 <*/
    pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16_HCOHBO;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_7;
      pframe->data.as_uint8[0] = (1<<5)|(9);  /**< HCO8-10 */
      pframe->data.as_uint8[1] = (4<<3)|(i>8000?2:0); /**< retry/status */
      pframe->data.as_uint8[2] = v>>8;        /**< v       */
      pframe->data.as_uint8[3] = v;
      pframe->data.as_uint8[4] = i>>8;        /**< i       */
      pframe->data.as_uint8[5] = i;
      pframe->data.as_uint8[6] = l;           /**< load    */
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_SENDHCO810);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*14));
    }
    /*> TODO: EMIT HCO25/HCO8/HBO 0x06D4 <*/
    pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16_HCOHBO;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_8;
      pframe->data.as_uint8[0] = (2<<5)|(0);  /**< HBO1        */
      pframe->data.as_uint8[1] = (10<<3)|(i>3000?2:0); /**< retry/status */
      pframe->data.as_uint8[2] = v>>8;        /**< v           */
      pframe->data.as_uint8[3] = v;
      pframe->data.as_uint8[4] = i>>8;        /**< low-side i  */
      pframe->data.as_uint8[5] = i;
      pframe->data.as_uint8[6] = i>>8;        /**< high-side i */
      pframe->data.as_uint8[7] = i;
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_SENDHBO1);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*15));
    }
    /*> TODO: EMIT HCO25/HCO8/HBO 0x06D4 <*/
    pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16_HCOHBO;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_8;
      pframe->data.as_uint8[0] = (2<<5)|(1);  /**< HBO2        */
      pframe->data.as_uint8[1] = (10<<3)|(i>3000?2:0); /**< retry/status */
      pframe->data.as_uint8[2] = v>>8;        /**< v           */
      pframe->data.as_uint8[3] = v;
      pframe->data.as_uint8[4] = i>>8;        /**< low-side i  */
      pframe->data.as_uint8[5] = i;
      pframe->data.as_uint8[6] = i>>8;        /**< high-side i */
      pframe->data.as_uint8[7] = i;
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_SENDHBO2);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*16));
    }

    CTL_EVENT_SET_t const evt =
      ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS , &e_pd16a,
                      EVT_PD16_HCO25_1 | EVT_PD16_HCO25_2 | EVT_PD16_HCO25_3 | EVT_PD16_HCO25_4 |
                      EVT_PD16_HCO8_1  | EVT_PD16_HCO8_2  | EVT_PD16_HCO8_3  | EVT_PD16_HCO8_4  |
                      EVT_PD16_HCO8_5  | EVT_PD16_HCO8_6  | EVT_PD16_HCO8_7  | EVT_PD16_HCO8_8  |
                      EVT_PD16_HCO8_9  | EVT_PD16_HCO8_10 | EVT_PD16_HBO_1   | EVT_PD16_HBO_2,
                      CTL_TIMEOUT_ABSOLUTE, t0+BASE_MS);
    (void)evt;  /**< knowing what unblocked us does not matter, time to tx all (for now) */
  }
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief handle PD16A diag info 0x6D6 emit per spec
 */
static __NO_RETURN void
  task_pd16a_diag_out(void *arg)
{
  enum { ID_PD16_DIAG = 0x6D6,
         BASE_MS  = 500,  /* 2Hz base emit rate              */
         PAUSE_MS = 5 };  /* forced pause between mux frames */
  static_assert((PAUSE_MS*6) < BASE_MS);

  (void)arg;
  while(~0) {
    unsigned const c = (255*sine)/100000;
    unsigned const l = (100*sine)/100000;
    unsigned const v = (16000*sine)/100000;
    uint64_t const i = (50000*(uint64_t)sine)/100000;
    CTL_TIME_t const t0 = ctl_get_current_time();

    /*> TODO: EMIT DIAG 0x06D6 <*/
    canframe_t *pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16_DIAG;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_8;
      pframe->data.as_uint8[0] = 0;           /**< DIAG0       */
      pframe->data.as_uint8[1] = (l>50?1:0);  /**< ign sw      */
      pframe->data.as_uint8[2] = v>>8;        /**< main rail v */
      pframe->data.as_uint8[3] = v;
      pframe->data.as_uint8[4] = v>>8;        /**< prot rail v */
      pframe->data.as_uint8[5] = v;
      pframe->data.as_uint8[6] = v>>8;        /**< ecr v       */
      pframe->data.as_uint8[7] = v;
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_SENDDIAG0);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*1));
    }
    /*> TODO: EMIT DIAG 0x06D6 <*/
    pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16_DIAG;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_8;
      pframe->data.as_uint8[0] = 1;             /**< DIAG1       */
      pframe->data.as_uint8[1] = 0;             /**< unspecified */
      pframe->data.as_uint8[2] = (v+10000)>>8;  /**< boost v     */
      pframe->data.as_uint8[3] = (v+10000);
      pframe->data.as_uint8[4] = v>>8;          /**< SEPIC v     */
      pframe->data.as_uint8[5] = v;
      pframe->data.as_uint8[6] = i>>8;          /**< ecr plug i  */
      pframe->data.as_uint8[7] = i;
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_SENDDIAG1);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*2));
    }
    /*> TODO: EMIT DIAG 0x06D6 <*/
    pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16_DIAG;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_8;
      pframe->data.as_uint8[0] = 2;        /**< DIAG2        */
      pframe->data.as_uint8[1] = 0;        /**< unspecified  */
      pframe->data.as_uint8[2] = 3300>>8;  /**< UVLO v       */
      pframe->data.as_uint8[3] = (uint8_t)3300;
      pframe->data.as_uint8[4] = v>>8;     /**< SENSOR GND v */
      pframe->data.as_uint8[5] = v;
      pframe->data.as_uint8[6] = v>>8;     /**< VDD v        */
      pframe->data.as_uint8[7] = v;
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_SENDDIAG2);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*3));
    }
    /*> TODO: EMIT DIAG 0x06D6 <*/
    pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16_DIAG;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_8;
      pframe->data.as_uint8[0] = 3;     /**< DIAG3          */
      pframe->data.as_uint8[1] = c;     /**< mail rail temp */
      pframe->data.as_uint8[2] = c;     /**< temp 1         */
      pframe->data.as_uint8[3] = c;     /**< temp 2         */
      pframe->data.as_uint8[4] = c;     /**< temp 3         */
      pframe->data.as_uint8[5] = c;     /**< cpu temp       */
      pframe->data.as_uint8[6] = i>>8;  /**< total I        */
      pframe->data.as_uint8[7] = i;
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_SENDDIAG3);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*4));
    }
    /*> TODO: EMIT DIAG 0x06D6 <*/
    pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16_DIAG;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_7;
      pframe->data.as_uint8[0] = 4;           /**< DIAG4         */
      pframe->data.as_uint8[1] = 0;           /**< unspecified   */
      pframe->data.as_uint8[2] = v>>8;        /**< batt v        */
      pframe->data.as_uint8[3] = v;
      pframe->data.as_uint8[4] = (t0/1000)>>8; /**< on time       */
      pframe->data.as_uint8[5] = (t0/1000);
      pframe->data.as_uint8[6] = 0b01010101;  /**< temp go/no-go */
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_SENDDIAG4);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*5));
    }
    /*> TODO: EMIT DIAG 0x06D6 <*/
    pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
    if(pframe) {
      pframe->txhdr.Identifier = ID_PD16_DIAG;
      pframe->txhdr.IdType = FDCAN_STANDARD_ID;
      pframe->txhdr.TxFrameType = FDCAN_DATA_FRAME;
      pframe->txhdr.BitRateSwitch = FDCAN_BRS_OFF;
      pframe->txhdr.FDFormat = FDCAN_CLASSIC_CAN;
      pframe->txhdr.DataLength = FDCAN_DLC_BYTES_8;
      pframe->data.as_uint8[0] = 5;     /**< DIAG5       */
      pframe->data.as_uint8[1] = 0;     /**< unspecified */
      pframe->data.as_uint8[2] = 0x00;  /**< sn#1/#2     */
      pframe->data.as_uint8[3] = 0x00;  /**< sn#3/#4     */
      pframe->data.as_uint8[4] = 0x08;  /**< sn#5/#6     */
      pframe->data.as_uint8[5] = 0x67;  /**< sn#7/#8     */
      pframe->data.as_uint8[6] = 0x53;  /**< sn#9/#10    */
      pframe->data.as_uint8[7] = 0x09;  /**< sn#11/#12   */
      if(!htbus_send(pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_PD16A_SENDDIAG5);
        ctl_memory_area_free(&frames, (void*)pframe); /**< _post failed! */
      }
      ctl_timeout_wait(t0+(PAUSE_MS*6));
    }

    CTL_EVENT_SET_t const evt =
      ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS , &e_pd16a,
                      EVT_PD16_DIAG0 | EVT_PD16_DIAG1 | EVT_PD16_DIAG2 |
                      EVT_PD16_DIAG3 | EVT_PD16_DIAG4,
                      CTL_TIMEOUT_ABSOLUTE, t0+BASE_MS);
    (void)evt;  /**< knowing what unblocked us does not matter, time to tx all (for now) */
  }
}

///**
//  * @brief  Initializes the FDCAN3 MSP.
//  * @param  hfdcan pointer to an FDCAN_HandleTypeDef structure that contains
//  *         the configuration information for the specified FDCAN.
//  * @retval None
//  */
//void atebus_init(FDCAN_HandleTypeDef *atebus)
//{
//  assert(atebus);
//
//  ErrorStatus e = ERROR;
//  LL_GPIO_InitTypeDef pin;
//  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
//
//  /* PB.3  - FDCAN3_RX (ATEBUS BUS) [AF11] */
//  LL_GPIO_StructInit(&pin);
//  pin.Mode      = LL_GPIO_MODE_ALTERNATE;
//  pin.Alternate = LL_GPIO_AF_9;
//  pin.Pin       = LL_GPIO_PIN_3;
//  pin.Pull      = LL_GPIO_PULL_UP;
//  e = LL_GPIO_Init(GPIOB, &pin);
//  assert(e == SUCCESS);
//
//  /* PB.4  - FDCAN3_TX (ATEBUS BUS) [AF11] */
//  LL_GPIO_StructInit(&pin);
//  pin.Mode      = LL_GPIO_MODE_ALTERNATE;
//  pin.Alternate = LL_GPIO_AF_11;
//  pin.Pin       = LL_GPIO_PIN_4;
//  pin.Speed     = LL_GPIO_SPEED_FREQ_VERY_HIGH;
//  pin.Pull      = LL_GPIO_PULL_NO;
//  e = LL_GPIO_Init(GPIOB, &pin);
//  assert(e == SUCCESS);
//
//  NVIC_SetPriority(FDCAN3_IT0_IRQn, IRQPRI_FDCAN);
//  NVIC_EnableIRQ(FDCAN3_IT0_IRQn);
//  NVIC_SetPriority(FDCAN3_IT1_IRQn, IRQPRI_FDCAN);
//  NVIC_EnableIRQ(FDCAN3_IT1_IRQn);
//}

/** ---------------------------------------------------------------------------
 * @internal
 */
static FDCAN_HandleTypeDef htbus = { .Instance = FDCAN2,
  .Init = {
    .ClockDivider = FDCAN_CLOCK_DIV1,/**< PDIV := 0000b (/1)         */
    .FrameFormat = FDCAN_FRAME_CLASSIC,
    .Mode = FDCAN_MODE_NORMAL,
    .AutoRetransmission = ENABLE,
    .TransmitPause = DISABLE,
    .ProtocolException = ENABLE,
                                     /*  170MHz/NBRP = 1/34MHz tq    */
    .NominalPrescaler = 5,           /*  NBRP   := /5                */
    .NominalSyncJumpWidth = 8,       /*  NSJW   := 8                 */
    .NominalTimeSeg1 = 23,           /*  NTSEG1 := 23                */
    .NominalTimeSeg2 = 10,           /*  NTSEG2 := 10                */
                                     /* [1+23+10]tq = 1MBit(+/-8tq)  */

                                     /*  170MHz/DBRP = 1/170MHz tq   */
    .DataPrescaler = 1,              /*  DBRP   := /1                */
    .DataSyncJumpWidth = 8,          /*  DSJW   := 8                 */
    .DataTimeSeg1 = 11,              /*  DTSEG1 := 11                */
    .DataTimeSeg2 = 9,               /*  DTSEG2 := 9                 */
                                     /* [1+11+9]tq = 8.1MBit(+/-8tq) */
                                     /* 123.5ns bit time ~1.2% error */
    .StdFiltersNbr = 28,
    .ExtFiltersNbr = 0,
    .TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION
  }
};

/** ---------------------------------------------------------------------------
 * @internal
 */
void FDCAN2_IT1_IRQHandler(void) {
  ctl_enter_isr();
  HAL_FDCAN_IRQHandler(&htbus);
  ctl_exit_isr();
}
void FDCAN2_IT0_IRQHandler(void) {
  ctl_enter_isr();
  HAL_FDCAN_IRQHandler(&htbus);
  ctl_exit_isr();
}

/** ---------------------------------------------------------------------------
 * @internal - called from IRQ context
 */
void htbus_TxEventFifoCallback(FDCAN_HandleTypeDef *htbus, uint32_t TxEventFifoITs)
{
  (void)ctl_message_queue_post_nb(&codes, (void*)CODE_TXEFIFO);
}

/** ---------------------------------------------------------------------------
 * @internal - called from IRQ context
 */
void htbus_RxFifo0Callback(FDCAN_HandleTypeDef *hnd, uint32_t RxFifo0ITs)
{
  assert(hnd == &htbus);

  /* drain hardware fifo into rxq */
  if((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) || (RxFifo0ITs & FDCAN_IT_RX_FIFO0_FULL)) {
    uint32_t level = HAL_FDCAN_GetRxFifoFillLevel(hnd, FDCAN_RX_FIFO0); assert(level <= 3);
    while(level--) {
      canframe_t * const pframe = (canframe_t*)ctl_memory_area_allocate(&frames);
      if(!pframe) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_RXFIFO0_ALLOC);
        break;
      }
      if(HAL_FDCAN_GetRxMessage(hnd, FDCAN_RX_FIFO0, &(pframe->rxhdr), &(pframe->data.as_uint8[0])) != HAL_OK) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_RXFIFO0);
        ctl_memory_area_free(&frames, (unsigned*)pframe); /**< avoid leak! */
        break;
      }
      if(!ctl_message_queue_post_nb(&rxq, (void*)pframe)) {
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_RXQFULL);
        ctl_memory_area_free(&frames, (unsigned*)pframe); /**< avoid leak! */
        break;
      }
    }
  }

  if(RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) {
    /* signal: likely to occur only when we're debugging and the FDCAN  */
    /* peripheral continues to work away in the background - should not */
    /* happen in the day-to-day, but want to catch regardless           */
    (void)ctl_message_queue_post_nb(&codes, (void*)CODE_FRAMELOST);
  }
}

/** ---------------------------------------------------------------------------
 * @internal - called from IRQ context
 */
void htbus_RxFifo1Callback(FDCAN_HandleTypeDef *htbus, uint32_t RxFifo1ITs)
{
  /* maybe assign/filter for future keypads/canopen-stack? */
  (void)ctl_message_queue_post_nb(&codes, (void*)CODE_RXFIFO1);
}

/** ---------------------------------------------------------------------------
 * @internal - called from IRQ context
 */
void htbus_TxBufferCompleteCallback(FDCAN_HandleTypeDef *htbus, uint32_t BufferIndexes)
{
  unsigned level = HAL_FDCAN_GetTxFifoFreeLevel(htbus);
  while(level--) {    /**< always top off FIFO */
    canframe_t *pframe = 0;
    if(ctl_message_queue_receive_nb(&txq, (void**)&pframe)) {
      assert(pframe);
      if(HAL_FDCAN_AddMessageToTxFifoQ(htbus, &(pframe->txhdr), &(pframe->data.as_uint8[0])) != HAL_OK)
        (void)ctl_message_queue_post_nb(&codes, (void*)CODE_TXFIFO);
      ctl_memory_area_free(&frames, (unsigned*)pframe); /**< avoid leak! */
    }
    else break; /* maybe that was the last thing in the queue ¯\_(ツ)_/¯ */
  }
  ctl_events_pulse(&e, EVT_HTBUS_TXQWD);
}

/** ---------------------------------------------------------------------------
 * @internal - called from IRQ context
 */
void htbus_TxBufferAbortCallback(FDCAN_HandleTypeDef *htbus, uint32_t BufferIndexes)
{
  (void)ctl_message_queue_post_nb(&codes, (void*)CODE_TXBUF_ABT);
}

/** ---------------------------------------------------------------------------
 * @internal - called from IRQ context
 */
void htbus_ErrorStatusCallback(FDCAN_HandleTypeDef *htbus, uint32_t ErrorStatusITs)
{
  if(ErrorStatusITs & FDCAN_IT_ERROR_PASSIVE)
    (void)ctl_message_queue_post_nb(&codes, (void*)CODE_ERR_PASSIVE);

  if(ErrorStatusITs & FDCAN_IT_ERROR_WARNING)
    (void)ctl_message_queue_post_nb(&codes, (void*)CODE_ERR_WARNING);

  if(ErrorStatusITs & FDCAN_IT_BUS_OFF)
    (void)ctl_message_queue_post_nb(&codes, (void*)CODE_BUS_OFF);
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief place (or queue) a canframe on the htbus - NONBLOCKING
 * @retval non-zero upon success
 *
 * @note on success, it's implied that the caller gives up ownership of the frame
 */
unsigned htbus_send(canframe_t *pframe)
{
  assert(pframe);
  assert(pframe->txhdr.FDFormat == FDCAN_FRAME_CLASSIC);
  assert(pframe->txhdr.IdType == FDCAN_STANDARD_ID);
  assert(pframe->txhdr.DataLength <= FDCAN_DLC_BYTES_8);

  HAL_StatusTypeDef const status =
    HAL_FDCAN_AddMessageToTxFifoQ(&htbus, &(pframe->txhdr), &(pframe->data.as_uint8[0]));

  if(status == HAL_OK) {                                  /**< made it into hardware FIFO */
    ctl_memory_area_free(&frames, (unsigned*)pframe);     /**< return frame to pool       */
    return ~0;
  }

  unsigned const err = HAL_FDCAN_GetError(&htbus);
  if(err & HAL_FDCAN_ERROR_FIFO_FULL)
    if(ctl_message_queue_post_nb(&txq, pframe)) {         /**< made it into queue         */
      return ~0;
    }

  (void)ctl_message_queue_post_nb(&codes, (void*)CODE_TXQFULL);
  /* queue full - this is a pretty exceptional situation,               */
  /* it's possible to be real, or maybe there is a problem on the bus   */
  /*                                                                    */
  /* I think the right thing to do is depend on a watchdog task that is */
  /* periodically tripped to check the state of the txq and if it's     */
  /* full, but a TxBufferCompleteCallback hasn't occured in a while     */
  /* then flush the txq and lets let things attempt to heal themselves  */
  return 0;
}

/**
  * @brief  Initializes the FDCAN2 bus.
  * @param  htbus pointer to an FDCAN_HandleTypeDef structure that contains
  *         the configuration information for the specified FDCAN.
  * @retval None
  */
void htbus_init(FDCAN_HandleTypeDef *htbus)
{
  assert(htbus);
  ErrorStatus e = ERROR;
  LL_GPIO_InitTypeDef pin;
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

  /* PB.5  - FDCAN2_RX (HALTECH BUS) [AF9] */
  LL_GPIO_StructInit(&pin);
  pin.Mode      = LL_GPIO_MODE_ALTERNATE;
  pin.Alternate = LL_GPIO_AF_9;
  pin.Pin       = LL_GPIO_PIN_5;
  pin.Pull      = LL_GPIO_PULL_UP;
  e = LL_GPIO_Init(GPIOB, &pin);
  assert(e == SUCCESS);

  /* PB.6  - FDCAN2_TX (HALTECH BUS) [AF9] */
  LL_GPIO_StructInit(&pin);
  pin.Mode      = LL_GPIO_MODE_ALTERNATE;
  pin.Alternate = LL_GPIO_AF_9;
  pin.Pin       = LL_GPIO_PIN_6;
  pin.Speed     = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  pin.Pull      = LL_GPIO_PULL_NO;
  e = LL_GPIO_Init(GPIOB, &pin);
  assert(e == SUCCESS);

  NVIC_SetPriority(FDCAN2_IT0_IRQn, IRQPRI_FDCAN);
  NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
//NVIC_SetPriority(FDCAN2_IT1_IRQn, IRQPRI_FDCAN);
//NVIC_EnableIRQ(FDCAN2_IT1_IRQn);
}

/** ---------------------------------------------------------------------------
 * @internal
 * @brief handles everything fdcan (queues/errors/irqs/etc)
 *
 * @note Basically, the FDCAN HAL_xxxx api is not threadsafe, so EVERYTHING
 *       needs to be done from thread context (for mutexes to properly guard
 *       the underlying hardware state too). Even running the IRQ handler needs
 *       to be done here.
 *
 *       The only thing the IRQ is used for is to kickoff this thread.
 */
static __NO_RETURN void
  task_htbus(void *arg)
{
  CTL_TIME_t const to = (CTL_TIME_t)arg; assert(to >= 500);
  HAL_StatusTypeDef result;
  HAL_FDCAN_StateTypeDef state;

  __HAL_FDCAN_RESET_HANDLE_STATE(&htbus);
  result = HAL_FDCAN_RegisterCallback(&htbus, HAL_FDCAN_MSPINIT_CB_ID, htbus_init);            assert(result == HAL_OK);
  result = HAL_FDCAN_Init(&htbus);                                                             assert(result == HAL_OK);
  state  = HAL_FDCAN_GetState(&htbus);                                                         assert(state  == HAL_FDCAN_STATE_READY);
  result = HAL_FDCAN_RegisterTxEventFifoCallback(&htbus, htbus_TxEventFifoCallback);           assert(result == HAL_OK);
  result = HAL_FDCAN_RegisterRxFifo0Callback(&htbus, htbus_RxFifo0Callback);                   assert(result == HAL_OK);
  result = HAL_FDCAN_RegisterRxFifo1Callback(&htbus, htbus_RxFifo1Callback);                   assert(result == HAL_OK);
  result = HAL_FDCAN_RegisterErrorStatusCallback(&htbus, htbus_ErrorStatusCallback);           assert(result == HAL_OK);
  result = HAL_FDCAN_RegisterTxBufferAbortCallback(&htbus, htbus_TxBufferAbortCallback);       assert(result == HAL_OK);
  result = HAL_FDCAN_RegisterTxBufferCompleteCallback(&htbus, htbus_TxBufferCompleteCallback); assert(result == HAL_OK);
  result = HAL_FDCAN_EnableEdgeFiltering(&htbus);                                              assert(result == HAL_OK);

  result = HAL_FDCAN_ConfigGlobalFilter(&htbus, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
  assert(result == HAL_OK); /* FUTURE: tighten up filters once arch is worked out */

  result = HAL_FDCAN_ConfigInterruptLines(&htbus,
    FDCAN_IT_GROUP_RX_FIFO0 | FDCAN_IT_GROUP_RX_FIFO1 | FDCAN_IT_GROUP_SMSG | FDCAN_IT_GROUP_TX_FIFO_ERROR |
    FDCAN_IT_GROUP_MISC | FDCAN_IT_GROUP_BIT_LINE_ERROR | FDCAN_IT_GROUP_PROTOCOL_ERROR, FDCAN_INTERRUPT_LINE0);
  assert(result == HAL_OK); /* FUTURE: narrow this down once arch is worked out */

  result = HAL_FDCAN_ActivateNotification(&htbus,
    FDCAN_IT_TX_COMPLETE | FDCAN_IT_TX_ABORT_COMPLETE | FDCAN_IT_TX_FIFO_EMPTY | FDCAN_IT_RX_HIGH_PRIORITY_MSG |
    FDCAN_IT_TIMESTAMP_WRAPAROUND | FDCAN_IT_TIMEOUT_OCCURRED | FDCAN_IT_TX_EVT_FIFO_ELT_LOST | FDCAN_IT_TX_EVT_FIFO_FULL |
    FDCAN_IT_TX_EVT_FIFO_NEW_DATA | FDCAN_IT_RX_FIFO0_MESSAGE_LOST | FDCAN_IT_RX_FIFO0_FULL | FDCAN_IT_RX_FIFO0_NEW_MESSAGE |
    FDCAN_IT_RX_FIFO1_MESSAGE_LOST | FDCAN_IT_RX_FIFO1_FULL | FDCAN_IT_RX_FIFO1_NEW_MESSAGE | FDCAN_IT_RAM_ACCESS_FAILURE |
    FDCAN_IT_ERROR_LOGGING_OVERFLOW | FDCAN_IT_RAM_WATCHDOG | FDCAN_IT_ARB_PROTOCOL_ERROR | FDCAN_IT_DATA_PROTOCOL_ERROR |
    FDCAN_IT_RESERVED_ADDRESS_ACCESS | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING | FDCAN_IT_BUS_OFF,
    FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2);
  assert(result == HAL_OK); /* FUTURE: narrow this down once arch is worked out */

  ctl_message_queue_setup_events(&rxq, &e, EVT_HTBUS_RXQ_NOTEMPTY, EVT_NONE);
  result = HAL_FDCAN_Start(&htbus); assert(result == HAL_OK);
  while(~0) {
    unsigned const evt =
      ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS, &e, EVT_HTBUS_RXQ_NOTEMPTY | EVT_HTBUS_TXQWD, CTL_TIMEOUT_DELAY, to);

    if(evt & EVT_HTBUS_RXQ_NOTEMPTY) {
      canframe_t * pframe;
      while(ctl_message_queue_receive_nb(&rxq, (void**)&pframe)) {
        assert(pframe);
        filter(pframe);
        ctl_memory_area_free(&frames, (unsigned*)pframe);
      }
    }

    if(evt == EVT_NONE) {                                               /**< timeout?                 */
      (void)ctl_message_queue_post_nb(&codes, (void*)CODE_TXTIMEOUT);   /* assume the worst and flush */
      if(HAL_FDCAN_Stop(&htbus) == HAL_OK) {
        canframe_t * pframe;
        while(ctl_message_queue_receive_nb(&txq, (void**)&pframe)) {
          assert(pframe);
          ctl_memory_area_free(&frames, (unsigned*)pframe);
        }
        (void)HAL_FDCAN_Start(&htbus);
      }
    }
  }
}