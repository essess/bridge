/**
 * Copyright (c) 2024 Sean Stasiak. All rights reserved.
 * Developed by: Sean Stasiak <sean@atypeng.com>
 * Refer to license terms in license.txt; In the absence of such a file,
 * contact me at the above email address and I can provide you with one.
 */

#ifndef   _can_h_
#define   _can_h_

#ifdef __cplusplus
extern "C"
{
#endif
typedef struct _can_t can_t;

typedef struct _canframe_t {
  unsigned id;
  unsigned dlc;
  unsigned extended;
  union {
    uint8_t  as_uint8[8];
    uint32_t as_uint32[2];
  } data;
} canframe_t;

typedef enum _canid_t {
  CANID_FDCAN1 = 0,
  CANID_FDCAN2 = 1,
  CANID_FDCAN3 = 2
} canid_t;

/*! assert helper */
static inline int
  is_canid_t(canid_t t)
{
  return((t == CANID_FDCAN1) || (t == CANID_FDCAN1) || (t == CANID_FDCAN1));
}

/** -------------------------------------------------------------------------
 * @public
 * @brief init selected controller
 */
can_t *can_init(canid_t id);
void can_init_cb(void);               /**< _init() callback */
int can_init_byid_cb(canid_t id);     /**< _init() callback */

/** -------------------------------------------------------------------------
 * @public
 * @brief take controller out of init and resync to bus
 */
void can_start(can_t *can);

/** -------------------------------------------------------------------------
 * @public
 * @brief non-blocking poll for an available frame
 *        returns non-zero on success
 */
int can_in_rxf0(can_t *can, canframe_t *frame);

/** -------------------------------------------------------------------------
 * @public
 * @brief any room in the tx fifo?
 *        returns non-zero if room available
 */
int can_peek(can_t *can);

/** -------------------------------------------------------------------------
 * @public
 * @brief non-blocking send of supplied frame
 *        returns non-zero on success
 */
int can_out(can_t *can, canframe_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* _can_h_ */