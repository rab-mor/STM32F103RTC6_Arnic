#ifndef LINK_PROTO_H
#define LINK_PROTO_H
/**
 * @file  link_proto.h
 * @brief Wire format for the H755 <-> F103 SPI link. Shared verbatim by both
 *        projects. No HAL, no RTOS, no project-local includes.
 *
 * Contract obligations on the master (H7):
 *   - Hold CS high >= 50 us between transactions so the slave can re-arm DMA.
 *   - A frame failing the magic check means "slave was not armed", not
 *     "slave is broken". Re-poll. Escalate only after several consecutive.
 *   - A repeated frame_seq means the slave had no new frame built. Discard,
 *     do not treat as an error.
 *
 * Two independent sequence spaces, do not conflate:
 *   - frame_seq (u8, header)  : link-level, detects duplicate/stale frames.
 *   - cmd_seq   (u16, payload): command-level, matches relay acks.
 *
 *
 *
 *  Note: there will be macros in this header that are common to other source files. These are completely isolated.
 */
/*--------std libraries ----------*/
#include <stdint.h>
#include <stddef.h>

/*---------macros-----------------*/
#define F10COMM_MAGIC 			0xA55Au
#define F10COMM_PROTO_VERSION   1u
#define F10COMM_FRAME_SIZE      64u
#define F10COMM_PAYLOAD_SIZE    56u
#define F10COMM_CRC_SPAN  (F10COMM_FRAME_SIZE - sizeof(uint16_t))


#define LINK_RELAY_COUNT            8u
#define LINK_MAX_EVENTS_PER_FRAME   3u

/*---------enums-----------*/
typedef enum {
	F10_TYPE_POLL 		= 0u,
	F10_TYPE_COMMAND 	= 1u,
	F10_TYPE_METER_DATA = 2u
} F10Comm_Type_t;


typedef enum {
	LINK_EVENT_ACCEPTED = 0u, /* Command queued, coil not yet fired */
	LINK_EVENT_VERIFIED = 1u, 	/* queue full &|| badindex &|| busy  */
	LINK_EVENT_REJECTED = 2u,	/* current confirms new state */
	LINK_EVENT_FAULT 	= 3u		/* coil stuck, verification timed out */
} link_event_type_t;


/*---------structs-----------*/
typedef struct {
    uint8_t  type;
    uint8_t  relay_idx;
    uint16_t cmd_seq;
    int32_t  measured_ma;
} link_wire_event_t;


typedef struct {
    uint32_t          window_id;
    int16_t           relay_ma[LINK_RELAY_COUNT];
    uint16_t          rail_24v_mv;
    uint16_t          rail_5v_mv;
    uint8_t           relay_state;
    uint8_t           fault_flags;
    uint16_t          dropped_events;
    uint8_t           event_count;
    link_wire_event_t events[LINK_MAX_EVENTS_PER_FRAME];
} link_status_t;


typedef struct {
	uint8_t  relay_idx;
	uint8_t  target;
	uint16_t cmd_seq;
} link_cmd_t;


typedef struct {
	uint16_t magic;
	uint8_t  version;
	uint8_t  type;
	uint8_t  seq;               /* frame_seq */
	uint8_t  reserved;
	uint8_t  payload[F10COMM_PAYLOAD_SIZE];
	uint16_t crc16;
} F10Comm_Frame_t;

_Static_assert(sizeof(F10Comm_Frame_t) == 64, "frame layout changed");

/* ================= Payload: status (slave -> master) =================
 * (bytes)
 * off  size  field
 *   0    4   window_id        u32
 *   4   16   relay_ma[8]      i16
 *  20    2   rail_24v_mv      u16
 *  22    2   rail_5v_mv       u16
 *  24    1   relay_state      u8  (bit n = relay n energized)
 *  25    1   fault_flags      u8
 *  26    2   dropped_events   u16 (cumulative)
 *  28    1   event_count      u8  (0..3)
 *  29   24   events[3]        8 bytes each
 *  53    3   reserved
 */

/*-------Function Prototypes ---------*/
uint16_t F10Comm_Crc16(const uint8_t *data, size_t len);

void link_encode_status(uint8_t *payload, const link_status_t *st);
void link_encode_cmd(uint8_t *payload, const link_cmd_t *cmd);

int link_decode_status(const uint8_t *payload, link_status_t *st);
int link_decode_cmd(const uint8_t *payload, link_cmd_t *cmd);


#endif /* LINK_PROTO_H */
