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
/* Frame types. Master (H7) -> slave: POLL, COMMAND, OTA. Slave -> master:
 * METER_DATA, OTA_STATUS, HARMONICS. A slave that does not know a type
 * ignores it and answers with METER_DATA, so older F103 builds keep working. */
typedef enum {
	F10_TYPE_POLL 		= 0u,   /* payload: u16 f0_centi_hz at LINK_F0_OFF_POLL      */
	F10_TYPE_COMMAND 	= 1u,   /* payload: link_cmd_t, u16 f0 at LINK_F0_OFF_COMMAND */
	F10_TYPE_METER_DATA = 2u,   /* payload: link_status_t                              */
	F10_TYPE_OTA        = 3u,   /* payload: link_ota_req_t                             */
	F10_TYPE_OTA_STATUS = 4u,   /* payload: link_ota_status_t                          */
	F10_TYPE_HARMONICS  = 5u    /* payload: link_harm_part_t                           */
} F10Comm_Type_t;

/* Line frequency from the ADE9000 in 0.01 Hz (6000 = 60.00 Hz), 0 = unknown.
 * The F103 places its harmonic detectors on multiples of it. */
#define LINK_F0_OFF_POLL            0u
#define LINK_F0_OFF_COMMAND         4u


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
    uint8_t           relay_known;      /* bit n: position established      */
    uint8_t           relay_evidence;   /* bit n: corrected by current      */
    uint8_t           store_flags;      /* STORE_F_* (F103 app_shared.h)    */
} link_status_t;


/* 4 byte packet */
typedef struct {
	uint8_t  relay_idx;
	uint8_t  target;
	uint16_t cmd_seq;
} link_cmd_t;


/* ================= Firmware update (F10_TYPE_OTA / OTA_STATUS) =========
 * The H7 sends one request per frame; the F103 acts on it (f1_ota.c) before
 * it re-arms, so the status comes back in the very next frame, echoing
 * op_seq. A page erase keeps the F103 busy for up to 40 ms, during which it
 * is not armed: the H7 polls until it sees its op_seq come back.
 *
 * link_ota_req_t (payload bytes)
 *   0  op       LINK_OTA_*
 *   1  op_seq   echoed in the status
 *   2  len      data bytes (DATA)
 *   3  -        0
 *   4  offset   u32: DATA = image offset, BEGIN = image size
 *   8  data     DATA: up to LINK_OTA_DATA_MAX image bytes
 *               BEGIN: u32 crc32, u32 fw_version, u8 board_type
 *
 * link_ota_status_t (payload bytes)
 *   0  op_seq, 1 result (F1OTA_*), 2 state (F1OTA_ST_*), 3 board_type,
 *   4  next_offset u32, 8 fw_version u16 (running firmware, major << 8 | minor)
 */
#define LINK_OTA_QUERY              0u  /* just report: version, state */
#define LINK_OTA_BEGIN              1u
#define LINK_OTA_DATA               2u
#define LINK_OTA_END                3u
#define LINK_OTA_ABORT              4u
#define LINK_OTA_DATA_MAX           48u

typedef struct {
	uint8_t  op;
	uint8_t  op_seq;
	uint8_t  len;
	uint32_t offset;
	uint8_t  data[LINK_OTA_DATA_MAX];
} link_ota_req_t;

typedef struct {
	uint8_t  op_seq;
	uint8_t  result;
	uint8_t  state;
	uint8_t  board_type;
	uint32_t next_offset;
	uint16_t fw_version;
} link_ota_status_t;

/* ================= Relay harmonics (F10_TYPE_HARMONICS) ==================
 * Once a second the F103 measures h1..h9 of every relay's current over three
 * line cycles (harmonics.c) and sends the set in LINK_HARM_PARTS frames, in
 * poll slots that carry no new measurement window.
 *
 * link_harm_part_t (payload bytes)
 *   0  set_id       u8, +1 per set
 *   1  part         0..LINK_HARM_PARTS-1: relays 2*part and 2*part+1
 *   2  f0_centi_hz  u16, the frequency the detectors were placed on
 *   4  rec[2]       link_harm_t, 26 bytes each:
 *        0  h1_ma      u16  fundamental, mA RMS (0 = no current)
 *        2  ratio[8]   u16  h2..h9 RMS as a share of h1, 0.01 % units
 *       18  phase[8]   i8   h2..h9 phase minus h x (h1 phase), 360/256 degrees
 *      ratio and phase are 0 when h1 is below HARM_MIN_H1_MA.
 */
#define LINK_HARM_RATIOS            8u
#define LINK_HARM_REC_BYTES         26u
#define LINK_HARM_RELAYS_PER_PART   2u
#define LINK_HARM_PARTS             (LINK_RELAY_COUNT / LINK_HARM_RELAYS_PER_PART)

typedef struct {
	uint16_t h1_ma;
	uint16_t ratio[LINK_HARM_RATIOS];
	int8_t   phase[LINK_HARM_RATIOS];
} link_harm_t;

typedef struct {
	uint8_t     set_id;
	uint8_t     part;
	uint16_t    f0_centi_hz;
	link_harm_t rec[LINK_HARM_RELAYS_PER_PART];
} link_harm_part_t;

_Static_assert(4u + LINK_HARM_RELAYS_PER_PART * LINK_HARM_REC_BYTES <= 56u, "harmonics part must fit a frame");

/* 64 byte packet. payload is 54 bytes */
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
 *  53    1   relay_known      u8  (bit n = relay n's position is established:
 *                                  a pulse, the saved record or current evidence)
 *  54    1   relay_evidence   u8  (bit n = position corrected by current evidence
 *                                  since that relay's last command)
 *  55    1   store_flags      u8  (bit 0 EEPROM present, 1 restored from a record,
 *                                  2 last save failed, 3 save pending)
 *
 * Bytes 53..55 were reserved (zero) in older F103 builds, so an H7 reading an
 * old F103 sees relay_known == 0 and adopts nothing.
 */

/*-------Function Prototypes ---------*/
uint16_t F10Comm_Crc16(const uint8_t *data, size_t len);

void link_encode_status(uint8_t *payload, const link_status_t *st);
void link_encode_cmd(uint8_t *payload, const link_cmd_t *cmd);

int link_decode_status(const uint8_t *payload, link_status_t *st);
int link_decode_cmd(const uint8_t *payload, link_cmd_t *cmd);

void     link_put_f0(uint8_t *payload, uint8_t off, uint16_t f0_centi_hz);
uint16_t link_get_f0(const uint8_t *payload, uint8_t off);

void link_encode_ota_req(uint8_t *payload, const link_ota_req_t *r);
int  link_decode_ota_req(const uint8_t *payload, link_ota_req_t *r);
void link_encode_ota_status(uint8_t *payload, const link_ota_status_t *s);
void link_decode_ota_status(const uint8_t *payload, link_ota_status_t *s);

/* One 26-byte record; shared with the RS485 encoder's layout. */
void link_encode_harm(uint8_t *p, const link_harm_t *h);
void link_decode_harm(const uint8_t *p, link_harm_t *h);
void link_encode_harm_part(uint8_t *payload, const link_harm_part_t *hp);
int  link_decode_harm_part(const uint8_t *payload, link_harm_part_t *hp);


#endif /* LINK_PROTO_H */
