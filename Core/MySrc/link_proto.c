#include "link_proto.h"
#include <string.h>



static inline void put_u16(uint8_t *p, uint16_t v) {
	p[0] = (uint8_t)(v);
	p[1] = (uint8_t)(v >> 8);
}


static inline void put_u32(uint8_t *p, uint32_t v) {
	p[0] = (uint8_t)(v);
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16);
	p[3] = (uint8_t)( v >> 24);
}


static inline uint16_t get_u16(const uint8_t *p)  {
	return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}


static inline uint32_t get_u32(const uint8_t *p) {
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}


void link_encode_status(uint8_t *payload, const link_status_t *st) {

	memset(payload, 0, F10COMM_PAYLOAD_SIZE);
	put_u32(&payload[0], st->window_id);

	for(uint8_t i = 0; i < LINK_RELAY_COUNT; i++) {
		put_u16(&payload[4 + (i * 2u)], (uint16_t)st->relay_ma[i]);	 // put the current readings into our payload indexing by 2 bytes at a time
	}

	put_u16(&payload[20], st->rail_24v_mv);
	put_u16(&payload[22], st->rail_5v_mv);
	payload[24] = st->relay_state;
	payload[25] = st->fault_flags;
	put_u16(&payload[26], st->dropped_events);

	uint8_t n = st->event_count;
	if(n > LINK_MAX_EVENTS_PER_FRAME) n = LINK_MAX_EVENTS_PER_FRAME;
	payload[28] = n;

	for (uint8_t i = 0u; i < n; i++) {
	        uint8_t *e = &payload[29 + (i * 8u)];
	        e[0] = st->events[i].type;
	        e[1] = st->events[i].relay_idx;
	        put_u16(&e[2], st->events[i].cmd_seq);
	        put_u32(&e[4], (uint32_t)st->events[i].measured_ma);
	    }

	payload[53] = st->relay_known;
	payload[54] = st->relay_evidence;
	payload[55] = st->store_flags;
}


void link_encode_cmd(uint8_t *payload, const link_cmd_t *cmd) {
	memset(payload, 0, F10COMM_PAYLOAD_SIZE);
	payload[0] = cmd->relay_idx;
	payload[1] = cmd->target;
	put_u16(&payload[2], cmd->cmd_seq);
}



int link_decode_status(const uint8_t *payload, link_status_t *st) {

	st->window_id = get_u32(&payload[0]);

	for(uint8_t i = 0; i < LINK_RELAY_COUNT; i++) {
        st->relay_ma[i] = (int16_t)get_u16(&payload[4 + (i * 2u)]);
	}
	/* Load the rest of our packet. */
	st->rail_24v_mv    = get_u16(&payload[20]);
	st->rail_5v_mv     = get_u16(&payload[22]);
	st->relay_state    = payload[24];
	st->fault_flags    = payload[25];
	st->dropped_events = get_u16(&payload[26]);
	st->relay_known    = payload[53];
	st->relay_evidence = payload[54];
	st->store_flags    = payload[55];

    const uint8_t n = payload[28];
	if(n > LINK_MAX_EVENTS_PER_FRAME) {
		st->event_count = 0u; return -1;
	}

	st->event_count = n;
	for(uint8_t i = 0u; i < n; i++) {
		const uint8_t *e = &payload[29 + (i * 8u)];
		st->events[i].type        = e[0];
		st->events[i].relay_idx   = e[1];
		st->events[i].cmd_seq     = get_u16(&e[2]);
		st->events[i].measured_ma = (int32_t)get_u32(&e[4]);
	}
	return 0;
}


int link_decode_cmd(const uint8_t *payload, link_cmd_t *cmd) {
	cmd->relay_idx = payload[0];
	cmd->target    = payload[1];
	cmd->cmd_seq   = get_u16(&payload[2]);

	if (cmd->relay_idx >= LINK_RELAY_COUNT) return -1;
	if (cmd->target > 1u)                   return -1;

	return 0;
}


uint16_t F10Comm_Crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;

    for (size_t i = 0u; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0u; bit < 8u; bit++) {
            if (crc & 0x8000u)  {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            	} else {
            	crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}
