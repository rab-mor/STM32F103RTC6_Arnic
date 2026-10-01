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


void link_put_f0(uint8_t *payload, uint8_t off, uint16_t f0_centi_hz)
{
	put_u16(&payload[off], f0_centi_hz);
}

uint16_t link_get_f0(const uint8_t *payload, uint8_t off)
{
	return get_u16(&payload[off]);
}


void link_encode_ota_req(uint8_t *payload, const link_ota_req_t *r)
{
	memset(payload, 0, F10COMM_PAYLOAD_SIZE);
	uint8_t n = r->len;
	if (n > LINK_OTA_DATA_MAX) n = LINK_OTA_DATA_MAX;
	payload[0] = r->op;
	payload[1] = r->op_seq;
	payload[2] = n;
	put_u32(&payload[4], r->offset);
	memcpy(&payload[8], r->data, n);
}

int link_decode_ota_req(const uint8_t *payload, link_ota_req_t *r)
{
	r->op     = payload[0];
	r->op_seq = payload[1];
	r->len    = payload[2];
	r->offset = get_u32(&payload[4]);
	if (r->len > LINK_OTA_DATA_MAX) return -1;
	memcpy(r->data, &payload[8], LINK_OTA_DATA_MAX);
	return 0;
}

void link_encode_ota_status(uint8_t *payload, const link_ota_status_t *s)
{
	memset(payload, 0, F10COMM_PAYLOAD_SIZE);
	payload[0] = s->op_seq;
	payload[1] = s->result;
	payload[2] = s->state;
	payload[3] = s->board_type;
	put_u32(&payload[4], s->next_offset);
	put_u16(&payload[8], s->fw_version);
}

void link_decode_ota_status(const uint8_t *payload, link_ota_status_t *s)
{
	s->op_seq      = payload[0];
	s->result      = payload[1];
	s->state       = payload[2];
	s->board_type  = payload[3];
	s->next_offset = get_u32(&payload[4]);
	s->fw_version  = get_u16(&payload[8]);
}


void link_encode_harm(uint8_t *p, const link_harm_t *h)
{
	put_u16(&p[0], h->h1_ma);
	for (uint8_t k = 0u; k < LINK_HARM_RATIOS; k++) {
		put_u16(&p[2u + 2u * k], h->ratio[k]);
		p[18u + k] = (uint8_t)h->phase[k];
	}
}

void link_decode_harm(const uint8_t *p, link_harm_t *h)
{
	h->h1_ma = get_u16(&p[0]);
	for (uint8_t k = 0u; k < LINK_HARM_RATIOS; k++) {
		h->ratio[k] = get_u16(&p[2u + 2u * k]);
		h->phase[k] = (int8_t)p[18u + k];
	}
}

void link_encode_harm_part(uint8_t *payload, const link_harm_part_t *hp)
{
	memset(payload, 0, F10COMM_PAYLOAD_SIZE);
	payload[0] = hp->set_id;
	payload[1] = hp->part;
	put_u16(&payload[2], hp->f0_centi_hz);
	for (uint8_t i = 0u; i < LINK_HARM_RELAYS_PER_PART; i++) {
		link_encode_harm(&payload[4u + LINK_HARM_REC_BYTES * i], &hp->rec[i]);
	}
}

int link_decode_harm_part(const uint8_t *payload, link_harm_part_t *hp)
{
	hp->set_id      = payload[0];
	hp->part        = payload[1];
	hp->f0_centi_hz = get_u16(&payload[2]);
	if (hp->part >= LINK_HARM_PARTS) return -1;
	for (uint8_t i = 0u; i < LINK_HARM_RELAYS_PER_PART; i++) {
		link_decode_harm(&payload[4u + LINK_HARM_REC_BYTES * i], &hp->rec[i]);
	}
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
