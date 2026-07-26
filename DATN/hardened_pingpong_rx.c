#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "delay.h"
#include "timer.h"
#include "radio.h"
#include "tremo_system.h"
#include "tremo_uart.h"

#if defined( REGION_AS923 )
// ... keep freq defs ...
#define RF_FREQUENCY                                923000000
#elif defined( REGION_AU915 )
#define RF_FREQUENCY                                915000000
#elif defined( REGION_CN470 )
#define RF_FREQUENCY                                470000000
#elif defined( REGION_CN779 )
#define RF_FREQUENCY                                779000000
#elif defined( REGION_EU433 )
#define RF_FREQUENCY                                433000000
#elif defined( REGION_EU868 )
#define RF_FREQUENCY                                868000000
#elif defined( REGION_KR920 )
#define RF_FREQUENCY                                920000000
#elif defined( REGION_IN865 )
#define RF_FREQUENCY                                865000000
#elif defined( REGION_US915 )
#define RF_FREQUENCY                                915000000
#elif defined( REGION_US915_HYBRID )
#define RF_FREQUENCY                                915000000
#else
    #error "Please define a frequency band in the compiler options."
#endif

#define TX_OUTPUT_POWER                             14

#if defined( USE_MODEM_LORA )
#define LORA_BANDWIDTH                              0
#define LORA_SPREADING_FACTOR                       7
#define LORA_CODINGRATE                             1
#define LORA_PREAMBLE_LENGTH                        8
#define LORA_SYMBOL_TIMEOUT                         0
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false
#elif defined( USE_MODEM_FSK )
#define FSK_FDEV                                    25000
#define FSK_DATARATE                                50000
#define FSK_BANDWIDTH                               50000
#define FSK_AFC_BANDWIDTH                           83333
#define FSK_PREAMBLE_LENGTH                         5
#define FSK_FIX_LENGTH_PAYLOAD_ON                   false
#else
    #error "Please define a modem in the compiler options."
#endif

typedef enum { LOWPOWER, RX, RX_TIMEOUT, RX_ERROR, TX, TX_TIMEOUT } States_t;

#define RX_TIMEOUT_VALUE        3000
#define BUFFER_SIZE             255
#define CHUNK_MAX               230

#define MAX_JUMP_THRESHOLD          20000  // Cho phép gap lớn hơn trong mạng yếu
#define ANTI_REPLAY_WINDOW_SEC      300
#define CLOCK_SKEW_TOLERANCE_SEC    5
#define MAX_NODES                   10
#define WINDOW_SIZE 32  // Khớp với 32-bit bitmap (uint32_t)

#define UART_INST UART0
#define UART_BAUD 115200

#define SECRET_KEY_LEN 16
static const uint8_t SECRET_KEY[SECRET_KEY_LEN] = {
    0x13,0x37,0xAA,0x55,0x99,0x42,0xDE,0xAD,
    0xBE,0xEF,0x77,0x21,0x66,0x88,0xCC,0x10
};

// ===== STANDALONE COMPACT HMAC-SHA256 (TRUNCATED 4-BYTE) =====
static const uint32_t K256[64] = {
    0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
    0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
    0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
    0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
    0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
    0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
    0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
    0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2
};

#define ROTR(x,n) (((x) >> (n)) | ((x) << (32 - (n))))
#define CH(x,y,z) (((x) & (y)) ^ (~(x) & (z)))
#define MAJ(x,y,z) (((x) & (y)) ^ ((x) & (z)) ^ ((y) & (z)))
#define EP0(x) (ROTR(x,2) ^ ROTR(x,13) ^ ROTR(x,22))
#define EP1(x) (ROTR(x,6) ^ ROTR(x,11) ^ ROTR(x,25))
#define SIG0(x) (ROTR(x,7) ^ ROTR(x,18) ^ ((x) >> 3))
#define SIG1(x) (ROTR(x,17) ^ ROTR(x,19) ^ ((x) >> 10))

typedef struct {
    uint32_t state[8];
    uint64_t count;
    uint8_t buffer[64];
} SHA256_CTX;

static void sha256_init(SHA256_CTX *ctx) {
    ctx->state[0] = 0x6a09e667; ctx->state[1] = 0xbb67ae85;
    ctx->state[2] = 0x3c6ef372; ctx->state[3] = 0xa54ff53a;
    ctx->state[4] = 0x510e527f; ctx->state[5] = 0x9b05688c;
    ctx->state[6] = 0x1f83d9ab; ctx->state[7] = 0x5be0cd19;
    ctx->count = 0;
}

static void sha256_transform(SHA256_CTX *ctx, const uint8_t data[64]) {
    uint32_t a, b, c, d, e, f, g, h, i, j, t1, t2, m[64];
    for (i = 0, j = 0; i < 16; ++i, j += 4)
        m[i] = (data[j] << 24) | (data[j + 1] << 16) | (data[j + 2] << 8) | (data[j + 3]);
    for (; i < 64; ++i)
        m[i] = SIG1(m[i - 2]) + m[i - 7] + SIG0(m[i - 15]) + m[i - 16];

    a = ctx->state[0]; b = ctx->state[1]; c = ctx->state[2]; d = ctx->state[3];
    e = ctx->state[4]; f = ctx->state[5]; g = ctx->state[6]; h = ctx->state[7];

    for (i = 0; i < 64; ++i) {
        t1 = h + EP1(e) + CH(e, f, g) + K256[i] + m[i];
        t2 = EP0(a) + MAJ(a, b, c);
        h = g; g = f; f = e; e = d + t1;
        d = c; c = b; b = a; a = t1 + t2;
    }
    ctx->state[0] += a; ctx->state[1] += b; ctx->state[2] += c; ctx->state[3] += d;
    ctx->state[4] += e; ctx->state[5] += f; ctx->state[6] += g; ctx->state[7] += h;
}

static void sha256_update(SHA256_CTX *ctx, const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
        ctx->buffer[ctx->count % 64] = data[i];
        ctx->count++;
        if (ctx->count % 64 == 0)
            sha256_transform(ctx, ctx->buffer);
    }
}

static void sha256_final(SHA256_CTX *ctx, uint8_t digest[32]) {
    uint32_t i = ctx->count % 64;
    ctx->buffer[i++] = 0x80;
    if (i > 56) {
        while (i < 64) ctx->buffer[i++] = 0x00;
        sha256_transform(ctx, ctx->buffer);
        i = 0;
    }
    while (i < 56) ctx->buffer[i++] = 0x00;
    uint64_t bits = ctx->count * 8;
    for (int j = 7; j >= 0; j--) {
        ctx->buffer[56 + j] = bits & 0xFF;
        bits >>= 8;
    }
    sha256_transform(ctx, ctx->buffer);
    for (i = 0; i < 4; ++i) {
        digest[i]      = (ctx->state[0] >> (24 - i * 8)) & 0x000000ff;
        digest[i + 4]  = (ctx->state[1] >> (24 - i * 8)) & 0x000000ff;
        digest[i + 8]  = (ctx->state[2] >> (24 - i * 8)) & 0x000000ff;
        digest[i + 12] = (ctx->state[3] >> (24 - i * 8)) & 0x000000ff;
        digest[i + 16] = (ctx->state[4] >> (24 - i * 8)) & 0x000000ff;
        digest[i + 20] = (ctx->state[5] >> (24 - i * 8)) & 0x000000ff;
        digest[i + 24] = (ctx->state[6] >> (24 - i * 8)) & 0x000000ff;
        digest[i + 28] = (ctx->state[7] >> (24 - i * 8)) & 0x000000ff;
    }
}

static void hmac_sha256(const uint8_t *key, size_t key_len, const uint8_t *data, size_t data_len, uint8_t mac[32]) {
    uint8_t k_pad[64];
    uint8_t k_ipad[64];
    uint8_t k_opad[64];
    memset(k_pad, 0, sizeof(k_pad));
    if (key_len > 64) {
        SHA256_CTX ctx;
        sha256_init(&ctx);
        sha256_update(&ctx, key, key_len);
        sha256_final(&ctx, k_pad);
    } else {
        memcpy(k_pad, key, key_len);
    }
    for (int i = 0; i < 64; i++) {
        k_ipad[i] = k_pad[i] ^ 0x36;
        k_opad[i] = k_pad[i] ^ 0x5c;
    }
    uint8_t inner_hash[32];
    SHA256_CTX in_ctx;
    sha256_init(&in_ctx);
    sha256_update(&in_ctx, k_ipad, 64);
    sha256_update(&in_ctx, data, data_len);
    sha256_final(&in_ctx, inner_hash);

    SHA256_CTX out_ctx;
    sha256_init(&out_ctx);
    sha256_update(&out_ctx, k_opad, 64);
    sha256_update(&out_ctx, inner_hash, 32);
    sha256_final(&out_ctx, mac);
}

static uint32_t compute_auth_tag(uint8_t node_id, uint16_t session_id, uint32_t seq, const uint8_t* payload, uint16_t len) {
    uint8_t auth_buf[BUFFER_SIZE];
    int p = 0;
    auth_buf[p++] = node_id;
    auth_buf[p++] = (session_id >> 0) & 0xFF;
    auth_buf[p++] = (session_id >> 8) & 0xFF;
    auth_buf[p++] = (seq >> 0) & 0xFF;
    auth_buf[p++] = (seq >> 8) & 0xFF;
    auth_buf[p++] = (seq >> 16) & 0xFF;
    auth_buf[p++] = (seq >> 24) & 0xFF;
    memcpy(&auth_buf[p], payload, len); p += len;

    uint8_t mac[32];
    hmac_sha256(SECRET_KEY, SECRET_KEY_LEN, auth_buf, p, mac);

    // 4-byte Truncated HMAC-SHA256 (32-bit uint32_t)
    uint32_t tag = (uint32_t)mac[0]
                 | ((uint32_t)mac[1] << 8)
                 | ((uint32_t)mac[2] << 16)
                 | ((uint32_t)mac[3] << 24);
    return tag;
}

uint8_t Buffer[BUFFER_SIZE];
uint16_t BufferSize = 0;

static uint8_t  LoraBuf[BUFFER_SIZE];
static uint16_t LoraLen = 0;

volatile States_t State = LOWPOWER;
static volatile uint8_t  txDone = 0;
int8_t RssiValue = 0;
int8_t SnrValue = 0;
uint32_t ChipId[2] = {0};

static uint8_t LOCAL_NODE_ID = 0; // Gateway is node 0
static uint32_t tx_sequence = 0;

static RadioEvents_t RadioEvents;

static char uart_json_buffer[300];
static uint16_t uart_json_idx = 0;

static uint32_t valid_packets = 0;
static uint32_t invalid_seq = 0;
static uint32_t invalid_crc = 0;
static uint32_t invalid_jump = 0;

// === DEMO 3: Timing Tracking & Edge DoS Filter ===
static uint32_t demo_start_time = 0;
static uint32_t last_rx_time[MAX_NODES];
#define TIMED_LOG(fmt, ...) do { \
    uint32_t elapsed = TimerGetCurrentTime() - demo_start_time; \
    printf("[%7lu ms] " fmt "\r\n", (unsigned long)elapsed, ##__VA_ARGS__); \
} while(0)

typedef struct {
    uint8_t  node_id;
    uint16_t last_session;
    uint32_t last_seq;
    uint32_t bitmap;
    uint32_t packets_received;
    uint32_t packets_rejected;
    int8_t   last_rssi;
    uint32_t last_rx_timestamp;
} PerNodeState_t;

typedef struct {
    uint8_t  node_id;
    uint16_t session_id;
    uint32_t seq;
    uint16_t payload_len;
    uint8_t  payload[CHUNK_MAX];
    uint32_t hmac_tag;
    int      valid;
    char     reject_reason[80];
} ParsedPacket_t;

static PerNodeState_t node_states[MAX_NODES];
static uint8_t node_count = 0;

void OnTxDone(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void OnTxTimeout(void);
void OnRxTimeout(void);
void OnRxError(void);

static void uart_init_wrapper(uint32_t baud)
{
    uart_config_t cfg;
    uart_config_init(&cfg);

    cfg.baudrate     = baud;
    cfg.data_width   = UART_DATA_WIDTH_8;
    cfg.stop_bits    = UART_STOP_BITS_1;
    cfg.parity       = UART_PARITY_NO;
    cfg.flow_control = UART_FLOW_CONTROL_DISABLED;
    cfg.mode         = UART_MODE_TXRX;
    cfg.fifo_mode    = 1;

    uart_init(UART_INST, &cfg);
    uart_cmd(UART_INST, true);
}

static int uart_available_wrapper(void)
{
    return uart_get_flag_status(UART_INST, UART_FLAG_RX_FIFO_EMPTY) ? 0 : 1;
}

static int uart_readbyte_wrapper(void)
{
    if (!uart_available_wrapper()) return -1;
    return (int)uart_receive_data(UART_INST);
}

static const char* uart_read_json_line(uint32_t timeout_ms)
{
    uint32_t start = TimerGetCurrentTime();

    while (TimerGetElapsedTime(start) < timeout_ms) {
        int byte = uart_readbyte_wrapper();

        if (byte >= 0) {
            if (byte == '\n') {
                uart_json_buffer[uart_json_idx] = 0;

                if (uart_json_idx > 0) {
                    // Gateway UART RX is receiving ACK from ESP32
                    uart_json_idx = 0;
                    return uart_json_buffer;
                }
                uart_json_idx = 0;
            } else if (byte == '\r') {
            } else if (uart_json_idx < (sizeof(uart_json_buffer) - 1)) {
                uart_json_buffer[uart_json_idx++] = (char)byte;
            } else {
                uart_json_idx = 0;
            }
        }
    }

    return NULL;
}

static void radio_send_blocking(const uint8_t* data, uint16_t len)
{
    txDone = 0;
    Radio.Send((uint8_t*)data, len);
    while (!txDone) { Radio.IrqProcess(); }
}

static void send_enhanced_secure_packet(const uint8_t* plaintext, uint16_t plain_len) {
    if (plain_len > CHUNK_MAX) return;

    uint8_t packet[BUFFER_SIZE];
    int pkt_pos = 0;
    uint32_t current_seq = tx_sequence;
    uint16_t gateway_session = 0;

    packet[pkt_pos++] = LOCAL_NODE_ID;

    packet[pkt_pos++] = (gateway_session >> 0) & 0xFF;
    packet[pkt_pos++] = (gateway_session >> 8) & 0xFF;

    packet[pkt_pos++] = (current_seq >>  0) & 0xFF;
    packet[pkt_pos++] = (current_seq >>  8) & 0xFF;
    packet[pkt_pos++] = (current_seq >> 16) & 0xFF;
    packet[pkt_pos++] = (current_seq >> 24) & 0xFF;
    
    tx_sequence++;

    packet[pkt_pos++] = (plain_len >> 0) & 0xFF;
    packet[pkt_pos++] = (plain_len >> 8) & 0xFF;

    memcpy(&packet[pkt_pos], plaintext, plain_len);
    pkt_pos += plain_len;

    uint32_t auth = compute_auth_tag(LOCAL_NODE_ID, gateway_session, current_seq, plaintext, plain_len);
    packet[pkt_pos++] = (auth >>  0) & 0xFF;
    packet[pkt_pos++] = (auth >>  8) & 0xFF;
    packet[pkt_pos++] = (auth >> 16) & 0xFF;
    packet[pkt_pos++] = (auth >> 24) & 0xFF;

    radio_send_blocking(packet, (uint16_t)pkt_pos);
    printf("[TX SECURE] Gateway sent ACK, seq=%lu, HMAC=0x%08lX\r\n", (unsigned long)current_seq, (unsigned long)auth);
}

static uint16_t crc16_calc(const uint8_t* data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (int j = 0; j < 8; j++) {
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
            crc &= 0xFFFF;
        }
    }
    return crc;
}

static void per_node_init(void)
{
    memset(node_states, 0, sizeof(node_states));
    node_count = 0;
}

static PerNodeState_t* per_node_get_or_create(uint8_t node_id)
{
    for (uint8_t i = 0; i < node_count; i++) {
        if (node_states[i].node_id == node_id) {
            return &node_states[i];
        }
    }

    if (node_count < MAX_NODES) {
        node_states[node_count].node_id = node_id;
        node_states[node_count].last_seq = 0;
        node_states[node_count].packets_received = 0;
        node_states[node_count].packets_rejected = 0;
        node_states[node_count].last_rssi = 0;
        return &node_states[node_count++];
    }

    return NULL;
}

static int check_and_update_seq(PerNodeState_t* t, uint32_t seq)
{
    if (seq > t->last_seq) {
        uint32_t shift = seq - t->last_seq;

        if (shift > MAX_JUMP_THRESHOLD) {
            printf("[DROP] Jump too large: %lu > %lu\r\n", (unsigned long)shift, (unsigned long)MAX_JUMP_THRESHOLD);
            return 0;
        }

        if (shift >= WINDOW_SIZE) {
            t->bitmap = 0;
        } else {
            t->bitmap <<= shift;
        }

        t->bitmap |= 1;
        t->last_seq = seq;
        return 1;
    }

    uint32_t diff = t->last_seq - seq;

    if (diff >= WINDOW_SIZE) {
        printf("[DROP] Too old\r\n");
        return 0;
    }

    if (t->bitmap & (1UL << diff)) {
        printf("[DROP] Replay\r\n");
        return 0;
    }

    t->bitmap |= (1UL << diff);
    return 1;
}

static void parse_and_validate_packet(const uint8_t* raw, uint16_t raw_len, ParsedPacket_t* pkt)
{
    memset(pkt, 0, sizeof(*pkt));
    pkt->valid = 0;

    // Minimum packet size: NodeID (1B) + SessionID (2B) + Seq (4B) + PayloadLen (2B) + AuthTag (4B) = 13B
    if (raw_len < 13) {
        snprintf(pkt->reject_reason, sizeof(pkt->reject_reason), "Packet quá nhỏ (%u < 13)", raw_len);
        return;
    }

    int pos = 0;
    pkt->node_id = raw[pos++];

    pkt->session_id = (uint16_t)raw[pos+0] | ((uint16_t)raw[pos+1] << 8);
    pos += 2;

    pkt->seq = (uint32_t)raw[pos+0]
             | ((uint32_t)raw[pos+1] << 8)
             | ((uint32_t)raw[pos+2] << 16)
             | ((uint32_t)raw[pos+3] << 24);
    pos += 4;

    pkt->payload_len = (uint16_t)raw[pos+0] | ((uint16_t)raw[pos+1] << 8);
    pos += 2;

    if (pkt->payload_len > CHUNK_MAX || pos + pkt->payload_len + 4 > raw_len) {
        snprintf(pkt->reject_reason, sizeof(pkt->reject_reason), "Payload len không hợp lệ");
        return;
    }

    memcpy(pkt->payload, &raw[pos], pkt->payload_len);
    pos += pkt->payload_len;

    pkt->hmac_tag = (uint32_t)raw[pos+0]
                  | ((uint32_t)raw[pos+1] << 8)
                  | ((uint32_t)raw[pos+2] << 16)
                  | ((uint32_t)raw[pos+3] << 24);

    // BƯỚC 1: Kiểm tra HMAC-SHA256 4-byte Truncated Auth Tag
    {
        uint32_t expected = compute_auth_tag(pkt->node_id, pkt->session_id, pkt->seq, pkt->payload, pkt->payload_len);
        if (expected != pkt->hmac_tag) {
            snprintf(pkt->reject_reason, sizeof(pkt->reject_reason), "AUTH FAIL (HMAC mismatch)");
            return;
        }
    }

    // BƯỚC 2: Kiểm tra Anti-Replay, Node Reboot & Edge DoS Rate Limiting (1s)
    {
        PerNodeState_t* node = per_node_get_or_create(pkt->node_id);
        if (!node) {
            snprintf(pkt->reject_reason, sizeof(pkt->reject_reason), "Max nodes exceeded");
            return;
        }

        uint32_t now_ms = TimerGetCurrentTime();

        // 2a. Edge DoS Rate Limiting: Drop if packet interval < 1000 ms
        if (node->last_rx_timestamp > 0 && (now_ms - node->last_rx_timestamp) < 1000) {
            snprintf(pkt->reject_reason, sizeof(pkt->reject_reason), "Rate limit exceeded (DoS)");
            node->packets_rejected++;
            return;
        }

        // 2b. Kiểm tra Session ID & Sequence Anti-Replay
        if (node->packets_received == 0) {
            // Node mới xuất hiện lần đầu
            node->last_session = pkt->session_id;
            node->last_seq     = pkt->seq;
            node->bitmap       = 1;
            pkt->valid         = 1;
        }
        else if (pkt->session_id > node->last_session) {
            // Node TX mới reset/reboot! Nhận phiên mới an toàn
            TIMED_LOG("[SESSION RESET] Node %u rebooted (Old Session: %u -> New Session: %u)",
                      pkt->node_id, node->last_session, pkt->session_id);
            node->last_session = pkt->session_id;
            node->last_seq     = pkt->seq;
            node->bitmap       = 1;
            pkt->valid         = 1;
        }
        else if (pkt->session_id == node->last_session) {
            // Phiên hiện tại: Chạy Sliding Window Bitmap check
            if (check_and_update_seq(node, pkt->seq)) {
                pkt->valid = 1;
            } else {
                snprintf(pkt->reject_reason, sizeof(pkt->reject_reason), "Replay/Too old");
                node->packets_rejected++;
                return;
            }
        }
        else {
            // session_id < last_session: Gói tin cũ từ phiên trước
            snprintf(pkt->reject_reason, sizeof(pkt->reject_reason), "Stale session");
            node->packets_rejected++;
            return;
        }

        // Cập nhật chỉ số nếu hợp lệ
        if (pkt->valid) {
            // Kiểm tra biến động bất thường RSSI (RF Impersonation)
            if (node->packets_received > 1) {
                int diff_rssi = abs((int)RssiValue - (int)node->last_rssi);
                if (diff_rssi > 15) {
                    TIMED_LOG("[WARNING] Node %u RSSI shift: %d -> %d dBm (Delta: %d dB)",
                              pkt->node_id, (int)node->last_rssi, (int)RssiValue, diff_rssi);
                }
            }
            node->last_rssi = (int8_t)RssiValue;
            node->last_rx_timestamp = now_ms;
            node->packets_received++;
        }
    }
}

int app_start(void)
{
    (void)system_get_chip_id(ChipId);
    printf("Gateway Chip ID: 0x%08lX-%08lX\r\n",
           (unsigned long)ChipId[0],
           (unsigned long)ChipId[1]);
    printf("\r\n");

    per_node_init();
    memset(last_rx_time, 0, sizeof(last_rx_time));
    demo_start_time = TimerGetCurrentTime();

    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);

#if defined( USE_MODEM_LORA )
    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
#elif defined( USE_MODEM_FSK )
    Radio.SetTxConfig(MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                      FSK_DATARATE, 0,
                      FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, 0, 3000);

    Radio.SetRxConfig(MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                      0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                      0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                      0, 0, false, true);
#endif

    Radio.Rx(RX_TIMEOUT_VALUE);

    uart_init_wrapper(UART_BAUD);
    printf("Listening for Follower Nodes...\r\n");
    printf("Max nodes: %u | Max payload: %u bytes\r\n", MAX_NODES, CHUNK_MAX);
    printf("==============================================\r\n\r\n");

    static uint32_t total_accepted = 0;
    static uint32_t total_rejected = 0;

    while (1)
    {
        switch (State)
        {
        case RX:
            {
                ParsedPacket_t pkt;
                parse_and_validate_packet(LoraBuf, LoraLen, &pkt);

                if (pkt.valid) {
                    total_accepted++;
                    uint32_t now_ms = TimerGetCurrentTime();
                    uint32_t delta_ms = (last_rx_time[pkt.node_id] > 0) ? 
                        (now_ms - last_rx_time[pkt.node_id]) : 0;
                    last_rx_time[pkt.node_id] = now_ms;
                    
                    TIMED_LOG("[RX OK] node=%u, seq=%lu, len=%u, delta=%lu ms, rssi=%d dBm, snr=%d dB",
                           (unsigned)pkt.node_id,
                           (unsigned long)pkt.seq,
                           (unsigned)pkt.payload_len,
                           (unsigned long)delta_ms,
                           (int)RssiValue,
                           (int)SnrValue);

                    if (pkt.payload_len > 0) {
                        char payload_str[CHUNK_MAX + 1];
                        memcpy(payload_str, (char*)pkt.payload, pkt.payload_len);
                        payload_str[pkt.payload_len] = '\0';
                        printf("        Payload: %s\r\n", payload_str);
                    }
                } else {
                    total_rejected++;
                    TIMED_LOG("[RX DROP] node=%u, seq=%lu, rssi=%d (Reason: %s)",
                           (unsigned)pkt.node_id,
                           (unsigned long)pkt.seq,
                           (int)RssiValue,
                           pkt.reject_reason);
                }
            }
            Radio.Rx(RX_TIMEOUT_VALUE);
            State = LOWPOWER;
            break;
        case TX:
            Radio.Rx(RX_TIMEOUT_VALUE);
            State = LOWPOWER;
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            Radio.Rx(RX_TIMEOUT_VALUE);
            State = LOWPOWER;
            break;
        case TX_TIMEOUT:
            Radio.Rx(RX_TIMEOUT_VALUE);
            State = LOWPOWER;
            break;
        case LOWPOWER:
        default:
            break;
        }

        Radio.IrqProcess();

        const char* json_line = uart_read_json_line(10);
        if (json_line != NULL && strlen(json_line) > 0) {
            if (strlen(json_line) <= CHUNK_MAX) {
                send_enhanced_secure_packet((uint8_t*)json_line, strlen(json_line));
                Radio.Rx(RX_TIMEOUT_VALUE); // Back to listening
            }
        }

        {
            static uint32_t last_stats = 0;
            uint32_t now = TimerGetCurrentTime();
            if (TimerGetElapsedTime(last_stats) > 30000) {
                printf("\r\n");
                TIMED_LOG("=== GATEWAY STATISTICS (30sec) ===");
                TIMED_LOG("Total accepted: %lu | Total rejected: %lu", 
                    (unsigned long)total_accepted, (unsigned long)total_rejected);
                TIMED_LOG("Active nodes:   %u/%u", node_count, MAX_NODES);
                printf("\r\n");

                TIMED_LOG("Per-Node Status:");
                for (uint8_t i = 0; i < node_count; i++) {
                    uint32_t total = node_states[i].packets_received + node_states[i].packets_rejected;
                    uint32_t rate = (total > 0) ? (100U * node_states[i].packets_received / total) : 0;
                    TIMED_LOG("  Node %u: seq=%lu, ok=%lu, bad=%lu, rate=%lu%%, rssi=%d dBm",
                           (unsigned)node_states[i].node_id,
                           (unsigned long)node_states[i].last_seq,
                           (unsigned long)node_states[i].packets_received,
                           (unsigned long)node_states[i].packets_rejected,
                           (unsigned long)rate,
                           (int)node_states[i].last_rssi);
                }
                TIMED_LOG("==================================\r\n");

                last_stats = now;
            }
        }
    }
}

void OnTxDone(void)
{
    Radio.Sleep();
    txDone = 1;
    State = TX;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    Radio.Sleep();

    uint16_t use_len = (size > BUFFER_SIZE) ? BUFFER_SIZE : size;
    memcpy(LoraBuf, payload, use_len);
    LoraLen = use_len;
    RssiValue = rssi;
    SnrValue = snr;

    State = RX;
}

void OnTxTimeout(void)
{
    Radio.Sleep();
    txDone = 1;
    State = TX_TIMEOUT;
}

void OnRxTimeout(void)
{
    Radio.Sleep();
    State = RX_TIMEOUT;
}

void OnRxError(void)
{
    Radio.Sleep();
    State = RX_ERROR;
}
