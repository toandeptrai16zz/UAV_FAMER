#include "uav_config.h"
#include "flight_pid.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* Fixed size, bounded, nonblocking parser; IRQ only puts bytes in the ring.
 * TX uses HAL_UART_Transmit_IT only when HAL owns no other TX. */
#define SYNC_A 0xC3u
#define SYNC_B 0x3Cu
#define MAX_PAYLOAD 60u
#define TX_MAX 80u
#define RX_BUDGET 96u
#define GAINS_WIRE_SIZE 52u
#define FLASH_CFG_MAGIC 0x55415650u
#define FLASH_CFG_COMMIT UINT64_C(0xC0DEC0DEAA55AA55)
#define FLASH_CFG_PAGE0 0x0807F000u
#define FLASH_CFG_PAGE1 0x0807F800u

/* User must reserve 0x0807F000..0x0807FFFF in ALL active linker scripts. */
typedef struct {
    uint32_t magic;
    uint32_t generation;
    FlightPIDGains gains;
    uint32_t checksum;
    uint64_t commit;
} ConfigRecord;
_Static_assert(sizeof(FlightPIDGains) == GAINS_WIRE_SIZE, "Gain format must be 52 bytes");
_Static_assert(offsetof(ConfigRecord, commit) == 64u, "Record layout changed");
_Static_assert(sizeof(ConfigRecord) == 72u, "Flash doubleword layout changed");

static UART_HandleTypeDef *link_uart;
static uint8_t uart_rx_byte;
static volatile uint8_t ring[256];
static volatile uint8_t ring_head, ring_tail;
static volatile uint8_t rx_restart;
static uint8_t request[7u + MAX_PAYLOAD + 2u];
static uint8_t rx_pos, rx_expected;
static uint8_t tx_packet[TX_MAX];
static uint8_t tx_size;
static volatile bool tx_active;
static bool tx_pending;
static uint32_t stored_generation;
static bool boot_armed;
static uint32_t boot_acked_at;

static uint16_t crc16(const uint8_t *bytes, uint32_t n)
{
    uint16_t crc = 0xFFFFu;
    for (uint32_t i = 0; i < n; ++i) {
        crc ^= (uint16_t)bytes[i] << 8;
        for (uint32_t b = 0; b < 8u; ++b)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}
static uint32_t hash32(const uint8_t *bytes, uint32_t size)
{
    uint32_t hash = 2166136261u;
    for (uint32_t i = 0; i < size; ++i) { hash ^= bytes[i]; hash *= 16777619u; }
    return hash;
}
static bool flash_dual_bank(void)
{
#if defined(FLASH_OPTR_DBANK) && defined(SYSCFG_MEMRMP_FB_MODE)
    return ((FLASH->OPTR & FLASH_OPTR_DBANK) != 0u) &&
           ((SYSCFG->MEMRMP & SYSCFG_MEMRMP_FB_MODE) == 0u) &&
           (*(const uint16_t *)FLASHSIZE_BASE == 512u);
#else
    return false;
#endif
}
static bool record_valid(const ConfigRecord *r)
{
    if (r->magic != FLASH_CFG_MAGIC || r->commit != FLASH_CFG_COMMIT) return false;
    if (hash32((const uint8_t *)r, offsetof(ConfigRecord, checksum)) != r->checksum) return false;
    return FlightPID_ValidateGains(&r->gains);
}
static void load_gains(void)
{
    if (!flash_dual_bank()) return;
    const ConfigRecord *a = (const ConfigRecord *)FLASH_CFG_PAGE0;
    const ConfigRecord *b = (const ConfigRecord *)FLASH_CFG_PAGE1;
    const bool va = record_valid(a), vb = record_valid(b);
    if (va && (!vb || (int32_t)(a->generation - b->generation) > 0)) {
        (void)FlightPID_SetGains(&a->gains); stored_generation = a->generation;
    } else if (vb) {
        (void)FlightPID_SetGains(&b->gains); stored_generation = b->generation;
    }
}
static bool persist_gains(void)
{
    if (!flash_dual_bank()) return false;
    const ConfigRecord *a = (const ConfigRecord *)FLASH_CFG_PAGE0;
    const ConfigRecord *b = (const ConfigRecord *)FLASH_CFG_PAGE1;
    const bool va = record_valid(a), vb = record_valid(b);
    /* Always write the other page first, so a brown-out preserves old record. */
    const uint32_t dest = (!va || (vb && (int32_t)(b->generation - a->generation) > 0))
                        ? FLASH_CFG_PAGE0 : FLASH_CFG_PAGE1;
    ConfigRecord fresh;
    memset(&fresh, 0xFF, sizeof(fresh));
    fresh.magic = FLASH_CFG_MAGIC;
    fresh.generation = stored_generation + 1u;
    FlightPID_GetGains(&fresh.gains);
    fresh.checksum = hash32((const uint8_t *)&fresh, offsetof(ConfigRecord, checksum));
    fresh.commit = FLASH_CFG_COMMIT;

    FLASH_EraseInitTypeDef erase = {0};
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_2;
    erase.Page = (dest - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
    erase.NbPages = 1u;
    uint32_t error_page = 0u;
    if (HAL_FLASH_Unlock() != HAL_OK) return false;
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);
    bool ok = HAL_FLASHEx_Erase(&erase, &error_page) == HAL_OK;
    /* Commit marker is programmed last; never treat a partial write as valid. */
    for (uint32_t off = 0u; ok && off < sizeof(fresh); off += 8u) {
        uint64_t word;
        memcpy(&word, ((const uint8_t *)&fresh) + off, sizeof(word));
        ok = (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, dest + off, word) == HAL_OK);
    }
    (void)HAL_FLASH_Lock();
    if (ok) {
        ok = record_valid((const ConfigRecord *)dest) &&
             ((const ConfigRecord *)dest)->generation == fresh.generation;
    }
    if (ok) stored_generation = fresh.generation;
    return ok;
}
static void rx_arm(void)
{
    if (link_uart && link_uart->RxState == HAL_UART_STATE_READY &&
        HAL_UART_Receive_IT(link_uart, &uart_rx_byte, 1u) == HAL_OK)
        rx_restart = 0u;
}
void UAVConfig_Init(UART_HandleTypeDef *uart)
{
    link_uart = uart;
    ring_head = ring_tail = 0u; rx_pos = 0u; rx_expected = 0u;
    tx_active = false; tx_pending = false; tx_size = 0u;
    boot_armed = false; boot_acked_at = 0u;
    stored_generation = 0u;
    load_gains();
    rx_restart = 1u; rx_arm();
}
void UAVConfig_OnRxComplete(UART_HandleTypeDef *uart)
{
    if (uart != link_uart) return;
    const uint8_t next = (uint8_t)(ring_head + 1u);
    if (next != ring_tail) { ring[ring_head] = uart_rx_byte; ring_head = next; }
    rx_restart = 1u;
    rx_arm();
}
void UAVConfig_OnTxComplete(UART_HandleTypeDef *uart)
{
    if (uart == link_uart && tx_active) {
        tx_active = false;
        if (boot_armed) boot_acked_at = HAL_GetTick();
    }
}
void UAVConfig_OnError(UART_HandleTypeDef *uart)
{
    if (uart == link_uart) { tx_active = false; rx_restart = 1u; boot_armed = false; }
}
static void make_reply(uint8_t request_type, uint16_t seq, uint8_t status, bool safe, bool include_gains)
{
    if (tx_pending || tx_active) return;
    uint8_t len = 2u;
    tx_packet[0] = SYNC_A; tx_packet[1] = SYNC_B; tx_packet[2] = 1u;
    tx_packet[3] = request_type | 0x80u;
    tx_packet[4] = (uint8_t)seq; tx_packet[5] = (uint8_t)(seq >> 8);
    tx_packet[7] = status;
    tx_packet[8] = safe ? 1u : 0u;
    if (include_gains) {
        FlightPIDGains gains;
        FlightPID_GetGains(&gains);
        memcpy(&tx_packet[9], &gains, sizeof(gains));
        len += (uint8_t)sizeof(gains);
    }
    tx_packet[6] = len;
    tx_size = (uint8_t)(7u + len + 2u);
    const uint16_t check = crc16(tx_packet, tx_size - 2u);
    tx_packet[tx_size - 2u] = (uint8_t)check;
    tx_packet[tx_size - 1u] = (uint8_t)(check >> 8);
    tx_pending = true;
}
static void handle_request(bool safe)
{
    const uint8_t type = request[3];
    const uint16_t seq = (uint16_t)request[4] | ((uint16_t)request[5] << 8);
    const uint8_t len = request[6];
    uint8_t status = UAV_CONFIG_STATUS_OK;
    bool with_gains = true;
    if (tx_active || tx_pending) return;
    switch (type) {
    case UAV_CONFIG_CMD_GET:
        if (len != 0u) status = UAV_CONFIG_STATUS_LENGTH;
        break;
    case UAV_CONFIG_CMD_APPLY:
        if (!safe) status = UAV_CONFIG_STATUS_LOCKED;
        else if (len != sizeof(FlightPIDGains)) status = UAV_CONFIG_STATUS_LENGTH;
        else {
            FlightPIDGains gains;
            memcpy(&gains, &request[7], sizeof(gains));
            if (!FlightPID_SetGains(&gains)) status = UAV_CONFIG_STATUS_RANGE;
        }
        break;
    case UAV_CONFIG_CMD_SAVE:
        if (!safe) status = UAV_CONFIG_STATUS_LOCKED;
        else if (len != 0u) status = UAV_CONFIG_STATUS_LENGTH;
        else if (!persist_gains()) status = UAV_CONFIG_STATUS_FLASH;
        break;
    case UAV_CONFIG_CMD_BOOT:
        with_gains = false;
        if (!safe) status = UAV_CONFIG_STATUS_LOCKED;
        else if (len != 0u) status = UAV_CONFIG_STATUS_LENGTH;
#if UAV_CONFIG_ENABLE_ROM_OTA
        else { boot_armed = true; boot_acked_at = 0u; }
#else
        else status = UAV_CONFIG_STATUS_DISABLED;
#endif
        break;
    default:
        status = UAV_CONFIG_STATUS_DISABLED;
        with_gains = false;
        break;
    }
    make_reply(type, seq, status, safe, with_gains);
}
static void parse_byte(uint8_t c, bool safe)
{
    if (rx_pos == 0u) { if (c == SYNC_A) request[rx_pos++] = c; return; }
    if (rx_pos == 1u) {
        if (c == SYNC_B) request[rx_pos++] = c;
        else rx_pos = (c == SYNC_A) ? 1u : 0u;
        return;
    }
    if (rx_pos >= sizeof(request)) { rx_pos = 0u; return; }
    request[rx_pos++] = c;
    if (rx_pos == 7u) {
        if (request[2] != 1u || request[6] > MAX_PAYLOAD) { rx_pos = 0u; return; }
        rx_expected = (uint8_t)(7u + request[6] + 2u);
    }
    if (rx_expected && rx_pos == rx_expected) {
        const uint16_t expected = (uint16_t)request[rx_expected - 2u] |
                                  ((uint16_t)request[rx_expected - 1u] << 8);
        if (crc16(request, rx_expected - 2u) == expected) handle_request(safe);
        rx_pos = 0u; rx_expected = 0u;
    }
}
void UAVConfig_Service(bool safe)
{
    if (rx_restart) rx_arm();
    for (uint32_t i = 0u; i < RX_BUDGET && ring_tail != ring_head; ++i) {
        const uint8_t value = ring[ring_tail]; ring_tail = (uint8_t)(ring_tail + 1u);
        parse_byte(value, safe);
    }
    if (tx_pending && !tx_active && link_uart && link_uart->gState == HAL_UART_STATE_READY) {
        tx_active = true;
        if (HAL_UART_Transmit_IT(link_uart, tx_packet, tx_size) == HAL_OK) {
            tx_pending = false;
        } else { tx_active = false; }
    }
}

/* ROM bootloader entry for STM32G474. Requires a validated USART2 boot on PA2/PA3
 * and a backup ST-LINK recovery plan. Disabled in the default flight candidate.
 * The boot ACK has left the UART before taking this path. */
static void jump_rom_loader(void)
{
#if UAV_CONFIG_ENABLE_ROM_OTA
    const uint32_t rom = 0x1FFF0000u;
    const uint32_t msp = *(volatile const uint32_t *)rom;
    const uint32_t reset = *(volatile const uint32_t *)(rom + 4u);
    if ((msp & 0x2FFF0000u) != 0x20000000u || (reset & 1u) == 0u) return;
    /* Stop outputs BEFORE teardown, and ensure interrupts cannot invoke DShot. */
    __disable_irq();
    for (uint32_t n = 0; n < 8u; ++n) NVIC->ICER[n] = 0xFFFFFFFFu;
    SysTick->CTRL = 0u;
    (void)HAL_RCC_DeInit();
    /* ROM bootloader is started with its expected vector table and main stack. */
    SCB->VTOR = rom;
    __set_MSP(msp);
    __DSB(); __ISB();
    ((void (*)(void))reset)();
    for (;;) { }
#endif
}
void UAVConfig_ServiceBoot(bool safe)
{
    if (!boot_armed) return;
    if (!safe) { boot_armed = false; return; }
    if (boot_acked_at && !tx_active && !tx_pending &&
        (uint32_t)(HAL_GetTick() - boot_acked_at) >= 200u) {
        boot_armed = false;
        jump_rom_loader();
    }
}
