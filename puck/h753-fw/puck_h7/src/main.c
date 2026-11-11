/*
 * puck_h7 v0.1 - dev-kit puck firmware for NUCLEO-H753ZI
 *
 * Links (all 115200 8N1, 3.3 V):
 *   USART3 PD8/PD9   ST-LINK virtual COM port (debug console on the PC)
 *   UART7  PE8 TX / PE7 RX  -> DK1 Arduino D0/D1 (DK1 /dev/ttySTM2)
 *   USART6 PC6 TX / PC7 RX  -> ESP32 UART0 (GPIO3 RX0 / GPIO1 TX0)
 *   PE9  ESP32 EN     (open-drain)
 *   PE11 ESP32 GPIO0  (open-drain)
 *   I2C1 PB8 SCL / PB9 SDA  sensors (LSM6DSV16X, LPS22DF, LIS2MDL / LSM303AGR)
 *
 * Text commands, one per line, on the DK1 link or the ST-LINK console:
 *   PING                 -> OK PONG ...
 *   SCAN                 -> I2C bus scan
 *   STREAM ON|OFF        -> sensor lines on/off
 *   ESP RESET            -> reset ESP32 into its app
 *   ESP BOOT             -> reset ESP32 into its ROM bootloader
 *   ESP SEND <text>      -> send <text>\n to the ESP32
 *   BRIDGE               -> ESP32 into bootloader, then raw byte bridge between the
 *                           port that sent the command and the ESP32 (for esptool).
 *                           Ends after 3 s with no bytes either way, then resets the
 *                           ESP32 into its app.
 *   BRIDGE APP           -> same raw bridge without entering the bootloader
 *   WIRES                -> wiring finder: lists every header pin that another board is
 *                           driving high (idle UART TX lines, powered pins) and every pin
 *                           that toggles during a 10 s watch. Read-only, nothing is driven.
 * Lines the ESP32 prints are forwarded to both ports as "ESP: <line>".
 * Sensor lines: $IMU,ms,ax,ay,az[mg],gx,gy,gz[mdps]  $MAG,ms,x,y,z[mG]
 *               $BAR,ms,p[Pa],t[cdegC]  $ACC,ms,ax,ay,az[mg] (LSM303AGR accel)
 */
#include "stm32h7xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#define FW_VERSION "puck_h7 v0.2"

/* ---------------- ring buffers ---------------- */
#define RB_SIZE 4096u
typedef struct { volatile uint16_t head, tail; uint8_t buf[RB_SIZE]; } ring_t;
static inline int rb_put(ring_t *r, uint8_t c) {
  uint16_t n = (uint16_t)((r->head + 1u) % RB_SIZE);
  if (n == r->tail) return 0;
  r->buf[r->head] = c; r->head = n; return 1;
}
static inline int rb_get(ring_t *r, uint8_t *c) {
  if (r->tail == r->head) return 0;
  *c = r->buf[r->tail]; r->tail = (uint16_t)((r->tail + 1u) % RB_SIZE); return 1;
}
static inline uint16_t rb_free(ring_t *r) {
  return (uint16_t)((r->tail + RB_SIZE - r->head - 1u) % RB_SIZE);
}

/* ---------------- ports ---------------- */
enum { P_VCP = 0, P_DK1 = 1, P_ESP = 2, NPORTS = 3 };
typedef struct {
  USART_TypeDef *u;
  ring_t rx, tx;
  char line[256]; uint16_t len;
  uint32_t overruns;
} port_t;
static port_t ports[NPORTS];
static UART_HandleTypeDef huart[NPORTS];
static I2C_HandleTypeDef hi2c1;

static void port_irq(port_t *p) {
  uint32_t isr = p->u->ISR;
  if (isr & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) {
    if (isr & USART_ISR_ORE) p->overruns++;
    p->u->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
  }
  while (p->u->ISR & USART_ISR_RXNE_RXFNE) {
    uint8_t c = (uint8_t)p->u->RDR;
    if (!rb_put(&p->rx, c)) p->overruns++;
  }
}
void USART3_IRQHandler(void) { port_irq(&ports[P_VCP]); }
void UART7_IRQHandler(void)  { port_irq(&ports[P_DK1]); }
void USART6_IRQHandler(void) { port_irq(&ports[P_ESP]); }
void SysTick_Handler(void) { HAL_IncTick(); }
void HardFault_Handler(void) { while (1) {} }

/* drain TX rings without blocking */
static void tx_service(void) {
  for (int i = 0; i < NPORTS; i++) {
    port_t *p = &ports[i];
    uint8_t c;
    while ((p->u->ISR & USART_ISR_TXE_TXFNF) && rb_get(&p->tx, &c)) p->u->TDR = c;
  }
}
static void port_write(int i, const void *d, uint16_t n) {
  const uint8_t *b = d;
  for (uint16_t k = 0; k < n; k++) {
    uint32_t t0 = HAL_GetTick();
    while (!rb_put(&ports[i].tx, b[k])) {  /* ring full: wait up to 50 ms */
      tx_service();
      if (HAL_GetTick() - t0 > 50) return;
    }
  }
  tx_service();
}
static void port_puts(int i, const char *s) { port_write(i, s, (uint16_t)strlen(s)); }
/* print to both the PC console and the DK1 */
static void say(const char *fmt, ...) {
  char b[200];
  va_list ap; va_start(ap, fmt);
  int n = vsnprintf(b, sizeof b - 2, fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if (n > (int)sizeof b - 3) n = sizeof b - 3;
  b[n++] = '\r'; b[n++] = '\n'; b[n] = 0;
  port_puts(P_VCP, b); port_puts(P_DK1, b);
}

/* ---------------- clocks / init ---------------- */
static void clock_init(void) {
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /* stay on HSI 64 MHz: simple and plenty for v0 */
  SystemCoreClockUpdate();
}

static void gpio_af(GPIO_TypeDef *g, uint32_t pins, uint32_t af, uint32_t mode, uint32_t pull) {
  GPIO_InitTypeDef gi = {0};
  gi.Pin = pins; gi.Mode = mode; gi.Pull = pull;
  gi.Speed = GPIO_SPEED_FREQ_MEDIUM; gi.Alternate = af;
  HAL_GPIO_Init(g, &gi);
}

static void uart_init(int i, USART_TypeDef *u, uint32_t baud, IRQn_Type irq) {
  ports[i].u = u;
  UART_HandleTypeDef *h = &huart[i];
  h->Instance = u;
  h->Init.BaudRate = baud;
  h->Init.WordLength = UART_WORDLENGTH_8B;
  h->Init.StopBits = UART_STOPBITS_1;
  h->Init.Parity = UART_PARITY_NONE;
  h->Init.Mode = UART_MODE_TX_RX;
  h->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  h->Init.OverSampling = UART_OVERSAMPLING_16;
  h->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  h->Init.ClockPrescaler = UART_PRESCALER_DIV1;
  h->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(h);
  HAL_UARTEx_DisableFifoMode(h);
  u->CR1 |= USART_CR1_RXNEIE_RXFNEIE;
  u->CR3 |= USART_CR3_EIE;
  HAL_NVIC_SetPriority(irq, 1, 0);
  HAL_NVIC_EnableIRQ(irq);
}

static void periph_init(void) {
  __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE(); __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_USART3_CLK_ENABLE(); __HAL_RCC_UART7_CLK_ENABLE();
  __HAL_RCC_USART6_CLK_ENABLE(); __HAL_RCC_I2C1_CLK_ENABLE();

  /* LEDs: LD1 PB0 green, LD3 PB14 red, LD2 PE1 yellow */
  GPIO_InitTypeDef gi = {0};
  gi.Mode = GPIO_MODE_OUTPUT_PP; gi.Speed = GPIO_SPEED_FREQ_LOW;
  gi.Pin = GPIO_PIN_0 | GPIO_PIN_14; HAL_GPIO_Init(GPIOB, &gi);
  gi.Pin = GPIO_PIN_1; HAL_GPIO_Init(GPIOE, &gi);

  /* ESP32 EN (PE9) and GPIO0 (PE11): open-drain, released (high via ESP pull-ups) */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9 | GPIO_PIN_11, GPIO_PIN_SET);
  gi.Mode = GPIO_MODE_OUTPUT_OD; gi.Pull = GPIO_NOPULL;
  gi.Pin = GPIO_PIN_9 | GPIO_PIN_11; HAL_GPIO_Init(GPIOE, &gi);

  gpio_af(GPIOD, GPIO_PIN_8 | GPIO_PIN_9, GPIO_AF7_USART3, GPIO_MODE_AF_PP, GPIO_PULLUP);
  gpio_af(GPIOE, GPIO_PIN_7 | GPIO_PIN_8, GPIO_AF7_UART7, GPIO_MODE_AF_PP, GPIO_PULLUP);
  gpio_af(GPIOC, GPIO_PIN_6 | GPIO_PIN_7, GPIO_AF7_USART6, GPIO_MODE_AF_PP, GPIO_PULLUP);
  gpio_af(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_AF4_I2C1, GPIO_MODE_AF_OD, GPIO_PULLUP);

  uart_init(P_VCP, USART3, 115200, USART3_IRQn);
  uart_init(P_DK1, UART7, 115200, UART7_IRQn);
  uart_init(P_ESP, USART6, 115200, USART6_IRQn);

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10707DBC;  /* ~100 kHz from 64 MHz */
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);
}

/* ---------------- ESP32 control ---------------- */
static void esp_pins(int en, int io0) {
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, en ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, io0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static void esp_reset(int bootloader) {
  esp_pins(1, bootloader ? 0 : 1);
  HAL_Delay(5);
  esp_pins(0, bootloader ? 0 : 1);   /* hold in reset */
  HAL_Delay(100);
  esp_pins(1, bootloader ? 0 : 1);   /* release reset, GPIO0 sampled now */
  HAL_Delay(100);
  esp_pins(1, 1);                    /* release GPIO0 */
  /* flush whatever the ESP32 printed during reset */
  uint8_t c; while (rb_get(&ports[P_ESP].rx, &c)) {}
  ports[P_ESP].len = 0;
}

/* raw bridge between port 'src' and the ESP32 until 3 s idle */
static void bridge(int src, int bootloader) {
  say("OK BRIDGE %s (ends after 3 s idle)", bootloader ? "BOOT" : "APP");
  tx_service();
  uint32_t t0 = HAL_GetTick(); while (HAL_GetTick() - t0 < 20) tx_service();
  if (bootloader) esp_reset(1);
  port_t *a = &ports[src], *e = &ports[P_ESP];
  uint32_t last = HAL_GetTick(), got = 0;
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
  for (;;) {
    uint8_t c; int moved = 0;
    while (rb_free(&e->tx) && rb_get(&a->rx, &c)) { rb_put(&e->tx, c); moved = 1; }
    while (rb_free(&a->tx) && rb_get(&e->rx, &c)) { rb_put(&a->tx, c); moved = 1; }
    tx_service();
    if (moved) { last = HAL_GetTick(); got = 1; }
    uint32_t idle = HAL_GetTick() - last;
    if (idle > (got ? 3000u : 10000u)) break;   /* wait up to 10 s for the first byte */
  }
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
  if (bootloader) esp_reset(0);
  a->len = 0;
  say("OK BRIDGE END");
}

/* ---------------- sensors ---------------- */
static int i2c_rd(uint8_t a, uint8_t reg, uint8_t *d, uint16_t n) {
  return HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(a << 1), reg, I2C_MEMADD_SIZE_8BIT, d, n, 10) == HAL_OK;
}
static int i2c_wr(uint8_t a, uint8_t reg, uint8_t v) {
  return HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(a << 1), reg, I2C_MEMADD_SIZE_8BIT, &v, 1, 10) == HAL_OK;
}
static int whoami(uint8_t a, uint8_t reg, uint8_t expect) {
  uint8_t v = 0; return i2c_rd(a, reg, &v, 1) && v == expect;
}
static uint8_t imu_a, baro_a, mag_a, acc_a;   /* 0 = not found */
static int stream_on = 1;

static void sensors_probe(void) {
  imu_a = baro_a = mag_a = acc_a = 0;
  if (whoami(0x6A, 0x0F, 0x70)) imu_a = 0x6A; else if (whoami(0x6B, 0x0F, 0x70)) imu_a = 0x6B;
  if (whoami(0x5C, 0x0F, 0xB4)) baro_a = 0x5C; else if (whoami(0x5D, 0x0F, 0xB4)) baro_a = 0x5D;
  if (whoami(0x1E, 0x4F, 0x40)) mag_a = 0x1E;          /* LIS2MDL or LSM303AGR mag */
  if (whoami(0x19, 0x0F, 0x33)) acc_a = 0x19;          /* LSM303AGR accel */
  if (imu_a) {                       /* LSM6DSV16X */
    i2c_wr(imu_a, 0x12, 0x44);       /* CTRL3: BDU, IF_INC */
    i2c_wr(imu_a, 0x17, 0x02);       /* CTRL8: +-8 g */
    i2c_wr(imu_a, 0x15, 0x04);       /* CTRL6: +-2000 dps */
    i2c_wr(imu_a, 0x10, 0x06);       /* CTRL1: XL 120 Hz HP */
    i2c_wr(imu_a, 0x11, 0x06);       /* CTRL2: G 120 Hz HP */
  }
  if (baro_a) {                      /* LPS22DF */
    i2c_wr(baro_a, 0x11, 0x08);      /* CTRL2: BDU */
    i2c_wr(baro_a, 0x10, 0x20);      /* CTRL1: 25 Hz */
  }
  if (mag_a) {                       /* LIS2MDL / LSM303AGR mag */
    i2c_wr(mag_a, 0x62, 0x10);       /* CFG_C: BDU */
    i2c_wr(mag_a, 0x60, 0x8C);       /* CFG_A: temp comp, 100 Hz, continuous */
  }
  if (acc_a) {                       /* LSM303AGR accel */
    i2c_wr(acc_a, 0x23, 0x88);       /* CTRL4: BDU, HR, +-2 g */
    i2c_wr(acc_a, 0x20, 0x57);       /* CTRL1: 100 Hz, XYZ */
  }
  say("SENSORS imu=%s baro=%s mag=%s acc303=%s",
      imu_a ? "LSM6DSV16X" : "none", baro_a ? "LPS22DF" : "none",
      mag_a ? "LIS2MDL/LSM303" : "none", acc_a ? "LSM303AGR" : "none");
}

static int16_t s16(const uint8_t *b) { return (int16_t)(b[0] | (b[1] << 8)); }

static void sensors_poll(void) {
  static uint32_t t_imu, t_mag, t_bar;
  uint32_t now = HAL_GetTick();
  uint8_t b[12];
  if (!stream_on) return;
  if (imu_a && now - t_imu >= 20) {            /* 50 Hz */
    t_imu = now;
    if (i2c_rd(imu_a, 0x22, b, 12)) {
      long gx = s16(b) * 70L, gy = s16(b + 2) * 70L, gz = s16(b + 4) * 70L;           /* mdps */
      long ax = s16(b + 6) * 244L / 1000, ay = s16(b + 8) * 244L / 1000, az = s16(b + 10) * 244L / 1000; /* mg */
      say("$IMU,%lu,%ld,%ld,%ld,%ld,%ld,%ld", now, ax, ay, az, gx, gy, gz);
    }
  }
  if ((mag_a || acc_a) && now - t_mag >= 50) {  /* 20 Hz */
    t_mag = now;
    if (mag_a && i2c_rd(mag_a, 0x68, b, 6))
      say("$MAG,%lu,%ld,%ld,%ld", now, s16(b) * 3L / 2, s16(b + 2) * 3L / 2, s16(b + 4) * 3L / 2);
    if (acc_a && i2c_rd(acc_a, 0x28 | 0x80, b, 6))
      say("$ACC,%lu,%d,%d,%d", now, s16(b) >> 4, s16(b + 2) >> 4, s16(b + 4) >> 4);
  }
  if (baro_a && now - t_bar >= 100) {           /* 10 Hz */
    t_bar = now;
    if (i2c_rd(baro_a, 0x28, b, 5)) {
      int32_t p = (int32_t)(b[0] | (b[1] << 8) | ((uint32_t)b[2] << 16));
      if (p & 0x800000) p |= (int32_t)0xFF000000;
      long pa = (long)((int64_t)p * 100 / 4096);
      say("$BAR,%lu,%ld,%d", now, pa, s16(b + 3));
    }
  }
}

static void i2c_scan(void) {
  char out[180]; int n = snprintf(out, sizeof out, "SCAN");
  int found = 0;
  for (uint8_t a = 0x08; a < 0x78; a++) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1), 1, 5) == HAL_OK) {
      found++;
      if (n < (int)sizeof out - 6) n += snprintf(out + n, sizeof out - n, " 0x%02X", a);
    }
  }
  if (!found) snprintf(out + n, sizeof out - n, " none (check SDA/SCL/3V3/GND)");
  say("%s", out);
}


/* ---------------- wiring finder (read-only) ---------------- */
static GPIO_TypeDef *const wports[] = {GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG};
static const char wnames[] = "ABCDEFG";
/* pins never touched: SWD, ST-LINK VCP, LSE/HSE oscillator */
static int wire_skip(int port, int pin) {
  if (port == 0 && (pin == 13 || pin == 14)) return 1;          /* PA13/PA14 SWD */
  if (port == 3 && (pin == 8 || pin == 9)) return 1;            /* PD8/PD9 VCP */
  if (port == 2 && (pin == 14 || pin == 15)) return 1;          /* PC14/PC15 LSE */
  return 0;
}
static const char *wire_note(int port, int pin) {
  static const struct { char p; int n; const char *t; } notes[] = {
    {'A',1,"ETH"},{'A',2,"ETH"},{'A',7,"ETH"},{'C',1,"ETH"},{'C',4,"ETH"},{'C',5,"ETH"},
    {'G',11,"ETH"},{'G',13,"ETH"},{'B',13,"ETH"},{'A',9,"USB"},{'A',10,"USB"},{'A',11,"USB"},
    {'A',12,"USB"},{'B',0,"LED"},{'B',14,"LED"},{'E',1,"LED"},{'C',13,"BTN"},{'G',7,"USB"},
    {'C',6,"=CN7-1"},{'C',7,"=CN7-11"},{'E',7,"=CN10-20"},{'E',8,"=CN10-18"},{'E',9,"=CN10-4"},
    {'E',11,"=CN10-6"},{'B',8,"=CN7-2"},{'B',9,"=CN7-4"},{'B',6,"=D1 CN10-14"},{'B',7,"=D0 CN10-16"}};
  for (unsigned i = 0; i < sizeof notes / sizeof notes[0]; i++)
    if (notes[i].p == wnames[port] && notes[i].n == pin) return notes[i].t;
  return "";
}
static void wires(void) {
  uint32_t moder[7], pupdr[7];
  say("WIRES start: reading every header pin, nothing is driven");
  for (int k = 0; k < 7; k++) { moder[k] = wports[k]->MODER; pupdr[k] = wports[k]->PUPDR; }
  __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOF_CLK_ENABLE(); __HAL_RCC_GPIOG_CLK_ENABLE();
  uint16_t high_pd[7] = {0}, low_pu[7] = {0};
  for (int k = 0; k < 7; k++) {
    GPIO_TypeDef *g = wports[k];
    uint32_t m = g->MODER, pu = g->PUPDR, pd = g->PUPDR;
    for (int n = 0; n < 16; n++) {
      if (wire_skip(k, n)) continue;
      m &= ~(3u << (2 * n));
      pd = (pd & ~(3u << (2 * n))) | (2u << (2 * n));
      pu = (pu & ~(3u << (2 * n))) | (1u << (2 * n));
    }
    g->MODER = m; g->PUPDR = pd; HAL_Delay(3);
    uint32_t idr_pd = g->IDR;
    g->PUPDR = pu; HAL_Delay(3);
    uint32_t idr_pu = g->IDR;
    for (int n = 0; n < 16; n++) {
      if (wire_skip(k, n)) continue;
      if (idr_pd & (1u << n)) high_pd[k] |= (uint16_t)(1u << n);
      if (!(idr_pu & (1u << n))) low_pu[k] |= (uint16_t)(1u << n);
    }
  }
  for (int k = 0; k < 7; k++) for (int n = 0; n < 16; n++)
    if (high_pd[k] & (1u << n)) say("WIRES driven-high P%c%d %s", wnames[k], n, wire_note(k, n));
  for (int k = 0; k < 7; k++) for (int n = 0; n < 16; n++)
    if (low_pu[k] & (1u << n)) say("WIRES pulled-low P%c%d %s", wnames[k], n, wire_note(k, n));
  say("WIRES watching for toggles for 10 s (press the ESP32 EN/RST button now)");
  tx_service();
  uint32_t edges[7][16]; memset(edges, 0, sizeof edges);
  uint32_t last[7];
  for (int k = 0; k < 7; k++) last[k] = wports[k]->IDR;
  uint32_t t0 = HAL_GetTick();
  while (HAL_GetTick() - t0 < 10000) {
    for (int k = 0; k < 7; k++) {
      uint32_t v = wports[k]->IDR, d = v ^ last[k];
      if (d) { last[k] = v; for (int n = 0; n < 16; n++) if (d & (1u << n)) edges[k][n]++; }
    }
    tx_service();
  }
  int any = 0;
  for (int k = 0; k < 7; k++) for (int n = 0; n < 16; n++)
    if (edges[k][n] && !wire_skip(k, n)) { any = 1; say("WIRES toggled P%c%d x%lu %s", wnames[k], n, edges[k][n], wire_note(k, n)); }
  if (!any) say("WIRES toggled: none");
  for (int k = 0; k < 7; k++) { wports[k]->PUPDR = pupdr[k]; wports[k]->MODER = moder[k]; }
  say("WIRES done");
}

/* ---------------- commands ---------------- */
static void handle_line(int src, char *s) {
  while (*s == ' ') s++;
  if (!*s) return;
  if (!strcmp(s, "PING")) say("OK PONG %s up=%lus", FW_VERSION, HAL_GetTick() / 1000);
  else if (!strcmp(s, "SCAN")) i2c_scan();
  else if (!strcmp(s, "PROBE")) sensors_probe();
  else if (!strcmp(s, "STREAM ON")) { stream_on = 1; say("OK STREAM ON"); }
  else if (!strcmp(s, "STREAM OFF")) { stream_on = 0; say("OK STREAM OFF"); }
  else if (!strcmp(s, "ESP RESET")) { esp_reset(0); say("OK ESP RESET"); }
  else if (!strcmp(s, "ESP BOOT")) { esp_reset(1); say("OK ESP BOOT"); }
  else if (!strncmp(s, "ESP SEND ", 9)) { port_puts(P_ESP, s + 9); port_puts(P_ESP, "\n"); say("OK ESP SEND"); }
  else if (!strcmp(s, "BRIDGE")) bridge(src, 1);
  else if (!strcmp(s, "BRIDGE APP")) bridge(src, 0);
  else if (!strcmp(s, "WIRES")) wires();
  else if (!strcmp(s, "STATS")) say("STATS ovr vcp=%lu dk1=%lu esp=%lu",
                                    ports[P_VCP].overruns, ports[P_DK1].overruns, ports[P_ESP].overruns);
  else say("ERR unknown command: %s", s);
}

static void lines_service(int i) {
  port_t *p = &ports[i];
  uint8_t c;
  while (rb_get(&p->rx, &c)) {
    if (c == '\r' || c == '\n') {
      if (p->len) {
        p->line[p->len] = 0;
        if (i == P_ESP) say("ESP: %s", p->line);
        else handle_line(i, p->line);
        p->len = 0;
        if (i != P_ESP && ports[i].u == NULL) return;
      }
    } else if (p->len < sizeof p->line - 1) {
      p->line[p->len++] = (char)c;
    }
  }
}

int main(void) {
  HAL_Init();
  clock_init();
  HAL_InitTick(TICK_INT_PRIORITY);
  periph_init();
  HAL_Delay(50);
  say("BOOT %s core=%luMHz", FW_VERSION, SystemCoreClock / 1000000);
  sensors_probe();
  uint32_t t_led = 0;
  for (;;) {
    lines_service(P_VCP);
    lines_service(P_DK1);
    lines_service(P_ESP);
    sensors_poll();
    tx_service();
    if (HAL_GetTick() - t_led >= 500) {
      t_led = HAL_GetTick();
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
    }
  }
}
