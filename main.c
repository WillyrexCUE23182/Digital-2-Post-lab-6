/*
 * Proyecto : Control de videojuego por UART (ATmega328P - Arduino Nano)
 * Archivo  : main.c
 * Autor    : Willy Cuellar 
 * Fecha    : 2025-09-17
 * Descripción:
 *   - 6 botones con detección por interrupciones (INT0/INT1 y PCINT).
 *   - Antirrebote robusto mediante máquina de estados por botón (press-once-per-hold).
 *   - Temporizador 1 en CTC a 1 ms para confirmar pulsación/liberación.
 *   - Envío por UART 9600-8N1: 'U','D','L','R','A','B'.
 *
 * Mapa de pines (activo en LOW, pull-up interno):
 *   Arriba    -> D3  (PD3 / INT1)     -> 'U'
 *   Abajo     -> A1  (PC1 / PCINT9)   -> 'D'
 *   Izquierda -> D2  (PD2 / INT0)     -> 'L'
 *   Derecha   -> A0  (PC0 / PCINT8)   -> 'R'
 *   Acción A  -> D7  (PD7 / PCINT23)  -> 'A'
 *   Acción B  -> A2  (PC2 / PCINT10)  -> 'B'
 *
 * Notas:
 *   - Sin polling en el bucle principal: toda la lógica ocurre en ISRs.
 *   - Requiere F_CPU = 16 MHz (Arduino Nano clásico).
 */

#define F_CPU 16000000UL

/* ======================= Encabezado (libraries) ======================= */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <stdbool.h>

/* ======================= Parámetros del sistema ======================= */
#define UART_BAUD  9600UL
#define UART_UBRR  ((F_CPU / (16UL * UART_BAUD)) - 1UL)

#define T_PRESS    15U   // ms de estabilidad para confirmar pulsación
#define T_RELEASE  15U   // ms de estabilidad para confirmar liberación

/* ======================= Prototipos ================================== */
static void UART_Init(void);
static void UART_Tx(uint8_t c);

static void GPIO_Buttons_Init(void);
static void SysTick1ms_Init(void);
static void ExtInt_Init(void);
static void PCInt_Init(void);

/* ======================= Tipos y estructuras ========================== */
typedef enum { BTN_UP=0, BTN_DOWN, BTN_LEFT, BTN_RIGHT, BTN_A, BTN_B, BTN_COUNT } btn_id_t;
typedef enum { ST_IDLE=0, ST_FALLING, ST_HELD, ST_RISING } btn_state_t;

typedef struct {
  volatile uint8_t *pin;   // puntero al registro PINx
  uint8_t           bit;   // bit dentro de PINx
  char              code;  // byte a enviar por UART
  volatile btn_state_t st; // estado actual
  volatile uint32_t t0;    // marca de tiempo (ms)
} Button;

/* ======================= Helpers de lectura (seguras en C) ============ */
static inline bool read_low_ptr(volatile uint8_t *pin, uint8_t bit) {
  return ((*pin & (1U << bit)) == 0);
}
static inline bool read_high_ptr(volatile uint8_t *pin, uint8_t bit) {
  return ((*pin & (1U << bit)) != 0);
}

/* ======================= Tabla de botones ============================= */
static Button btns[BTN_COUNT] = {
  [BTN_UP]    = { &PIND, PIND3, 'U', ST_IDLE, 0 }, // D3 / INT1
  [BTN_DOWN]  = { &PINC, PINC1, 'D', ST_IDLE, 0 }, // A1 / PCINT9
  [BTN_LEFT]  = { &PIND, PIND2, 'L', ST_IDLE, 0 }, // D2 / INT0
  [BTN_RIGHT] = { &PINC, PINC0, 'R', ST_IDLE, 0 }, // A0 / PCINT8
  [BTN_A]     = { &PIND, PIND7, 'A', ST_IDLE, 0 }, // D7 / PCINT23
  [BTN_B]     = { &PINC, PINC2, 'B', ST_IDLE, 0 }, // A2 / PCINT10
};

/* ======================= Variables globales =========================== */
volatile uint32_t g_ms = 0;  // contador de milisegundos (Timer1 CTC)

/* ======================= UART ======================================== */
static void UART_Init(void)
{
  UBRR0H = (uint8_t)(UART_UBRR >> 8);
  UBRR0L = (uint8_t)(UART_UBRR & 0xFF);
  UCSR0A = 0x00;
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8N1
  UCSR0B = (1 << TXEN0);                  // habilitar TX
}

static void UART_Tx(uint8_t c)
{
  while (!(UCSR0A & (1 << UDRE0))) { /* esperar buffer vacío */ }
  UDR0 = c;
}

/* ======================= GPIO & Interrupciones ======================== */
static void GPIO_Buttons_Init(void)
{
  // D2, D3, D7 como entrada con pull-up
  DDRD  &= ~((1 << DDD2) | (1 << DDD3) | (1 << DDD7));
  PORTD |=  (1 << PORTD2) | (1 << PORTD3) | (1 << PORTD7);

  // PC0, PC1, PC2 como entrada con pull-up
  DDRC  &= ~((1 << DDC0) | (1 << DDC1) | (1 << DDC2));
  PORTC |=  (1 << PORTC0) | (1 << PORTC1) | (1 << PORTC2);
}

static void SysTick1ms_Init(void)
{
  // Timer1 en CTC para 1 ms: OCR1A = 249 con prescaler 64
  TCCR1A = 0x00;
  TCCR1B = 0x00;
  OCR1A  = 249;
  TCCR1B = (1 << WGM12);                  // CTC
  TIMSK1 = (1 << OCIE1A);                 // habilitar IRQ compare A
  TCCR1B |= (1 << CS11) | (1 << CS10);    // prescaler 64
}

static void ExtInt_Init(void)
{
  // INT0 (PD2, Izquierda) y INT1 (PD3, Arriba) en flanco de bajada
  EICRA = (1 << ISC01) | (1 << ISC11);    // ISC00=0, ISC10=0
  EIFR  = (1 << INTF0) | (1 << INTF1);    // limpiar flags pendientes
  EIMSK = (1 << INT0)  | (1 << INT1);     // habilitar INT0/INT1
}

static void PCInt_Init(void)
{
  // Habilitar grupos PCINT para puerto C (PCINT8..14) y puerto D (PCINT16..23)
  PCICR  = (1 << PCIE1) | (1 << PCIE2);
  // Mascara para PC0..PC2
  PCMSK1 = (1 << PCINT8) | (1 << PCINT9) | (1 << PCINT10);
  // Mascara para PD7
  PCMSK2 = (1 << PCINT23);
  // Limpiar flags
  PCIFR  = (1 << PCIF1) | (1 << PCIF2);
}

/* ======================= Lógica de armado (desde INT/PCINT) =========== */
static inline void arm_falling(btn_id_t id)
{
  Button *b = &btns[id];
  if (b->st == ST_IDLE && read_low_ptr(b->pin, b->bit)) {
    b->st = ST_FALLING;
    b->t0 = g_ms;
  }
}

/* ======================= ISRs ======================================== */
// Tick de 1 ms: ejecuta la FSM de todos los botones
ISR(TIMER1_COMPA_vect)
{
  g_ms++;

  for (uint8_t i = 0; i < BTN_COUNT; ++i) {
    Button *b = &btns[i];

    switch (b->st) {
      case ST_FALLING:
        if ((g_ms - b->t0) >= T_PRESS) {
          if (read_low_ptr(b->pin, b->bit)) {
            UART_Tx(b->code);   // emitir UNA sola vez
            b->st = ST_HELD;    // queda ocupado mientras siga LOW
          } else {
            b->st = ST_IDLE;    // falso disparo
          }
        }
        break;

      case ST_HELD:
        if (read_high_ptr(b->pin, b->bit)) {
          b->st = ST_RISING;
          b->t0 = g_ms;
        }
        break;

      case ST_RISING:
        if ((g_ms - b->t0) >= T_RELEASE) {
          if (read_high_ptr(b->pin, b->bit)) {
            b->st = ST_IDLE;    // liberación confirmada
          } else {
            b->st = ST_HELD;    // regresó a LOW: sigue presionado
          }
        }
        break;

      default: /* ST_IDLE */ break;
    }
  }
}

/* INT0 -> D2 (Izquierda) */
ISR(INT0_vect)  { arm_falling(BTN_LEFT); }
/* INT1 -> D3 (Arriba)   */
ISR(INT1_vect)  { arm_falling(BTN_UP);   }

/* PCINT1 -> PC0..PC2: Derecha, Abajo, B */
ISR(PCINT1_vect)
{
  if (read_low_ptr(&PINC, PINC0)) arm_falling(BTN_RIGHT);
  if (read_low_ptr(&PINC, PINC1)) arm_falling(BTN_DOWN);
  if (read_low_ptr(&PINC, PINC2)) arm_falling(BTN_B);
}

/* PCINT2 -> PD7: Acción A */
ISR(PCINT2_vect)
{
  if (read_low_ptr(&PIND, PIND7)) arm_falling(BTN_A);
}

/* ======================= Función principal ============================ */
int main(void)
{
  UART_Init();
  GPIO_Buttons_Init();
  SysTick1ms_Init();
  ExtInt_Init();
  PCInt_Init();

  sei(); // habilitar interrupciones globales

  UART_Tx('\n'); UART_Tx('O'); UART_Tx('K'); UART_Tx('\n');

  for (;;) {
    // Sin polling.
  }
}
