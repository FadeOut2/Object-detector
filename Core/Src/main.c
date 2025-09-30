// STM32F767 + SSD1306 (I2C1 PB8/PB9) + HW-511 object detector PF13 + LEDs + Buzzer
#include "stm32f767xx.h"
#include <stdint.h>
#include <string.h>

/* ---------- I2C / OLED ---------- */
#define OLED_ADDR_7BIT   0x3C //Device address of SSD1306 or 0x3D 

#define OLED_ADDR_WR     (OLED_ADDR_7BIT << 1) 
//I2C protocol 8 bit address in order to write and thats what it expects
#define OLED_CMD         0x00 // Control byte for commands
#define OLED_BYTE_CONTROL       0x40 // Control byte data for SSD1306
#define OLED_W           128 // Width
#define OLED_H           64 // Height
#define I2C1_TIMING      0x20303E5D      // 100 kHz @ PCLK1=48 MHz  CLOCK/PRESC=1/4, SCLL=0x3E, SCLH=0x3E, SDADEL=0x2, SCLDEL=0x5

/* ---------- PINS ---------- */
#define PB8 8    // I2C1 SCL (AF4)
#define PB9 9    // I2C1 SDA (AF4)
#define PF13 13  // Sensor DO (HW-511)
#define PA3 3    // Green LED
#define PC0 0    // Red LED
#define PC3 3    // Buzzer

/* ---------- Small utils ---------- */
static void delay_cycles(volatile uint32_t n){ while(n--) __NOP(); }
static void delay_ms(uint32_t ms){ for(volatile uint32_t i=0;i<ms*4000;i++); }

/* ---------- LED/BUZZ macros ---------- */
#define GREEN_ON()    (GPIOA->BSRR = (1U<<PA3))
#define GREEN_OFF()   (GPIOA->BSRR = (1U<<(PA3+16)))
#define RED_ON()      (GPIOC->BSRR = (1U<<PC0))
#define RED_OFF()     (GPIOC->BSRR = (1U<<(PC0+16)))
#define BUZ_ON()      (GPIOC->BSRR = (1U<<PC3))
#define BUZ_OFF()     (GPIOC->BSRR = (1U<<(PC3+16)))

/* ---------- GPIO for LEDs/Buzzer/Sensor ---------- */
static void IO_Init(void){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOFEN;

    // LEDs + Buzzer
    GPIOA->MODER &= ~(3U<<(PA3*2)); GPIOA->MODER |= (1U<<(PA3*2)); // PA3 out
    GPIOC->MODER &= ~(3U<<(PC0*2)); GPIOC->MODER |= (1U<<(PC0*2)); // PC0 out
    GPIOC->MODER &= ~(3U<<(PC3*2)); GPIOC->MODER |= (1U<<(PC3*2)); // PC3 out
    GREEN_OFF(); RED_OFF(); BUZ_OFF();

    // PF13 input with pull-up (helps open-collector style outputs)
    GPIOF->MODER &= ~(3U<<(PF13*2));          // input
    GPIOF->PUPDR &= ~(3U<<(PF13*2));          // clear
    GPIOF->PUPDR |=  (1U<<(PF13*2));          // pull-up
}

/* ---------- I2C1 PB8/PB9 (hardware) ---------- */
static void I2C1_Init(void){
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // PB8=SCL, PB9=SDA: AF4, OD, PU, High speed
    GPIOB->MODER   &= ~((3U<<(8*2))|(3U<<(9*2)));
    GPIOB->MODER   |=  ((2U<<(8*2))|(2U<<(9*2)));            // AF
    GPIOB->AFR[1]  &= ~((0xFU<<0)|(0xFU<<4)); // clear AF 16 bits
    GPIOB->AFR[1]  |=  ((4U<<0)|(4U<<4));                    // AF4 (I2C1)
    GPIOB->OTYPER  |=  (1U<<8)|(1U<<9);                      // open-drain
    GPIOB->OSPEEDR |=  (3U<<(8*2))|(3U<<(9*2));              // high speed
    GPIOB->PUPDR   &= ~((3U<<(8*2))|(3U<<(9*2)));
    GPIOB->PUPDR   |=  (1U<<(8*2))|(1U<<(9*2));              // pull-up

    I2C1->CR1 &= ~I2C_CR1_PE; // clear PE
    I2C1->TIMINGR = I2C1_TIMING;
    I2C1->CR1 = I2C_CR1_PE;  

}

/* Write exactly 2 bytes: [a0, a1] with AUTOEND */
static int I2C1_Write2(uint8_t a0, uint8_t a1){
    I2C1->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF; // clear interrupts flags register
    I2C1->CR2 = (OLED_ADDR_WR) | // slave device address + write(0) by default
                (2U << I2C_CR2_NBYTES_Pos) | // 2 bytes & num of bytes to transfer
                I2C_CR2_START | I2C_CR2_AUTOEND; // start init transaction then autoend 
    // autoend: automatic STOP generation after NBYTES have been transferred

    while(!(I2C1->ISR & (I2C_ISR_TXIS|I2C_ISR_NACKF)));// wait TXIS or NACK
    if(I2C1->ISR & I2C_ISR_NACKF){ I2C1->ICR=I2C_ICR_NACKCF; return 0; } // if NACK, clear and exit
    I2C1->TXDR = a0; // transmit first byte
    while(!(I2C1->ISR & (I2C_ISR_TXIS|I2C_ISR_NACKF))); // wait TXIS or NACK
    if(I2C1->ISR & I2C_ISR_NACKF){ I2C1->ICR=I2C_ICR_NACKCF; return 0; } // if NACK, clear and exit
    I2C1->TXDR = a1; // transmit second byte
    while(!(I2C1->ISR & I2C_ISR_STOPF)); // wait STOP
    I2C1->ICR = I2C_ICR_STOPCF; /* clear STOP*/
    return 1; /* sucess*/
}
 
/* Burst write: [CONTROL=0x40][buf...] */
static int I2C1_DataBurst(const uint8_t* buf, uint16_t len){
    I2C1->ICR = I2C_ICR_STOPCF | I2C_ICR_NACKCF; // clear interrupts flags reg
    I2C1->CR2 = (OLED_ADDR_WR) | // slave device address + write(0)
                ((uint32_t)(len+1) << I2C_CR2_NBYTES_Pos) | // NBYTES = len+1 (data + control byte)
                I2C_CR2_START | I2C_CR2_AUTOEND;
                // the plus one is the control byte 0x40 for ssd1306 data
                // len is data bytes to send after control byte

    while(!(I2C1->ISR & (I2C_ISR_TXIS|I2C_ISR_NACKF)));
    if(I2C1->ISR & I2C_ISR_NACKF){ I2C1->ICR=I2C_ICR_NACKCF; return 0; } // if NACK, clear and exit
    I2C1->TXDR = OLED_BYTE_CONTROL; // first byte = control byte always 0x40 for ssd1306 data
    for(uint16_t i=0;i<len;i++){
        while(!(I2C1->ISR & (I2C_ISR_TXIS|I2C_ISR_NACKF)));
        if(I2C1->ISR & I2C_ISR_NACKF){ I2C1->ICR=I2C_ICR_NACKCF; return 0; } // clear and exit
        // wait for transmit interrupt status or NACK if true abort
        I2C1->TXDR = buf[i]; // send data byte
    }
    while(!(I2C1->ISR & I2C_ISR_STOPF)); // wait STOP
    I2C1->ICR = I2C_ICR_STOPCF; /* clear STOP*/
    return 1; // end success
}

/* ---------- OLED (page addressing) ---------- */
static int OLED_Cmd(uint8_t c){ return I2C1_Write2(OLED_CMD, c); }
static void OLED_SetPageCol(uint8_t page, uint8_t x){ // page:0-7, x:0-127
    
    OLED_Cmd(0xB0 | (page & 0x07)); // set page address
    OLED_Cmd(0x00 | (x & 0x0F)); // set lower column address
    OLED_Cmd(0x10 | ((x >> 4) & 0x0F)); // set higher column address
}
static void OLED_Clear(void){
    uint8_t zeros[OLED_W]; for(int i=0;i<OLED_W;i++) zeros[i]=0x00;
    for(uint8_t p=0;p<8;p++){ OLED_SetPageCol(p, 0); I2C1_DataBurst(zeros, OLED_W); }
}
static void OLED_Init(void){
    I2C1_Init(); delay_ms(120);
    OLED_Cmd(0xAE);
    OLED_Cmd(0x20); OLED_Cmd(0x02);     // Page Addressing
    OLED_Cmd(0xA1); OLED_Cmd(0xC8);
    OLED_Cmd(0x81); OLED_Cmd(0x7F);
    OLED_Cmd(0xA6);
    OLED_Cmd(0xA8); OLED_Cmd(0x3F);
    OLED_Cmd(0xD3); OLED_Cmd(0x00);
    OLED_Cmd(0xD5); OLED_Cmd(0x80);
    OLED_Cmd(0xD9); OLED_Cmd(0x22);
    OLED_Cmd(0xDA); OLED_Cmd(0x12);
    OLED_Cmd(0xDB); OLED_Cmd(0x20);
    OLED_Cmd(0x8D); OLED_Cmd(0x14);
    OLED_Cmd(0xAF);
    OLED_Clear();
}

/* ---------- tiny 5x7 glyphs (we only need a few) ---------- */
static const uint8_t G_SP[5]={0,0,0,0,0}; // SPACE
static const uint8_t G_A[5]={0x7E,0x11,0x11,0x11,0x7E}; 
static const uint8_t G_B[5]={0x7F,0x49,0x49,0x49,0x36};
static const uint8_t G_C[5]={0x3E,0x41,0x41,0x41,0x22};
static const uint8_t G_D[5]={0x7F,0x41,0x41,0x22,0x1C};
static const uint8_t G_E[5]={0x7F,0x49,0x49,0x49,0x41};
static const uint8_t G_I[5]={0x00,0x41,0x7F,0x41,0x00};
static const uint8_t G_J[5]={0x20,0x40,0x41,0x3F,0x01};
static const uint8_t G_N[5]={0x7F,0x04,0x08,0x10,0x7F};
static const uint8_t G_O[5]={0x3E,0x41,0x41,0x41,0x3E};
static const uint8_t G_R[5]={0x7F,0x09,0x19,0x29,0x46};
static const uint8_t G_T[5]={0x01,0x01,0x7F,0x01,0x01};
static const uint8_t G_P[5]= {0x7F,0x09,0x09,0x09,0x06};
static const uint8_t G_W[5]= {0x7F,0x20,0x18,0x20,0x7F};
static const uint8_t G_COL[5]={0x00,0x00,0x24,0x00,0x00}; // ':'
// this is an array of 5 bytes, each byte is a column of 8 pixels (LSB at top)
// each glyph is 5 pixels wide, we add 1 pixel gap when printing

static const uint8_t* glyph(char c){ // returns pointer to 5 bytes glyph
    if(c>='a'&&c<='z') c = (char)('A'+(c-'a')); // to upper
    switch(c){
        case ' ': return G_SP;
        case 'A': return G_A; case 'B': return G_B; case 'C': return G_C;
        case 'D': return G_D; case 'E': return G_E; case 'I': return G_I;
        case 'J': return G_J; case 'N': return G_N; case 'O': return G_O;
        case 'R': return G_R; case 'T': return G_T; case ':': return G_COL;
        case 'W': return G_W; case 'P': return G_P;
        default:  return G_SP;
    }
}
static void OLED_PrintAt(uint8_t x, uint8_t page, const char* s){
    OLED_SetPageCol(page, x);
    while(*s){
        const uint8_t* g = glyph(*s++);
        I2C1_DataBurst(g, 5);
        uint8_t gap = 0x00; I2C1_DataBurst(&gap, 1);
    }
}
static void OLED_PrintCentered(uint8_t page, const char* s){
    uint8_t w = (uint8_t)strlen(s)*6;
    uint8_t x = (w<OLED_W)?(uint8_t)((OLED_W - w)/2):0;
    OLED_PrintAt(x, page, s);
}

/* ---------- Sensor / Display ---------- */
static inline int sensor_raw(void){ return (GPIOF->IDR & (1U<<PF13)) ? 1 : 0; }

static void show_raw_line(int raw){
    char line[16]; // "RAW: 0" or "RAW: 1"
    line[0]='R'; line[1]='A'; line[2]='W'; line[3]=':'; line[4]=' ';
    line[5]= raw ? '1' : '0'; line[6]=0;
    // Clear top line only
    uint8_t zeros[128]={0}; // width of display
    OLED_SetPageCol(0,0); I2C1_DataBurst(zeros,128); // clear line 0, then print
    OLED_PrintAt(0,0,line);
}

static int detected_stable_active_low(void){ // option for active LOW sensors
    // we do multiple samples and majority voting
    // FC-51 typical: active LOW = object
    int votes=0; // 
    for(int i=0;i<8;i++){ if(sensor_raw()==0) votes++; delay_cycles(80000); } // ~20ms total
    // if sensor == 0 then increment votes that means object detected
    return (votes>=6);  // at least 6/8 votes but why not 5/8?
    // adjust threshold here if needed
    // but why should we do votes?
    // because some cheap sensors are noisy and jittery
    // so we sample multiple times and do majority voting
    // this is a simple form of debouncing
}

/* ---------- MAIN ---------- */
int main(void){
    SystemInit();
    IO_Init();
    OLED_Init();

    OLED_PrintCentered(2, "POWER ON");
    delay_ms(400);

    int last = -1;
    while(1){
        int raw = sensor_raw(); // this detects if there is an input signal from the sensor
        show_raw_line(raw); // show raw value on top line

        int detected = detected_stable_active_low(); // adjust here if your board is active high
        if(detected != last){
            OLED_Clear();
            show_raw_line(raw);
            if(detected){
                GREEN_ON();  RED_OFF(); BUZ_ON(); 
                OLED_PrintCentered(2, "OBJECT DETECTED");
            } else {
                GREEN_OFF(); RED_ON();  BUZ_OFF();
                OLED_PrintCentered(2, "NO OBJECT");
                OLED_PrintCentered(4, "DETECTED");
            }
            last = detected;
        }
        delay_ms(20);
    }
} 

void _init(void){}

// pin headers
// PB8 - I2C1 SCL
// PB9 - I2C1 SDA
// PF13 - Sensor DO (HW-511)
// PA3 - Green LED
// PC0 - Red LED
// PC3 - Buzzer

//--REAL PINS
// D15 FOR SCL
// D14 FOR SDA
// D7 FOR SENSOR
// A0 FOR GREEN LED
// A1 FOR RED LED
// A2 FOR BUZZER
//--END REAL PINS

