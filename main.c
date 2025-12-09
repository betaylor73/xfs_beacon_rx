//#############################################################################
//
// FILE:   main.c
//
// TITLE:  XFS Beacon Monitor Board FW
//
// Main module for the XFS Beacon Monitor Board FW.
//
//#############################################################################

//
// Included Files
//
#include <stddef.h>
#include <ctype.h>

#include "driverlib.h"
#include "board.h"
#include "c2000ware_libraries.h"

#include "sine_table.h"
#include "queue.h"
#include "diagport.h"
#include "fsk_correlator.h"
#include "hal_afe031.h"

//
// A N-Point sine table is decimated and fed to the AFE031 DAC via DMA at a
// 1MHz rate to produce the desired MARK and SPACE frequencies for each bit.
// Decimation calculations for SPACE and MARK frequencies w/1MHz output rate:
// Step Size = #Total/(#Points/Cycle), where #Points/Cycle = DAC_Hz/Target_Hz
// MARK Step = N/(1000KHz/131.25KHz) = N*0.13125 (537.6 for 4096pt sine wave)
// SPACE Step = N/(1000KHz/143.75KHz) = N*0.14375 (588.8 for 4096pt sine wave)
// PWM1 paces the 1us DMA DAC Output from 50MHz Base Clock (TBPRD=50, DIV=1)
// PWM2 paces the 5.12ms Bit Frame from 50MHz Base Clock (TBRD=16000, DIV=16)
// REFERENCES:
// 1. The SunSpec Rapid Shutdown Application Note can be found here:
// http://www.ti.com/lit/ug/tidue68/tidue68.pdf
// 2. A sine table calculator can be found here:
// https://daycounter.com/Calculators/Sine-Generator-Calculator.phtml
//

// Number of points in the sine table
#define N_POINT (sizeof(sine_table)/sizeof(sine_table[0]))

// Tones 
#define TONE_MARK  (1u)
#define TONE_SPACE (0u)

//
// NOTE: Firmware gain values calibrated w/1500nF, 1.5kV line coupling capacitor
// and multiplied by 1024 to enable faster integer division (i.e. logical shift).
//
static const uint32_t GAIN_SPACE_x1024 = 676;   // SPACE Gain x 1024 = 0.660
static const uint32_t GAIN_MARK_x1024  = 558;   // MARK Gain x 1024  = 0.545

//
// Offset DAC code to keep output in linear region (0.1V < DAC_OUT < 3.2V).
// NOTE: The DAC offset limits the firmware gain values (1024 - 2*offset).
//
static uint32_t DAC_CODE_OFFSET = 64;

// Tone frequencies (Hz).
#define SPACE_HZ  143750u
#define MARK_HZ   131250u

// Output sample rate (Hz) used by the NCO update loop.
#define FS_HZ (1000000u)

static uint32_t amplitude_gain_x1024 = 1024u;  // unity gain in Q10

static float phase_accumulator = 0;
static float tuning_word = 0;

#define TW_HZ(f_hz) ((float)N_POINT*((float)f_hz/(float)FS_HZ))

static const float mark_tuning_word  = TW_HZ(MARK_HZ);
static const float space_tuning_word = TW_HZ(SPACE_HZ);

static inline void nco_set_tone(uint16_t tone)
{
    switch (tone) {
        case TONE_MARK:  tuning_word = mark_tuning_word;  break;
        case TONE_SPACE: tuning_word = space_tuning_word; break;
        default:         tuning_word = 0;                 break;
    }
}

static inline void nco_set_gain(uint32_t gain_x1024)
{
    amplitude_gain_x1024 = gain_x1024;
}

static bool    led_enabled = true;
static uint8_t led_toggle_counter = 0;
static uint32_t message_timer = 0;

// locate dac_buffer_a in ramgs0 so that DMA is able to access it
#pragma DATA_SECTION(dac_buffer_a, "ramgs0");
#pragma DATA_SECTION(dac_buffer_b, "ramgs0");
uint16_t dac_buffer_a[16];
uint16_t dac_buffer_b[16];

static uint16_t dac_buffer_select = 0;  // 0 == A; 1 == B

static uint16_t current_bit = TONE_MARK;
static uint16_t last_bit    = TONE_SPACE;

#ifdef _FLASH
extern uint16_t SineTableLoadStart;
extern uint16_t SineTableLoadSize;
extern uint16_t SineTableRunStart;
#endif

static void pwm_enable(bool);
static void rx_enable(bool);


__interrupt void bit_tx_isr(void);
__interrupt void dac_dma_isr(void);
__interrupt void dac_adc_isr(void);
__interrupt void bit_rx_isr(void);
__interrupt void timer_expire_isr(void);

static void DMA_init();
static void DAC_SPI_TX_DMA_init();
static void nco_fill_dma_block(uint16_t *, uint16_t);
static void nco_reset_dma_buffers(void);

#define DAC_SPI_TX_DMA_BASE DMA_CH1_BASE
#define MESSAGE_RX_TIMER_BASE CPUTIMER2_BASE

static FskCorrState fsk_corr_state;

#define RX_BUFFER_SIZE 96

static bool has_printed = false;
static volatile int16_t rx_bit_buffer[RX_BUFFER_SIZE] = {0};
//static volatile int16_t bit_buffer[100] = {0};

static volatile uint16_t bit_index = 0;

// new 12-byte receiver buffer and write indices (accumulate bits MSB-first)
static uint8_t rx_msg_buf[12];
static uint16_t rx_msg_byte_index = 0;
static uint8_t rx_msg_bit_pos = 0; // number of bits currently in current byte (0..7)

static void print_rx_bits_as_string(void);
static void init_timer(void);

static bool accumulating_bits = false;
static bool message_started = false;
static volatile bool timer_expired = false;

static spsc_queue_t bit_queue;

typedef enum {
    RX_STATE_DISABLED,
    RX_STATE_WAITING_FOR_GAP,
    RX_STATE_WAITING_FOR_MSG_START,
    RX_STATE_RECEIVING_MESSAGE,
    RX_STATE_MESSAGE_TIMEOUT,
    RX_STATE_MESSAGE_COMPLETE
} rx_state_t;

static rx_state_t rx_state = RX_STATE_DISABLED;

static void main_loop(void);

//
// Main
//
void main(void)
{

    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Disable pin locks and enable internal pull-ups.
    //
    Device_initGPIO();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //

    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // PinMux and Peripheral Initializatione
    //
    Board_init();
    EPWM_disableADCTrigger(DAC_PWM_BASE, EPWM_SOC_A);

    // make sure that the sine table is in RAM
    #ifdef _FLASH
    memcpy(&SineTableRunStart, &SineTableLoadStart, (size_t)&SineTableLoadSize);
    #endif

    //
    // C2000Ware Library initialization
    //
    C2000Ware_libraries_init();

    fsk_corr_init(&fsk_corr_state);
    
    // set up interrupt handler for bit timer
    Interrupt_register(INT_EPWM4, &bit_tx_isr);
    
    // enable bit timer interrupt
    Interrupt_enable(INT_EPWM4);

    Interrupt_register(INT_DMA_CH1, &dac_dma_isr);
    Interrupt_enable(INT_DMA_CH1);

    Interrupt_register(INT_ADCA1, &dac_adc_isr);
    Interrupt_enable(INT_ADCA1);

    Interrupt_register(INT_EPWM7, &bit_rx_isr);
    Interrupt_enable(INT_EPWM7);

    Interrupt_register(INT_TIMER2, &timer_expire_isr);
    Interrupt_enable(INT_TIMER2);

    // make sure DMA can access SPI 
    SysCtl_selectSecController(SYSCTL_SEC_CONTROLLER_CLA, SYSCTL_SEC_CONTROLLER_DMA);

    // configure AFE031 PLC Analog Front-End
    AFE031_initialize();

    // TODO: temp
    // uint16_t gainsel   = AFE031_read_register(HAL_AFE031_GAINSEL_REG);
    // uint16_t dieid     = AFE031_read_register(HAL_AFE031_DIEID_REG);
    // uint16_t rev       = AFE031_read_register(HAL_AFE031_REV_REG);
    // uint16_t reset_val = AFE031_read_register(HAL_AFE031_RESET_REG);
    // AFE031_write_register( HAL_AFE031_GAINSEL_REG, 0x2A );
    // gainsel = AFE031_read_register(HAL_AFE031_GAINSEL_REG);

    AFE031_disable_transmit();
    AFE031_enable_receive();
    AFE031_enable_interrupts();

    init_timer();

    spsc_init(&bit_queue);

    //
    // Enable Global Interrupt (INTM) and real time interrupt (DBGM)
    //
    EINT;
    ERTM;

    // turn off sonalert!
    GPIO_writePin(SONALERT, 0);

    DIAGPORT_initialize();

    DMA_init();
    //DMA_startChannel(DAC_SPI_TX_DMA_BASE);

    EPWM_enableADCTrigger(DAC_PWM_BASE, EPWM_SOC_A);

    // TODO: do phase synchronization across EPMW1 and EPMW2

    // enable TBCLK for EPWM modules 
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC);

    EPWM_clearADCTriggerFlag(DAC_PWM_BASE, EPWM_SOC_A);

    // TODO: TEMP
    pwm_enable(false);
    nco_set_tone(TONE_SPACE);
    nco_set_gain(GAIN_SPACE_x1024);
    nco_reset_dma_buffers();
    pwm_enable(true);
    rx_enable(true);

    main_loop();
    // while (true)
    // {
    //     int16_t latest_bit = 0;
    //     while (spsc_dequeue(&bit_queue, &latest_bit))
    //     {
    //         // TODO: process bits
    //     }

    //     if (accumulating_bits) {
    //         if ((! message_started) && (bit_index > 0)) {
    //             message_started = true;
    //             //CPUTimer_startTimer(MESSAGE_RX_TIMER_BASE);
    //         }
    //     }

    //     if (message_started) {
    //         message_timer++;
    //         if (message_timer >= 300) {
    //             timer_expired = true;
    //         }
    //     }

    //     if (timer_expired) 
    //     {
    //         timer_expired = false;
    //         message_started = false;
    //         accumulating_bits = false;
    //         bit_index = 0;
    //         SCI_writeCharArray(DEBUG_SCI_BASE, "Timer Expired\r\n", 15);
    //     }   

    //     if (current_bit != last_bit) {
    //         pwm_enable(false);
    //         nco_set_tone( current_bit );
    //         nco_set_gain( (current_bit == TONE_MARK) ? GAIN_MARK_x1024 : GAIN_SPACE_x1024 );
    //         nco_reset_dma_buffers();
    //         pwm_enable(true);

    //         last_bit = current_bit;
    //     }

    //     // toggle health LED every 250 ms (every 25 * 10 ms)
    //     if (led_enabled) {
    //         if (++led_toggle_counter >= 25)
    //         {
    //             GPIO_togglePin(HLTH_LED);
    //             led_toggle_counter = 0;
    //         }
    //     }
    //     else 
    //     {
    //         GPIO_writePin(HLTH_LED, 0);
    //     }

    //     if ((bit_index == sizeof(rx_bit_buffer)/sizeof(rx_bit_buffer[0])) && ! has_printed) {
    //         message_started = false;
    //         CPUTimer_stopTimer(MESSAGE_RX_TIMER_BASE);
    //         print_rx_bits_as_string();
    //         has_printed = true;
    //     }

    //     // write out debug message
    //     //SCI_writeCharArray(DEBUG_SCI_BASE, "Hello\r\n", 7);
    //     //DIAGPORT_service();

    //     //SPI_writeDataBlockingNonFIFO(DAC_SPI_BASE, (0x200 << 6)/*tx_data_toggle ? (0x200 << 6) : (0x100 << 6)*/);
    //     //tx_data_toggle = !tx_data_toggle;

    //     //DMA_forceTrigger(DAC_SPI_TX_DMA_BASE);

    //     // delay for 10 milliseconds
    //     DEVICE_DELAY_US(10000);
    // }
}

#define GAP_DETECTION_THRESHOLD 50u  // number of consecutive idle bits to detect gap
#define MESSAGE_RX_TIMEOUT_MS 3000u  // message timeout in ms (3 seconds)

static uint16_t gap_counter = 0;

void main_loop(void) 
{
    while (true) 
    {
        switch (rx_state) 
        {
            case RX_STATE_DISABLED:
            {
                // TODO: for now, just jump directly to waiting for gap; in the future
                // should require a user action (e.g. via diagport command) to enable 
                // reception
                spsc_clear(&bit_queue);
                gap_counter = 0;

                rx_state = RX_STATE_WAITING_FOR_GAP;
                break;
            }

            case RX_STATE_WAITING_FOR_GAP:
            {
                int16_t latest_bit = BIT_IDLE;

                // wait for gap in transmission
                while (spsc_dequeue(&bit_queue, &latest_bit)) {

                    if (latest_bit != BIT_DECIDING) {
                        if ((latest_bit != BIT_MARK) && (latest_bit != BIT_SPACE)) {
                            gap_counter++;
                            if (gap_counter >= GAP_DETECTION_THRESHOLD) {
                                rx_state = RX_STATE_WAITING_FOR_MSG_START;
                                gap_counter = 0;
                                break;
                            }
                        } else {
                            gap_counter = 0; // idle bits must be consecutive
                        } 
                    }   
                }

                break;
            }

            case RX_STATE_WAITING_FOR_MSG_START:
            {
                int16_t latest_bit = BIT_IDLE;

                // wait for start of message
                while (spsc_dequeue(&bit_queue, &latest_bit)) {
                    if ((latest_bit == BIT_MARK) || (latest_bit == BIT_SPACE)) {
                        // initialize receiver buffer
                        for (int i = 0; i < (int)sizeof(rx_msg_buf); ++i) rx_msg_buf[i] = 0;
                        rx_msg_byte_index = 0;
                        // store first bit as LSB value 0x01 for mark(1) or 0x00 for space(0)
                        rx_msg_buf[0] = (latest_bit == BIT_MARK) ? 0x01 : 0x00;
                        rx_msg_bit_pos    = 1; // one bit already stored
                        accumulating_bits = true;
                        message_started   = true;

                        // transition to receiving state to continue filling buffer
                        rx_state = RX_STATE_RECEIVING_MESSAGE;
                        break;
                    }
                }

                message_timer = 0;
                break;
            }

            case RX_STATE_RECEIVING_MESSAGE:
            {
                int16_t latest_bit = BIT_IDLE;

                // receive message bits and pack into rx_msg_buf MSB-first
                while (spsc_dequeue(&bit_queue, &latest_bit)) {

                    // for now, ignore any bit that is not +1 or -1
                    if ((latest_bit == BIT_MARK) || (latest_bit == BIT_SPACE)) {
                        uint8_t bit = (latest_bit == BIT_MARK) ? 1u : 0u; // treat -1 or 0 as 0

                        // guard against overflow
                        if (rx_msg_byte_index >= sizeof(rx_msg_buf)) {
                            // buffer full, mark complete and stop
                            rx_state = RX_STATE_MESSAGE_COMPLETE;
                            break;
                        }

                        rx_msg_buf[rx_msg_byte_index] = (uint8_t)((rx_msg_buf[rx_msg_byte_index] << 1) | bit);
                        rx_msg_bit_pos++;

                        if (rx_msg_bit_pos >= 8) {
                            rx_msg_bit_pos = 0;
                            rx_msg_byte_index++;
                            if (rx_msg_byte_index >= sizeof(rx_msg_buf)) {
                                // full message received
                                rx_state = RX_STATE_MESSAGE_COMPLETE;
                                break;
                            }
                        }
                    }
                }

                if (rx_state != RX_STATE_MESSAGE_COMPLETE) {
                    message_timer++;
                    if (message_timer >= MESSAGE_RX_TIMEOUT_MS) {
                        rx_state = RX_STATE_MESSAGE_TIMEOUT;
                    }
                }

                break;
            }

            case RX_STATE_MESSAGE_TIMEOUT:
            {
                // handle message timeout
                // TODO: dump message and state
                SCI_writeCharArray(DEBUG_SCI_BASE, "Timer Expired\r\n", 15);
                
                rx_state = RX_STATE_WAITING_FOR_GAP;
                break;
            }
            
            case RX_STATE_MESSAGE_COMPLETE:
            {
                // Format the received 12-byte message into a printable string and send it
                char out[13];
                for (size_t i = 0; i < sizeof(rx_msg_buf); ++i) {
                    uint8_t b = rx_msg_buf[i];
                    out[i] = isprint(b) ? (char)b : '.';
                }
                out[sizeof(rx_msg_buf)] = '\0';

                // Send the message over SCI (explicit length)
                SCI_writeCharArray(DEBUG_SCI_BASE, (const uint16_t *)out, (int)sizeof(rx_msg_buf));
                const char crlf[] = "\r\n";
                SCI_writeCharArray(DEBUG_SCI_BASE, (const uint16_t *)crlf, 2);

                // Reset reception state
                accumulating_bits = false;
                message_started = false;
                timer_expired = false;
                rx_msg_byte_index = 0;
                rx_msg_bit_pos = 0;
                for (size_t i = 0; i < sizeof(rx_msg_buf); ++i) rx_msg_buf[i] = 0;

                // Return to waiting for gap
                rx_state = RX_STATE_WAITING_FOR_GAP;
                break;
            }

            default:
            {
                // unknown state, reset to disabled
                rx_state = RX_STATE_DISABLED;
                break;
            }
        }

        if (led_enabled) {
            if (++led_toggle_counter >= 250)
            {
                GPIO_togglePin(HLTH_LED);
                led_toggle_counter = 0;
            }
        }
        else 
        {
            GPIO_writePin(HLTH_LED, 0);
        }
    
        // TODO: service diagnostic port here

        DEVICE_DELAY_US(1000);      // 1 ms
    }
}

void MAIN_enable_health_led( bool enable )
{
    led_enabled = enable;
}

void pwm_enable( bool enable )
{
    EPWM_setTimeBaseCounterMode(DAC_PWM_BASE, enable ? EPWM_COUNTER_MODE_UP_DOWN : EPWM_COUNTER_MODE_STOP_FREEZE);
    EPWM_setTimeBaseCounterMode(BIT_PWM_BASE, enable ? EPWM_COUNTER_MODE_UP_DOWN : EPWM_COUNTER_MODE_STOP_FREEZE);
}

void rx_enable( bool enable )
{
    EPWM_setTimeBaseCounterMode(ADC_PWM_BASE, enable ? EPWM_COUNTER_MODE_UP : EPWM_COUNTER_MODE_STOP_FREEZE);
    EPWM_setTimeBaseCounterMode(DEMOD_PWM_BASE, enable ? EPWM_COUNTER_MODE_UP : EPWM_COUNTER_MODE_STOP_FREEZE);
}

// this interrupt should fire every 5.12 ms (the bit time for SunSpec RSS)
#pragma CODE_SECTION(bit_tx_isr,".TI.ramfunc");
__interrupt void bit_tx_isr(void) 
{
    static uint16_t temp_transition_counter = 0;

    // TODO:TEMP: toggle a debug gpio to test interrupt period
    GPIO_togglePin(DEBUG_TP23);

    if (++temp_transition_counter > 100) {
        current_bit = (current_bit == TONE_MARK) ? TONE_SPACE : TONE_MARK;
        temp_transition_counter = 0;        
    }

    // Clear INT flag for this timer
    EPWM_clearEventTriggerInterruptFlag(BIT_PWM_BASE);

    // Acknowledge this interrupt to allow more interrupts from group 3
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);
}

#define DAC_SPI_TX_DMA_ADDRESS ((uint16_t *)(SPIA_BASE + SPI_O_TXBUF))

#pragma CODE_SECTION(dac_dma_isr,".TI.ramfunc");
__interrupt void dac_dma_isr(void) 
{
    // TODO: TEMP
    //DMA_stopChannel(DAC_SPI_TX_DMA_BASE);

    // switch buffers and refill the inactive buffer
    if (dac_buffer_select)
    {
        DMA_configAddresses(DAC_SPI_TX_DMA_BASE, DAC_SPI_TX_DMA_ADDRESS, dac_buffer_b);
        nco_fill_dma_block(dac_buffer_a, sizeof(dac_buffer_a)/sizeof(dac_buffer_a[0]));
        dac_buffer_select = 0;
    }
    else
    {
        DMA_configAddresses(DAC_SPI_TX_DMA_BASE, DAC_SPI_TX_DMA_ADDRESS, dac_buffer_a);
        nco_fill_dma_block(dac_buffer_b, sizeof(dac_buffer_b)/sizeof(dac_buffer_b[0]));
        dac_buffer_select = 1;
    }

    // Acknowledge this interrupt to allow more interrupts from group 7
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP7);
}

#define ADC_PU_SCALE_FACTOR (float)(1.0 / 4096.0)   // ADC Scaling Factor
#define SCALE_ADC_INPUT(adc_input)  (((float)(adc_input))*ADC_PU_SCALE_FACTOR)*2.0

static float last_scaled_input = 0.0;

#pragma CODE_SECTION(dac_adc_isr,".TI.ramfunc");
__interrupt void dac_adc_isr(void)
{
    // read ADC result
    uint16_t adc_result = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);
    last_scaled_input = SCALE_ADC_INPUT(adc_result);        // scales from 0.0 to 2.0

    // TODO: TEMP keep compiler from optimizing out last_scaled_input
    dac_buffer_a[1] = (uint16_t)(last_scaled_input * 1000.0);

    // pass scaled sample to FSK correlator
    fsk_corr_step(last_scaled_input, &fsk_corr_state);

    // Clear ADC interrupt flag
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

    // Acknowledge this interrupt to allow more interrupts from group 1
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);

    // TODO: TEMP: toggle a debug gpio to test interrupt period
    //GPIO_togglePin(DEBUG_TP21);
}

static const float bit_detection_threshold = 0.1;

static uint16_t idle_count = 0;

#pragma CODE_SECTION(bit_rx_isr,".TI.ramfunc");
__interrupt void bit_rx_isr(void)
{
    int16_t detected_bit = fsk_corr_detect(&fsk_corr_state, bit_detection_threshold);

    spsc_enqueue(&bit_queue, detected_bit);

    // if (detected_bit == 1) {
    //     if (accumulating_bits && (bit_index < sizeof(rx_bit_buffer)/sizeof(rx_bit_buffer[0]))) {
    //         rx_bit_buffer[bit_index] = 1;
    //         bit_index++;

    //         // if (! message_started) {
    //         //     message_started = true;
    //         //     //CPUTimer_startTimer(MESSAGE_RX_TIMER_BASE);
    //         // }
    //     }
    //     GPIO_writePin(DEBUG_TP16, 1);
    //     GPIO_writePin(DEBUG_TP21, 1);
    // } else if (detected_bit == -1) {
    //     if (accumulating_bits && (bit_index < sizeof(rx_bit_buffer)/sizeof(rx_bit_buffer[0]))) {
    //         rx_bit_buffer[bit_index] = -1;
    //         bit_index++;

    //         // if (! message_started) {
    //         //     message_started = true;
    //         //     //CPUTimer_startTimer(MESSAGE_RX_TIMER_BASE);
    //         // }
    //     }
    //     GPIO_writePin(DEBUG_TP16, 0);
    //     GPIO_writePin(DEBUG_TP21, 1);
    // } else {
    //     idle_count++;
    //     if ((idle_count > 10) && ! accumulating_bits) { 
    //         accumulating_bits = true;
    //     }
    //     GPIO_writePin(DEBUG_TP21, 0);
    // }

    // Clear EPWM interrupt flag
    EPWM_clearEventTriggerInterruptFlag(DEMOD_PWM_BASE);

    // Acknowledge this interrupt to allow more interrupts from group 3
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP3);

    // TODO: TEMP: toggle a debug gpio to test interrupt period
    GPIO_togglePin(DEBUG_TP19);

}

// syscfg is not flexible enough for us to use it to configure DMA
// for example, I was not able to determine how to use EPWM1 as the trigger
// if SPI and DMA were 'linked' by syscfg, it insisted on using SPI_TX as the trigger
// if SPI_TX was the destination address

#define DAC_SPI_TX_DMA_TRIGGER DMA_TRIGGER_EPWM1SOCA

void DMA_init()
{
    DMA_initController();
	DAC_SPI_TX_DMA_init();
}

void DAC_SPI_TX_DMA_init()
{
    DMA_setEmulationMode(DMA_EMULATION_FREE_RUN);
    DMA_configAddresses(DAC_SPI_TX_DMA_BASE, DAC_SPI_TX_DMA_ADDRESS, dac_buffer_a);
    DMA_configBurst(DAC_SPI_TX_DMA_BASE, 1U, 1, 0);
    DMA_configTransfer(DAC_SPI_TX_DMA_BASE, 16U, 1, 0);
    DMA_configMode(DAC_SPI_TX_DMA_BASE, DAC_SPI_TX_DMA_TRIGGER, DMA_CFG_ONESHOT_DISABLE | DMA_CFG_CONTINUOUS_ENABLE | DMA_CFG_SIZE_16BIT);
    DMA_setInterruptMode(DAC_SPI_TX_DMA_BASE, DMA_INT_AT_END);
    DMA_enableInterrupt(DAC_SPI_TX_DMA_BASE);
    DMA_disableOverrunInterrupt(DAC_SPI_TX_DMA_BASE);
    DMA_enableTrigger(DAC_SPI_TX_DMA_BASE);
    DMA_stopChannel(DAC_SPI_TX_DMA_BASE);
}

#pragma CODE_SECTION(nco_fill_dma_block,".TI.ramfunc");
void nco_fill_dma_block(uint16_t *buffer, uint16_t buffer_size)
{
    uint16_t entry;

    // set up next data points in sine wave for transmission
    for (uint16_t x=0; x < buffer_size; x++)
    {
        // NOTE: shift left 6 + shift right 10 (x1024 gain factor) yields net shift right 4.
        // 0xFFC0 masks all but most 
        entry = ((amplitude_gain_x1024 * sine_table[(uint16_t)phase_accumulator]) >> 4) & 0xffc0;

        // offset to ensure linear region (0.1V < DAC_OUT < 3.2V)
        buffer[x] = entry + (DAC_CODE_OFFSET << 6);

        // Calculate next step
        phase_accumulator += tuning_word;

        // Check for overflow
        if (phase_accumulator > (N_POINT-1))
        {
            phase_accumulator -= (N_POINT-1);
        }
    }
}

// this function is called when the transmitted frequency changes
void nco_reset_dma_buffers(void)
{
    // avoid DMA transfer count reset glitch if last transfer was not completed 
    // when this function is called
    EALLOW;
    HWREGH(DAC_SPI_TX_DMA_BASE + DMA_O_TRANSFER_COUNT) = 15;
    EDIS;

    DMA_configAddresses(DAC_SPI_TX_DMA_BASE, DAC_SPI_TX_DMA_ADDRESS, dac_buffer_a);

    nco_fill_dma_block(&dac_buffer_a[0], sizeof(dac_buffer_a)/sizeof(dac_buffer_a[0]));
    nco_fill_dma_block(&dac_buffer_b[0], sizeof(dac_buffer_b)/sizeof(dac_buffer_b[0]));

    dac_buffer_select = 1;
}

// new helper: pack bits (rx_bit_buffer) MSB-first into bytes and send as a string.
// non-printable bytes are replaced with '.'
void print_rx_bits_as_string(void)
{
    size_t max_bits  = sizeof(rx_bit_buffer) / sizeof(rx_bit_buffer[0]);
    size_t max_bytes = max_bits / 8;
    char out[256] = {0};//char out[max_bytes + 1];

    size_t num_bits = bit_index;
    size_t num_bytes = num_bits / 8; // drop trailing bits if not a full byte

    if (num_bytes == 0) {
        // nothing to print
        return;
    }

    for (size_t i = 0; i < num_bytes; ++i) {
        uint8_t byte = 0;
        // pack 8 bits MSB-first: first collected bit -> bit7
        for (int b = 0; b < 8; ++b) {
            size_t idx = i * 8 + b;
            int16_t v = rx_bit_buffer[idx];
            uint8_t bit = (v == 1) ? 1u : 0u; // treat -1 or 0 as 0
            byte = (uint8_t)((byte << 1) | bit);
        }
        out[i] = isprint(byte) ? (char)byte : '.';
    }
    out[num_bytes] = '\0';

    // send the string (no null terminator required by SCI, length passed explicitly)
    SCI_writeCharArray(DEBUG_SCI_BASE, (uint16_t *)out, (int)num_bytes);
    // send CRLF
    const char crlf[] = "\r\n";
    SCI_writeCharArray(DEBUG_SCI_BASE, (uint16_t *)crlf, 2);
}

#pragma CODE_SECTION(timer_expire_isr,".TI.ramfunc");
__interrupt void timer_expire_isr(void)
{
    // Handle timer expiration
    //timer_expired = true;

    //
    // The CPU acknowledges the interrupt for CPUTIMER2
    //
}

static void init_timer(void)
{
    const float freq   = DEVICE_SYSCLK_FREQ;
    const float period = 2000000.0; // 2 seconds

    //
    // Initialize timer period:
    //
    uint32_t temp = (uint32_t)(freq / 1000000 * period);
    CPUTimer_setPeriod(MESSAGE_RX_TIMER_BASE, temp);

    //
    // Set pre-scale counter to divide by 1 (SYSCLKOUT):
    //
    CPUTimer_setPreScaler(MESSAGE_RX_TIMER_BASE, 0);

    //
    // Initializes timer control register. The timer is stopped, reloaded,
    // free run disabled, and interrupt enabled.
    // Additionally, the free and soft bits are set
    //
    CPUTimer_stopTimer(MESSAGE_RX_TIMER_BASE);
    CPUTimer_reloadTimerCounter(MESSAGE_RX_TIMER_BASE);
    CPUTimer_setEmulationMode(MESSAGE_RX_TIMER_BASE, CPUTIMER_EMULATIONMODE_STOPAFTERNEXTDECREMENT);
    CPUTimer_enableInterrupt(MESSAGE_RX_TIMER_BASE);
}


// TODO:
// - Add a means of allowing to use to specify:
//    - Both RX gain values for AFE031
//    - RX mode (continous vs burst)
//    - message buffer size???
// - Improve diagport command parsing
