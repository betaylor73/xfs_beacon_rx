#ifndef _HAL_AFE031_H
#define _HAL_AFE031_H

#include <stdint.h>

#define HAL_AFE031_CMD_SHIFT 8
#define HAL_AFE031_CMD_MASK  0xFF
#define HAL_AFE031_DATA_SHIFT 0
#define HAL_AFE031_DATA_MASK  0xFF

#define HAL_AFE031_CMD_WR 0x0
#define HAL_AFE031_CMD_RD 0x1
#define HAL_AFE031_CMD_RW_SHIFT 7
#define HAL_AFE031_CMD_RW_MASK  0x1

// SPI transaction lengths
#define HAL_AFE031_SPI_DAC_LEN     10
#define HAL_AFE031_SPI_DEFAULT_LEN 16

// data registers
#define HAL_AFE031_ENABLE_REG   0x01
#define HAL_AFE031_ENABLE2_REG  0x03
#define HAL_AFE031_GAINSEL_REG  0x02
#define HAL_AFE031_CTRL_REG     0x04
#define HAL_AFE031_CTRL2_REG    0x05
#define HAL_AFE031_RESET_REG    0x09
#define HAL_AFE031_DIEID_REG    0x0A
#define HAL_AFE031_REV_REG      0x0B

typedef struct
{
    uint16_t PA       :1;  // b0 - PA enable
    uint16_t TX       :1;  // b1 - TX enable
    uint16_t RX       :1;  // b2 - RX enable
    uint16_t ERX      :1;  // b3 - ERX enable
    uint16_t ETX      :1;  // b4 - ETX enable
    uint16_t DAC      :1;  // b5 - DAC enable
    uint16_t reserved :10; // reserved
}
HAL_afe031_enable1_t;

typedef struct
{
  uint16_t RXG      :4;  // b1:0 - RX PGA1 gain
                         // b3:2 - RX PGA2 gain
  uint16_t TXG      :2;  // b5:4 - TX PGA gain
  uint16_t reserved :10; // b15:6 reserved
}
HAL_afe031_gain_sel_t;

typedef struct
{  
    uint16_t ZC       :1;  // b0 - ZC enable
    uint16_t REF      :2;  // b2:1 - Ref1:2Ref1 enable
    uint16_t PA_OUT   :1;  // b3 - PA out enable
    uint16_t reserved :12; // reserved
}
HAL_afe031_enable2_t;

#define HAL_AFE031_BIAS_ON_DELAY        2000  //in us

typedef struct
{
    uint16_t TX_CAL    :1; // b0 - TX calibration enable
    uint16_t RX_CAL    :1; // b1 - RX calibration enable
    uint16_t reserved0 :1; // reserved
    uint16_t CA_CBCD   :1; // b3 - Cenelec A (0) or BCD (1) band
    uint16_t reserved1 :2; // reserved
    uint16_t TX_FLAG   :1; // b6 - TX flag status (read only)
    uint16_t RX_FLAG   :1; // b7 - RX flag status (read only)
    uint16_t reserved2 :8; // reserved
}
HAL_afe031_control1_t;

#define HAL_AFE031_BAND_CENELEC_A   0
#define HAL_AFE031_BAND_CENELEC_BCD 1

typedef struct
{
    uint16_t reserved0 :5; // b4:0 - reserved
    uint16_t TFLAG_EN  :1; // b5 - TFlag enable
    uint16_t IFLAG_EN  :1; // b6 - IFlag enable
    uint16_t reserved1 :9; // reserved
}HAL_afe031_control2_t;

typedef struct
{
    uint16_t reserved0 :2; // b1:0 - reserved
    uint16_t SOFT_RST  :3; // b4:2 - Writing 101 to soft reset
    uint16_t T_FLAG    :1; // b5 - TFlag status (write 0 to clear)
    uint16_t I_FLAG    :1; // b6 - IFlag status (write 0 to clear)
    uint16_t reserved1 :9; // reserved
}
HAL_afe031_reset_t;

typedef struct
{
    union
    {
        uint16_t             all;
        HAL_afe031_enable1_t bits;
    }
    enable_1;    
    
    union
    {
        uint16_t              all;
        HAL_afe031_gain_sel_t bits;
    }
    gain_select;

    union
    {
        uint16_t             all;   	
        HAL_afe031_enable2_t bits;
    }
    enable_2;

    union
    {
        uint16_t              all;
        HAL_afe031_control1_t bits;
    }
    control_1;

    union
    {
        uint16_t              all;   	
        HAL_afe031_control2_t bits;
    }
    control_2;
    
    union
    {
        uint16_t           all;   	
        HAL_afe031_reset_t bits;
    }
    reset; 
    
    uint16_t die_id;  
    uint16_t revision;
}    
HAL_afe031_reg_t;

void     AFE031_initialize( void );

uint16_t AFE031_read_register( uint16_t );
void     AFE031_write_register( uint16_t, uint16_t );

void     AFE031_soft_reset( void );
void     AFE031_enable_bias( void );
void     AFE031_select_band( uint16_t );
void     AFE031_disable_interrupts( void );
void     AFE031_enable_interrupts( void );
void     AFE031_enable_zero_crossing( void );
void     AFE031_write_gain( uint16_t, uint16_t );

void     AFE031_enable_dac_mode( void );
void     AFE031_enable_dac_mode_transmit( void );
void     AFE031_disable_transmit( void );
void     AFE031_enable_receive( void );

#endif
