#include "driverlib.h"
#include "board.h"
#include "c2000ware_libraries.h"

#include "hal_afe031.h"

static HAL_afe031_reg_t afe031_registers;

void AFE031_initialize( void )
{
    // configuration of pinmux and SPI performed in Board_init()
    
    AFE031_soft_reset();
    DEVICE_DELAY_US(2);

    AFE031_enable_bias();
    DEVICE_DELAY_US(2);

    AFE031_select_band( HAL_AFE031_BAND_CENELEC_BCD );
    DEVICE_DELAY_US(2);

    AFE031_disable_interrupts();
    DEVICE_DELAY_US(2);

    AFE031_enable_zero_crossing();
    DEVICE_DELAY_US(2);

    // set default RX/TX gains
    AFE031_write_gain( 6/*4*/, 1 ); // TODO: change when function is correctly implemented
    DEVICE_DELAY_US(2);
}

uint16_t AFE031_read_register( uint16_t reg )
{
    uint16_t tx_data = ((HAL_AFE031_CMD_RD << HAL_AFE031_CMD_RW_SHIFT) | reg) << HAL_AFE031_CMD_SHIFT;

    SPI_writeDataBlockingNonFIFO(DAC_SPI_BASE, tx_data);

    DEVICE_DELAY_US(2);
    SPI_writeDataBlockingNonFIFO(DAC_SPI_BASE, 0xFFFF);     // dummy write

    DEVICE_DELAY_US(2);
    uint16_t rx_data = SPI_readDataBlockingNonFIFO(DAC_SPI_BASE);

    return (rx_data & 0x00FF);
}

void AFE031_write_register( uint16_t addr, uint16_t data )
{
    uint16_t tx_data = ((HAL_AFE031_CMD_WR << HAL_AFE031_CMD_RW_SHIFT) | addr) << HAL_AFE031_CMD_SHIFT;
    tx_data |= ((data & HAL_AFE031_DATA_MASK) << HAL_AFE031_DATA_SHIFT);

    SPI_writeDataBlockingNonFIFO(DAC_SPI_BASE, tx_data);

    DEVICE_DELAY_US(2);
}

void AFE031_soft_reset( void )
{
    afe031_registers.reset.bits.SOFT_RST = 5;   // write b101 to soft reset
    AFE031_write_register( HAL_AFE031_RESET_REG, afe031_registers.reset.all );
}

void AFE031_enable_bias( void )
{
    afe031_registers.enable_2.bits.REF = 3;     // enable both references
    AFE031_write_register( HAL_AFE031_ENABLE2_REG, afe031_registers.enable_2.all );

    DEVICE_DELAY_US( HAL_AFE031_BIAS_ON_DELAY );
}

// band shold be HAL_AFE031_BAND_CENELEC_A or HAL_AFE031_BAND_CENELEC_BCD
void AFE031_select_band( uint16_t band )
{
    afe031_registers.control_1.bits.CA_CBCD = band;
    AFE031_write_register( HAL_AFE031_CTRL_REG, afe031_registers.control_1.all );
}

void AFE031_disable_interrupts( void )
{
    afe031_registers.control_2.bits.TFLAG_EN = 0;
    afe031_registers.control_2.bits.IFLAG_EN = 0;
    AFE031_write_register( HAL_AFE031_CTRL2_REG, afe031_registers.control_2.all );
}

void AFE031_enable_interrupts( void )
{
    // disable and clear interrupts first
    AFE031_disable_interrupts();
    DEVICE_DELAY_US(2);
    AFE031_write_register( HAL_AFE031_RESET_REG, 0 );
    DEVICE_DELAY_US(2);

    // now enable all interrupts
    afe031_registers.control_2.bits.TFLAG_EN = 1;
    afe031_registers.control_2.bits.IFLAG_EN = 1;
    AFE031_write_register( HAL_AFE031_CTRL2_REG, afe031_registers.control_2.all );
    DEVICE_DELAY_US(2);
}

void AFE031_enable_zero_crossing( void )
{
    afe031_registers.enable_2.bits.ZC = 1;
    AFE031_write_register( HAL_AFE031_ENABLE2_REG, afe031_registers.enable_2.all );
}

void AFE031_write_gain( uint16_t rx_gain, uint16_t tx_gain )
{
    // TODO: implement based on proof of concept code
    afe031_registers.gain_select.bits.TXG = tx_gain & 0x0F;
    afe031_registers.gain_select.bits.RXG = rx_gain & 0x0F;
    AFE031_write_register( HAL_AFE031_GAINSEL_REG, afe031_registers.gain_select.all );
}

void AFE031_enable_dac_mode( void )
{
    // enable DAC block
    afe031_registers.enable_1.bits.DAC = 1; // enable DAC (digital-to-analog converter) block
    AFE031_write_register( HAL_AFE031_ENABLE_REG, afe031_registers.enable_1.all );
    DEVICE_DELAY_US(2);

    // configure DAC-mode SPI word length
    SPI_setcharLength(DAC_SPI_BASE, HAL_AFE031_SPI_DAC_LEN);

    // drive DAC mode pin high
    GPIO_writePin(PLC_DAC_MODE_SEL, 1);
}

void AFE031_enable_dac_mode_transmit( void )
{
    afe031_registers.enable_1.bits.TX  = 1; // enable TX block (TX programmable gain amplifier and TX filter)
    afe031_registers.enable_1.bits.PA  = 1; // enable PA (power amplifier) block
    afe031_registers.enable_1.bits.DAC = 1; // enable DAC (digital-to-analog converter) block
    AFE031_write_register( HAL_AFE031_ENABLE_REG, afe031_registers.enable_1.all );
    DEVICE_DELAY_US(2);

    // enable PA output stage
    afe031_registers.enable_2.bits.PA_OUT = 1;
    AFE031_write_register( HAL_AFE031_ENABLE2_REG, afe031_registers.enable_2.all );
}

void AFE031_disable_transmit( void )
{
    // disable PA output stage
    afe031_registers.enable_2.bits.PA_OUT = 0;
    AFE031_write_register( HAL_AFE031_ENABLE2_REG, afe031_registers.enable_2.all );
    DEVICE_DELAY_US(2);

    afe031_registers.enable_1.bits.TX  = 0; // disable TX block (TX programmable gain amplifier and TX filter)
    afe031_registers.enable_1.bits.PA  = 0; // disable PA (power amplifier) block
    AFE031_write_register( HAL_AFE031_ENABLE_REG, afe031_registers.enable_1.all );
    DEVICE_DELAY_US(2);    
}

void AFE031_enable_receive( void )
{
    afe031_registers.enable_1.bits.RX  = 1; // enable RX block (RX programmable gain amplifier and RX filter)
    AFE031_write_register( HAL_AFE031_ENABLE_REG, afe031_registers.enable_1.all );
}
