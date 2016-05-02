/*
Copyright (c) 2014 Tobias Schramm
Copyright (c) 2016 Brian Kauke

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include <nrf24l01/NRF24L01.h>
#include <nrf24l01/types.h>
#include <nrf24l01/constants.h>

#include <mbed-drivers/mbed.h>

using namespace nrf24l01;

NRF24L01::NRF24L01(RadioType radio_type, PinName mosi, PinName miso, PinName sclk, PinName csn, PinName ce)
    : radio_type(radio_type),
      _ce(ce),
      _csn(csn)
{
    ce_low();
    csn_high();
    spi_init(&_spi, mosi, miso, sclk);
    spi_format(&_spi, 8, 0, SPI_MSB);
    spi_frequency(&_spi, 1000000);
}

/************************************************************************/
/* Public methods                                                       */
/************************************************************************/

///Basic NRF24L01 initialization. Enables pipe 0 and pipe 1
void NRF24L01::init(DataRate data_rate, TxPwr tx_pwr, uint8_t channel, uint8_t width,
                    bool enable_rx, bool enable_rt_irq, bool enable_tx_irq, bool enable_rx_irq)
{
    uint8_t preset_baudrate_high, preset_baudrate_low;
    uint8_t preset_tx_pwr;

    flush_rx();
    flush_tx();
    set_register(NRF24L01_REG_STATUS, NRF24L01_MASK_STATUS_MAX_RT | NRF24L01_MASK_STATUS_RX_DR | NRF24L01_MASK_STATUS_TX_DS);

    set_tx_pwr(tx_pwr);

    switch (data_rate)
    {
        case DataRate::DR_250:
            assert(radio_type == RadioType::NRF24L01_PLUS);
            preset_baudrate_high = 0;
            preset_baudrate_low = 1;
            break;
        case DataRate::DR_1000:
            preset_baudrate_high = 0;
            preset_baudrate_low = 0;
            break;
        //case DataRate::DR_2000:
        default:    
            preset_baudrate_high = 1;
            preset_baudrate_low = 0;
            break;
    }

    switch (tx_pwr)
    {
        case TxPwr::ZERO_DB:
            preset_tx_pwr = 0b11;
            break;
        case TxPwr::MINUS_6_DB:
            preset_tx_pwr = 0b10;
            break;
        case TxPwr::MINUS_12_DB:
            preset_tx_pwr = 0x01;
            break;
        //case TxPwr::MINUS_18_DB:
        default:
            preset_tx_pwr = 0x00;
            break;
    }

    nrf24l01_config_t* config = (nrf24l01_config_t*)malloc(sizeof(nrf24l01_config_t));
    config->value = 0;
    config->prim_rx = (uint8_t)(enable_rx ? 1 : 0);
    config->pwr_up = 1;
    config->crco = 1;
    config->mask_max_rt = (uint8_t)(enable_rt_irq ? 1 : 0);
    config->mask_tx_ds = (uint8_t)(enable_tx_irq ? 1 : 0);
    config->mask_rx_dr = (uint8_t)(enable_rx_irq ? 1 : 0);
    config->en_crc = 1;
    set_register(NRF24L01_REG_CONFIG, config->value);
    free(config);

    nrf24l01_rf_ch_t* rf_ch = (nrf24l01_rf_ch_t*)malloc(sizeof(nrf24l01_rf_ch_t));
    rf_ch->value = 0;
    rf_ch->rf_ch = channel;
    set_register(NRF24L01_REG_RF_CH, rf_ch->value);
    free(rf_ch);

    nrf24l01_rf_setup_t* rf_setup = (nrf24l01_rf_setup_t*)malloc(sizeof(nrf24l01_rf_setup_t));
    rf_setup->value = 0;
    rf_setup->rf_pwr = preset_tx_pwr;
    rf_setup->rf_dr_high = preset_baudrate_high;
    rf_setup->rf_dr_low = preset_baudrate_low;
    set_register(NRF24L01_REG_RF_SETUP, rf_setup->value);
    free(rf_setup);

    nrf24l01_rx_pw_t* payload_width = (nrf24l01_rx_pw_t*)malloc(sizeof(nrf24l01_rx_pw_t));
    payload_width->rx_pw = width;
    set_register(NRF24L01_REG_RX_PW_P0, payload_width->value);
    set_register(NRF24L01_REG_RX_PW_P1, payload_width->value);
    free(payload_width);

    ce_high();
}

///Initializes data transmission. Automatically sets the NRF24L01 to TX mode
void NRF24L01::send_data(uint8_t* data, uint8_t len)
{
    ce_low();
    set_tx();
    csn_low();
    spi_fast_shift(NRF24L01_CMD_FLUSH_TX);
    csn_high();
    csn_low();
    spi_fast_shift(NRF24L01_CMD_W_TX_PAYLOAD);
    spi_transmit_sync(data, len);
    csn_high();
    ce_high();
    wait_us(10);
    ce_low();
}

///Wakes the NRF24L01 from standby mode.
void NRF24L01::power_up(void)
{
    uint8_t config = get_register(NRF24L01_REG_STATUS);
    config |= NRF24L01_MASK_CONFIG_PWR_UP;
    set_register(NRF24L01_REG_STATUS, config);
}

///Puts the NRF24L01 into standby mode.
void NRF24L01::power_down(void)
{
    uint8_t config = get_register(NRF24L01_REG_STATUS);
    config &= ~NRF24L01_MASK_CONFIG_PWR_UP;
    set_register(NRF24L01_REG_STATUS, config);
}

///Sets the frequency channel the NRF24L01 operates on.
///Must be inside range [0 .. 125]
void NRF24L01::set_channel(uint8_t channel)
{
    uint8_t rf_ch = channel;
    rf_ch &= NRF24L01_MASK_RF_CH_RF_CH;
    set_register(NRF24L01_REG_RF_CH, rf_ch);
}

///Sets the tx power of the NRF24L01
///Must be inside range [0 .. 3]
///0 = -18 dBm -> 0.016 mW
///1 = -12 dBm -> 0.063 mW
///2 = - 6 dBm -> 0.251 mW
///3 =   0 dBm -> 1.000 mW
void NRF24L01::set_tx_pwr(TxPwr tx_pwr)
{
    uint8_t value;
    switch (tx_pwr)
    {
        case TxPwr::ZERO_DB:
            value = 0b11;
            break;
        case TxPwr::MINUS_6_DB:
            value = 0b10;
            break;
        case TxPwr::MINUS_12_DB:
            value = 0x01;
            break;
        //case TxPwr::MINUS_18_DB:
        default:
            value = 0x00;
            break;
    }

    uint8_t rf_setup = get_register(NRF24L01_REG_RF_SETUP);
    rf_setup &= ~NRF24L01_MASK_RF_SETUP_RF_PWR;
    rf_setup |= (value << 1) & NRF24L01_MASK_RF_SETUP_RF_PWR;
    rf_setup &= NRF24L01_MASK_RF_SETUP;
    set_register(NRF24L01_REG_RF_SETUP, rf_setup);
}

///Enables the dynamic payload feature.
void NRF24L01::enable_dyn_pld(bool enable)
{
    uint8_t features = get_register(NRF24L01_REG_FEATURE);
    features &= ~NRF24L01_MASK_FEATURE_EN_DPL;
    features |= NRF24L01_MASK_FEATURE_EN_DPL & ((enable ? 1 : 0) << 2);
    set_register(NRF24L01_REG_FEATURE, features);
}

///Enables the dynamic payload feature for the given pipe.
///Overrides the payload width setting for the pipe.
void NRF24L01::enable_dyn_pld_pipe(uint8_t pipe, uint8_t state)
{
    uint8_t dynpld_pipes = get_register(NRF24L01_REG_DYNPD);
    if(state)
        dynpld_pipes |= (1 << pipe);
    else
        dynpld_pipes &= ~(1 << pipe);
    set_register(NRF24L01_REG_DYNPD, dynpld_pipes);
}

///Enables the acknowledge payload feature. Should only be used with dynamic
///payload size. Otherwise ack payloads my be discarded by the receiver
void NRF24L01::enable_ack_pld(bool enable)
{
    uint8_t features = get_register(NRF24L01_REG_FEATURE);
    features &= ~NRF24L01_MASK_FEATURE_EN_ACK_PAY;
    features |= NRF24L01_MASK_FEATURE_EN_ACK_PAY & ((enable ? 1 : 0) << 1);
    set_register(NRF24L01_REG_FEATURE, features);
}

///Enables the dynamic acknowledge feature. This feature allows single packets to
///contain a 'do not acknowledge' flag. The receiver will then not send an automatic
///acknowledgment for this packet.
void NRF24L01::enable_dyn_ack(bool enable)
{
    uint8_t features = get_register(NRF24L01_REG_FEATURE);
    features &= ~NRF24L01_MASK_FEATURE_EN_DYN_ACK;
    features |= NRF24L01_MASK_FEATURE_EN_DYN_ACK & (enable ? 1 : 0);
    set_register(NRF24L01_REG_FEATURE, features);
}

///Enables / Disables auctomatic packet acknowledgement for the givent pipe.
///By default all pipes have this feature turned on.
void	NRF24L01::set_autoack_pipe(uint8_t pipe, uint8_t state)
{
    uint8_t auto_ack_pipes = get_register(NRF24L01_REG_EN_AA);
    auto_ack_pipes &= ~(1<<pipe);
    auto_ack_pipes |= (1<<pipe);
    set_register(NRF24L01_REG_EN_AA, auto_ack_pipes);
}

///Sets the data rate used on the RF side
///Must be inside range [0 .. 2]
///0 = 0.25 Mbps
///1 = 1.00 Mbps
///2 = 2.00 Mbps
///0.25 Mbps mode works only with NRF24L01+
void NRF24L01::set_rf_dr(DataRate data_rate)
{
    uint8_t rf_setup = get_register(NRF24L01_REG_RF_SETUP);
    switch(data_rate)
    {
        case DataRate::DR_250:
            assert(radio_type == RadioType::NRF24L01_PLUS);
            rf_setup &=	~NRF24L01_MASK_RF_SETUP_RF_DR_HIGH;
            rf_setup |=	NRF24L01_MASK_RF_SETUP_RF_DR_LOW;
            break; //250 kbps
        case DataRate::DR_1000:
            rf_setup &=	~NRF24L01_MASK_RF_SETUP_RF_DR_HIGH;
            rf_setup &=	~NRF24L01_MASK_RF_SETUP_RF_DR_LOW;
            break; //1 Mbps
        case DataRate::DR_2000:
            rf_setup |=	NRF24L01_MASK_RF_SETUP_RF_DR_HIGH;
            rf_setup &= ~NRF24L01_MASK_RF_SETUP_RF_DR_LOW;
            break; //2 Mbps
    }
    rf_setup &= NRF24L01_MASK_RF_SETUP;
    set_register(NRF24L01_REG_RF_SETUP, rf_setup);
}

///Reads fifo status and returns if rx fifo is not empty
bool NRF24L01::data_ready(void)
{
    uint8_t fifo_status = get_register(NRF24L01_REG_FIFO_STATUS);
    return (fifo_status & NRF24L01_MASK_FIFO_STATUS_RX_EMPTY) != 0;
}

///Reads data received by the NRF24L01
void NRF24L01::get_received_data(uint8_t* data, uint8_t len)
{
    csn_low();
    spi_fast_shift(NRF24L01_CMD_R_RX_PAYLOAD);
    spi_transfer_sync(data, data, len);
    csn_high();
    set_register(NRF24L01_REG_STATUS, NRF24L01_MASK_STATUS_RX_DR);
}

///Reads the NRF24L01's actual status
uint8_t NRF24L01::get_status(void)
{
    return read_byte(NRF24L01_CMD_NOP);
}

///Reads the pipe data was received on from the NRF24L01's status
uint8_t	NRF24L01::get_pipe_from_status(uint8_t status)
{
    return (uint8_t)((status & NRF24L01_MASK_STATUS_RX_P_NO) << 1);
}

uint8_t NRF24L01::get_payload_len()
{
    uint8_t len;
    csn_low();
    len = spi_fast_shift(NRF24L01_CMD_R_RX_PL_WID);
    csn_high();
    return len;
}

///Writes the additional payload to be transmitted together with the auto
///acknowledgment data on the given pipe. Works only when dynamic payload
///and acknowledgment payload feature are turned on
void NRF24L01::write_ack_payload(uint8_t pipe, uint8_t* data, uint8_t len)
{
    csn_low();
    spi_fast_shift((uint8_t)NRF24L01_CMD_W_ACK_PAYLOAD | pipe);
    spi_transmit_sync(data, len);
    csn_high();
}


///NRF24L01 ONLY
///Calling this function toggles the availability of the acknowledgment payload, dynamic
///payload and dynamic acknowledgment bits int he feature register. Disabled by default.
void NRF24L01::activate(void)
{
    csn_low();
    spi_fast_shift(NRF24L01_CMD_ACTIVATE);
    spi_fast_shift(0x73);
    csn_high();
}

///Enables / Disables single data pipes. By default pipe 0 and pipe 1 are enabled
void NRF24L01::enable_pipe(uint8_t pipe, uint8_t state)
{
    uint8_t pipes = get_register(NRF24L01_REG_EN_RXADDR);
    if(state)
        pipes |= (1<<pipe);
    else
        pipes &= ~(1<<pipe);
    set_register(NRF24L01_REG_EN_RXADDR, pipes);
}

///Sets the tx address of the NRF24L01
void NRF24L01::set_tx_addr(uint8_t* addr, uint8_t len)
{
    write_register(NRF24L01_REG_TX_ADDR, addr, len);
}

///Sets the rx address for the specified pipe
void NRF24L01::set_rx_addr(uint8_t pipe, uint8_t* addr, uint8_t len)
{
    write_register((uint8_t)NRF24L01_REG_RX_ADDR_P0 + pipe, addr, len);
}

///Sets the payload length for the specified pipe
void NRF24L01::set_payload_width(uint8_t pipe, uint8_t width)
{
    set_register((uint8_t)NRF24L01_REG_RX_PW_P0 + pipe, width & (uint8_t)NRF24L01_MASK_RX_PW_P0);
}

///Puts the NRF24L01 into receive mode
void NRF24L01::set_rx(void)
{
    uint8_t config = get_register(NRF24L01_REG_CONFIG);
    config |= NRF24L01_MASK_CONFIG_PRIM_RX;
    config |= NRF24L01_MASK_CONFIG_PWR_UP;
    config &= NRF24L01_MASK_CONFIG;
    set_register(NRF24L01_REG_CONFIG, config);
    ce_high();
}

///Puts the NRF24L01 into transmit mode
void NRF24L01::set_tx(void)
{
    uint8_t config = get_register(NRF24L01_REG_CONFIG);
    config &= ~NRF24L01_MASK_CONFIG_PRIM_RX;
    config |= NRF24L01_MASK_CONFIG_PWR_UP;
    config &= NRF24L01_MASK_CONFIG;
    set_register(NRF24L01_REG_CONFIG, config);
}

///Flushes the NRF's receive packet buffer
void NRF24L01::flush_rx()
{
    csn_low();
    spi_fast_shift(NRF24L01_CMD_FLUSH_RX);
    csn_high();
}

///Flushes the NRF's transmit packet buffer
void NRF24L01::flush_tx()
{
    csn_low();
    spi_fast_shift(NRF24L01_CMD_FLUSH_TX);
    csn_high();
}

/************************************************************************/
/* Private methods                                                      */
/************************************************************************/

///Sets one byte in the NRF24L01's config registers
void NRF24L01::set_register(uint8_t regaddr, uint8_t val)
{
    csn_low();
    spi_fast_shift((uint8_t)NRF24L01_CMD_W_REGISTER | regaddr);
    spi_fast_shift(val);
    csn_high();
}

///Gets one byte from the NRF24L01's config registers
uint8_t NRF24L01::get_register(uint8_t regaddr)
{
    uint8_t byte;
    read_register(regaddr, &byte, 1);
    return byte;
}

///Sets multi byte registers in the NRF24L01
void NRF24L01::write_register(uint8_t regaddr, uint8_t* data, uint8_t len)
{
    csn_low();
    spi_fast_shift((uint8_t)NRF24L01_CMD_W_REGISTER | regaddr);
    spi_transmit_sync(data, len);
    csn_high();
}

///Reads multi byte registers from the NRF24L01
void NRF24L01::read_register(uint8_t regaddr, uint8_t* data, uint8_t len)
{
    csn_low();
    spi_fast_shift((uint8_t)NRF24L01_CMD_R_REGISTER | regaddr);
    spi_transfer_sync(data, data, len);
    csn_high();
}

///Sends a single one byte command to the NRF24L01 and gets the single byte response
uint8_t NRF24L01::read_byte(uint8_t cmd)
{
    csn_low();
    uint8_t data = spi_fast_shift(cmd);
    csn_high();
    return data;
}

uint8_t NRF24L01::spi_fast_shift(uint8_t data)
{
    return (uint8_t)spi_master_write(&_spi, data);
}

void NRF24L01::spi_transmit_sync(uint8_t *data, uint8_t len)
{
    for (uint8_t i=0; i < len; ++i)
    {
        spi_master_write(&_spi, data[i]);
    }
}

void NRF24L01::spi_transfer_sync(uint8_t *data_in, uint8_t *data_out, uint8_t len)
{
    for (uint8_t i=0; i < len; ++i)
    {
        data_out[i] = (uint8_t)spi_master_write(&_spi, data_in[i]);
    }
}
