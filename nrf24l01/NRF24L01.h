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

#ifndef __NRF24L01_H__
#define __NRF24L01_H__

#include <mbed-drivers/DigitalOut.h>
#include <mbed-hal/spi_api.h>

namespace nrf24l01
{

enum class DataRate { DR_250 = 0, DR_1000, DR_2000 };

enum class TxPwr { ZERO_DB, MINUS_6_DB, MINUS_12_DB, MINUS_18_DB };

enum class RadioType { NRF24L01, NRF24L01_PLUS };

/** Driver for NRF24L01/NRF24L01+ packet radio modules
 *
 */
class NRF24L01
{
    NRF24L01 & operator = (const NRF24L01 & other) = delete;
    NRF24L01(const NRF24L01 & other) = delete;
public:
    const RadioType radio_type;

    /** Create a driver object for a radio module connected to the specified pins
     * @param radio_type the radio type (NRF24L01 or NRF24L01+)
     * @param mosi SPI Master Out, Slave In pin
     * @param miso SPI Master In, Slave Out pin
     * @param sclk SPI Clock pin
     * @param ce Chip Enable pin
     * @param csn Chip Select pin
     */
    NRF24L01(RadioType radio_type, PinName mosi, PinName miso, PinName sclk, PinName ce, PinName csn);

    /** Initialize the radio module
     * @param data_rate Wireless data rate to set at initialization
     * @param tx_pwr Transmission power to set at initialization
     * @param channel Wireless channel to use
     * @param width Payload width
     * @param enable_rx Enable/disable reciever at initialization
     * @param enable_rt_irq
     * @param enable_tx_irq
     * @param enable_rx_irq
     */
    void init(DataRate data_rate, TxPwr tx_pwr, uint8_t channel, uint8_t width, bool enable_rx,
              bool enable_rt_irq, bool enable_tx_irq, bool enable_rx_irq);
    uint8_t get_status();
    void send_data(uint8_t *data, uint8_t len);
    void get_received_data(uint8_t *data, uint8_t len);
    uint8_t get_pipe_from_status(uint8_t status);
    uint8_t get_payload_len();
    void write_ack_payload(uint8_t pipe, uint8_t* data, uint8_t len);
    void activate(void);
    void enable_pipe(uint8_t pipe, uint8_t state);
    void set_tx(void);
    void set_rx(void);
    void set_tx_addr(uint8_t* addr, uint8_t len);
    void set_rx_addr(uint8_t pipe, uint8_t* addr, uint8_t len);
    void set_autoack_pipe(uint8_t pipe, uint8_t state);
    void set_payload_width(uint8_t pipe, uint8_t width);
    void flush_rx();
    void flush_tx();
    void set_channel(uint8_t channel);
    void set_tx_pwr(TxPwr tx_pwr);
    void enable_dyn_pld(bool enable);
    void enable_ack_pld(bool enable);
    void enable_dyn_ack(bool enable);
    void enable_dyn_pld_pipe(uint8_t pipe, uint8_t state);
    void power_up(void);
    void power_down(void);
    void set_rf_dr(DataRate data_rate);
    bool data_ready(void);
private:
    void set_register(uint8_t regaddr, uint8_t value);
    uint8_t get_register(uint8_t regaddr);
    void write_register(uint8_t regaddr, uint8_t *data, uint8_t len);
    void read_register(uint8_t regaddr, uint8_t *data, uint8_t len);
    uint8_t read_byte(uint8_t cmd);
    // SPI support methods
    uint8_t spi_fast_shift(uint8_t data);
    void spi_transmit_sync(uint8_t *data, uint8_t len);
    void spi_transfer_sync(uint8_t *data_in, uint8_t *data_out, uint8_t len);
    // IO support methods
    inline void csn_low() { _csn.write(0); }
    inline void csn_high() { _csn.write(1); }
    inline void ce_low() { _ce.write(0); }
    inline void ce_high() { _ce.write(1); }
    // Data members
    mbed::DigitalOut _ce;
    mbed::DigitalOut _csn;
    spi_t _spi;
};

}
#endif
