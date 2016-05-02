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
     * @param csn Chip Select pin
     * @param ce Chip Enable pin
     */
    NRF24L01(RadioType radio_type, PinName mosi, PinName miso, PinName sclk, PinName csn, PinName ce);

    /** Initialize the radio module. Enables pipe 0 and pipe 1.
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
    
    /** Read the module status
     * @return Status byte
     */
    uint8_t get_status();
    
    /** Initiate data transmission. Automatically sets the radio to TX mode.
     * @param data Pointer to data to send
     * @param len Length of data in bytes
     */
    void send_data(uint8_t *data, uint8_t len);
    
    /** Read data received by the module
     * @param data Destination pointer for data
     * @param len Length of data to read
     */
    void get_received_data(uint8_t *data, uint8_t len);
    
    /** From a status byte, get the pipe data was last received on
     * @param status Status byte
     * @return Pipe number
     */
    uint8_t get_pipe_from_status(uint8_t status);
    
    /** Get the currently configured payload length
     * @return Paylod length
     */
    uint8_t get_payload_len();
    
    /** Writes the additional payload to be transmitted together with the auto
     *  acknowledgment data on the given pipe. Works only when dynamic payload 
     *  and acknowledgment payload feature are turned on.
     * @param pipe Pipe to configure
     * @data Pointer to data for autoack payload
     * @len Length of data for autoack payload
     */
    void write_ack_payload(uint8_t pipe, uint8_t* data, uint8_t len);
    

    /** Calling this function toggles the availability of the acknowledgment payload, dynamic 
     *  payload and dynamic acknowledgment bits int he feature register. Disabled by default.
     *  Note: NRF24L01 ONLY
     */
    void activate(void);
    
    /** Enable or disable a single data pipe. By default pipe 0 and pipe 1 are enabled.
     * @param pipe Pipe to configure
     * @param enable True to enable, false to disable
     */
    void enable_pipe(uint8_t pipe, bool enable);
    
    /** Put the module into transmit mode
     */
    void set_tx(void);
    
    /** Put the module into receive mode
     */
    void set_rx(void);
    
    /** Set the transmit address of the module
     * @param addr Pointer to address
     * @param len Length of the address in bytes
     */
    void set_tx_addr(uint8_t* addr, uint8_t len);

    /** Set the receive address for the specified pipe
     * @param pipe Pipe to configure
     * @param addr Pointer to address
     * @param len Length of the address in bytes
     */
    void set_rx_addr(uint8_t pipe, uint8_t* addr, uint8_t len);

    /** Enable or disable automatic packet acknowledgement for the given pipe.
     *  By default all pipes have this feature turned on.
     * @param pipe Pipe to configure
     * @param enable True to enable, false to disable
     */
    void set_autoack_pipe(uint8_t pipe, bool enable);

    /** Set the payload width for the specified pipe
     * @param pipe Pipe to configure
     * @param width Payload width
     */
    void set_payload_width(uint8_t pipe, uint8_t width);
    
    /** Flush the module's receive packet buffer
     */
    void flush_rx();
    
    /** Flush the module's transmit packet buffer
     */
    void flush_tx();
    
    /** Set the channel the radio operates on
     * @param channel Radio channel
     */
    void set_channel(uint8_t channel);

    /** Set the transmit power of the radio
     * @param tx_pwr Transmit power
     */
    void set_tx_pwr(TxPwr tx_pwr);
    
    /** Enable or disable the dynamic payload feature
     * @param enable True to enable, false to disable
     */
    void enable_dyn_pld(bool enable);

    /** Enable or disable the acknowledge payload feature.
     *  Should only be used with dynamic payload size. Otherwise ack payloads
     *  may be discarded by the reciever.
     * @param enable True for enable, false for disable
     */
    void enable_ack_pld(bool enable);

    /** Enable or disable the dynamic acknowledge feature.
     *  This feature allows single packets to contain a 'do not acknowledge' flag.
     *  The reciever will then not send an automatic acknowledgement for this packet.
     * @param enable True to enable, false to disable
     */
    void enable_dyn_ack(bool enable);

    /** Enable or disable the dynamic payload feature for the given pipe.
     *  Overrides the payload width setting for the pipe.
     * @param pipe Pipe to configure
     * @param enable True to enable, false to disable
     */
    void enable_dyn_pld_pipe(uint8_t pipe, bool enable);
    
    /** Wake the module from standby mode
     */
    void power_up(void);
    
    /** Put the module into standby mode
     */
    void power_down(void);
    
    /** Set the data rate used on the RF side
     * @param data_rate Data rate
     */
    void set_rf_dr(DataRate data_rate);
    
    /** Reads FIFO status and returns true if the rx FIFO is not empty
     */
    bool data_ready(void);
    
private:
    //
    // Low-level functions
    //
    void set_register(uint8_t regaddr, uint8_t value);
    uint8_t get_register(uint8_t regaddr);
    void write_register(uint8_t regaddr, uint8_t *data, uint8_t len);
    void read_register(uint8_t regaddr, uint8_t *data, uint8_t len);
    uint8_t read_byte(uint8_t cmd);
    
    //
    // SPI support methods
    //
    uint8_t spi_fast_shift(uint8_t data);
    void spi_transmit_sync(uint8_t *data, uint8_t len);
    void spi_transfer_sync(uint8_t *data_in, uint8_t *data_out, uint8_t len);
    
    //
    // IO support methods
    //
    inline void csn_low() { _csn.write(0); }
    inline void csn_high() { _csn.write(1); }
    inline void ce_low() { _ce.write(0); }
    inline void ce_high() { _ce.write(1); }
    
    //
    // Data members
    //
    mbed::DigitalOut _ce;
    mbed::DigitalOut _csn;
    spi_t _spi;
};

}
#endif
