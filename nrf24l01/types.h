/*
Copyright (c) 2014 Tobias Schramm

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

#ifndef __NRF24L01_TYPES_H__
#define __NRF24L01_TYPES_H__

	//Typedefs
	typedef union
	{
		struct
		{
			uint8_t tx_full:1;
			uint8_t rx_p_no:3;
			uint8_t max_rt:1;
			uint8_t tx_ds:1;
			uint8_t rx_dr:1;
			uint8_t reserved:1;
		};
		uint8_t value;
	}
	nrf24l01_status_t;

	typedef union
	{
		struct
		{
			uint8_t prim_rx:1;
			uint8_t pwr_up:1;
			uint8_t crco:1;
			uint8_t en_crc:1;
			uint8_t mask_max_rt:1;
			uint8_t mask_tx_ds:1;
			uint8_t mask_rx_dr:1;
			uint8_t reserved:1;
		};
		uint8_t value;
	}
	nrf24l01_config_t;

	typedef union
	{
		struct
		{
			uint8_t enaa_p0:1;
			uint8_t enaa_p1:1;
			uint8_t enaa_p2:1;
			uint8_t enaa_p3:1;
			uint8_t enaa_p4:1;
			uint8_t enaa_p5:1;
			uint8_t reserved:2;
		};
		uint8_t value;
	}
	nrf24l01_shockburst_t;

	typedef union
	{
		struct
		{
			uint8_t erx_p0:1;
			uint8_t erx_p1:1;
			uint8_t erx_p2:1;
			uint8_t erx_p3:1;
			uint8_t erx_p4:1;
			uint8_t erx_p5:1;
			uint8_t reserved:2;
		};
		uint8_t value;
	}
	nrf24l01_en_rxaddr_t;

	typedef union
	{
		struct
		{
			uint8_t aw:2;
			uint8_t reserved:6;
		};
		uint8_t value;
	}
	nrf24l01_setup_aw_t;

	typedef union
	{
		struct
		{
			uint8_t arc:4;
			uint8_t ard:4;
		};
		uint8_t value;
	}
	nrf24l01_setup_retr_t;

	typedef union
	{
		struct
		{
			uint8_t rf_ch:7;
			uint8_t reserved:1;
		};
		uint8_t value;
	}
	nrf24l01_rf_ch_t;

	typedef union
	{
		struct
		{
			uint8_t lna_hcurr:1;
			uint8_t rf_pwr:2;
			uint8_t rf_dr_high:1;
			uint8_t pll_lock:1;
			uint8_t rf_dr_low:1;
			uint8_t reserved:1;
			uint8_t cont_wave:1;
		};
		uint8_t value;
	}
	nrf24l01_rf_setup_t;

	typedef union
	{
		struct
		{
			uint8_t arc_cnt:4;
			uint8_t plos_cnt:4;
		};
		uint8_t value;
	}
	nrf24l01_observe_tx_t;

	typedef union
	{
		struct
		{
			uint8_t cd:1;
			uint8_t reserved:7;
		};
		uint8_t value;
	}
	nrf24l01_cd_t;

	typedef union
	{
		struct
		{
			uint8_t rx_pw:6;
			uint8_t reserved:2;
		};
		uint8_t value;
	}
	nrf24l01_rx_pw_t;

	typedef union
	{
		struct
		{
			uint8_t rx_empty:1;
			uint8_t rx_full:1;
			uint8_t reserved:2;
			uint8_t tx_empty:1;
			uint8_t tx_full:1;
			uint8_t tx_reuse:1;
			uint8_t reserved1:1;
		};
		uint8_t value;
	}
	nrf24l01_fifo_status_t;

	typedef union
	{
		struct
		{
			uint8_t dpl_p0:1;
			uint8_t dpl_p1:1;
			uint8_t dpl_p2:1;
			uint8_t dpl_p3:1;
			uint8_t dpl_p4:1;
			uint8_t dpl_p5:1;
			uint8_t reserved:2;
		};
		uint8_t value;
	}
	nrf24l01_dynpd_t;

	typedef union
	{
		struct
		{
			uint8_t en_dyn_ack:1;
			uint8_t en_ack_pay:1;
			uint8_t en_dpl:1;
			uint8_t reserved:5;
		};
		uint8_t value;
	}
	nrf24l01_feature_t;

#endif
