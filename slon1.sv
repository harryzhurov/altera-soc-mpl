


module slon1
(
    input           clk,
	
	output	[14:0]	memory_mem_a,
	output	[2:0]	memory_mem_ba,
	output		    memory_mem_ck,
	output	    	memory_mem_ck_n,
	output		    memory_mem_cke,
	output		    memory_mem_cs_n,
	output		    memory_mem_ras_n,
	output		    memory_mem_cas_n,
	output		    memory_mem_we_n,
	output		    memory_mem_reset_n,
	inout	[7:0]	memory_mem_dq,
	inout		    memory_mem_dqs,
	inout		    memory_mem_dqs_n,
	output	    	memory_mem_odt,
	output		    memory_mem_dm,
	input		    memory_oct_rzqin,
    
	input	    	hps_io_hps_io_uart0_inst_RX,
	output   		hps_io_hps_io_uart0_inst_TX,

	inout		    hps_io_hps_io_qspi_inst_IO0,
	inout		    hps_io_hps_io_qspi_inst_IO1,
	inout	    	hps_io_hps_io_qspi_inst_IO2,
	inout		    hps_io_hps_io_qspi_inst_IO3,
	output		    hps_io_hps_io_qspi_inst_SS0,
	output		    hps_io_hps_io_qspi_inst_CLK,

    
	inout		    hps_io_hps_io_gpio_inst_GPIO53,
	inout		    hps_io_hps_io_gpio_inst_GPIO54,
	inout		    hps_io_hps_io_gpio_inst_GPIO55,
	inout		    hps_io_hps_io_gpio_inst_GPIO56
);




wire reset_n = 1'b1;

slon1_soc u0 
(
	.clk_clk       (clk),       //   clk.clk
	.reset_reset_n (reset_n),   // reset.reset_n

    .memory_mem_a                   (memory_mem_a        ),           // memory.mem_a
    .memory_mem_ba                  (memory_mem_ba       ),           //       .mem_ba
    .memory_mem_ck                  (memory_mem_ck       ),           //       .mem_ck
    .memory_mem_ck_n                (memory_mem_ck_n     ),           //       .mem_ck_n
    .memory_mem_cke                 (memory_mem_cke      ),           //       .mem_cke
    .memory_mem_cs_n                (memory_mem_cs_n     ),           //       .mem_cs_n
    .memory_mem_ras_n               (memory_mem_ras_n    ),           //       .mem_ras_n
    .memory_mem_cas_n               (memory_mem_cas_n    ),           //       .mem_cas_n
    .memory_mem_we_n                (memory_mem_we_n     ),           //       .mem_we_n
    .memory_mem_reset_n             (memory_mem_reset_n  ),           //       .mem_reset_n
    .memory_mem_dq                  (memory_mem_dq       ),           //       .mem_dq
    .memory_mem_dqs                 (memory_mem_dqs      ),           //       .mem_dqs
    .memory_mem_dqs_n               (memory_mem_dqs_n    ),           //       .mem_dqs_n
    .memory_mem_odt                 (memory_mem_odt      ),           //       .mem_odt
    .memory_mem_dm                  (memory_mem_dm       ),           //       .mem_dm
    .memory_oct_rzqin               (memory_oct_rzqin    ),           //       .oct_rzqin

    .hps_io_hps_io_uart0_inst_RX    (hps_io_hps_io_uart0_inst_RX),    // hps_io.hps_io_uart0_inst_RX
    .hps_io_hps_io_uart0_inst_TX    (hps_io_hps_io_uart0_inst_TX),    //       .hps_io_uart0_inst_TX

    .hps_io_hps_io_qspi_inst_IO0    (hps_io_hps_io_qspi_inst_IO0),    // hps_io.hps_io_qspi_inst_IO0
    .hps_io_hps_io_qspi_inst_IO1    (hps_io_hps_io_qspi_inst_IO1),    //       .hps_io_qspi_inst_IO1
    .hps_io_hps_io_qspi_inst_IO2    (hps_io_hps_io_qspi_inst_IO2),    //       .hps_io_qspi_inst_IO2
    .hps_io_hps_io_qspi_inst_IO3    (hps_io_hps_io_qspi_inst_IO3),    //       .hps_io_qspi_inst_IO3
    .hps_io_hps_io_qspi_inst_SS0    (hps_io_hps_io_qspi_inst_SS0),    //       .hps_io_qspi_inst_SS0
    .hps_io_hps_io_qspi_inst_CLK    (hps_io_hps_io_qspi_inst_CLK),    //       .hps_io_qspi_inst_CLK

	.hps_io_hps_io_gpio_inst_GPIO53(hps_io_hps_io_gpio_inst_GPIO53),
	.hps_io_hps_io_gpio_inst_GPIO54(hps_io_hps_io_gpio_inst_GPIO54),
	.hps_io_hps_io_gpio_inst_GPIO55(hps_io_hps_io_gpio_inst_GPIO55),
	.hps_io_hps_io_gpio_inst_GPIO56(hps_io_hps_io_gpio_inst_GPIO56)
	
);

endmodule
