`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: York University
// Engineer: John Bagshaw
// Create Date:    05-07-2022
// Module Name:    nt1065_v5_top
// Project Name: GPS Receiver NT1065 Front-end
//////////////////////////////////////////////////////////////////////////////////
module nt1065_v5_top (
    //! global reset
    input  glbl_rst    ,
    input  clk_in      ,  
    // SPI Port
    output SPI_SS      ,
    output SPI_MOSI    ,
    input  SPI_MISO    ,
    output SPI_CLK     ,
    //		-- clk from NT1065
    input  CLK_OUT_1   ,
    input  CLK_OUT_2   ,
    //		-- NT1065 data chenel
    input  DATA_MagN1_p,
    input  DATA_MagN1_n,
    input  DATA_SigN1_p,
    input  DATA_SigN1_n,
    //UART Port
    input  UART_RX     ,
    output UART_TX
  );

  //===========================================================
  parameter      ZERO_STATE	= 6'b000000,
                 IDLE       = 6'b000001,
                 MY_VER     = 6'b000010,
                 MY_TX		= 6'b000011,
                 GEN_ACK	= 6'b000100,
                 GEN_ACK1	= 6'b000101,
                 GEN_ACK2	= 6'b000110,
                 WR_ACK		= 6'b000111,

                 SPI_WR		= 6'b001000,
                 SPI_WR1	= 6'b001001,
                 SPI_WR2	= 6'b001010,
                 SPI_WR3	= 6'b001011,
                 SPI_WR4	= 6'b001100,
                 SPI_WR5	= 6'b001101,

                 SPI_RD		= 6'b010000,
                 SPI_RD1	= 6'b010001,
                 SPI_RD2	= 6'b010010,
                 SPI_RD3	= 6'b010011,
                 SPI_RD4	= 6'b010100,
                 SPI_RD5	= 6'b010101,

                 DATA_WR	= 6'b010111,

                 SPI_SEL    = 6'b011111,

                 RAM_ADDR	= 6'b100000,
                 RAM_RD_DEL	= 6'b100001,
                 RAM_TX		= 6'b100011,

                 SET_DEL	= 6'b110010,
                 SET_DEL1	= 6'b110011,

                 UART_SPEED_1    = 115200,
                 UART_SPEED_2    = 460800,
                 RAM_SIZE		 = 15'd32767;

  wire [7:0] spi_data_read;
  wire       SPI_SS_int, SPI_MOSI_int, SPI_CLK_int;
  wire       SPI_MISO_int ;
  wire       glbl_rstn    ;

  reg [ 5:0] gen_state             = IDLE ;
  reg [ 3:0] tx_cnt                       ;
  reg [ 7:0] data2tx        [11:0]        ;
  reg [ 2:0] ram_rd_cnt                   ;
  reg        LED_S_int             = 1'b0 ;
  reg [14:0] ram_addr_buf                 ;
  reg [ 1:0] xcs_sum                      ;
  reg [ 2:0] adc_spi_cnt                  ;
  reg [ 2:0] select_ADC_data              ;
  reg [ 1:0] sel_calibr_ch         = 2'b00;
  
  reg	del_ld = 1'b0;
  wire  idelctrl_rdy;
  reg 	[4:0] del_value = 5'b00000;
  wire	[4:0] del_out;

  //===============================================================================================
  //======= CRC =======
  //===============================================================================================
  reg [15:0] crc_data, crc_result;
  reg [ 7:0] crc_array[0:3];

  function [15:0] crc_calc;
    input n;
    integer n;
    reg  [15:0] result, x, res1, res2;
    reg	[7:0] a;
    integer ind;
    begin
      result = 16'b1111111111111111;
      ind = 0;
      while (ind < n)
      begin
        a = crc_array[ind];
        x = (result >> 8) ^ ({8'b00000000,a});
        x = x ^ (x >> 4);
        res1 = (result << 8) ^ (x << 12);
        res2 = (x << 5) ^ x;
        result = res1 ^ res2;
        ind = ind + 1;
      end
      crc_calc = result;
    end
  endfunction

  //===============================================================================================
  //======= CLK =======
  //===============================================================================================
  wire CLK_200;
  //wire	CS_CLK;
  wire lvds_fr, lvds_ck;
  wire CLK_OUT_BUFF, PLL_LOCK;

  

  //===============================================================================================
  //======= UART =======
  //===============================================================================================
  reg  [7:0] data_from_uart, uart_data_in;
  reg        uart_data_in_stb, uart_data_out_ack;
  reg        uart_data_out_flag, uart_tmp, uart_tmp2;
  reg        uart_speed_sel     = 1'b0;
  wire       UART_TX_0, UART_RX_0, UART_TX_1, UART_RX_1;
  wire       uart_data_out_stb0, uart_data_out_stb1, uart_data_in_ack0, uart_data_in_ack1;
  wire       uart_data_in_stb0, uart_data_in_stb1, uart_data_out_ack0, uart_data_out_ack1;
  wire [7:0] uart_data_out, uart_data_out0, uart_data_out1, uart_data_in0, uart_data_in1;
  wire       uart_data_out_stb, uart_data_in_ack;

  UART #(
         .BAUD_RATE      (UART_SPEED_1), // Virtex4 : 230400 - OK;			Virtex7 : 921600 - OK
         .CLOCK_FREQUENCY(200000000   )
       ) UART_inst0 (
         .CLOCK              (CLK_200           ),
         .RESET              (glbl_rst          ),
         .DATA_STREAM_IN     (uart_data_in0     ), // in
         .DATA_STREAM_IN_STB (uart_data_in_stb0 ), // in
         .DATA_STREAM_IN_ACK (uart_data_in_ack0 ), // out
         .DATA_STREAM_OUT    (uart_data_out0    ), // out
         .DATA_STREAM_OUT_STB(uart_data_out_stb0), // out
         .DATA_STREAM_OUT_ACK(uart_data_out_ack0), // in
         .TX                 (UART_TX_0         ),
         .RX                 (UART_RX_0         )
       );

  UART #(
         .BAUD_RATE      (UART_SPEED_2), // Virtex4 : 230400 - OK;			Virtex7 : 921600 - OK
         .CLOCK_FREQUENCY(200000000   )
       ) UART_inst1 (
         .CLOCK              (CLK_200           ),
         .RESET              (glbl_rst          ),
         .DATA_STREAM_IN     (uart_data_in1     ), // in
         .DATA_STREAM_IN_STB (uart_data_in_stb1 ), // in
         .DATA_STREAM_IN_ACK (uart_data_in_ack1 ), // out
         .DATA_STREAM_OUT    (uart_data_out1    ), // out
         .DATA_STREAM_OUT_STB(uart_data_out_stb1), // out
         .DATA_STREAM_OUT_ACK(uart_data_out_ack1), // in
         .TX                 (UART_TX_1         ),
         .RX                 (UART_RX_1         )
       );

  assign UART_TX           = uart_speed_sel ? UART_TX_1 : UART_TX_0;
  assign UART_RX_0         = uart_speed_sel ? 1'b0 : UART_RX;
  assign UART_RX_1         = uart_speed_sel ? UART_RX : 1'b0;
  assign uart_data_out     = uart_speed_sel ? uart_data_out1 : uart_data_out0;
  assign uart_data_out_stb = uart_speed_sel ? uart_data_out_stb1 : uart_data_out_stb0;
  assign uart_data_in_ack  = uart_speed_sel ? uart_data_in_ack1 : uart_data_in_ack0;

  assign uart_data_in0 = uart_speed_sel ? 1'b0 : uart_data_in;
  assign uart_data_in1 = uart_speed_sel ? uart_data_in : 1'b0;

  assign uart_data_in_stb0 = uart_speed_sel ? 1'b0 : uart_data_in_stb;
  assign uart_data_in_stb1 = uart_speed_sel ? uart_data_in_stb : 1'b0;

  assign uart_data_out_ack0 = uart_speed_sel ? 1'b0 : uart_data_out_ack;
  assign uart_data_out_ack1 = uart_speed_sel ? uart_data_out_ack : 1'b0;

  //===========================================================
  always @(posedge CLK_200)
  begin
    if (glbl_rst == 1'b1)
    begin
      uart_data_out_ack <= 1'b0;
    end
    else
    begin
      uart_data_out_ack <= 1'b0;
      if (uart_data_out_stb == 1'b1)
      begin
        uart_data_out_ack <= 1'b1;
        data_from_uart    <= uart_data_out;
      end
    end
  end

  always @(posedge CLK_200)
  begin
    uart_tmp           <= uart_data_out_stb;
    uart_tmp2          <= ~uart_tmp;
    uart_data_out_flag <= uart_tmp & uart_tmp2;
  end

  


  //===============================================================================================
  //======= SPI =======
  //===============================================================================================
  wire       spi_ready       ;
  reg  [6:0] spi_addr        ;
  reg  [7:0] spi_data        ;
  reg        spi_wr    = 1'b0;
  reg        spi_rd    = 1'b0;

  SPI_Master SPI_Master_inst (
               .nrst     (glbl_rstn    ),
               .clk      (CLK_200      ), // CLK_200M
               .clk_div  (8'hC8        ), // 8'h7D = 125;  8'hC8 = 200;  8'h64 = 100;  8'h0A = 10;  8'h14 = 20

               .ready    (spi_ready    ),
               .Address  (spi_addr     ),
               .rd       (spi_rd       ),
               .wr       (spi_wr       ),
               .DataWrite(spi_data     ),
               .DataRead (spi_data_read),

               .SPI_SS   (SPI_SS_int   ),
               .SPI_MOSI (SPI_MOSI_int ),
               .SPI_MISO (SPI_MISO_int ), // in
               .SPI_CLK  (SPI_CLK_int  )
             );
  assign SPI_MISO_int = SPI_MISO;
  assign SPI_SS       = SPI_SS_int;
  assign SPI_MOSI     = SPI_MOSI_int;
  assign SPI_CLK      = SPI_CLK_int;

 

  //===============================================================================================
  //======= Input DiffBuff =======
  //===============================================================================================
  wire DATA_MagN1_BUFF, DATA_MagN2_BUFF, DATA_MagN3_BUFF, DATA_MagN4_BUFF;
  wire DATA_SigN1_BUFF, DATA_SigN2_BUFF, DATA_SigN3_BUFF, DATA_SigN4_BUFF;
  reg  Data_MagN1_reg, Data_MagN2_reg, Data_MagN3_reg, Data_MagN4_reg;
  reg  Data_SigN1_reg, Data_SigN2_reg, Data_SigN3_reg, Data_SigN4_reg;

  IBUFDS #(.DIFF_TERM("TRUE"), .IBUF_LOW_PWR("FALSE")) IBUFDS_DATA_MagN1_inst (.O(DATA_MagN1_BUFF), .I(DATA_MagN1_p), .IB(DATA_MagN1_n));
  IBUFDS #(.DIFF_TERM("TRUE"), .IBUF_LOW_PWR("FALSE")) IBUFDS_DATA_SigN1_inst (.O(DATA_SigN1_BUFF), .I(DATA_SigN1_p), .IB(DATA_SigN1_n));

  IBUFGDS #(.DIFF_TERM("TRUE"), .IBUF_LOW_PWR("FALSE")) IBUFGDS_CLK_OUT_inst (.O(CLK_OUT_BUFF), .I(CLK_OUT_1), .IB(CLK_OUT_2));

  //===============================================================================================
  //======= NT1065 Input Registers =======
  //===============================================================================================
  always @(posedge CLK_OUT_BUFF)
  begin
    Data_MagN1_reg <= DATA_MagN1_BUFF;
    Data_SigN1_reg <= DATA_SigN1_BUFF;
  end

  reg tmp_clkn_dell, clk_out_flag;
  always @(posedge clk_in)
  begin
    tmp_clkn_dell <= ~CLK_OUT_BUFF;
    clk_out_flag  <= CLK_OUT_BUFF & tmp_clkn_dell;
  end

  //--------------------------------------------------
  reg  [ 7:0] data2ram        ;
  wire [ 7:0] data4ram        ;
  reg         ram_wr          ;
  reg  [14:0] addr2ram = 15'd0;

  always @(posedge clk_in)
  begin
    case (gen_state)
      DATA_WR :
      begin
        if (clk_out_flag)
        begin
          if (addr2ram < RAM_SIZE)
          begin
            addr2ram <= addr2ram + 1'b1;
            //data2ram <= addr2ram[7:0];
            data2ram <= {6'd0,Data_SigN1_reg, Data_MagN1_reg};
          end
          else
            addr2ram <= RAM_SIZE;
        end
      end
      RAM_RD_DEL :
        addr2ram <= ram_addr_buf;
      IDLE :
      begin
        addr2ram <= 15'd0;
      end
    endcase
  end


  blk_mem_gen_0 ram_32k_8bit_inst (
  .clka(clk_in),    // input wire clka
  .ena(1'b1),      // input wire ena
  .wea(ram_wr),      // input wire [0 : 0] wea
  .addra(addr2ram),  // input wire [14 : 0] addra
  .dina(data2ram),    // input wire [7 : 0] dina
  .douta(data4ram)  // output wire [7 : 0] douta
);


  //===============================================================================================
  //======= MAIN STATE MACHINE =======
  //===============================================================================================
  always @(posedge CLK_200)
  begin
    if ((glbl_rst == 1'b0))
    case (gen_state)
      IDLE :
      begin
        if (uart_data_out_flag == 1'b1)
        begin
          case (data_from_uart)
            8'h11 :
            begin						// Return firmware version
              gen_state <= MY_VER;
            end
            8'h55 :
            begin						// Start of SPI command
              gen_state <= SPI_SEL;
            end
            //--------------------------------
            8'h91 :
            begin						// Start Capturing digital Data
              gen_state <= DATA_WR;
              ram_wr    <= 1'b1;
            end
            8'hA3 :
            begin						// Start transmitting digital Data
              gen_state <= RAM_ADDR;
            end
            8'h41 :
            begin						// Set UART High Speed
              uart_speed_sel = 1'b0;
            end
            8'h44 :
            begin						// Set UART Low Speed
              uart_speed_sel = 1'b1;
            end
            //--------------------------------
            8'hD0 :
            begin						// Manual Set Delay
              gen_state <= SET_DEL;
            end
            //--------------------------------
            default :
            begin
              gen_state <= IDLE;
            end
          endcase
        end
      end
      //===============================================================================================
      MY_VER :
      begin							// matlab command 'double('NT1065_us_v5')'
        data2tx[11] <= 8'd78;			// 'N'
        data2tx[10] <= 8'd84;			// 'T'
        data2tx[9]  <= 8'd49;			// '1'
        data2tx[8]  <= 8'd48;			// '0'
        data2tx[7]  <= 8'd54;			// '6'
        data2tx[6]  <= 8'd53;			// '5'
        data2tx[5]  <= 8'd95;			// '_'
        data2tx[4]  <= 8'd117;			// 'u'
        data2tx[3]  <= 8'd115;			// 's'
        data2tx[2]  <= 8'd95;			// '_'
        data2tx[1]  <= 8'd118;			// 'v'
        data2tx[0]  <= 8'd53;			// '5'
        tx_cnt      <= 4'b1011;
        gen_state   <= MY_TX;
      end
      MY_TX :
      begin
        uart_data_in     <= data2tx[tx_cnt];
        uart_data_in_stb <= 1'b1;
        if (uart_data_in_ack == 1'b1)
        begin
          uart_data_in_stb <= 1'b0;
          if (tx_cnt == 4'b0000)
          begin
            gen_state <= IDLE;
          end
          else
          begin
            tx_cnt    <= tx_cnt - 4'b0001;
            gen_state <= MY_TX;
          end
        end
      end
      //-----------------------------------------------------------------------------------------------
      SPI_SEL :
      begin
        if (uart_data_out_flag == 1'b1)
        case (data_from_uart)
          8'h01 :
          begin
            gen_state <= SPI_WR;
          end
          8'h02 :
          begin
            gen_state <= SPI_RD;
          end
          default :
          begin
            gen_state <= IDLE;
          end
        endcase
      end
      //-----------------------------------------------------------------------------------------------
      SPI_WR :
      begin
        if (uart_data_out_flag == 1'b1)
        begin
          spi_addr     <= data_from_uart[6:0];
          crc_array[0] <= 8'h55;
          crc_array[1] <= 8'h01;
          crc_array[2] <= data_from_uart;
          gen_state    <= SPI_WR1;
        end
      end
      SPI_WR1 :
      begin
        if (uart_data_out_flag == 1'b1)
        begin
          spi_data     <= data_from_uart;
          crc_array[3] <= data_from_uart;
          gen_state    <= SPI_WR2;
        end
      end
      SPI_WR2 :
      begin
        if (uart_data_out_flag == 1'b1)
        begin
          crc_result     <= crc_calc(4);
          crc_data[15:8] <= data_from_uart;
          gen_state      <= SPI_WR3;
        end
      end
      SPI_WR3 :
      begin
        if (uart_data_out_flag == 1'b1)
        begin
          crc_data[7:0] <= data_from_uart;
          gen_state     <= SPI_WR4;
        end
      end
      SPI_WR4 :
      begin
        if (crc_result == crc_data)
        begin
          LED_S_int <= 1'b0;
          spi_rd    <= 1'b0;
          spi_wr    <= 1'b1;
          gen_state <= SPI_WR5;
        end
        else
        begin
          LED_S_int <= 1'b1;
          gen_state <= IDLE;
        end
      end
      SPI_WR5 :
      begin
        crc_array[0] <= 8'hAA;
        crc_array[2] <= 8'h00;
        crc_array[3] <= 8'h00;
        spi_addr     <= 7'b0000000;
        spi_data     <= 8'h00;
        spi_rd       <= 1'b0;
        spi_wr       <= 1'b0;
        if (spi_ready == 1'b1)
          gen_state <= GEN_ACK;
      end
      //-----------------------------------------------------------------------------------------------
      SPI_RD :
      begin
        if (uart_data_out_flag == 1'b1)
        begin
          spi_addr     <= data_from_uart[6:0];
          crc_array[0] <= 8'h55;
          crc_array[1] <= 8'h02;
          crc_array[2] <= data_from_uart;
          spi_data     <= 8'h00;
          gen_state    <= SPI_RD1;

        end
      end
      SPI_RD1 :
      begin
        if (uart_data_out_flag == 1'b1)
        begin
          crc_result     <= crc_calc(3);
          crc_data[15:8] <= data_from_uart;
          gen_state      <= SPI_RD2;
        end
      end
      SPI_RD2 :
      begin
        if (uart_data_out_flag == 1'b1)
        begin
          crc_data[7:0] <= data_from_uart;
          gen_state     <= SPI_RD3;
        end
      end
      SPI_RD3 :
      begin
        if (crc_result == crc_data)
        begin
          LED_S_int <= 1'b0;
          spi_rd    <= 1'b1;
          spi_wr    <= 1'b0;
          gen_state <= SPI_RD4;
        end
        else
        begin
          LED_S_int <= 1'b1;
          gen_state <= IDLE;
        end
      end
      SPI_RD4 :
      begin
        crc_array[0] <= 8'hAA;
        crc_array[2] <= 8'h00;
        crc_array[3] <= 8'h00;
        gen_state    <= SPI_RD5;
      end
      SPI_RD5 :
      begin
        spi_rd <= 1'b0;
        spi_wr <= 1'b0;
        if (spi_ready == 1'b1)
          gen_state <= GEN_ACK;
      end
      //-----------------------------------------------------------------------------------------------
      GEN_ACK :
      begin
        crc_array[1] <= spi_data_read;
        gen_state    <= GEN_ACK1;
      end
      GEN_ACK1 :
      begin
        crc_result <= crc_calc(2);
        gen_state  <= GEN_ACK2;
      end
      GEN_ACK2 :
      begin
        data2tx[3] <= 8'hAA;					// header
        data2tx[2] <= spi_data_read;			// data
        data2tx[1] <= crc_result[15:8];		// CRC_H
        data2tx[0] <= crc_result[7:0];		// CRC_L
        tx_cnt     <= 4'b0011;
        gen_state  <= MY_TX;
      end
      //-----------------------------------------------------------------------------------------------
      DATA_WR :
      begin
        if ((addr2ram == RAM_SIZE))
        begin
          gen_state <= WR_ACK;
          ram_wr    <= 1'b0;
        end
      end

      WR_ACK :
      begin
        data2tx[4] <= 8'd87;			// 'W'
        data2tx[3] <= 8'd82;			// 'R'
        data2tx[2] <= 8'd95;			// '_'
        data2tx[1] <= 8'd79;			// 'O'
        data2tx[0] <= 8'd75;			// 'K'
        tx_cnt     <= 4'b0100;
        gen_state  <= MY_TX;
      end
      //-----------------------------------------------------------------------------------------------
      RAM_ADDR :
      begin
        if (uart_data_out_flag == 1'b1)
        begin
          ram_addr_buf <= {data_from_uart,7'b000000};
          gen_state    <= RAM_RD_DEL;
          ram_rd_cnt   <= 3'b110;
        end
      end

      RAM_RD_DEL :
      begin
        if (ram_rd_cnt == 3'b000)
          gen_state <= RAM_TX;
        else
          ram_rd_cnt <= ram_rd_cnt - 3'b001;
      end

      RAM_TX :
      begin
        uart_data_in     <= data4ram;
        uart_data_in_stb <= 1'b1;
        if (uart_data_in_ack == 1'b1)
        begin
          uart_data_in_stb <= 1'b0;
          if (ram_addr_buf == RAM_SIZE)
          begin
            gen_state <= IDLE;
          end
          else
          begin
            ram_addr_buf <= ram_addr_buf + 15'd1;
            gen_state    <= RAM_RD_DEL;
          end
        end
      end

      
      //-----------------------------------------------------------------------------------------------
      SET_DEL :
      begin
        if (uart_data_out_flag == 1'b1)
        begin
          del_value <= data_from_uart[4:0];
          del_ld    <= 1'b1;
          gen_state <= SET_DEL1;
        end
      end
      SET_DEL1 :
      begin
        if (del_value == del_out)
        begin
          del_ld    <= 1'b0;
          gen_state <= IDLE;
        end
      end
      //-----------------------------------------------------------------------------------------------
      ZERO_STATE :
      begin
        gen_state <= IDLE;
      end
      default :
      begin
        gen_state <= IDLE;
      end
    endcase
    else
    begin
      gen_state        <= IDLE;
      uart_data_in_stb <= 1'b0;
      uart_data_in     <= 8'b00000000;
    end
  end
  //===============================================================================================

  assign glbl_rstn = ~glbl_rst;


endmodule
