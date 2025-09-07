`timescale 1ns / 1ps

`define DEBUG
module i2c_master_top(

        input wire          clk,            //  100MHz input clock
        input wire          reset,          //  Active HIGH Reset
                    
        inout wire          scl,
        inout wire          sda,
        
        input wire [11:0]   dac_wr_data,
        input wire          dac_prog,
        output reg          dac_prog_done
        
        `ifdef DEBUG
        ,
        output wire [3:0]   i2c_state,
        output wire [3:0]   i2c_next_state,
        output wire         reg_sda_o,
        output wire         reg_scl_o,
        output wire [7:0]   addr,
        output wire         rw,
        output wire [15:0]  sub_addr_wire,
        output wire         sub_len_wire,
        output wire [23:0]  byte_len_wire,
        output wire [7:0]   bit_counter,
        output wire         en_scl,
        output wire         byte_sent,
        output wire [23:0]  num_byte_sent,
        output wire [2:0]   cntr,
        output wire [7:0]   byte_sr,
        output wire         read_sub_addr_sent_flag,
        output wire [7:0]   data_to_write_wire,
        output wire [7:0]   data_in_sr,
        
        output wire         clk_i2c,
        output wire [15:0]  clk_i2c_cntr,
        output wire [1:0]   clk_phase,
        
        output wire         sda_prev,
        output wire [1:0]   sda_curr,
        output wire         scl_prev,
        output wire [1:0]   scl_curr,
        output wire         ack_in_prog,
        output wire         ack_nack,
        output wire         en_end_indicator,
        output wire         grab_next_data,
        output wire         scl_is_high,
        output wire         scl_is_low
        
        `endif

    );
    
    
    
            reg [7:0]       i_addr_w_rw;    
            reg [15:0]      i_sub_addr;      
            reg             i_sub_len;          
            reg [23:0]      i_byte_len;    
            reg [7:0]       i_data_write;   
            reg             start_transaction;  
            wire [7:0]      data_out;
            wire            valid_out;
            wire            req_data_chunk;   
            wire            busy;    
            wire            nack;
            reg [1:0]	    wr_byte_index;
              
            wire [7:0]      write_byte_0;
            wire [7:0]      write_byte_1;
            wire [7:0]      write_byte_2;
            wire [7:0]      current_byte;
            
            reg [2:0]       state;
            
            wire            sda_in_debug;
            wire            sda_out_debug;
 
    
    
    
    
    
    
        //---------------------------------------------------
        //      DAC I2C WRITE TRANSACTION
        //---------------------------------------------------
        
        
        localparam  VIRT_SLAVE_ADDR     =   8'b1001_0000,       //  Slave A - 0x48 {1001_000}  , Slave B - 0x49 {1001_001} 
                    REG_ADDR            =   8'b0010_0010;       
        
        
        localparam  WAIT_TRANSACTION    =   16'd10000;       //  100us wait counter after I2C Word sent
                    
        reg [7:0]	write_bytes [0:3];
        reg [15:0]  wait_tx_count;
        
        
        
        
        //  DEBUG PROBES
        
        assign write_byte_0 =   write_bytes[0];
        assign write_byte_1 =   write_bytes[1];
        assign write_byte_2 =   write_bytes[2];
        
        assign current_byte =   write_bytes[wr_byte_index];
    
    
        
    
        always@(posedge clk)
        begin
            if (reset)
            begin
                write_bytes[0] 	<=	16'b0;
                write_bytes[1] 	<=	16'b0;
                write_bytes[2] 	<=	16'b0;
            end
            else 
            begin
                write_bytes[0]	<=	{ 3'b010, 2'b00, 2'b00, 1'b0};	//	BYTE 2: 	WRITE_TYPE (C2 C1 C0) + XX + (PD1 PD0) + X		
                write_bytes[1]	<=	dac_wr_data[11:4];				//	BYTE 3:		MSB 8 BITS OF "DAC WRITE DATA" 
                write_bytes[2]	<=	{dac_wr_data[3:0], 4'b0};		//	BYTE 4: 	LSB 4 BITS OF "DAC WRITE DATA" + 4'bXXXX
            end
        end
    
    
    
        //  STATE ENCODING FOR THE WRITE TRANSACTION
        
        localparam  WR_IDLE			=   3'b000,
                    WR_ADDR_BYTE	=	3'b001,
                    WR_DATA_BYTES	=	3'b010,
                    WR_INDEX		=	3'b011,
                    WR_DONE     	=   3'b100; 
        
        
        
        // STATE MACHINE
    
        //	reg [2:0] 	wr_byte_index;
    
        
        always@(posedge clk)
        begin
            if (reset)
            begin
                
                i_addr_w_rw			<=	8'b0;    
                i_sub_addr			<=	16'b0;      
                i_sub_len			<=	1'b0;          
                i_byte_len			<=	24'b0;    
                i_data_write		<=	8'b0;   
                start_transaction	<=	1'b0;  
            
                wr_byte_index		<=	2'b0;
                state 				<=	WR_IDLE;
                
                dac_prog_done	    <=  1'b0;  
                wait_tx_count       <=  16'b0;
            
            end
            else
            begin
                
                i_addr_w_rw			<=	VIRT_SLAVE_ADDR;   
                i_sub_addr			<=	REG_ADDR;           //	SUB-ADDRESS (NA - 0)
                i_sub_len			<=	1'b0;          		//	LENGTH OF SUB-ADDRESS
                i_byte_len			<=	24'd3;				//	3 WRITE BYTES TO BE SENT    
                i_data_write		<=	write_bytes[wr_byte_index];            //   THREE DATA BYTES         
                
                
                start_transaction	<=	1'b0;
                wait_tx_count       <=  16'b0;
                dac_prog_done	    <=  1'b0;
            
                case (state)
                    
                    WR_IDLE:
                    begin
                        
                        wr_byte_index 	<=	2'b0;
                        state			<=	WR_ADDR_BYTE;
                    end
    
                    WR_ADDR_BYTE:
                    begin
                        if (!busy && dac_prog)
                        begin
                            start_transaction	<=  1'b1;
                            state				<=	WR_DATA_BYTES;
                        end
                        else
                        begin
                            start_transaction	<=	1'b0;
                            state				<=	WR_ADDR_BYTE;
                        end
                    end
    
                    WR_DATA_BYTES:
                    begin
                        
                        if (req_data_chunk) 
                            state		<=	WR_INDEX;
                        else
                            state		<=	WR_DATA_BYTES;
                    end
                
                    WR_INDEX:
                    begin
                        
                        if (wr_byte_index < 2'd2)          //  THREE WRITE BYTES APART FROM THE FIRST ADDRESS BYTE SENT
                        begin  
    
                            wr_byte_index	<=	wr_byte_index	+	2'b1;
                            state			<=	WR_DATA_BYTES;
                            
                        end
                        else
                            state			<=	WR_DONE;
                    end
    
                    WR_DONE:
                    begin
                        //  tx_done asserted high for 3 clocks for proper latching
                        dac_prog_done   <=  (wait_tx_count < 16'd3) ? 1'b1 : 1'b0; 
                        
                        if (wait_tx_count < WAIT_TRANSACTION)
                        begin
                            wait_tx_count   <=  wait_tx_count + 1'b1;
                            state	        <=	WR_DONE;
                        end
                        else
                            state           <=  WR_IDLE;
                    end
    
                endcase
            end
        end
        
        
    
    
    
    
    
    
    //--------------------------------------
    //      I2C Master Instance
    //--------------------------------------
    

        i2c_master_str i2c_master_inst (
        
            .i_clk          (clk),
            .reset_n        (~reset),
            .i_addr_w_rw    (i_addr_w_rw),
            .i_sub_addr     (i_sub_addr),
            .i_sub_len      (i_sub_len),
            .i_byte_len     (i_byte_len),
            .i_data_write   (i_data_write),
            .req_trans      (start_transaction),
        
            .data_out       (data_out),
            .valid_out      (valid_out),
        
            .scl_o          (scl),
            .sda_o          (sda),
        
            .req_data_chunk(req_data_chunk),
            .busy           (busy),
            .nack           (nack),
        
            .sda_out_debug  (sda_out_debug),
            .sda_in_debug   (sda_in_debug)
        

        
        
        `ifdef DEBUG
            ,
            .state                      (i2c_state),
            .next_state                 (i2c_next_state),
            .reg_sda_o                  (reg_sda_o),
            .reg_scl_o                  (reg_scl_o),
            .addr                       (addr),
            .rw                         (rw),
            .sub_addr                   (sub_addr_wire),
            .sub_len                    (sub_len_wire),
            .byte_len                   (byte_len_wire),
            .bit_counter                (bit_counter),
            .en_scl                     (en_scl),
            .byte_sent                  (byte_sent),
            .num_byte_sent              (num_byte_sent),
            .cntr                       (cntr),
            .byte_sr                    (byte_sr),
            .read_sub_addr_sent_flag    (read_sub_addr_sent_flag),
            .data_to_write              (data_to_write_wire),
            .data_in_sr                 (data_in_sr),
        
            .clk_i2c                    (clk_i2c),
            .clk_i2c_cntr               (clk_i2c_cntr),
            .clk_phase                  (clk_phase),
            
            .sda_prev                   (sda_prev),
            .sda_curr                   (sda_curr),
            .scl_prev                   (scl_prev),
            .scl_curr                   (scl_curr),
            .ack_in_prog                (ack_in_prog),
            .ack_nack                   (ack_nack),
            .en_end_indicator           (en_end_indicator),
            .grab_next_data             (grab_next_data),
            .scl_is_high                (scl_is_high),
            .scl_is_low                 (scl_is_low)
            
        `endif
        
        
        );
    
    
    
endmodule
