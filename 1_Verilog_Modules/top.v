`timescale 1ns / 1ps

`default_nettype none
//`define DEBUG

module top
    #(
        NUM_BYTES_WR    =   2'd3        //  3 byte write transaction
    )
(
    input wire          clk,            //  100MHz System Clock
    input wire          reset_n,        //  Active High System Reset
    
    inout wire          scl_m,          //  I2C Master
    inout wire          sda_m,
    
    inout wire          scl_a,          //  I2C Slave A - 0x48
    inout wire          sda_a,
    
    inout wire          scl_b,          //  I2C Slave B - 0x49 
    inout wire          sda_b  
    
    
    
    `ifdef DEBUG
    ,
    
    //  Upstream Slave Debug Probes
    
    output wire [3:0]       state_us, 
    output wire [3:0]       next_state_us, 
    
    output wire [1:0]       sda_sync_us,
    output wire [1:0]       scl_sync_us,
    output wire             sda_prev_us, 
    output wire             scl_prev_us,
    
    output wire             sda_level_us,
    output wire             scl_level_us,
    output wire             sda_rise_us,
    output wire             sda_fall_us,
    output wire             scl_rise_us,
    output wire             scl_fall_us, 
    output wire             start_seen_us, 
    output wire             stop_seen_us,
     
    output wire             sda_in_us, 
    output wire             scl_in_us,
    output wire             sda_oe_us,          // 1 = drive low_us, 0 = release (tristate)
    output wire             scl_oe_us,          // for clock stretching
    
    output wire             sda_oe_next_us,
    output wire             scl_oe_next_us,
    
    output wire             ack_hold_us,
    output wire             stretching_us,
    output wire             stretch_req_us,
    output wire             ack_req_us,
    output wire             data_ready_us,
    output wire             force_release_us,
    output wire             will_ack_us,
    output wire             tx_bit_us,
    
    output wire [3:0]       bit_cnt_us,
    output wire [1:0]       byte_cnt_us,
    output wire             busy_us,       
    output wire             tx_en_us,          //  Start sending bit-by-bit data to the master
    output wire             rx_en_us,          //  Start receiving bit-by-bit data coming from the master
    output wire             addr_vld_us,       //  Flag marking device address validation 
    output wire             subaddr_vld_us,    //  Flag marking register address validation in the slave
    output wire [1:0]       ack_phase_us,      //  Used to mark the normal data bit shift & ACK/NACK bit shift 
    
    output wire             rw_mode_us,
    output wire [7:0]       addr_reg_us,
    output wire [7:0]       subaddr_reg_us,
    output wire [7:0]       r_data_reg_us,
    output wire [15:0]      shift_reg_us,
    
    output wire             addr_ack_wait_us,
    output wire             rx_next_byte_us,
    output wire             remove_str_resume_req,
    
    
    //  Downstream Master Debug Probes
    
    output wire          addr_translate_done,
    output wire [1:0]    slave_sel_dm,
    output wire          nack_dm,
     
    output wire          ack_ready_dm,
    output wire          will_ack_dm,
    output wire [7:0]    data_out_dm,
    output wire          valid_out_dm,
    output wire          req_data_chunk_dm,
    output wire          busy_dm,
    output wire [3:0]    state_dm,
    output wire [3:0]    next_state_dm,
    output wire          reg_sda_a_dm,
    output wire          reg_scl_a_dm,
    output wire          reg_sda_b_dm,
    output wire          reg_scl_b_dm,
    output wire [7:0]    addr_dm,
    output wire          rw_dm,
    
    output wire [15:0]   sub_addr_dm,
    output wire          sub_len_dm,
    output wire [23:0]   byte_len_dm,
    output wire [7:0]    bit_counter_dm,
    output wire          en_scl_dm,
    output wire          byte_sent_dm,
    output wire [23:0]   num_byte_sent_dm,
    output wire [2:0]    cntr_dm,
    output wire [7:0]    byte_sr_dm,
    output wire          read_sub_addr_sent_flag_dm,
    output wire [7:0]    data_to_write_dm,
    output wire [7:0]    data_in_sr_dm,
    
    //400KHz clock generation
    output wire          clk_i2c_dm,
    output wire [15:0]   clk_i2c_cntr_dm,
    output wire [1:0]    clk_phase_dm,
    
    
    //sampling sda and scl
    output wire          sda_prev_a_dm,
    output wire [1:0]    sda_curr_a_dm,
    output wire          scl_prev_a_dm,
    output wire [1:0]    scl_curr_a_dm,
    output wire          sda_prev_b_dm,
    output wire [1:0]    sda_curr_b_dm,
    output wire          scl_prev_b_dm,
    output wire [1:0]    scl_curr_b_dm,
    output wire          ack_in_prog_dm,
    output wire          ack_nack_dm,
    output wire          en_end_indicator_dm,
    output wire          grab_next_data_dm,
    output wire          scl_a_is_high_dm,
    output wire          scl_b_is_high_dm,
    output wire          scl_a_is_low_dm,
    output wire          scl_b_is_low_dm
    
    `endif
    
    
    
    );
    
    
    
    `ifndef DEBUG
        
        //  Upstream Slave Acknowledgement Probes
        wire        start_seen_us; 
        wire        stop_seen_us; 
        wire [3:0]  state_us; 
        wire [3:0]  next_state_us;
        wire        stretch_req_us;
        wire        remove_str_resume_req;
        wire [7:0]  addr_reg_us;
        wire [7:0]  subaddr_reg_us;
        wire [7:0]  r_data_reg_us;
        wire [15:0] shift_reg_us;
        wire        rx_next_byte_us;
        
        
        //  Downstream Master Acknowledgement Probes
        wire        addr_translate_done;
        wire [1:0]  slave_sel_dm;
        wire        nack_dm;
        wire        ack_ready_dm;
        wire        will_ack_dm;

     
    `endif
    
    
    
    //--------------------------------------
    //      Upstream Slave
    //--------------------------------------
    
        
        upstream_slave 
        #(
            .NUM_BYTES_WR   (NUM_BYTES_WR)
        )up_slave(
        
        .clk                    (clk),
        .reset_n                (reset_n),
        
        .scl_m                  (scl_m),
        .sda_m                  (sda_m),
        
        .start_seen             (start_seen_us), 
        .stop_seen              (stop_seen_us), 
        .state                  (state_us), 
        .next_state             (next_state_us),
        .stretch_req            (stretch_req_us),
        .remove_str_resume_req  (remove_str_resume_req),
        .addr_reg               (addr_reg_us),
        .subaddr_reg            (subaddr_reg_us),
        .r_data_reg             (r_data_reg_us),
        .shift_reg              (shift_reg_us),
        .rx_next_byte           (rx_next_byte_us),
        
        
        //  Downstream Master Acknowledgement Probes
        
        .addr_translate_done    (addr_translate_done),
        .slave_sel              (slave_sel_dm),
        .nack_dm                (nack_dm),
        .ack_ready_dm           (ack_ready_dm),
        .will_ack_dm            (will_ack_dm)
        
        
        `ifdef DEBUG
        ,
        .sda_sync       (sda_sync_us), 
        .scl_sync       (scl_sync_us), 
        .sda_prev       (sda_prev_us), 
        .scl_prev       (scl_prev_us), 
        
        .sda_level      (sda_level_us), 
        .scl_level      (scl_level_us), 
        .sda_rise       (sda_rise_us), 
        .sda_fall       (sda_fall_us), 
        .scl_rise       (scl_rise_us), 
        .scl_fall       (scl_fall_us), 
        
        
        .sda_in         (sda_in_us), 
        .scl_in         (scl_in_us), 
        .sda_oe         (sda_oe_us), 
        .scl_oe         (scl_oe_us),
        
        .sda_oe_next    (sda_oe_next_us),
        .scl_oe_next    (scl_oe_next_us),
        
        .ack_hold       (ack_hold_us), 
        .stretching     (stretching_us), 
         
        .ack_req        (ack_req_us), 
        .data_ready     (data_ready_us), 
        .force_release  (force_release_us), 
        .will_ack       (will_ack_us), 
        .tx_bit         (tx_bit_us), 
        
        .bit_cnt        (bit_cnt_us), 
        .byte_cnt       (byte_cnt_us), 
        .busy           (busy_us), 
        .tx_en          (tx_en_us), 
        .rx_en          (rx_en_us), 
        .ack_phase      (ack_phase_us), 
        .addr_vld       (addr_vld_us), 
        .subaddr_vld    (subaddr_vld_us), 
        .rw_mode        (rw_mode_us),
        .addr_ack_wait  (addr_ack_wait_us)
        
        
        
        `endif
        
        
        );
    
    
    
    
    
    
    //--------------------------------------
    //      Downstream Master
    //--------------------------------------
    
        downstream_master down_master(
        
        .clk            (clk),
        .reset_n        (reset_n),
        
        .scl_a          (scl_a),
        .sda_a          (sda_a),
        
        .scl_b          (scl_b),
        .sda_b          (sda_b),
        
        
        //  Input from Upstream Slave
        
        .start_seen_us      (start_seen_us),
        .stop_seen_us       (stop_seen_us),
        .state_us           (state_us),
        .next_state_us      (next_state_us),
        .stretch_req_us     (stretch_req_us), 
        .remove_str_resume_req      (remove_str_resume_req), 
        
        .addr_reg_us        (addr_reg_us),
        .subaddr_reg_us     (subaddr_reg_us),
        .r_data_reg_us      (r_data_reg_us),
        .shift_reg_us       (shift_reg_us),
        .rx_next_byte_us    (rx_next_byte_us),
        
        
        //  Outputs from Downstream Master
        
        .addr_translate_done(addr_translate_done),
        .slave_sel          (slave_sel_dm),
        .nack               (nack_dm),
        .ack_ready_dm       (ack_ready_dm),
        .will_ack_dm        (will_ack_dm)
        
        
        `ifdef DEBUG
        ,
        .data_out           (data_out_dm),
        .valid_out          (valid_out_dm),
        .req_data_chunk     (req_data_chunk_dm),
        .busy               (busy_dm),
        
        .state              (state_dm),
        .next_state         (next_state_dm),
        .reg_sda_a          (reg_sda_a_dm),
        .reg_scl_a          (reg_scl_a_dm),
        .reg_sda_b          (reg_sda_b_dm),
        .reg_scl_b          (reg_scl_b_dm),
        .addr               (addr_dm),
        .rw                 (rw_dm),
        
        .sub_addr                  (sub_addr_dm),
        .sub_len                   (sub_len_dm),
        .byte_len                  (byte_len_dm),
        .bit_counter               (bit_counter_dm),
        .en_scl                    (en_scl_dm),
        .byte_sent                 (byte_sent_dm),
        .num_byte_sent             (num_byte_sent_dm),
        .cntr                      (cntr_dm),
        .byte_sr                   (byte_sr_dm),
        .read_sub_addr_sent_flag   (read_sub_addr_sent_flag_dm),
        .data_to_write             (data_to_write_dm),
        .data_in_sr                (data_in_sr_dm),
        
        // 400 kHz clock generation
        .clk_i2c                   (clk_i2c_dm),
        .clk_i2c_cntr              (clk_i2c_cntr_dm),
        .clk_phase                 (clk_phase_dm),
        
        // sampling sda and scl
        .sda_prev_a                (sda_prev_a_dm),
        .sda_curr_a                (sda_curr_a_dm),
        .scl_prev_a                (scl_prev_a_dm),
        .scl_curr_a                (scl_curr_a_dm),
        .sda_prev_b                (sda_prev_b_dm),
        .sda_curr_b                (sda_curr_b_dm),
        .scl_prev_b                (scl_prev_b_dm),
        .scl_curr_b                (scl_curr_b_dm),
        .ack_in_prog               (ack_in_prog_dm),
        .ack_nack                  (ack_nack_dm),
        .en_end_indicator          (en_end_indicator_dm),
        .grab_next_data            (grab_next_data_dm),
        .scl_a_is_high             (scl_a_is_high_dm),
        .scl_b_is_high             (scl_b_is_high_dm),
        .scl_a_is_low              (scl_a_is_low_dm),
        .scl_b_is_low              (scl_b_is_low_dm)
        
        `endif
        
        );
    
    
    
endmodule
