`timescale 1ns / 1ps

module sim_i2c();
    
    reg         clk, reset_n;
    reg [11:0]  dac_wr_data;
    reg         dac_prog;
    wire        dac_prog_done;
    
    
    tri1    scl_m, sda_m;
    tri1    scl_a, sda_a;
    tri1    scl_b, sda_b;
    
    
    //  i2c master debug probes
    
    wire [3:0]  i2c_state;
    wire [3:0]  i2c_next_state;
    wire        reg_sda_o;
    wire        reg_scl_o;
    wire [7:0]  addr;
    wire        rw;
    wire [15:0] sub_addr_wire;
    wire        sub_len_wire;
    wire [23:0] byte_len_wire;
    wire [7:0]  bit_counter;
    wire        en_scl;
    wire        byte_sent;
    wire [23:0] num_byte_sent;
    wire [2:0]  cntr;
    wire [7:0]  byte_sr;
    wire        read_sub_addr_sent_flag;
    wire [7:0]  data_to_write_wire;
    wire [7:0]  data_in_sr;
    
    wire        clk_i2c;
    wire [15:0] clk_i2c_cntr;
    wire [1:0]  clk_phase;
    
    wire        sda_prev;
    wire [1:0]  sda_curr;
    wire        scl_prev;
    wire [1:0]  scl_curr;
    wire        ack_in_prog;
    wire        ack_nack;
    wire        en_end_indicator;
    wire        grab_next_data;
    wire        scl_is_high;
    wire        scl_is_low;
    
    
    
    
    
    
    //  Upstream Slave Debug Probes
    
    wire [3:0]       state_us; 
    wire [3:0]       next_state_us; 
    
    wire [1:0]       sda_sync_us;
    wire [1:0]       scl_sync_us;
    wire             sda_prev_us; 
    wire             scl_prev_us;
    
    wire             sda_level_us;
    wire             scl_level_us;
    wire             sda_rise_us;
    wire             sda_fall_us;
    wire             scl_rise_us;
    wire             scl_fall_us; 
    wire             start_seen_us; 
    wire             stop_seen_us; 
    
    wire             sda_in_us; 
    wire             scl_in_us;
    wire             sda_oe_us;          // 1 = drive low_us; 0 = release (tristate)
    wire             scl_oe_us;          // for clock stretching
   
    wire             sda_oe_next_us;
    wire             scl_oe_next_us;
   
    wire             ack_hold_us;
    wire             stretching_us;
    wire             stretch_req_us;
    wire             ack_req_us;
    wire             data_ready_us;
    wire             force_release_us;
    wire             will_ack_us;
    wire             tx_bit_us;
    
    wire [3:0]       bit_cnt_us;
    wire [1:0]       byte_cnt_us;
    wire             busy_us;       
    wire             tx_en_us;          //  Start sending bit-by-bit data to the master
    wire             rx_en_us;          //  Start receiving bit-by-bit data coming from the master
    wire             addr_vld_us;       //  Flag marking device address validation 
    wire             subaddr_vld_us;    //  Flag marking register address validation in the slave
    wire [1:0]       ack_phase_us;      //  Used to mark the normal data bit shift & ACK/NACK bit shift 
    
    wire             rw_mode_us;
    wire [7:0]       addr_reg_us;
    wire [7:0]       subaddr_reg_us;
    wire [7:0]       r_data_reg_us;
    wire [15:0]      shift_reg_us;
    
    wire             addr_ack_wait_us;
    wire             rx_next_byte_us;
    wire            remove_str_resume_req;
    
    
    //      Downstream Master Debug Probes
    
    wire          addr_translate_done;
    wire [1:0]    slave_sel_dm;
    wire          nack_dm;
    
    wire          ack_ready_dm;
    wire          will_ack_dm;
    wire [7:0]    data_out_dm;
    wire          valid_out_dm;
    wire          req_data_chunk_dm;
    wire          busy_dm;
    wire [3:0]    state_dm;
    wire [3:0]    next_state_dm;
    wire          reg_sda_a_dm;
    wire          reg_scl_a_dm;
    wire          reg_sda_b_dm;
    wire          reg_scl_b_dm;
    wire [7:0]    addr_dm;
    wire          rw_dm;
    
    wire [15:0]   sub_addr_dm;
    wire          sub_len_dm;
    wire [23:0]   byte_len_dm;
    wire [7:0]    bit_counter_dm;
    wire          en_scl_dm;
    wire          byte_sent_dm;
    wire [23:0]   num_byte_sent_dm;
    wire [2:0]    cntr_dm;
    wire [7:0]    byte_sr_dm;
    wire          read_sub_addr_sent_flag_dm;
    wire [7:0]    data_to_write_dm;
    wire [7:0]    data_in_sr_dm;
    
    //400KHz clock generation
    wire          clk_i2c_dm;
    wire [15:0]   clk_i2c_cntr_dm;
    wire [1:0]    clk_phase_dm;
    
    
    //sampling sda and scl
    wire          sda_prev_a_dm;
    wire [1:0]    sda_curr_a_dm;
    wire          scl_prev_a_dm;
    wire [1:0]    scl_curr_a_dm;
    wire          sda_prev_b_dm;
    wire [1:0]    sda_curr_b_dm;
    wire          scl_prev_b_dm;
    wire [1:0]    scl_curr_b_dm;
    wire          ack_in_prog_dm;
    wire          ack_nack_dm;
    wire          en_end_indicator_dm;
    wire          grab_next_data_dm;
    wire          scl_a_is_high_dm;
    wire          scl_b_is_high_dm;
    wire          scl_a_is_low_dm;
    wire          scl_b_is_low_dm;
    
    
    
    //  Slave A Probes
    wire [3:0]       state_a; 
    wire [1:0]       sda_sync_a;
    wire [1:0]       scl_sync_a;
    wire             sda_prev_a; 
    wire             scl_prev_a;
    wire             sda_level_a;
    wire             scl_level_a;
    wire             sda_rise_a;
    wire             sda_fall_a;
    wire             scl_rise_a;
    wire             scl_fall_a; 
    wire             start_seen_a; 
    wire             stop_seen_a; 
    wire             sda_in_a; 
    wire             scl_in_a;
    wire             sda_oe_a;          // 1 = drive low_a; 0 = release (tristate)
    wire             scl_oe_a;          // for clock stretching
    wire             ack_hold_a;
    wire             stretching_a;
    wire             stretch_req_a;
    wire             ack_req_a;
    wire             data_ready_a;
    wire             force_release_a;
    wire             will_ack_a;
    wire             tx_bit_a;
    wire [3:0]       bit_cnt_a;
    wire [1:0]       byte_cnt_a;
    wire             busy_a;       
    wire             tx_en_a;          //  Start sending bit-by-bit data to the master
    wire             rx_en_a;          //  Start receiving bit-by-bit data coming from the master
    wire             addr_vld_a;       //  Flag marking device address validation 
    wire             subaddr_vld_a;    //  Flag marking register address validation in the slave
    wire [1:0]       ack_phase_a;      //  Used to mark the normal data bit shift & ACK/NACK bit shift 
    wire             rw_mode_a;
    wire [7:0]       addr_reg_a;
    wire [7:0]       subaddr_reg_a;
    wire [7:0]       r_data_reg_a;
    wire [15:0]      shift_reg_a;
    wire             addr_ack_wait_a;
    
    
    //  Slave B Probes
    
    wire [3:0]       state_b; 
    wire [1:0]       sda_sync_b;
    wire [1:0]       scl_sync_b;
    wire             sda_prev_b; 
    wire             scl_prev_b;
    wire             sda_level_b;
    wire             scl_level_b;
    wire             sda_rise_b;
    wire             sda_fall_b;
    wire             scl_rise_b;
    wire             scl_fall_b; 
    wire             start_seen_b; 
    wire             stop_seen_b; 
    wire             sda_in_b; 
    wire             scl_in_b;
    wire             sda_oe_b;          // 1 = drive low_b; 0 = release (tristate)
    wire             scl_oe_b;          // for clock stretching
    wire             ack_hold_b;
    wire             stretching_b;
    wire             stretch_req_b;
    wire             ack_req_b;
    wire             data_ready_b;
    wire             force_release_b;
    wire             will_ack_b;
    wire             tx_bit_b;
    wire [3:0]       bit_cnt_b;
    wire [1:0]       byte_cnt_b;
    wire             busy_b;       
    wire             tx_en_b;          //  Start sending bit-by-bit data to the master
    wire             rx_en_b;          //  Start receiving bit-by-bit data coming from the master
    wire             addr_vld_b;       //  Flag marking device address validation 
    wire             subaddr_vld_b;    //  Flag marking register address validation in the slave
    wire [1:0]       ack_phase_b;      //  Used to mark the normal data bit shift & ACK/NACK bit shift 
    wire             rw_mode_b;
    wire [7:0]       addr_reg_b;
    wire [7:0]       subaddr_reg_b;
    wire [7:0]       r_data_reg_b;
    wire [15:0]      shift_reg_b;
    wire             addr_ack_wait_b;
    
    
    
    
    
    
    //------------------------------------
    //  DUT - TRANSLATOR INSTANCE
    //------------------------------------
    
    
        top dut (
        
        .clk            (clk),
        .reset_n        (reset_n),
        
        .scl_m          (scl_m),
        .sda_m          (sda_m),
        .scl_a          (scl_a),
        .sda_a          (sda_a),
        .scl_b          (scl_b),
        .sda_b          (sda_b),
        
        
        
        //---------------------------------
        //  Upstream Slave Debug Probes
        //---------------------------------
             
        .state_us          (state_us), 
        .next_state_us     (next_state_us), 
            
        .sda_sync_us       (sda_sync_us), 
        .scl_sync_us       (scl_sync_us), 
        .sda_prev_us       (sda_prev_us), 
        .scl_prev_us       (scl_prev_us), 
        
        .sda_level_us      (sda_level_us), 
        .scl_level_us      (scl_level_us), 
        .sda_rise_us       (sda_rise_us), 
        .sda_fall_us       (sda_fall_us), 
        .scl_rise_us       (scl_rise_us), 
        .scl_fall_us       (scl_fall_us), 
        .start_seen_us     (start_seen_us), 
        .stop_seen_us      (stop_seen_us), 
        
        .sda_in_us         (sda_in_us), 
        .scl_in_us         (scl_in_us), 
        .sda_oe_us         (sda_oe_us), 
        .scl_oe_us         (scl_oe_us),
        
        .sda_oe_next_us    (sda_oe_next_us),
        .scl_oe_next_us    (scl_oe_next_us),
        
        .ack_hold_us       (ack_hold_us), 
        .stretching_us     (stretching_us), 
        .stretch_req_us    (stretch_req_us), 
        .ack_req_us        (ack_req_us), 
        .data_ready_us     (data_ready_us), 
        .force_release_us  (force_release_us), 
        .will_ack_us       (will_ack_us), 
        .tx_bit_us         (tx_bit_us), 
        
        .bit_cnt_us        (bit_cnt_us), 
        .byte_cnt_us       (byte_cnt_us), 
        .busy_us           (busy_us), 
        .tx_en_us          (tx_en_us), 
        .rx_en_us          (rx_en_us), 
        .addr_vld_us       (addr_vld_us), 
        .subaddr_vld_us    (subaddr_vld_us), 
        .ack_phase_us      (ack_phase_us), 
        
        .rw_mode_us        (rw_mode_us),
        .addr_reg_us       (addr_reg_us),
        .subaddr_reg_us    (subaddr_reg_us),
        .r_data_reg_us     (r_data_reg_us),
        .shift_reg_us      (shift_reg_us),
        
        .addr_ack_wait_us  (addr_ack_wait_us),
        .rx_next_byte_us   (rx_next_byte_us),
        .remove_str_resume_req   (remove_str_resume_req),
        
        
        //------------------------------------
        //  Downstream Master Debug Probes
        //------------------------------------
        
        .addr_translate_done    (addr_translate_done),
        .slave_sel_dm           (slave_sel_dm),
        .nack_dm                (nack_dm),
        
        .ack_ready_dm           (ack_ready_dm),
        .will_ack_dm            (will_ack_dm),
        .data_out_dm            (data_out_dm),
        .valid_out_dm           (valid_out_dm),
        .req_data_chunk_dm      (req_data_chunk_dm),
        .busy_dm                (busy_dm),
        .state_dm               (state_dm),
        .next_state_dm          (next_state_dm),
        .reg_sda_a_dm           (reg_sda_a_dm),
        .reg_scl_a_dm           (reg_scl_a_dm),
        .reg_sda_b_dm           (reg_sda_b_dm),
        .reg_scl_b_dm           (reg_scl_b_dm),
        .addr_dm                (addr_dm),
        .rw_dm                  (rw_dm),
        
        .sub_addr_dm                    (sub_addr_dm),
        .sub_len_dm                     (sub_len_dm),
        .byte_len_dm                    (byte_len_dm),
        .bit_counter_dm                 (bit_counter_dm),
        .en_scl_dm                      (en_scl_dm),
        .byte_sent_dm                   (byte_sent_dm),
        .num_byte_sent_dm               (num_byte_sent_dm),
        .cntr_dm                        (cntr_dm),
        .byte_sr_dm                     (byte_sr_dm),
        .read_sub_addr_sent_flag_dm     (read_sub_addr_sent_flag_dm),
        .data_to_write_dm               (data_to_write_dm),
        .data_in_sr_dm                  (data_in_sr_dm),
        
        // 400 kHz clock generation
        .clk_i2c_dm                     (clk_i2c_dm),
        .clk_i2c_cntr_dm                (clk_i2c_cntr_dm),
        .clk_phase_dm                   (clk_phase_dm),
        
        // sampling sda and scl 
        .sda_prev_a_dm                  (sda_prev_a_dm),
        .sda_curr_a_dm                  (sda_curr_a_dm),
        .scl_prev_a_dm                  (scl_prev_a_dm),
        .scl_curr_a_dm                  (scl_curr_a_dm),
        .sda_prev_b_dm                  (sda_prev_b_dm),
        .sda_curr_b_dm                  (sda_curr_b_dm),
        .scl_prev_b_dm                  (scl_prev_b_dm),
        .scl_curr_b_dm                  (scl_curr_b_dm),
        .ack_in_prog_dm                 (ack_in_prog_dm),
        .ack_nack_dm                    (ack_nack_dm),
        .en_end_indicator_dm            (en_end_indicator_dm),
        .grab_next_data_dm              (grab_next_data_dm),
        .scl_a_is_high_dm               (scl_a_is_high_dm),
        .scl_b_is_high_dm               (scl_b_is_high_dm),
        .scl_a_is_low_dm                (scl_a_is_low_dm),
        .scl_b_is_low_dm                (scl_b_is_low_dm)
        
        
        );
    
    
    
    
    
    
    
    
    
    
    //--------------------------------
    //      I2C Master Instance
    //--------------------------------
    
        i2c_master_top i2c_master_top (
            
            .clk            (clk),
            .reset          (~reset_n),
            
            .scl            (scl_m),
            .sda            (sda_m),
            
            .dac_wr_data    (dac_wr_data),
            .dac_prog       (dac_prog),
            .dac_prog_done  (dac_prog_done),
            
            
            .i2c_state               (i2c_state),
            .i2c_next_state          (i2c_next_state),
            .reg_sda_o               (reg_sda_o),
            .reg_scl_o               (reg_scl_o),
            .addr                    (addr),
            .rw                      (rw),
            .sub_addr_wire           (sub_addr_wire),
            .sub_len_wire            (sub_len_wire),
            .byte_len_wire           (byte_len_wire),
            .bit_counter             (bit_counter),
            .en_scl                  (en_scl),
            .byte_sent               (byte_sent),
            .num_byte_sent           (num_byte_sent),
            .cntr                    (cntr),
            .byte_sr                 (byte_sr),
            .read_sub_addr_sent_flag (read_sub_addr_sent_flag),
            .data_to_write_wire      (data_to_write_wire),
            .data_in_sr              (data_in_sr),
        
            .clk_i2c                 (clk_i2c),
            .clk_i2c_cntr            (clk_i2c_cntr),
            .clk_phase               (clk_phase),
        
            .sda_prev                (sda_prev),
            .sda_curr                (sda_curr),
            .scl_prev                (scl_prev),
            .scl_curr                (scl_curr),
            .ack_in_prog             (ack_in_prog),
            .ack_nack                (ack_nack),
            .en_end_indicator        (en_end_indicator),
            .grab_next_data          (grab_next_data),
            .scl_is_high             (scl_is_high),
            .scl_is_low              (scl_is_low)
            

        );
    
    
    
    //---------------------------------
    //      I2C Slave A Instance 
    //---------------------------------
    
        i2c_slave_a a_i2c_slave (
        
            .clk            (clk),
            .reset_n        (reset_n),
            
            .scl_s          (scl_a),
            .sda_s          (sda_a)
        
        
            `ifdef DEBUG
            , 
            .state          (state_a), 
            
            .sda_sync       (sda_sync_a), 
            .scl_sync       (scl_sync_a), 
            .sda_prev       (sda_prev_a), 
            .scl_prev       (scl_prev_a), 
            
            .sda_level      (sda_level_a), 
            .scl_level      (scl_level_a), 
            .sda_rise       (sda_rise_a), 
            .sda_fall       (sda_fall_a), 
            .scl_rise       (scl_rise_a), 
            .scl_fall       (scl_fall_a), 
            .start_seen     (start_seen_a), 
            .stop_seen      (stop_seen_a), 
            
            .sda_in         (sda_in_a), 
            .scl_in         (scl_in_a), 
            .sda_oe         (sda_oe_a), 
            .scl_oe         (scl_oe_a),
            
            .ack_hold       (ack_hold_a), 
            .stretching     (stretching_a), 
            .stretch_req    (stretch_req_a), 
            .ack_req        (ack_req_a), 
            .data_ready     (data_ready_a), 
            .force_release  (force_release_a), 
            .will_ack       (will_ack_a), 
            .tx_bit         (tx_bit_a), 
            
            .bit_cnt        (bit_cnt_a), 
            .byte_cnt       (byte_cnt_a), 
            .busy           (busy_a), 
            .tx_en          (tx_en_a), 
            .rx_en          (rx_en_a), 
            .addr_vld       (addr_vld_a), 
            .subaddr_vld    (subaddr_vld_a), 
            .ack_phase      (ack_phase_a), 
            .rw_mode        (rw_mode_a),
                
            .addr_reg       (addr_reg_a),
            .subaddr_reg    (subaddr_reg_a),
            .r_data_reg     (r_data_reg_a),
            .shift_reg      (shift_reg_a),
            .addr_ack_wait  (addr_ack_wait_a)
            
            `endif
        );
    
   
   
    //----------------------------------
    //      I2C Slave B Instance 
    //----------------------------------
    
        i2c_slave_b b_i2c_slave (
        
            .clk            (clk),
            .reset_n        (reset_n),
            
            .scl_s          (scl_b),
            .sda_s          (sda_b)
        
        
            `ifdef DEBUG
            , 
            .state          (state_b), 
            
            .sda_sync       (sda_sync_b), 
            .scl_sync       (scl_sync_b), 
            .sda_prev       (sda_prev_b), 
            .scl_prev       (scl_prev_b), 
            
            .sda_level      (sda_level_b), 
            .scl_level      (scl_level_b), 
            .sda_rise       (sda_rise_b), 
            .sda_fall       (sda_fall_b), 
            .scl_rise       (scl_rise_b), 
            .scl_fall       (scl_fall_b), 
            .start_seen     (start_seen_b), 
            .stop_seen      (stop_seen_b), 
            
            .sda_in         (sda_in_b), 
            .scl_in         (scl_in_b), 
            .sda_oe         (sda_oe_b), 
            .scl_oe         (scl_oe_b),
            
            .ack_hold       (ack_hold_b), 
            .stretching     (stretching_b), 
            .stretch_req    (stretch_req_b), 
            .ack_req        (ack_req_b), 
            .data_ready     (data_ready_b), 
            .force_release  (force_release_b), 
            .will_ack       (will_ack_b), 
            .tx_bit         (tx_bit_b), 
            
            .bit_cnt        (bit_cnt_b), 
            .byte_cnt       (byte_cnt_b), 
            .busy           (busy_b), 
            .tx_en          (tx_en_b), 
            .rx_en          (rx_en_b), 
            .addr_vld       (addr_vld_b), 
            .subaddr_vld    (subaddr_vld_b), 
            .ack_phase      (ack_phase_b), 
            .rw_mode        (rw_mode_b),
                
            .addr_reg       (addr_reg_b),
            .subaddr_reg    (subaddr_reg_b),
            .r_data_reg     (r_data_reg_b),
            .shift_reg      (shift_reg_b),
            .addr_ack_wait  (addr_ack_wait_b)
            
            `endif
        ); 

    
    always #5 clk = ~clk;
    
    initial begin
        reset_n =   0;
        clk     =   0;
        dac_wr_data     =   0;
        dac_prog        =   0;
        #500;
        
        reset_n   =  1;
        #5000;
        
        dac_wr_data     =   12'd151;
        dac_prog        =   1;
        
            
    end
    
  
    
endmodule
