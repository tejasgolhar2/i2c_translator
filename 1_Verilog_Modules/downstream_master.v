`timescale 1ns / 1ps

//`define DEBUG
module downstream_master
(
    input wire          clk,                //  100MHz System Clock
    input wire          reset_n,            //  Active High System Reset
    
    inout wire          scl_a,              //  I2C Slave A - 0x48
    inout wire          sda_a,
    
    inout wire          scl_b,              //  I2C Slave B - 0x49
    inout wire          sda_b,
    
    //  Upstream Slave Inputs
    
    input wire          start_seen_us,
    input wire          stop_seen_us,
    input wire [3:0]    state_us,
    input wire [3:0]    next_state_us,
    input wire          stretch_req_us,
    input wire          remove_str_resume_req,
    
    input wire [7:0]    addr_reg_us,
    input wire [7:0]    subaddr_reg_us,
    input wire [7:0]    r_data_reg_us,
    input wire [15:0]   shift_reg_us,
    input wire          rx_next_byte_us,
       
       
    //  Downstream Master Outputs   
        
    
    output reg          addr_translate_done,    //  Pipeline Acknowledgement Probes
    output reg [1:0]    slave_sel,
    output reg          nack,
    output reg          ack_ready_dm,
    output reg          will_ack_dm
    
    
    
    `ifdef DEBUG
    ,
    
    output reg [7:0]    data_out,
    output reg          valid_out,
    output reg          req_data_chunk,
    output reg          busy,
        
    output reg [3:0]    state,
    output reg [3:0]    next_state,
    output reg          reg_sda_a,
    output reg          reg_scl_a,
    output reg          reg_sda_b,
    output reg          reg_scl_b,
    output reg [7:0]    addr,
    output reg          rw,
    
    output reg [15:0]   sub_addr,
    output reg          sub_len,
    output reg [23:0]   byte_len,
    output reg [7:0]    bit_counter,
    output reg          en_scl,
    output reg          byte_sent,
    output reg [23:0]   num_byte_sent,
    output reg [2:0]    cntr,
    output reg [7:0]    byte_sr,
    output reg          read_sub_addr_sent_flag,
    output reg [7:0]    data_to_write,
    output reg [7:0]    data_in_sr,
    
    //400KHz clock generation
    output reg          clk_i2c,
    output reg [15:0]   clk_i2c_cntr,
    output reg [1:0]    clk_phase,
    
    
    //sampling sda and scl
    output reg          sda_prev_a,
    output reg [1:0]    sda_curr_a,
    output reg          scl_prev_a,
    output reg [1:0]    scl_curr_a,
    output reg          sda_prev_b,
    output reg [1:0]    sda_curr_b,
    output reg          scl_prev_b,
    output reg [1:0]    scl_curr_b,
    output reg          ack_in_prog,
    output reg          ack_nack,
    output reg          en_end_indicator,
    output reg          grab_next_data,
    output reg          scl_a_is_high,
    output reg          scl_b_is_high,
    output reg          scl_a_is_low,
    output reg          scl_b_is_low
    
    `endif
    
    );
    
    
    
    `ifndef DEBUG
        
        reg [7:0]    data_out;
        reg          valid_out;
        reg          req_data_chunk;
        reg          busy;
        
        
        reg [3:0]    state;
        reg [3:0]    next_state;
        reg          reg_sda_a;
        reg          reg_scl_a;
        reg          reg_sda_b;
        reg          reg_scl_b;
        reg [7:0]    addr;
        reg          rw;
        
        reg [15:0]   sub_addr;
        reg          sub_len;
        reg [23:0]   byte_len;
        reg [7:0]    bit_counter;
        reg          en_scl;
        reg          byte_sent;
        reg [23:0]   num_byte_sent;
        reg [2:0]    cntr;
        reg [7:0]    byte_sr;
        reg          read_sub_addr_sent_flag;
        reg [7:0]    data_to_write;
        reg [7:0]    data_in_sr;
        
        //400KHz clock generation
        reg          clk_i2c;
        reg [15:0]   clk_i2c_cntr;
        reg [1:0]    clk_phase;
        
        
        //sampling sda and scl
        reg          sda_prev_a;
        reg [1:0]    sda_curr_a;
        reg          scl_prev_a;
        reg [1:0]    scl_curr_a;
        reg          sda_prev_b;
        reg [1:0]    sda_curr_b;
        reg          scl_prev_b;
        reg [1:0]    scl_curr_b;
        reg          ack_in_prog;
        reg          ack_nack;
        reg          en_end_indicator;
        reg          grab_next_data;
        reg          scl_a_is_high;
        reg          scl_b_is_high;
        reg          scl_a_is_low;
        reg          scl_b_is_low;
        
    `endif
    
    
    
    //  State Machine Encoding
                     
    localparam [3:0] IDLE        = 4'd0,
                     SLAVE_VLD   = 4'd1,
                     START       = 4'd2,
                     RESTART     = 4'd3,
                     SLAVE_ADDR  = 4'd4,
                     SUB_ADDR    = 4'd5,
                     
                     READ        = 4'd6,
                     WRITE       = 4'd7,
                     GRAB_DATA   = 4'd8,
                     ACK_NACK_RX = 4'd9,
                     ACK_NACK_TX = 4'hA,
                     STOP        = 4'hB,
                     RELEASE_BUS = 4'hC,
                     CLK_STRETCH = 4'hD;
    
    
    //Modify Parameter as per Target Reqiurement
    
    localparam [15:0] DIV_100MHZ = 16'd125;         //desire 400KHz, have 100MHz
    localparam [7:0]  START_IND_SETUP  = 70,        //Time before negedge of scl
                      START_IND_HOLD   = 60,        //Time after posedge of clock when start occurs (not used)
                      DATA_SETUP_TIME  =  2,        //Time needed before posedge of scl 
                      DATA_HOLD_TIME   =  3,        //Time after negedge that scl is held
                      STOP_IND_SETUP   = 60;        //Time after posedge of scl before stop occurs
    
    
    localparam [7:0]    VIRT_ADDR_A     =   8'b10010000,        //0x48  -   Virtual Address for Slave A
                        VIRT_ADDR_B     =   8'b10010010,        //0x49  -   Virtual Address for Slave B
                        COMMON_DEV_ADDR =   8'b10010000;        //0x48  -   for both physical slaves connected
    
    
  
    //------------------------------------------------------
    //  Local Timer for Stadard I2C Clock 400Kbps / 400KHz
    //  clk_i2c 400KHz is synchronous to clk
    //------------------------------------------------------
    
    reg     prev_clk_i2c;
    wire    clk_i2c_edge, clk_i2c_rise, clk_i2c_fall;
    
    
    assign    clk_i2c_edge = (prev_clk_i2c != clk_i2c);
    assign    clk_i2c_rise = (prev_clk_i2c == 0 && clk_i2c == 1);
    assign    clk_i2c_fall = (prev_clk_i2c == 1 && clk_i2c == 0);
    
    
    
        always@(posedge clk or negedge reset_n) begin
            if(!reset_n)
            begin
                prev_clk_i2c    <=  1'b1;
                {clk_i2c_cntr, clk_i2c} <= 17'b1;
            end
            else if(!en_scl)
            begin
                prev_clk_i2c    <=  1'b1;
                {clk_i2c_cntr, clk_i2c} <= 17'b1;
            end
            else begin
                prev_clk_i2c    <=  clk_i2c;
                clk_i2c_cntr <= clk_i2c_cntr + 1;
                if(clk_i2c_cntr == DIV_100MHZ-1) begin
                    clk_i2c <= !clk_i2c;
                    clk_i2c_cntr <= 0;
                end
            end
        end
    
    
    
    
    //---------------------------------------------------------------------
    //  Grabbing Data from Both Slave Lines - 2 Flop Synchronization Chain
    //---------------------------------------------------------------------
    
    
        always@(negedge clk or negedge reset_n)
        begin
            if (!reset_n)
            begin
                {sda_curr_a, sda_prev_a} <= 0;
                {scl_curr_a, scl_prev_a} <= 0;
                
                {sda_curr_b, sda_prev_b} <= 0;
                {scl_curr_b, scl_prev_b} <= 0;
            end
            else
            begin
                sda_curr_a <= {sda_curr_a[0], sda_a};  //2 flip flop synchronization chain
                sda_prev_a <= sda_curr_a[1];
                scl_curr_a <= {scl_curr_a[0], scl_a}; 
                scl_prev_a <= scl_curr_a[1];
                
                sda_curr_b <= {sda_curr_b[0], sda_b};  //2 flip flop synchronization chain
                sda_prev_b <= sda_curr_b[1];
                scl_curr_b <= {scl_curr_b[0], scl_b}; 
                scl_prev_b <= scl_curr_b[1];
            end
        end
    
    
    
    
    //-----------------------------------------------
    //      IO Buffer for Tristate I2C Slave Lines
    //-----------------------------------------------
  
    assign sda_a = reg_sda_a;
    assign scl_a = reg_scl_a ? 1'b0 : 1'bz;
    
    assign sda_b = reg_sda_b;
    assign scl_b = reg_scl_b ? 1'b0 : 1'bz;
    
    
    
    
    
    //-----------------------------------------------
    //      Downstream Master FSM
    //-----------------------------------------------
   
   
        always@(posedge clk or negedge reset_n)
        begin
            if (!reset_n)
            begin
                {data_out, valid_out} <= 0;
                {req_data_chunk, busy, nack} <= 0;
                {addr, rw, sub_addr, sub_len, byte_len, en_scl} <= 0;
                {byte_sent, num_byte_sent, cntr, byte_sr} <= 0;
                {read_sub_addr_sent_flag, data_to_write, data_in_sr} <= 0;
                {ack_nack, ack_in_prog, en_end_indicator} <= 0;
                {scl_a_is_high, scl_b_is_high, scl_a_is_low, scl_b_is_low, grab_next_data} <= 0;
                
                reg_scl_a   <= 1'b0;
                reg_sda_a   <= 1'bz;
                
                reg_scl_b   <= 1'b0;
                reg_sda_b   <= 1'bz;
                 
                slave_sel   <=  2'b0;
                addr_translate_done  <=  1'b0;
                 
                bit_counter <=  8'b0;
                clk_phase   <=  2'b0;
                ack_ready_dm  <= 1'b0;
                will_ack_dm   <= 1'b0;       
                
                state       <= IDLE;
                next_state  <= IDLE;
            end
            else
            begin
            
                valid_out <= 1'b0;
                req_data_chunk <= 1'b0;
                
                
                //  STOP DETECT > High Priority
                if (stop_seen_us)
                begin
                
                    {data_out, valid_out}   <=  0;
                    {req_data_chunk, busy, nack}    <=  0;
                    {addr, rw, sub_addr, sub_len, byte_len, en_scl}     <=  0;
                    {byte_sent, num_byte_sent, cntr, byte_sr}           <=  0;
                    {read_sub_addr_sent_flag, data_to_write, data_in_sr}<=  0;
                    {ack_nack, ack_in_prog, en_end_indicator}   <=  0;
                    {scl_a_is_high, scl_b_is_high, scl_a_is_low, scl_b_is_low, grab_next_data}   <=  0;
                    
                    reg_scl_a   <=  1'b0;
                    reg_sda_a   <=  1'bz;
                    
                    reg_scl_b   <=  1'b0;
                    reg_sda_b   <=  1'bz;
                     
                    slave_sel   <=  2'b0;
                    addr_translate_done  <=  1'b0;
                     
                    bit_counter <=  8'b0;
                    clk_phase   <=  2'b0;
                    ack_ready_dm    <=  1'b0;
                    will_ack_dm     <=  1'b0; 
                    state       <=  IDLE;
                    next_state  <=  IDLE;
                    
                end
                
                //  START DETECT > SECOND HIGH Priority
                else if (start_seen_us)
                begin
                
                    {data_out, valid_out}   <=  0;
                    {req_data_chunk, busy, nack}    <=  0;
                    {addr, rw, sub_addr, sub_len, byte_len, en_scl}     <=  0;
                    {byte_sent, num_byte_sent, cntr, byte_sr}           <=  0;
                    {read_sub_addr_sent_flag, data_to_write, data_in_sr}<=  0;
                    {ack_nack, ack_in_prog, en_end_indicator}   <=  0;
                    {scl_a_is_high, scl_b_is_high, scl_a_is_low, scl_b_is_low, grab_next_data}   <=  0;
                    
                    reg_scl_a   <=  1'b0;
                    reg_sda_a   <=  1'bz;
                    
                    reg_scl_b   <=  1'b0;
                    reg_sda_b   <=  1'bz;
                     
                    slave_sel   <=  2'b0;
                    addr_translate_done  <=  1'b0;
                     
                    bit_counter <=  8'b0;
                    clk_phase   <=  2'b0;
                    ack_ready_dm    <=  1'b0;
                    will_ack_dm     <=  1'b0; 
                    state       <=  IDLE;
                    next_state  <=  IDLE;
                    
                end
                
                //  Downstream_Master FSM 
                else 
                begin
                    case(state)
                    
                        /***
                         * State: IDLE
                         * Purpose: Moniter the master of this module's readiness to begin a new transaction
                         * How it works: clock generation of 400KHz clock is directly tied to beginning the enable line.
                         *               The 400KHz clock's cycle begins at high, 125 100MHz clock cyles pass before it is driven low,
                         *               therefore next state will seek to drive sda line low, signaling a start bit.
                         */
                        
                        IDLE: begin
                            if(!busy && stretch_req_us && next_state_us == 4'd2)    //  next_state_us = ADDRESS_ACK 
                            begin
                                byte_len    <=  24'd3;
                                
                                //set busy
                                busy        <=  1'b1;
                                
                                //Set FSM in motion
                                state       <=  SLAVE_VLD;
                                next_state  <=  START;
                                
                                //Set all master inputs to local registers to modify and or reference later
                                addr        <=  addr_reg_us;
                                rw          <=  addr_reg_us[0];

                                addr_translate_done  <=  1'b0;
                                
                                 // Reset I2C control signals for both slaves
                                
                                reg_scl_a   <=  1'b0;     // release SCL line (high-Z) 
                                reg_sda_a   <=  1'bz;     // SDA idle (high)
                                
                                reg_scl_b   <=  1'b0;
                                reg_sda_b   <=  1'bz;
                                
                                //Reset flags and or counters
                                nack        <= 1'b0;  
                                read_sub_addr_sent_flag     <= 1'b0;
                                num_byte_sent   <= 0;
                                byte_sent       <= 1'b0;
                                
                                bit_counter <=  8'b0;
                                clk_phase   <=  2'b0;
                                cntr        <=  2'b0;
                            end
                        end
                        
                        
                        /***
                         * State: SLAVE_VLD
                         * Purpose: Translate the SLAVE Address received from the I2C Master
                         */
                        
                        SLAVE_VLD:
                        begin
                            
                            //  slave_sel : 1 - Slave A; 2 - Slave B; 3 - INVALID SLAVE 
                            
                            if (addr_reg_us[7:1] == VIRT_ADDR_A[7:1])      //  Slave A
                            begin
                                slave_sel  <= 2'd1;                         // talk to A
                                addr       <= addr_reg_us;
                                rw         <= addr_reg_us[0];
                                state      <=  next_state;
                            end
                            else if (addr_reg_us[7:1] == VIRT_ADDR_B[7:1]) //  Slave B
                            begin                            
                                slave_sel  <= 2'd2;                         // talk to B
                                addr       <= addr_reg_us;
                                rw         <= addr_reg_us[0];
                                state      <=  next_state;
                            end
                            else
                            begin
                                //  Invalid Device Address Detected - NACK
                                slave_sel     <= 2'd3;          // invalid
                                nack          <= 1'b1;          // tell upstream to NACK
                                // no B-side activity; keep SDA/SCL released
                                state       <= RELEASE_BUS;     // centralized cleanup before IDLE
                            end
                            addr_translate_done <=  1'b1;
                            cntr                <=  2'b0;
                            en_scl              <=  1'b1;       //begin the 400kHz generation
                        end
                        
                        
                        RELEASE_BUS:
                        begin
                            cntr                <=  2'b0;
                            en_scl              <=  1'b1;
                        end
                        
                        
                        
                        /***
                         * State: START
                         * Purpose: Enable the start signal and move to next appropriate address
                         * How it works: Since this will only be utilized when starting a write or read,
                         *               we know that if read_sub_addr_sent_flag is high, then we are performing a
                         *               read, and that information would have been sent in the input addr. Else,
                         *               even if it was a write, it does not matter.
                         */
                        
                        START: begin
                            
                            //  Initiate transaction on the desired slave
                            //  Choose slave lines depending upon slave_sel
                            //  Unsed SLAVE LINES will be idle
                            
                            case (slave_sel)
                                2'b01: begin
                                    case (cntr)
                                        2'd0: begin
                                            reg_scl_a <= 1'b0;  // Release SCL 
                                            if (scl_curr_a[1] & scl_curr_a[0]) begin  // Wait until SCL is actually high
                                                cntr <= 2'd1;
                                            end 
                                        end
                                
                                        2'd1: begin
                                            // Hold SCL high, then pull SDA low 
                                            if (clk_i2c_edge) begin
                                                reg_sda_a <= 1'b0;  // SDA falling while SCL is high = START
                                                cntr <= 2'd2;
                                            end
                                        end
                                
                                        2'd2: begin
                                            // Hold SDA low for some time before driving SCL low
                                            if (clk_i2c_cntr == START_IND_HOLD) begin
                                                reg_scl_a <= 1'b1;  // Drive SCL low to start bit transmission
                                                byte_sr <= {COMMON_DEV_ADDR[7:1], rw};  // Load slave address
                                                cntr <= 2'd0;
                                                state <= SLAVE_ADDR;
                                                $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: START INDICATION!", $time);
                                            end
                                        
                                        end
                                    endcase 
                                end
                                
                                2'b10: begin
                                    case (cntr)
                                        2'd0: begin
                                            reg_scl_b <= 1'b0;  // Release SCL 
                                            if (scl_curr_b[1] & scl_curr_b[0]) begin  // Wait until SCL is actually high
                                                cntr <= 2'd1;
                                            end 
                                        end
                                
                                        2'd1: begin
                                            // Hold SCL high, then pull SDA low 
                                            if (clk_i2c_edge) begin
                                                reg_sda_b <= 1'b0;  // SDA falling while SCL is high = START
                                                cntr <= 2'd2;
                                            end 
                                        end
                                
                                        2'd2: begin
                                            // Hold SDA low for some time before driving SCL low
                                            if (clk_i2c_cntr == START_IND_HOLD) begin
                                                reg_scl_b <= 1'b1;  // Drive SCL low to start bit transmission
                                                byte_sr <= {COMMON_DEV_ADDR[7:1], rw};  // Load slave address
                                                cntr <= 2'd0;
                                                state <= SLAVE_ADDR;
                                                $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: START INDICATION!", $time);
                                            end
                                        end
                                    endcase 
                                end
                            endcase
                            
                        end
                        
                        
                        
                        /***
                         * State: SLAVE_ADDR
                         * Purpose: Write slave addr and based on state of system, move to sub_addr or read
                         * How it works: We know that this state will go to either read or to writing the sub addr.
                         *               If we reach this state again, the flag will be set, and we know we are performing
                         *               a read. The setup time is inconsequential, simply need to account for hold time
                         */
                        SLAVE_ADDR: 
                        begin
                        
                            case (cntr) 
                                
                                3'd0: begin 
                                
                                    case(slave_sel)
                                        
                                        2'b01: begin    
                                            if (clk_i2c_edge) 
                                            begin 
                                                if (clk_i2c_fall) begin
                                                    reg_scl_a <= 1'b0; // Release SCL (open-drain high) 
                                                    scl_a_is_low  <=  1'b0;
                                                end else if (clk_i2c_rise) begin
                                                    reg_scl_a <= 1'b1; // Pull SCL low // Wait for SCL to actually go high before sending SDA bit 
                                                    scl_a_is_low  <=  1'b1;
                                                end    
                                                    
                                                if (reg_scl_a == 1'b0 && ~scl_curr_a[1]) 
                                                begin // Wait: slave is stretching the clock 
                                                    clk_phase <= clk_phase; // Hold phase 
                                                end else 
                                                begin // Proceed when SCL is high 
                                                    clk_phase <= clk_phase + 1; 
                                                    if (clk_phase == 0) begin 
                                                        reg_sda_a <= byte_sr[7]; 
                                                    end else if (clk_phase == 1) begin 
                                                        byte_sr <= {byte_sr[6:0], 1'b0}; 
                                                        if (bit_counter == 7) begin 
                                                            cntr <= 3'd1; 
                                                        end else begin 
                                                            bit_counter <= bit_counter + 1; 
                                                            clk_phase <= 0; 
                                                        end 
                                                    end 
                                                end 
                                            end 
                                        end
                                        
                                        2'b10: begin
                                            if (clk_i2c_edge) 
                                            begin 
                                                if (clk_i2c_fall) begin
                                                    reg_scl_b <= 1'b0; // Release SCL (open-drain high) 
                                                    scl_b_is_low  <=  1'b0;
                                                end else if (clk_i2c_rise) 
                                                begin
                                                    reg_scl_b <= 1'b1; // Pull SCL low // Wait for SCL to actually go high before sending SDA bit 
                                                    scl_b_is_low  <=  1'b1;
                                                end    
                                                    
                                                if (reg_scl_b == 1'b0 && ~scl_curr_b[1]) 
                                                begin // Wait: slave is stretching the clock 
                                                    clk_phase <= clk_phase; // Hold phase 
                                                end else 
                                                begin // Proceed when SCL is high 
                                                    clk_phase <= clk_phase + 1; 
                                                    if (clk_phase == 0) begin 
                                                        reg_sda_b <= byte_sr[7]; 
                                                    end else if (clk_phase == 1) begin 
                                                        byte_sr <= {byte_sr[6:0], 1'b0}; 
                                                        if (bit_counter == 7) begin 
                                                            cntr <= 3'd1; 
                                                        end else begin 
                                                            bit_counter <= bit_counter + 1; 
                                                            clk_phase <= 0; 
                                                        end 
                                                    end 
                                                end 
                                                
                                            end 
                                        end
                                        
                                    endcase
                                                                         
                                end 
                                
                                3'd1: 
                                begin
                                    if (clk_i2c_cntr >= DATA_HOLD_TIME)
                                    begin
                                        //reg_sda_o <= 1'bz; // Release SDA for ACK 
                                        cntr <= 3'd2; 
                                    end 
                                    
                                end 
                                
                                3'd2: 
                                begin 
                                    state <= ACK_NACK_RX; 
                                    next_state <= read_sub_addr_sent_flag ? READ : SUB_ADDR; 
//                                    byte_sr <= sub_addr[15:8]; 
                                    bit_counter <= 0; 
                                    clk_phase <= 0; 
                                    cntr <= 0; 
                                    $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: SLAVE_ADDR SENT!", $time); 
                                end 
                                
                            endcase
             
                        end
                        
                        
                        
                        
                        
                        /***
                         * State: Sub_addr
                         * Purpose: to grab a sub address
                         * How it Works: Send out the MSB of the sub_addr. If it is 16 bit sub_addr, toggle the flag,
                         *               and then send MSB after receiving ACK. Once this state has finished sending
                         *               sub addr, set the associated flag high, so other states may move to appropriate
                         *               states.
                         */
                        SUB_ADDR: begin
                            
                            ack_ready_dm    <=  1'b0; 
                            will_ack_dm     <=  1'b0;
                            
                            case (cntr)
                                3'd0: begin
                                
                                    case (slave_sel)
                                        2'b01: begin
                                            //  First edge encounters is rising edge of clk_i2c
                                            if (clk_i2c_edge) begin
                                                if (clk_i2c_rise) begin
                                                    reg_scl_a   <= 1'b0;      // Release SCL (slave may stretch)
                                                    scl_a_is_low  <= 1'b0;
                                                end else if (clk_i2c_fall) begin
                                                    reg_scl_a   <= 1'b1;      // Drive SCL low
                                                    scl_a_is_low  <= 1'b1;
                                                end
                                
                                                // Wait if slave is stretching SCL
                                                if (reg_scl_a == 1'b0 && ~scl_curr_a[1]) begin
                                                    clk_phase <= clk_phase;  // Hold
                                                end else begin
                                                    clk_phase <= clk_phase + 1;
                                
                                                    if (clk_phase == 0) begin
                                                        reg_sda_a <= byte_sr[7];  // Present MSB on SDA
                                                    end else if (clk_phase == 1) begin
                                                        byte_sr <= {byte_sr[6:0], 1'b0};  // Shift byte
                                
                                                        if (bit_counter == 7) begin
                                                            cntr <= 3'd1;
                                                        end else begin
                                                            bit_counter <= bit_counter + 1;
                                                            clk_phase <= 0;
                                                        end
                                                    end
                                                end
                                            end
                                        end
                                        
                                        2'b10: begin
                                            //  First edge encounters is rising edge of clk_i2c
                                            if (clk_i2c_edge) begin
                                                if (clk_i2c_rise) begin
                                                    reg_scl_b   <= 1'b0;      // Release SCL (slave may stretch)
                                                    scl_b_is_low  <= 1'b0;
                                                end else if (clk_i2c_fall) begin
                                                    reg_scl_b   <= 1'b1;      // Drive SCL low
                                                    scl_b_is_low  <= 1'b1;
                                                end
                                
                                                // Wait if slave is stretching SCL
                                                if (reg_scl_b == 1'b0 && ~scl_curr_b[1]) begin
                                                    clk_phase <= clk_phase;  // Hold
                                                end else begin
                                                    clk_phase <= clk_phase + 1;
                                
                                                    if (clk_phase == 0) begin
                                                        reg_sda_b <= byte_sr[7];  // Present MSB on SDA
                                                    end else if (clk_phase == 1) begin
                                                        byte_sr <= {byte_sr[6:0], 1'b0};  // Shift byte
                                
                                                        if (bit_counter == 7) begin
                                                            cntr <= 3'd1;
                                                        end else begin
                                                            bit_counter <= bit_counter + 1;
                                                            clk_phase <= 0;
                                                        end
                                                    end
                                                end
                                            end
                                        end
                                    endcase
                                    
                                end
                        
                                3'd1: begin
                                    if (clk_i2c_cntr >= DATA_HOLD_TIME) begin
                                        //reg_sda_o <= 1'bz; // Release SDA for ACK
                                        cntr <= 3'd2;
                                    end
                                end
                        
                                3'd2: begin
                                    state <= ACK_NACK_RX;
                                    byte_sent <= 1'b1;
                                    cntr <= 3'd0;
                                    bit_counter <= 0;
                                    
                                    if (sub_len) begin
                                        next_state <= SUB_ADDR;
                                        sub_len <= 1'b0;
                                        byte_sr <= sub_addr[7:0];
                                        $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: MSB OF SUB ADDR SENT", $time);
                                    end else begin
                                        read_sub_addr_sent_flag <= 1'b1;
                        
                                        if (rw) begin
                                            next_state <= READ;
                                        end else begin
                                            next_state <= GRAB_DATA;
                                            grab_next_data <= 1'b1;
                                        end
                                        $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: SUB ADDR SENT", $time);
                                    end
                                end
                            endcase 
                            
                        end
                        
                        
                        
                        /***
                         * State: GRAB_DATA
                         * Purpose: Grab next 8 bit segment as needed
                         * How it works: dequeue data, then grab the word requested (dequeue is req_data_chunk)
                         */
                        GRAB_DATA: begin
                            if(grab_next_data) begin
                                req_data_chunk <= 1'b1;
                                grab_next_data <= 1'b0;
                            end
                            else begin
                                clk_phase <= 0;
                                state <= WRITE;
                                byte_sr <= shift_reg_us[7:0];
                            end
                        end
                        
                        
                        
                        /***
                         * State: Write
                         * Purpose: Write specified data words starting from address and incrementing by 1
                         * How it Works: Simply send data out 1 byte at a time, with corresponding acks form slave.
                         *               When all bytes are written, quit comms.
                         */
                        WRITE: begin
                            
                            ack_ready_dm    <=  1'b0; 
                            will_ack_dm     <=  1'b0;
                        
                            case (cntr)
                        
                                3'd0: begin
                                    
                                    case (slave_sel)
                                        2'b01: begin
                                            if (clk_i2c_edge) 
                                            begin
                                                case(num_byte_sent[0])
                                                    1'b0: begin
                                                        if (clk_i2c_fall) begin
                                                            reg_scl_a   <= 1'b0;   // Release SCL (allow it to go high)
                                                            scl_a_is_low  <= 1'b0;
                                                        end else if (clk_i2c_rise) begin
                                                            reg_scl_a   <= 1'b1;   // Pull SCL low actively
                                                            scl_a_is_low  <= 1'b1;
                                                        end
                                                    end
                                                        
                                                    1'b1: begin
                                                        if (clk_i2c_rise) begin
                                                            reg_scl_a   <= 1'b0;   // Release SCL (allow it to go high)
                                                            scl_a_is_low  <= 1'b0;
                                                        end else if (clk_i2c_fall) begin
                                                            reg_scl_a   <= 1'b1;   // Pull SCL low actively
                                                            scl_a_is_low  <= 1'b1;
                                                        end
                                                    end
                                                endcase
                                                
                                
                                                // Wait for SCL to go high (clock stretching)
                                                if (reg_scl_a == 1'b0 && ~scl_curr_a[1]) 
                                                begin
                                                    clk_phase <= clk_phase;  // Hold
                                                end else 
                                                begin
                                                    clk_phase <= clk_phase + 1;
                                
                                                    if (clk_phase == 0) 
                                                    begin
                                                        reg_sda_a <= byte_sr[7];  // Output MSB
                                                    end 
                                                    else if (clk_phase == 1) 
                                                    begin
                                                        byte_sr <= {byte_sr[6:0], 1'b0};  // Shift left
                                                        if (bit_counter == 7) begin
                                                            cntr <= 3'd1;
                                                        end else begin
                                                            bit_counter <= bit_counter + 1;
                                                            clk_phase <= 0;
                                                        end
                                                    end
                                                end
                                            end
                                        end
                                        
                                        2'b10: begin
                                            if (clk_i2c_edge) 
                                            begin
                                                case(num_byte_sent[0])
                                                    1'b0: begin
                                                        if (clk_i2c_fall) begin
                                                            reg_scl_b   <= 1'b0;   // Release SCL (allow it to go high)
                                                            scl_b_is_low  <= 1'b0;
                                                        end else if (clk_i2c_rise) begin
                                                            reg_scl_b   <= 1'b1;   // Pull SCL low actively
                                                            scl_b_is_low  <= 1'b1;
                                                        end
                                                    end
                                                        
                                                    1'b1: begin
                                                        if (clk_i2c_rise) begin
                                                            reg_scl_b   <= 1'b0;   // Release SCL (allow it to go high)
                                                            scl_b_is_low  <= 1'b0;
                                                        end else if (clk_i2c_fall) begin
                                                            reg_scl_b   <= 1'b1;   // Pull SCL low actively
                                                            scl_b_is_low  <= 1'b1;
                                                        end
                                                    end
                                                endcase
                                                
                                
                                                // Wait for SCL to go high (clock stretching)
                                                if (reg_scl_b == 1'b0 && ~scl_curr_b[1]) 
                                                begin
                                                    clk_phase <= clk_phase;  // Hold
                                                end else 
                                                begin
                                                    clk_phase <= clk_phase + 1;
                                
                                                    if (clk_phase == 0) 
                                                    begin
                                                        reg_sda_b <= byte_sr[7];  // Output MSB
                                                    end 
                                                    else if (clk_phase == 1) 
                                                    begin
                                                        byte_sr <= {byte_sr[6:0], 1'b0};  // Shift left
                                                        if (bit_counter == 7) begin
                                                            cntr <= 3'd1;
                                                        end else begin
                                                            bit_counter <= bit_counter + 1;
                                                            clk_phase <= 0;
                                                        end
                                                    end
                                                end
                                            end
                                        end
                                    endcase
                                    
                                end
                        
                                3'd1: begin
                                    if (clk_i2c_cntr >= DATA_HOLD_TIME) begin
            //                            reg_sda_o <= 1'bz;    // Release SDA for ACK
                                        byte_sent <= 1'b1;
                                        cntr <= 3'd2;
                                    end
                                end
                        
                                3'd2: begin
                                    next_state     <= (num_byte_sent == byte_len - 1) ? STOP : GRAB_DATA;
                                    num_byte_sent  <= num_byte_sent + 1;
                                    grab_next_data <= 1'b1;
                                    state          <= ACK_NACK_RX;
                        
                                    bit_counter    <= 0;
                                    clk_phase      <= 0;
                                    cntr           <= 0;  
                                    $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: WRITE BYTE #%d SENT!", $time, num_byte_sent);
                                end
                        
                            endcase
                        end
            
                        
                        
                        /***
                         * State: ACK_NACK_RX
                         * Purpose: Receive ack_nack from slave
                         * How it works: sda is already freed, simply look at posedges of scl, and look at data
                         *               remember low is considered an ack, and high is a nack
                         */
                        ACK_NACK_RX: begin
                            case (cntr)
            
                                3'd0: begin
                                    
                                    case(slave_sel)
                                        2'b01: begin
                                            case(next_state)
                                                //  Came from ADDR > waiting for acknowledgement of ADDR from slave
                                                SUB_ADDR: begin
                                                    if (clk_i2c_edge) begin
                                                        // Toggle SCL via clk_i2c
                                                        if (clk_i2c_fall) begin
                                                            reg_scl_a  <= 1'b0;     // Release SCL (let slave pull high)
                                                            scl_a_is_low <= 1'b0;
                                                            cntr <= 3'd1;           // Advance to sample SDA only after rising edge attempt
                                                        end 
                                                        else if (clk_i2c_rise) begin
                                                            reg_scl_a  <= 1'b1;     // Pull SCL low
                                                            reg_sda_a <= 1'bz;
                                                            scl_a_is_low <= 1'b1;
                                                        end
                                                    end
                                                end
                                                
                                                //  Came from SUB_ADDR > waiting for acknowledgement of SUB_ADDR from slave
                                                GRAB_DATA: begin
                                                    case(num_byte_sent[0])
                                                    
                                                        1'b0: begin
                                                            if (clk_i2c_edge) begin
                                                                // Toggle SCL via clk_i2c
                                                                if (clk_i2c_rise) begin
                                                                    reg_scl_a  <= 1'b0;     // Release SCL (let slave pull high)
                                                                    scl_a_is_low <= 1'b0;
                                                                    cntr <= 3'd1;           // Advance to sample SDA only after rising edge attempt
                                                                end 
                                                                else if (clk_i2c_fall) begin
                                                                    reg_scl_a  <= 1'b1;     // Pull SCL low
                                                                    reg_sda_a <= 1'bz;
                                                                    scl_a_is_low <= 1'b1;
                                                                end
                                                            end
                                                        end
                                                        
                                                        1'b1: begin
                                                            if (clk_i2c_edge) begin
                                                                // Toggle SCL via clk_i2c
                                                                if (clk_i2c_fall) begin
                                                                    reg_scl_a  <= 1'b0;     // Release SCL (let slave pull high)
                                                                    scl_a_is_low <= 1'b0;
                                                                    cntr <= 3'd1;           // Advance to sample SDA only after rising edge attempt
                                                                end 
                                                                else if (clk_i2c_rise) begin
                                                                    reg_scl_a  <= 1'b1;     // Pull SCL low
                                                                    reg_sda_a <= 1'bz;
                                                                    scl_a_is_low <= 1'b1;
                                                                end
                                                            end
                                                        end
                                                    endcase
                                                end
                                                 
                                                STOP: begin
                                                    if (clk_i2c_edge) begin
                                                        // Toggle SCL via clk_i2c
                                                        if (clk_i2c_fall) begin
                                                            reg_scl_a  <= 1'b0;     // Release SCL (let slave pull high)
                                                            scl_a_is_low <= 1'b0;
                                                            cntr <= 3'd1;           // Advance to sample SDA only after rising edge attempt
                                                        end 
                                                        else if (clk_i2c_rise) begin
                                                            reg_scl_a  <= 1'b1;     // Pull SCL low
                                                            reg_sda_a <= 1'bz;
                                                            scl_a_is_low <= 1'b1;
                                                        end
                                                    end
                                                end
                                                
                                            endcase 
                                        end
                                        
                                        2'b10: begin
                                            
                                            case(next_state)
                                                //  Came from ADDR > waiting for acknowledgement of ADDR from slave
                                                SUB_ADDR: begin
                                                    if (clk_i2c_edge) begin
                                                        // Toggle SCL via clk_i2c
                                                        if (clk_i2c_fall) begin
                                                            reg_scl_b  <= 1'b0;     // Release SCL (let slave pull high)
                                                            scl_b_is_low <= 1'b0;
                                                            cntr <= 3'd1;           // Advance to sample SDA only after rising edge attempt
                                                        end 
                                                        else if (clk_i2c_rise) begin
                                                            reg_scl_b  <= 1'b1;     // Pull SCL low
                                                            reg_sda_b <= 1'bz;
                                                            scl_b_is_low <= 1'b1;
                                                        end
                                                    end
                                                end
                                                
                                                //  Came from SUB_ADDR > waiting for acknowledgement of SUB_ADDR from slave
                                                GRAB_DATA: begin
                                                    case(num_byte_sent[0])
                                                    
                                                        1'b0: begin
                                                            if (clk_i2c_edge) begin
                                                                // Toggle SCL via clk_i2c
                                                                if (clk_i2c_rise) begin
                                                                    reg_scl_b  <= 1'b0;     // Release SCL (let slave pull high)
                                                                    scl_b_is_low <= 1'b0;
                                                                    cntr <= 3'd1;           // Advance to sample SDA only after rising edge attempt
                                                                end 
                                                                else if (clk_i2c_fall) begin
                                                                    reg_scl_b  <= 1'b1;     // Pull SCL low
                                                                    reg_sda_b <= 1'bz;
                                                                    scl_b_is_low <= 1'b1;
                                                                end
                                                            end
                                                        end
                                                        
                                                        1'b1: begin
                                                            if (clk_i2c_edge) begin
                                                                // Toggle SCL via clk_i2c
                                                                if (clk_i2c_fall) begin
                                                                    reg_scl_b  <= 1'b0;     // Release SCL (let slave pull high)
                                                                    scl_b_is_low <= 1'b0;
                                                                    cntr <= 3'd1;           // Advance to sample SDA only after rising edge attempt
                                                                end 
                                                                else if (clk_i2c_rise) begin
                                                                    reg_scl_b  <= 1'b1;     // Pull SCL low
                                                                    reg_sda_b <= 1'bz;
                                                                    scl_b_is_low <= 1'b1;
                                                                end
                                                            end
                                                        end
                                                    endcase
                                                end
                                                
                                                //  Come from WRITE_ACK state and BYTE_LEN bytes are sent <Hardcoded here> 
                                                STOP: begin
                                                    if (clk_i2c_edge) begin
                                                        // Toggle SCL via clk_i2c
                                                        if (clk_i2c_fall) begin
                                                            reg_scl_b  <= 1'b0;     // Release SCL (let slave pull high)
                                                            scl_b_is_low <= 1'b0;
                                                            cntr <= 3'd1;           // Advance to sample SDA only after rising edge attempt
                                                        end 
                                                        else if (clk_i2c_rise) begin
                                                            reg_scl_b  <= 1'b1;     // Pull SCL low
                                                            reg_sda_b <= 1'bz;
                                                            scl_b_is_low <= 1'b1;
                                                        end
                                                    end
                                                end
                                                
                                            endcase
                                            
                                        end
                                    endcase
                                
                                    
                                end
                        
                                3'd1: begin
                                    case(slave_sel)
                                        2'b01: begin
                                            // Wait until actual SCL line goes high (not just our intention)
                                            if (scl_curr_a[1] && (clk_i2c_cntr >= DATA_HOLD_TIME)) begin  // Real SCL is high, i.e., rising edge complete
                                                if (!sda_curr_a[1]) begin  // SDA low = ACK
                                                    will_ack_dm  <= 1'b1;
                                                    nack        <=  1'b0;
                                                    $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: RX ACK", $time);
                                                end else begin           // SDA high = NACK
                                                    will_ack_dm  <= 1'b0; 
                                                    nack       <= 1'b1;
                                                    busy       <= 1'b0;
                                                    reg_sda_a  <= 1'bz;
                                                    en_scl     <= 1'b0;
                                                    $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: RX NACK", $time);
                                                end
                                                ack_ready_dm <= 1'b1;
                                                cntr <= 3'd0;
                                                state <= CLK_STRETCH;
                                            end
                                        end
                                        
                                        2'b10: begin
                                            // Wait until actual SCL line goes high (not just our intention)
                                            if (scl_curr_b[1] && (clk_i2c_cntr >= DATA_HOLD_TIME)) begin  // Real SCL is high, i.e., rising edge complete
                                                if (!sda_curr_b[1]) begin  // SDA low = ACK
                                                    nack        <=  1'b0;
                                                    will_ack_dm  <= 1'b1;
                                                    $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: RX ACK", $time);
                                                end else begin           // SDA high = NACK
                                                    will_ack_dm  <= 1'b0;
                                                    nack       <= 1'b1;
                                                    busy       <= 1'b0;
                                                    reg_sda_b  <= 1'bz;
                                                    en_scl     <= 1'b0;
                                                    $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: RX NACK", $time);
                                                end
                                                ack_ready_dm <= 1'b1;
                                                cntr <= 3'd0;
                                                state <= CLK_STRETCH;
                                            end
                                        end
                                    endcase
                                end
                            endcase
                            
                        end
                       
                       
                        CLK_STRETCH:
                        begin
                        
                            //  hold scl low of respective low, if will_ack_dm, else IDLE
                            
                            if (rx_next_byte_us)
                            begin
                                state   <=  next_state;
                                case (next_state)
                                    
                                    SUB_ADDR: begin
                                        byte_sr <=  subaddr_reg_us;
                                    end
                                    
                                endcase
                            end
                            
                            if (remove_str_resume_req && clk_i2c_edge)
                            begin
                                ack_ready_dm    <=  1'b0;
                                will_ack_dm     <=  1'b0;
                            end
                             
                            case(slave_sel)
                            
                                2'd01: begin
                                    case (next_state)
                                        //  Clock stretching of slave after ADDR 
                                        SUB_ADDR: begin
                                            if (ack_ready_dm)
                                            begin
                                                if (will_ack_dm)
                                                begin
                                                    if (clk_i2c_rise) begin
                                                        reg_scl_a       <=  1'b1;     // Pull SCL low
                                                        reg_sda_a       <=  1'bz;
                                                        scl_a_is_low    <=  1'b1;
                                                    end
                                                end
                                                else
                                                begin
                                                    state   <=  IDLE;
                                                end
                                            end
                                        end
                                        
                                        //  Clock stretching of slave after SUB_ADDR
                                        GRAB_DATA: begin
                                            case(num_byte_sent[0])
                                                
                                                1'b0: begin
                                                    if (ack_ready_dm)
                                                    begin
                                                        if (will_ack_dm)
                                                        begin
                                                            if (clk_i2c_fall) begin
                                                                reg_scl_a       <=  1'b1;     // Pull SCL low
                                                                reg_sda_a       <=  1'bz;
                                                                scl_a_is_low    <=  1'b1;
                                                            end
                                                        end
                                                        else
                                                        begin
                                                            state   <=  IDLE;
                                                        end
                                                    end
                                                end
                                                
                                                1'b1: begin
                                                    if (ack_ready_dm)
                                                    begin
                                                        if (will_ack_dm)
                                                        begin
                                                            if (clk_i2c_rise) begin
                                                                reg_scl_a       <=  1'b1;     // Pull SCL low
                                                                reg_sda_a       <=  1'bz;
                                                                scl_a_is_low    <=  1'b1;
                                                            end
                                                        end
                                                        else
                                                        begin
                                                            state   <=  IDLE;
                                                        end
                                                    end
                                                end 
                                                
                                            endcase  
                                        end
                                        
                                        STOP: begin
                                            if (ack_ready_dm)
                                            begin
                                                if (will_ack_dm)
                                                begin
                                                    if (clk_i2c_rise) begin
                                                        reg_scl_a       <=  1'b1;     // Pull SCL low
                                                        reg_sda_a       <=  1'bz;
                                                        scl_a_is_low    <=  1'b1;
                                                    end
                                                end
                                                else
                                                begin
                                                    state   <=  IDLE;
                                                end
                                            end 
                                        end
                                    endcase
                                end
                                
                                2'b10: begin
                                    
                                    case (next_state)
                                        //  Clock stretching of slave after ADDR 
                                        SUB_ADDR: begin
                                            if (ack_ready_dm)
                                            begin
                                                if (will_ack_dm)
                                                begin
                                                    if (clk_i2c_rise) begin
                                                        reg_scl_b       <=  1'b1;     // Pull SCL low
                                                        reg_sda_b       <=  1'bz;
                                                        scl_b_is_low    <=  1'b1;
                                                    end
                                                end
                                                else
                                                begin
                                                    state   <=  IDLE;
                                                end
                                            end
                                        end
                                        
                                        //  Clock stretching of slave after SUB_ADDR
                                        GRAB_DATA: begin
                                            case(num_byte_sent[0])
                                                
                                                1'b0: begin
                                                    if (ack_ready_dm)
                                                    begin
                                                        if (will_ack_dm)
                                                        begin
                                                            if (clk_i2c_fall) begin
                                                                reg_scl_b       <=  1'b1;     // Pull SCL low
                                                                reg_sda_b       <=  1'bz;
                                                                scl_b_is_low    <=  1'b1;
                                                            end
                                                        end
                                                        else
                                                        begin
                                                            state   <=  IDLE;
                                                        end
                                                    end
                                                end
                                                
                                                1'b1: begin
                                                    if (ack_ready_dm)
                                                    begin
                                                        if (will_ack_dm)
                                                        begin
                                                            if (clk_i2c_rise) begin
                                                                reg_scl_b       <=  1'b1;     // Pull SCL low
                                                                reg_sda_b       <=  1'bz;
                                                                scl_b_is_low    <=  1'b1;
                                                            end
                                                        end
                                                        else
                                                        begin
                                                            state   <=  IDLE;
                                                        end
                                                    end
                                                end 
                                                
                                            endcase  
                                        end
                                        
                                        STOP: begin
                                            if (ack_ready_dm)
                                            begin
                                                if (will_ack_dm)
                                                begin
                                                    if (clk_i2c_rise) begin
                                                        reg_scl_b       <=  1'b1;     // Pull SCL low
                                                        reg_sda_b       <=  1'bz;
                                                        scl_b_is_low    <=  1'b1;
                                                    end
                                                end
                                                else
                                                begin
                                                    state   <=  IDLE;
                                                end
                                            end 
                                        end
                                    endcase
                                    
                                        
                                end
                                
                            endcase 
                            
                        end
                       
                       
                       
                       
                       
                    endcase
                end
                            
            end
        end
   
endmodule
