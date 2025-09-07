
`define DEBUG
`timescale 1fs/1fs
module i2c_master_str
                 (input wire             i_clk,              //input clock to the module @100MHz (or whatever crystal you have on the board)
                  input wire            reset_n,            //reset for creating a known start condition
                  input wire     [7:0]  i_addr_w_rw,        //7 bit address, LSB is the read write bit, with 0 being write, 1 being read
                  input wire     [15:0] i_sub_addr,         //contains sub addr to send to slave, partition is decided on bit_sel
                  input wire            i_sub_len,          //denotes whether working with an 8 bit or 16 bit sub_addr, 0 is 8bit, 1 is 16 bit
                  input wire     [23:0] i_byte_len,         //denotes whether a single or sequential read or write will be performed (denotes number of bytes to read or write)
                  input wire     [7:0]  i_data_write,       //Data to write if performing write action
                  input wire            req_trans,          //denotes when to start a new transaction
                  
                  /** For Reads **/
                  output reg [7:0]  data_out,
                  output reg        valid_out,
                  
                  /** I2C Lines **/
                  inout wire            scl_o,              //i2c clck line, output by this module, 400 kHz
                  inout wire            sda_o,              //i2c data line, set to 1'bz when not utilized (resistors will pull it high)
                  
                  /** Comms to Master Module **/
                  output reg        req_data_chunk ,    //Request master to send new data chunk in i_data_write
                  output reg        busy,               //denotes whether module is currently communicating with a slave
                  output reg        nack,               //denotes whether module is encountering a nack from slave (only activates when master is attempting to contact device)
                  
                  //    DEBUG PROBES FOR SDA INPUT AND OUTPUT
                  output reg        sda_out_debug,
                  output wire       sda_in_debug
                  
                  `ifdef DEBUG
                  ,
                  output reg [3:0]  state,
                  output reg [3:0]  next_state,
                  output reg        reg_sda_o,
                  output reg        reg_scl_o,
                  output reg [7:0]  addr,
                  output reg        rw,
                  output reg [15:0] sub_addr,
                  output reg        sub_len,
                  output reg [23:0] byte_len,
                  output reg [7:0]  bit_counter,
                  output reg        en_scl,
                  output reg        byte_sent,
                  output reg [23:0] num_byte_sent,
                  output reg [2:0]  cntr,
                  output reg [7:0]  byte_sr,
                  output reg        read_sub_addr_sent_flag,
                  output reg [7:0]  data_to_write,
                  output reg [7:0]  data_in_sr,
                  
                  //400KHz clock generation
                  output reg        clk_i2c,
                  output reg [15:0] clk_i2c_cntr,
                  output reg [1:0]  clk_phase,
                  
                  
                  //sampling sda and scl
                  output reg        sda_prev,
                  output reg [1:0]  sda_curr,
                  output reg        scl_prev,
                  output reg [1:0]  scl_curr,
                  output reg        ack_in_prog,
                  output reg        ack_nack,
                  output reg        en_end_indicator,
                  output reg        grab_next_data,
                  output reg        scl_is_high,
                  output reg        scl_is_low
                  `endif
                  );

//For state machine                 
localparam [3:0] IDLE        = 4'd0,
                 START       = 4'd1,
                 RESTART     = 4'd2,
                 SLAVE_ADDR  = 4'd3,
                 SUB_ADDR    = 4'd4,
                 
                 READ        = 4'd5,
                 WRITE       = 4'd6,
                 GRAB_DATA   = 4'd7,
                 ACK_NACK_RX = 4'd8,
                 ACK_NACK_TX = 4'd9,
                 STOP        = 4'hA,
                 RELEASE_BUS = 4'hB;
                 
//Modify These Parameters for other targets
localparam [15:0] DIV_100MHZ = 16'd125;         //desire 400KHz, have 100MHz, thus (1/(400*10^3)*100*10^6)/2, note div by 2 is for need to change in cycle
localparam [7:0]  START_IND_SETUP  = 70,  //Time before negedge of scl
                  START_IND_HOLD   = 60,  //Time after posedge of clock when start occurs (not used)
                  DATA_SETUP_TIME  =  2,  //Time needed before posedge of scl 
                  DATA_HOLD_TIME   =  3,  //Time after negedge that scl is held
                  STOP_IND_SETUP   = 60;  //Time after posedge of scl before stop occurs

localparam [7:0]    SCL_HIGH       = 125,           // 1250 ns
                    SCL_LOW        = 125;           // 1350 ns
                    
                  
`ifndef DEBUG
reg [3:0]  state;
reg [3:0]  next_state;
reg        reg_sda_o;
reg        reg_scl_o;
reg [7:0]  addr;
reg        rw;
reg [15:0] sub_addr;
reg        sub_len;
reg [23:0] byte_len;
reg [7:0]  bit_counter;
reg        en_scl;
reg        byte_sent;
reg [23:0] num_byte_sent;
reg [2:0]  cntr;
reg [7:0]  byte_sr;
reg        read_sub_addr_sent_flag;
reg [7:0]  data_to_write;
reg [7:0]  data_in_sr;

//For generation of 400KHz clock
reg clk_i2c;
reg [15:0] clk_i2c_cntr;
reg [1:0] clk_phase;

//For taking a sample of the scl and sda
reg [1:0] sda_curr;     //So this one is asynchronous especially with replies from the slave, must have synchronization chain of 2
reg       sda_prev;
reg [1:0] scl_curr;     //Required for scl also as master have to suppport clock stretching.
reg       scl_prev;          

reg ack_in_prog;      //For sending acks during read
reg ack_nack;
reg en_end_indicator;

reg grab_next_data;
reg scl_is_high;
reg scl_is_low;
`endif


//clk_i2c 400KHz is synchronous to i_clk, so no need for 2 reg synchronization chain in other blocks
//Note: For other input clks (125MHz) use fractional clock divider

reg     prev_clk_i2c;
wire    clk_i2c_edge = (prev_clk_i2c != clk_i2c);
wire    clk_i2c_rise = (prev_clk_i2c == 0 && clk_i2c == 1);
wire    clk_i2c_fall = (prev_clk_i2c == 1 && clk_i2c == 0);

always@(posedge i_clk or negedge reset_n) begin
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


// clk_i2c 400KHz > Duty cycle change utility as and when required
/*
always @(posedge i_clk or negedge reset_n) begin
    if (!reset_n) begin
        clk_i2c_cntr    <=  1;
        clk_i2c         <=  1;
        phase           <=  1;
    end else if (!en_scl) begin
        clk_i2c_cntr    <=  1;
        clk_i2c         <=  1;
        phase           <=  1;
    end else begin
        // Count clock cycles
        clk_i2c_cntr    <=  clk_i2c_cntr + 1;

        if ((phase && clk_i2c_cntr == SCL_HIGH) || (!phase && clk_i2c_cntr == SCL_LOW)) begin
            clk_i2c         <= ~clk_i2c;       // Toggle SCL
            phase           <= ~phase;     // Flip phase
            clk_i2c_cntr    <= 1;          // Reset count
        end
    end
end
*/

//Main FSM
always@(posedge i_clk or negedge reset_n) begin
    if(!reset_n) begin
        {data_out, valid_out} <= 0;
        {req_data_chunk, busy, nack} <= 0;
        {addr, rw, sub_addr, sub_len, byte_len, en_scl} <= 0;
        {byte_sent, num_byte_sent, cntr, byte_sr} <= 0;
        {read_sub_addr_sent_flag, data_to_write, data_in_sr} <= 0;
        {ack_nack, ack_in_prog, en_end_indicator} <= 0;
        {scl_is_high, scl_is_low, grab_next_data} <= 0;
        reg_sda_o <= 1'bz;
        bit_counter <=  8'b0;
        clk_phase   <=  2'b0;
        state <= IDLE;
        next_state <= IDLE;
    end
    else begin
        valid_out <= 1'b0;
        req_data_chunk <= 1'b0;
        case(state)
            /***
             * State: IDLE
             * Purpose: Moniter the master of this module's readiness to begin a new transaction
             * How it works: clock generation of 400KHz clock is directly tied to beginning the enable line.
             *               The 400KHz clock's cycle begins at high, 125 100MHz clock cyles pass before it is driven low,
             *               therefore next state will seek to drive sda line low, signaling a start bit.
             */
            IDLE: begin
                if(req_trans & !busy) begin
                    //set busy
                    busy <= 1'b1;
                    //Set FSM in motion
                    state <= START;
                    next_state <= SLAVE_ADDR;
                    
                    //Set all master inputs to local registers to modify and or reference later
                    addr <= i_addr_w_rw;
                    rw <= i_addr_w_rw[0];
                    sub_addr <= i_sub_len ? i_sub_addr : {i_sub_addr[7:0], 8'b0};
                    sub_len <= i_sub_len;
                    data_to_write <= i_data_write;
                    byte_len <= i_byte_len;

                     // Reset I2C control signals
                    reg_sda_o <= 1'b1;     // SDA idle (high)
                    reg_scl_o <= 1'b0;     // release SCL line (high-Z)
                    en_scl    <= 1'b1;     //begin the 400kHz generation
                    
                    //Reset flags and or counters
                    nack <= 1'b0;  
                    read_sub_addr_sent_flag <= 1'b0;
                    num_byte_sent <= 0;
                    byte_sent <= 1'b0;
                    
                    bit_counter <=  8'b0;
                    clk_phase   <=  2'b0;
                end
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
                
                case (cntr)
                    2'd0: begin
                        reg_scl_o <= 1'b0;  // Release SCL 
                        if (scl_curr[1] & scl_curr[0]) begin  // Wait until SCL is actually high
                            cntr <= 2'd1;
                        end
                    end
            
                    2'd1: begin
                        // Hold SCL high, then pull SDA low 
                        if (clk_i2c_edge) begin
                            reg_sda_o <= 1'b0;  // SDA falling while SCL is high = START
                            cntr <= 2'd2;
                        end 
                    end
            
                    2'd2: begin
                        // Hold SDA low for some time before driving SCL low
                        if (clk_i2c_cntr == START_IND_HOLD) begin
                            reg_scl_o <= 1'b1;  // Drive SCL low to start bit transmission
                            byte_sr <= {addr[7:1], rw};  // Load slave address
                            cntr <= 2'd0;
                            state <= SLAVE_ADDR;
                            $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: START INDICATION!", $time);
                        end
                    end
                endcase 
            end
            
            
            /***
             * State: restart > NOT UPDATED
             * Purpose: To toggle a repeat start
             * How it works: Must await the negedge of clk, and drive the line high.
             */
            RESTART: begin
                
                reg_scl_o <= 1'b0;  // Let slave stretch SCL if needed
                
                if (scl_curr[1] & scl_curr[0]) begin
                    // Step 3: Wait for START setup time
                    if (clk_i2c_cntr == START_IND_SETUP) begin
                        reg_sda_o <= 1'b0;   // SDA goes low while SCL is high → START
                        byte_sr <= addr;
                        reg_scl_o <= 1'b1;   // Pull SCL low to begin transmission
                        state <= SLAVE_ADDR;
                        $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: RESTART INDICATION!", $time);
                    end
                end
            end
            
            /***
             * State: SLAVE_ADDR
             * Purpose: Write slave addr and based on state of system, move to sub_addr or read
             * How it works: We know that this state will go to either read or to writing the sub addr.
             *               If we reach this state again, the flag will be set, and we know we are performing
             *               a read. The setup time is inconsequential, simply need to account for hold time
             */
            SLAVE_ADDR: begin
            
                case (cntr) 
                    //  First edge encounters is falling edge of clk_i2c
                    3'd0: begin 
                        if (clk_i2c_edge) 
                        begin 
                            if (clk_i2c_fall) 
                            begin
                                reg_scl_o <= 1'b0; // Release SCL (open-drain high) 
                                scl_is_low  <=  1'b0;
                            end
                            else if (clk_i2c_rise) 
                            begin
                                reg_scl_o <= 1'b1; // Pull SCL low // Wait for SCL to actually go high before sending SDA bit 
                                scl_is_low  <=  1'b1;
                            end    
                                
                            if (reg_scl_o == 1'b0 && ~scl_curr[1]) 
                            begin // Wait: slave is stretching the clock 
                                clk_phase <= clk_phase; // Hold phase 
                            end else 
                            begin // Proceed when SCL is high 
                                clk_phase <= clk_phase + 1; 
                                if (clk_phase == 0) 
                                begin 
                                    reg_sda_o <= byte_sr[7]; 
                                end else if (clk_phase == 1) 
                                begin 
                                    byte_sr <= {byte_sr[6:0], 1'b0}; 
                                    if (bit_counter == 7) 
                                    begin 
                                        cntr <= 3'd1; 
                                    end else 
                                    begin 
                                        bit_counter <= bit_counter + 1; 
                                        clk_phase <= 0; 
                                    end 
                                end 
                            end 
                        end 
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
                        byte_sr <= sub_addr[15:8]; 
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
                case (cntr)
            
                    3'd0: begin
                        //  First edge encounters is rising edge of clk_i2c
                        if (clk_i2c_edge) begin
                            if (clk_i2c_rise) begin
                                reg_scl_o   <= 1'b0;      // Release SCL (slave may stretch)
                                scl_is_low  <= 1'b0;
                            end else if (clk_i2c_fall) begin
                                reg_scl_o   <= 1'b1;      // Drive SCL low
                                scl_is_low  <= 1'b1;
                            end
            
                            // Wait if slave is stretching SCL
                            if (reg_scl_o == 1'b0 && ~scl_curr[1]) begin
                                clk_phase <= clk_phase;  // Hold
                            end else begin
                                clk_phase <= clk_phase + 1;
            
                                if (clk_phase == 0) begin
                                    reg_sda_o <= byte_sr[7];  // Present MSB on SDA
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
             * State: Reads > NOT UPDATED
             * Purpose: Read 1 byte messages that are set on posedge of i2c_clk
             * How it Works: Need to read all 8 bits, on posedge of clock. SDA will be
             *               stable high before this occurs, thus it's fine to grab sda_prev,
             *               which is synchronous to i_clk. Every 
             */
            READ: begin
            
                // --- Completed byte read ---
                if (byte_sent) 
                begin
                    byte_sent   <= 1'b0;
                    data_out    <= data_in_sr;
                    valid_out   <= 1'b1;
                    state       <= ACK_NACK_TX;
            
                    next_state  <= (num_byte_sent == byte_len - 1) ? STOP : READ;
                    ack_nack    <= (num_byte_sent == byte_len - 1);  // NACK on last byte
                    num_byte_sent <= num_byte_sent + 1;
                    ack_in_prog <= 1'b1;
            
                    $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: READ BYTE #%d RECEIVED!", $time, num_byte_sent);
                end
                else
                begin
                    // --- Sample bit on SCL rising edge (stretch aware) ---
                    if (!scl_prev && scl_curr) 
                    begin
                        scl_is_high <= 1'b1;
                    end
                
                    if (scl_is_high) 
                    begin
                        if (clk_i2c_cntr >= START_IND_SETUP) 
                        begin
                            valid_out <= 1'b0;
                
                            data_in_sr <= {data_in_sr[6:0], sda_prev};  // Shift in new bit (MSB first)
                            cntr <= cntr + 1;
                
                            if (cntr == 7) begin
                                byte_sent <= 1'b1;
                                cntr <= 0;
                            end
                
                            scl_is_high <= 1'b0;
                        end 
                    end  
                end
            end
            
            
            /***
             * State: Write
             * Purpose: Write specified data words starting from address and incrementing by 1
             * How it Works: Simply send data out 1 byte at a time, with corresponding acks form slave.
             *               When all bytes are written, quit comms.
             */
            WRITE: begin
                case (cntr)
            
                    3'd0: begin
                        
                        if (clk_i2c_edge) 
                        begin
                            case(num_byte_sent[0])
                                1'b0: begin
                                    if (clk_i2c_fall) begin
                                        reg_scl_o   <= 1'b0;   // Release SCL (allow it to go high)
                                        scl_is_low  <= 1'b0;
                                    end else if (clk_i2c_rise) begin
                                        reg_scl_o   <= 1'b1;   // Pull SCL low actively
                                        scl_is_low  <= 1'b1;
                                    end
                                end
                                
                                1'b1: begin
                                    if (clk_i2c_rise) begin
                                        reg_scl_o   <= 1'b0;   // Release SCL (allow it to go high)
                                        scl_is_low  <= 1'b0;
                                    end else if (clk_i2c_fall) begin
                                        reg_scl_o   <= 1'b1;   // Pull SCL low actively
                                        scl_is_low  <= 1'b1;
                                    end
                                end
                            endcase
                            
            
                            // Wait for SCL to go high (clock stretching)
                            if (reg_scl_o == 1'b0 && ~scl_curr[1]) begin
                                clk_phase <= clk_phase;  // Hold
                            end else begin
                                clk_phase <= clk_phase + 1;
            
                                if (clk_phase == 0) begin
                                    reg_sda_o <= byte_sr[7];  // Output MSB
                                end else if (clk_phase == 1) begin
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
                    byte_sr <= i_data_write;
                end
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
                        if (clk_i2c_edge) begin
                        
                            case(next_state)
                                
                                SUB_ADDR: begin
                                    // Toggle SCL via clk_i2c
                                    if (~read_sub_addr_sent_flag)
                                    begin
                                        if (clk_i2c_fall) begin
                                            reg_scl_o  <= 1'b0;     // Release SCL (let slave pull high)
                                            scl_is_low <= 1'b0;
                                            cntr <= 3'd1;           // Advance to sample SDA only after rising edge attempt
                                        end 
                                        else if (clk_i2c_rise) begin
                                            reg_scl_o  <= 1'b1;     // Pull SCL low
                                            reg_sda_o <= 1'bz;
                                            scl_is_low <= 1'b1;
                                            
                                        end
                                    end
                                    else
                                    begin
                                        if (clk_i2c_rise) begin
                                            reg_scl_o  <= 1'b0;     // Release SCL (let slave pull high)
                                            scl_is_low <= 1'b0;
                                            cntr <= 3'd1;           // Advance to sample SDA only after rising edge attempt
                                        end 
                                        else if (clk_i2c_fall) begin
                                            reg_scl_o  <= 1'b1;     // Pull SCL low
                                            reg_sda_o <= 1'bz;
                                            scl_is_low <= 1'b1;
                                            
                                        end
                                    end
                                    
                                end
                                
                                GRAB_DATA: begin
                                    case(num_byte_sent[0])
                                        1'b0: begin
                                            if (clk_i2c_rise) begin
                                                reg_scl_o  <= 1'b0;     // Release SCL (let slave pull high)
                                                scl_is_low <= 1'b0;
                                                cntr <= 3'd1;           // Advance to sample SDA only after rising edge attempt
                                            end 
                                            else if (clk_i2c_fall) begin
                                                reg_scl_o  <= 1'b1;     // Pull SCL low
                                                reg_sda_o <= 1'bz;
                                                scl_is_low <= 1'b1;
                                            end
                                        end
                                        
                                        1'b1: begin
                                            if (clk_i2c_fall) begin
                                                reg_scl_o  <= 1'b0;     // Release SCL (let slave pull high)
                                                scl_is_low <= 1'b0;
                                                cntr <= 3'd1;           // Advance to sample SDA only after rising edge attempt
                                            end 
                                            else if (clk_i2c_rise) begin
                                                reg_scl_o  <= 1'b1;     // Pull SCL low
                                                reg_sda_o <= 1'bz;
                                                scl_is_low <= 1'b1;
                                            end
                                        end
                                    endcase
                                    
                                end
                                
                                STOP: begin
                                    if (clk_i2c_fall) begin
                                        reg_scl_o  <= 1'b0;     // Release SCL (let slave pull high)
                                        scl_is_low <= 1'b0;
                                        cntr <= 3'd1;           // Advance to sample SDA only after rising edge attempt
                                    end 
                                    else if (clk_i2c_rise) begin
                                        reg_scl_o  <= 1'b1;     // Pull SCL low
                                        reg_sda_o <= 1'bz;
                                        scl_is_low <= 1'b1;
                                    end 
                                end
                                
                            endcase
            
                            
            
                        end
                    end
            
                    3'd1: begin
                        // Wait until actual SCL line goes high (not just our intention)
                        if (scl_curr[1] && (clk_i2c_cntr >= DATA_HOLD_TIME)) begin  // Real SCL is high, i.e., rising edge complete
                            if (!sda_curr[1]) begin  // SDA low = ACK
                                state <= next_state;
                                $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: RX ACK", $time);
                            end else begin           // SDA high = NACK
                                nack       <= 1'b1;
                                busy       <= 1'b0;
                                reg_sda_o  <= 1'bz;
                                en_scl     <= 1'b0;
                                state      <= IDLE;
                                $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: RX NACK", $time);
                            end
                            cntr <= 3'd0;
                        end
                    end
            
                endcase
            end
            
            
            /***
             * State: ACK_NACK_TX > NOT UPDATED
             * Purpose: Take hold of SDA to acknowledge the read
             * How it works: On first negedge, since previous state will move on posedge, 
             *               pull the line low for an ack. On second negedge, release sda.
             */
            ACK_NACK_TX: begin
            
            
                case (cntr)
                    3'd0: begin
                        reg_scl_o <= 1'b1;  // Pull SCL low
                        scl_is_low <= 1'b1;
                        cntr <= 3'd1;
                    end
            
                    3'd1: begin
                        if (scl_is_low) begin
                            if (clk_i2c_cntr >= DATA_HOLD_TIME) begin
                                reg_sda_o <= ack_nack;   // Set SDA to ACK/NACK value (0 or 1)
                                ack_in_prog <= 1'b0;
                                scl_is_low <= 1'b0;
                                cntr <= 3'd2;
                            end
                        end
                    end
            
                    3'd2: begin
                        reg_scl_o <= 1'b0;  // Release SCL
                        if (scl_curr[1]) begin
                            cntr <= 3'd3;
                        end
                    end
            
                    3'd3: begin
                        if (clk_i2c_cntr >= DATA_SETUP_TIME) begin
                            reg_sda_o <= (next_state == STOP) ? 1'b0 : 1'bz;
                            en_end_indicator <= (next_state == STOP);
                            state <= next_state;
                            cntr <= 3'd0;
                        end 
                    end
                endcase 
                
            end
            
            /***
             * State: STOP
             * Purpose: Pulls bus low on negedge, and waits for scl to be high
             *          drive sda to high from low, which is stop indication
             */
            STOP: begin 
                case (cntr)
            
                    // Phase 0: Pull SCL low
                    3'd0: begin
                        if (clk_i2c_edge && clk_i2c_rise) begin
                            reg_scl_o <= 1'b1;   // Actively pull SCL low
                            scl_is_low <= 1'b1;
                            cntr <= 3'd1;
                        end
                    end
            
                    // Phase 1: Set SDA low while SCL low
                    3'd1: begin
                        if (scl_is_low && clk_i2c_edge && clk_i2c_fall) begin
                            reg_sda_o <= 1'b0;            // Drive SDA low
                            en_end_indicator <= 1'b1;
                            cntr <= 3'd2;
                            scl_is_low <= 1'b0;
                        end
                    end
            
                    // Phase 2: Release SCL, wait for actual high (may stretch)
                    3'd2: begin
                        if (clk_i2c_edge && clk_i2c_rise) begin
                            reg_scl_o <= 1'b0;   // Release SCL (open-drain high)
                        end
            
                        if (scl_curr[1]) begin   // Wait for SCL to go high
                            cntr <= 3'd3;
                        end
                    end
            
                    // Phase 3: SDA high while SCL high = STOP condition
                    3'd3: begin
                        if (clk_i2c_edge && clk_i2c_rise) begin
                            reg_sda_o <= 1'b1;     // SDA rising while SCL high = STOP
                            state <= RELEASE_BUS;
                            cntr <= 3'd0;
                            $display("DUT: I2C MASTER | TIMESTAMP: %t | MESSAGE: STOP CONDITION", $time);
                        end
                    end
            
                endcase 
            end 

            
            
            /***
             * State: Release bus
             * Purpose: Release the bus
             * How it works: Turn off 400KHz out and release the sda line, go back to idle
             */
            RELEASE_BUS: begin
                if(clk_i2c_cntr == DIV_100MHZ-3) begin
                    en_scl <= 1'b0;
                    state <= IDLE;
                    reg_sda_o <= 1'bz;
                    busy <= 1'b0;
                end
            end
            
            
            default:
                state <= IDLE;
        endcase
    end
end

/*
 * Purpose: grabbing sda from slave
 */
always@(negedge i_clk or negedge reset_n) begin
    if(!reset_n) begin
        {sda_curr, sda_prev} <= 0;
        {scl_curr, scl_prev} <= 0;
    end
    else begin
        sda_curr <= {sda_curr[0], sda_o};  //2 flip flop synchronization chain
        sda_prev <= sda_curr[1];
        scl_curr <= {scl_curr[0], scl_o}; // clk_i2c
        scl_prev <= scl_curr[1];
    end
end

//inout cannot be reg
assign sda_o = reg_sda_o;
assign scl_o = reg_scl_o ? 1'b0 : 1'bz;


//  ASSIGN THE SDA DEBUG LINES
assign sda_in_debug = sda_prev;
always@(posedge i_clk)
begin
    sda_out_debug <= reg_sda_o;
end

endmodule

