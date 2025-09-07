`timescale 1ns / 1ps

//`define DEBUG
module upstream_slave
    #(
        NUM_BYTES_WR = 2'd3
    )
(
    input wire              clk,
    input wire              reset_n,
    
    inout wire              scl_m,
    inout wire              sda_m,
    
    output reg              start_seen, 
    output reg              stop_seen,
    output reg [3:0]        state,
    output reg [3:0]        next_state,
    output reg              stretch_req,
    output reg              remove_str_resume_req,
    output reg [7:0]        addr_reg,
    output reg [7:0]        subaddr_reg,
    output reg [7:0]        r_data_reg,
    output reg [15:0]       shift_reg,
    output reg              rx_next_byte,
    
    
    //  Acknowledgement Outputs from Downstream Master
    
    input wire              addr_translate_done,
    input wire [1:0]        slave_sel,
    input wire              nack_dm,
    input wire              ack_ready_dm,
    input wire              will_ack_dm
    
    
    `ifdef DEBUG
    ,
   
    output reg [1:0]        sda_sync,
    output reg [1:0]        scl_sync,
    output reg              sda_prev,
    output reg              scl_prev,
    
    output reg              sda_level,
    output reg              scl_level,
    output reg              sda_rise,
    output reg              sda_fall,
    output reg              scl_rise,
    output reg              scl_fall,
     
    output wire             scl_in,
    output wire             sda_in,
    output reg              scl_oe,
    output reg              sda_oe,
    output reg              sda_oe_next, 
    output reg              scl_oe_next,
    output reg              ack_hold,
    output reg              stretching,
    
    output reg              ack_req,
    output reg              data_ready,
    output reg              force_release,
    output reg              will_ack,
    output reg              tx_bit,
    
    output reg [3:0]        bit_cnt,
    output reg [1:0]        byte_cnt,
    output reg              busy,       
    output reg              tx_en,          //  Start sending bit-by-bit data to the master
    output reg              rx_en,          //  Start receiving bit-by-bit data coming from the master
    output reg [1:0]        ack_phase,      //  Used to mark the normal data bit shift & ACK/NACK bit shift 
    
    output reg              addr_vld,       //  Flag marking device address validation 
    output reg              subaddr_vld,    //  Flag marking register address validation in the slave
    
    output reg              rw_mode,
    output reg              addr_ack_wait
    
    `endif
    
    );
    
    
    `ifndef DEBUG
    
        reg [1:0]        sda_sync;
        reg [1:0]        scl_sync;
        reg              sda_prev;
        reg              scl_prev;
        
        reg              sda_level;
        reg              scl_level;
        reg              sda_rise;
        reg              sda_fall;
        reg              scl_rise;
        reg              scl_fall;
         
        wire             scl_in;
        wire             sda_in;
        reg              scl_oe;
        reg              sda_oe;
        reg              sda_oe_next; 
        reg              scl_oe_next;
        reg              ack_hold;
        reg              stretching;
        
        reg              ack_req;
        reg              data_ready;
        reg              force_release;
        reg              will_ack;
        reg              tx_bit;
        
        reg [3:0]        bit_cnt;
        reg [1:0]        byte_cnt;
        reg              busy;       
        reg              tx_en;          //  Start sending bit-by-bit data to the master
        reg              rx_en;          //  Start receiving bit-by-bit data coming from the master
        reg [1:0]        ack_phase;      //  Used to mark the normal data bit shift & ACK/NACK bit shift 
        
        reg              addr_vld;       //  Flag marking device address validation 
        reg              subaddr_vld;    //  Flag marking register address validation in the slave
        
        reg              rw_mode;
        reg              addr_ack_wait;
        
    `endif
    
    
    
    
    /*
    *  Purpose: Open Drain Padding for the I2C Lines
    */
  
    // Assignments (open drain)
    assign sda_m    = (sda_oe) ? 1'b0 : 1'bz;
    assign scl_m    = (scl_oe) ? 1'b0 : 1'bz;
    
    // Readback
    assign sda_in = sda_m;
    assign scl_in = scl_m;
    
    
    
    /*
    *  Purpose: 2Flop Synchronizer for sda and scl from master
    */
        
    always @(posedge clk or negedge reset_n) 
    begin
        if(!reset_n) 
        begin
            // idle bus is high
            sda_sync    <=  2'b11;              
            scl_sync    <=  2'b11;
            sda_prev    <=  1'b1;
            scl_prev    <=  1'b1;
        end else 
        begin
            // 2-FF synchronizers
            sda_sync    <= {sda_sync[0], sda_m};    
            scl_sync    <= {scl_sync[0], scl_m};
    
            // Save previous state for edge detect
            sda_prev    <= sda_sync[1];
            scl_prev    <= scl_sync[1];
        end
    end
    
    
    
     /*
     * Purpose: Detecting Edge & Level Transtions on the I2C Lines
     */

        always@(posedge clk or negedge reset_n)
        begin
            if (!reset_n)
            begin
                sda_level   <=  1'b0;
                scl_level   <=  1'b0;
                sda_rise    <=  1'b0;
                sda_fall    <=  1'b0;
                scl_rise    <=  1'b0;
                scl_fall    <=  1'b0;  
                start_seen  <=  1'b0; 
                stop_seen   <=  1'b0;
            end
            else
            begin
                // Stable levels
                sda_level <= sda_sync[1];
                scl_level <= scl_sync[1];
        
                // Edges
                sda_rise  <= (sda_sync[1] && ~sda_prev);
                sda_fall  <= (~sda_sync[1] && sda_prev);
                scl_rise  <= (scl_sync[1] && ~scl_prev);
                scl_fall  <= (~scl_sync[1] && scl_prev);
        
                // Start/Stop
                start_seen <= sda_fall && scl_sync[1];
                stop_seen  <= sda_rise && scl_sync[1];
            end
        end
    
    

    /*
    *  Purpose: Acknowledgement Sequencer
    */
    
    //      It will hold SDA low across 9th clock when we intend to acknowledgement
    //  Enter when FSM asserts ack_req (should be while SCL low, right after byte_done).
    //  Release on the next SCL falling edge.
    
    
        always @(posedge clk or negedge reset_n) 
        begin
            if (!reset_n) 
            begin
                ack_hold <= 1'b0;
            end else 
            begin
                if (ack_req)        ack_hold <= 1'b1;      // start holding low for the ACK bit
                else if (scl_fall)  ack_hold <= 1'b0;      // ACK bit completed (after SCL high)
                if (stop_seen)      ack_hold <= 1'b0;      // safety on STOP
            end
        end
  
  
  
  
    /*
    *  Purpose: Clock Stretching Control
    */
    // Start stretching only while SCL is low; release when 'data_ready' or on STOP.
    
    always @(posedge clk or negedge reset_n) 
    begin
        if (!reset_n) 
        begin
            stretching <= 1'b0;
        end else begin
            if (!stretching && stretch_req && !scl_level)
                stretching <= 1'b1;          // begin holding SCL low
            else if (data_ready || stop_seen)
                stretching <= 1'b0;          // release when ready, or on STOP
        end
    end
  
  
  
  
    /*
    * Purpose: Clock stretching - next state control
    */
    // SDA OE next-value (drive only when allowed by protocol)
    
    always @* 
    begin
        // Default: release
        sda_oe_next = 1'b0;
        
        if (force_release) 
        begin
            sda_oe_next = 1'b0;
        end else 
        begin
            // ACK bit: pull low across the 9th clock if we're ACKing
            if (ack_hold && will_ack)
                sda_oe_next = 1'b1;
            
            // READ data bit (slave -> master):
            // prepare during SCL low; for '0' pull low, for '1' release
            if (tx_en && !scl_level) 
            begin
                if (tx_bit == 1'b0)
                    sda_oe_next = 1'b1;   // drive 0
                // for '1' keep released
            end
            // NOTE: we do not change SDA while SCL is high 
        end
    end
    
    // SCL OE next-value (clock-stretch)
    always @* 
    begin
        // Default: release
        scl_oe_next = 1'b0;
        
        if (force_release) 
        begin
            scl_oe_next = 1'b0;
        end else 
        begin
            // Actively hold SCL low while stretching
            if (stretching)
                scl_oe_next = 1'b1;
        end
    end
    
    // Registered Output Enables 
    always @(posedge clk or negedge reset_n) 
    begin
        if (!reset_n) 
        begin
            sda_oe <= 1'b0;
            scl_oe <= 1'b0;
        end else 
        begin
            sda_oe <= sda_oe_next;
            scl_oe <= scl_oe_next;
        end
    end
    
  
    localparam  [7:0]   ACK_SU          =   8'd5;   //  Setup time 
  
  
  
    //---------------------------------------
    //  State Encoding for Upstream Slave
    //---------------------------------------
    
    localparam [3:0]    BUS_FREE         =  4'd0,   // bus idle (SCL=1, SDA=1) - wait for START
                        ADDRESS_SHIFT    =  4'd1,   // shift in 7 addr bits + R/W (8 bits total)
                        ADDRESS_ACK      =  4'd2,   // if match, drive ACK (SDA=0) or NACK (SDA=Z) as
                        SUBADDR_SHIFT    =  4'd3,   // (write path) shift-in register pointer byte (or N bytes if needed)
                        SUBADDR_ACK      =  4'd4,   // ACK after subaddress; decide continue write or expect repeated START
                        WRITE_SHIFT      =  4'd5,   // shift-in a data byte from master
                        WRITE_ACK        =  4'd6,   // slave drives ACK (or NACK if cannot accept)
                        READ_PREP        =  4'd7,   // load next data byte from register, may clock-stretch if not ready
                        READ_SHIFT       =  4'd8,   // shift-out data bit-by-bit to master
                        READ_ACKIN       =  4'd9,   // sample master's ACK/NACK; NACK → done, ACK → next READ_PREP
                        CLK_STRETCH      =  4'hA;  //  Clock stretch the Master to revert from acknowledgmenet from slave
    
    
    
    
    
    //-------------------------------------
    //  State Machine for Upstream Slave
    //-------------------------------------
    
    always@(posedge clk or negedge reset_n)
    begin
        if (!reset_n)
        begin
            state           <=  BUS_FREE;
            next_state      <=  BUS_FREE;
            
            bit_cnt         <=  4'h0;
            byte_cnt        <=  2'b0;
            busy            <=  1'b0;
            rx_en           <=  1'b0;
            tx_en           <=  1'b0;
            addr_vld        <=  1'b0;
            subaddr_vld     <=  1'b0;
            ack_phase       <=  2'b0;
            rw_mode         <=  1'b0;
            
            stretch_req     <=  1'b0;
            force_release   <=  1'b0;
            tx_bit          <=  1'b0;
            r_data_reg      <=  8'b0;
            
            addr_reg        <=  8'b0;
            subaddr_reg     <=  8'b0;
            shift_reg       <=  16'b0;
            will_ack        <=  1'b0;
            addr_ack_wait   <=  1'b0;
            ack_req         <=  1'b0;
            data_ready      <=  1'b0;
            rx_next_byte    <=  1'b0;
            remove_str_resume_req   <=  1'b0;
        end
        else
        begin
            
            //  First Priority > Stop Condition
            if (stop_seen)
            begin
                state           <=  BUS_FREE;
                next_state      <=  BUS_FREE;
                
                bit_cnt         <=  4'h0;
                byte_cnt        <=  2'b0;
                busy            <=  1'b0;
                rx_en           <=  1'b0;
                tx_en           <=  1'b0;
                ack_phase       <=  2'b0;
                rw_mode         <=  1'b0;
                addr_vld        <=  1'b0;
                subaddr_vld     <=  1'b0;
                will_ack        <=  1'b0;
                addr_ack_wait   <=  1'b0;
                ack_req         <=  1'b0;
                data_ready      <=  1'b0;
                rx_next_byte    <=  1'b0;
                remove_str_resume_req   <=  1'b0;
                
                stretch_req     <=  1'b0;
                force_release   <=  1'b0;
                tx_bit          <=  1'b0;
                r_data_reg      <=  8'b0;
                
                addr_reg        <=  8'b0;
                subaddr_reg     <=  8'b0;
                shift_reg       <=  16'b0;
                
            end
            
            //  Second Priority >  START Condition
            else if (start_seen)
            begin
                state           <=  ADDRESS_SHIFT;
                next_state      <=  ADDRESS_SHIFT;
                
                bit_cnt         <=  4'h0;
                byte_cnt        <=  2'b0;
                busy            <=  1'b1;               //  Assert busy when start of transaction is detected
                rx_en           <=  1'b1;
                tx_en           <=  1'b0;
                ack_phase       <=  2'b0;
//                rw_mode         <=  1'b0;
                will_ack        <=  1'b0;
                addr_ack_wait   <=  1'b0;
                ack_req         <=  1'b0;
                data_ready      <=  1'b0;
                rx_next_byte    <=  1'b0;
                remove_str_resume_req   <=  1'b0;
                
                stretch_req     <=  1'b0;
                force_release   <=  1'b0;
                tx_bit          <=  1'b0;
//                r_data_reg      <=  8'b0;
                
                addr_reg        <=  8'b0;
//                subaddr_reg     <=  8'b0;
//                shift_reg       <=  16'b0;
            
                addr_vld        <=  1'b0;             //  Repeated start will have the same addr and subaddr
                subaddr_vld     <=  1'b0;
            
            end
            
            //  States for the Slave Exec
            else
            begin
                
                case(state)
                
                    //  Makes the Bus Free when reset removed, waits for the start transaction
                    BUS_FREE:
                    begin
                        
                        busy            <=  1'b0;
                        rx_en           <=  1'b0;
                        tx_en           <=  1'b0;
                        ack_phase       <=  2'b0;
                        rw_mode         <=  1'b0;
                        bit_cnt         <=  4'd0;
                        byte_cnt        <=  2'd0;
                        addr_vld        <=  1'b0;
                        subaddr_vld     <=  1'b0;
                        will_ack        <=  1'b0;
                        addr_ack_wait   <=  1'b0;
                        ack_req         <=  1'b0;
                        data_ready      <=  1'b0;
                        rx_next_byte    <=  1'b0;
                        remove_str_resume_req   <=  1'b0;
                        
                        stretch_req     <=  1'b0;
                        force_release   <=  1'b0;
                        tx_bit          <=  1'b0;
                        r_data_reg      <=  8'b0;
                        
                        addr_reg        <=  8'b0;
                        subaddr_reg     <=  8'b0;
                        shift_reg       <=  16'b0;
            
                        // wait for START > handled externally
                        if (start_seen) begin
                          busy          <=  1'b1;
                          rx_en         <=  1'b1;       // begin capturing address byte
                          bit_cnt       <=  4'd0;
                          byte_cnt      <=  2'd0;
                          state         <=  ADDRESS_SHIFT;
                          next_state    <=  ADDRESS_SHIFT;         
                        end
                        
                        
                    end
                    
                    
                    ADDRESS_SHIFT:
                    begin
                        //  Sample SDA on every rising edge of SCL for 8 SCL times
                        
                        byte_cnt      <=  2'd0;
                        if (scl_rise) 
                        begin
                          addr_reg      <=  {addr_reg[6:0], sda_level}; // MSB first
                          bit_cnt       <=  bit_cnt + 4'd1;
                    
                          if (bit_cnt == 4'd7) 
                          begin
                            // 8th bit just captured → full byte available now
                            rw_mode     <=  sda_level;      // LSB of the byte
                            state       <=  CLK_STRETCH;
                            next_state  <=  ADDRESS_ACK;    // move to ACK phase next cycle
                          end
                        end
                                
                    end
                    
                    
                    ADDRESS_ACK:
                    begin
                    
                        //remove ack request
                        remove_str_resume_req   <=  1'b1;
                        
                        case (ack_phase)
                            2'd0: begin
                                // Step 1: Wait for SCL to go low before asserting ACK
                                if (!scl_level) begin
                                    ack_req   <= 1'b1;    // Triggers ack_hold logic
                                    ack_phase <= 2'd1;
                                end
                            end
                            
                            2'd1: begin
                                //State for data setup time on the master sda line
                                ack_phase   <=  2'd2;
                            end
                            
                            2'd2: begin
                                data_ready      <=  1'b1;
                                // Step 2: Wait for ACK clock (SCL high) to complete
                                if (scl_rise) begin
                                    ack_req     <=  1'b0;
                                    ack_phase   <=  2'd0;
                                    bit_cnt     <=  4'b0;
                                    
                                    // Only go to SUBADDR_SHIFT if address matched
                                    state       <= SUBADDR_SHIFT;// ? SUBADDR_SHIFT : BUS_FREE;
                                end
                            end
                        endcase 
                    end
                    
                    
                    SUBADDR_SHIFT:
                    begin
                        data_ready      <=  1'b0;
                        if (scl_rise) 
                        begin
//                          will_ack      <=  1'b0;
                          subaddr_reg   <=  {subaddr_reg[6:0], sda_level}; // MSB first
                          bit_cnt       <=  bit_cnt + 4'd1;
                    
                          if (bit_cnt == 4'd7) 
                          begin
                            //remove ack request
                            remove_str_resume_req   <=  1'b0;
                            // 8th bit just captured > full byte available now
                            rx_next_byte<=  1'b1;
                            state       <=  CLK_STRETCH;    // move to CLK_STRETCH phase next cycle to get response from slave
                            next_state  <=  SUBADDR_ACK;
                          end
                        end
                    end
                    
                    
                    SUBADDR_ACK:
                    begin
                        //remove ack request
                        remove_str_resume_req   <=  1'b1;
                    
                        case (ack_phase)
                            2'd0: begin
                                // Step 1: Eval address match
                                if (!scl_level) begin
                                    ack_req   <= 1'b1;    // Triggers ack_hold logic
                                    ack_phase <= 2'd1;
                                end
                            end
                    
                            2'd1: begin
                                // Step 2: Wait for SCL to go low before asserting ACK
                                ack_phase <= 2'd2;
                            end
                    
                            2'd2: begin
                                data_ready      <=  1'b1;
                                // Step 3: Wait for ACK clock (SCL high) to complete
                                if (scl_rise) begin
                                    ack_req     <=  1'b0;
                                    ack_phase   <=  2'd0;
                                    bit_cnt     <=  4'b0;
                                    
                                    if (will_ack)
                                    begin
                                        state   <=  rw_mode ? READ_PREP : WRITE_SHIFT;
                                    end
                                    else
                                        state   <=  BUS_FREE;
                                end
                            end
                        endcase 
                    end
                    
                    
                    WRITE_SHIFT:
                    begin
                        data_ready      <=  1'b0;
                        if (scl_rise) 
                        begin
//                          will_ack      <=  1'b0;
                          shift_reg     <=  {shift_reg[15:0], sda_level}; // MSB first
                          bit_cnt       <=  bit_cnt + 4'd1;
                    
                          if (bit_cnt == 4'd7) 
                          begin
                            //remove ack request
                            remove_str_resume_req   <=  1'b0;
                            rx_next_byte<=  1'b1;
                            // 8th bit just captured > full byte available now
                            byte_cnt    <=  byte_cnt + 2'b1;
                            state       <=  CLK_STRETCH;   
                            next_state  <=  WRITE_ACK;
                          end
                        end
                    end
                    
                    
                    WRITE_ACK:
                    begin
                        //remove ack request
                        remove_str_resume_req   <=  1'b1;
                    
                        case (ack_phase)
                            2'd0: begin
                                // Step 1: Evaluate address match
                                if (!scl_level) begin
                                    ack_req   <= 1'b1;    // Triggers ack_hold logic
                                    ack_phase <= 2'd1;
                                end
                            end
                    
                            2'd1: begin
                                // Step 2: Wait for SCL to go low before asserting ACK
                                ack_phase <= 2'd2;
                            end
                    
                            2'd2: begin
                                data_ready      <=  1'b1;
                                // Step 3: Wait for ACK clock (SCL high) to complete
                                if (scl_rise) begin
                                    ack_req     <=  1'b0;
                                    ack_phase   <=  2'd0;
                                    bit_cnt     <=  4'b0;
                                    state       <=  WRITE_SHIFT;//( byte_cnt == NUM_BYTES_WR ) ? BUS_FREE : WRITE_SHIFT;
                                end
                            end
                        endcase 
                    end
                    
                    
                    //  READ Operation States yet to be modelled
                    
                    READ_PREP:
                    begin
                        data_ready      <=  1'b0;
                        r_data_reg  <=  byte_cnt ? shift_reg[7:0] : shift_reg[15:8];
                        data_ready  <=  1'b1;
                        tx_en       <=  1'b1;
                        state       <=  READ_SHIFT;
                    end
                    
                    
                    READ_SHIFT:
                    begin
                        if(scl_fall)
                        begin
                            tx_bit  <=  r_data_reg[7-bit_cnt];
                            bit_cnt <=  bit_cnt + 1'b1;
                            if (bit_cnt == 4'd7)
                            begin
                                state   <=  READ_ACKIN;
                            end
                        end
                    end     
                      
                        
                    READ_ACKIN:
                    begin
                        tx_en   <=  1'b0;
                        if (scl_rise)
                        begin
                            if (sda_level == 1'b0)
                            begin
                                byte_cnt    <=  byte_cnt + 1'b1;
                                state       <=  READ_PREP;
                            end
                            else
                                state       <=  BUS_FREE;
                        end
                    end     
                      
                    CLK_STRETCH:
                    begin
                        stretch_req     <=  1'b1;
                        rx_next_byte    <=  1'b0;
                        
                        //  ADDRESS ACKNOWLEDGEMENT FROM SLAVE
                        //  Whether address valid or not, send nack after getting response from the slave
                        
                        case(next_state)
                            
                            //  Clock stretch master before getting ADDR response from acutal slave
                            ADDRESS_ACK: begin
                                if (addr_translate_done)
                                begin
                                    
                                    //  Received nack anytime OR got acknowledgement from dm
                                    if (nack_dm || (ack_ready_dm && !will_ack_dm))
                                    begin
                                        stretch_req     <=  1'b0;
                                        will_ack        <=  1'b0;
                                        data_ready      <=  1'b1;
                                        state           <=  BUS_FREE;  
                                    end  
                                    else if (ack_ready_dm & will_ack_dm)
                                    begin
                                        stretch_req     <=  1'b0;
                                        will_ack        <=  1'b1;
                                        state           <=  next_state;
                                    end
                                end
                            end
                            
                            //  Clock stretch master before getting SUB_ADDR response from acutal slave
                            SUBADDR_ACK: begin
                                //  Received nack anytime OR got acknowledgement from dm
                                if (nack_dm || (ack_ready_dm && !will_ack_dm))
                                begin
                                    stretch_req     <=  1'b0;
                                    will_ack        <=  1'b0;
                                    data_ready      <=  1'b1;
                                    state           <=  BUS_FREE;  
                                end  
                                else if (ack_ready_dm & will_ack_dm)
                                begin
                                    stretch_req     <=  1'b0;
                                    will_ack        <=  1'b1;
                                    state           <=  next_state;
                                end
                            end
                            
                            //  Clock stretch master before getting WRITE_SHIFT response from actual slave
                            WRITE_ACK: begin
                                //  Received nack anytime OR got acknowledgement from dm
                                if (nack_dm || (ack_ready_dm && !will_ack_dm))
                                begin
                                    stretch_req     <=  1'b0;
                                    will_ack        <=  1'b0;
                                    data_ready      <=  1'b1;
                                    state           <=  BUS_FREE;  
                                end  
                                else if (ack_ready_dm & will_ack_dm)
                                begin
                                    stretch_req     <=  1'b0;
                                    will_ack        <=  1'b1;
                                    state           <=  next_state;
                                end
                            end
                            
                        endcase
                        
                    end
                    
                endcase
            
            end
            
        end
    end
    
    
    
    
    
    
    
    
endmodule
