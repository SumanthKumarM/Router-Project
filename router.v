// Router Top Block
module Router_1_3(
    output [7:0]data_out_0,data_out_1,data_out_2,
    output valid_out_0,valid_out_1,valid_out_2,error,busy,
    input clock,reset,
    read_enb_0,read_enb_1,read_enb_2,pkt_valid,
    input [7:0]data_in);

    wire detect_addr,ld_state,laf_state,
    full_state,lfd_state,write_enb_reg,
    rst_int_reg,low_pkt_valid,parity_done,fifo_full;
    wire [2:0] soft_reset,fifo_empty,full,write_enb;
    wire [7:0]d_out;

    controller FSM(detect_addr,ld_state,laf_state,full_state,lfd_state,
    write_enb_reg,rst_int_reg,busy,clock,reset,pkt_valid,
    parity_done,fifo_full,soft_reset[0],soft_reset[1],
    soft_reset[2],fifo_empty[0],fifo_empty[1],fifo_empty[2],
    low_pkt_valid,data_in[1:0]);

    Register REGISTER(parity_done,low_pkt_valid,error,
    d_out,clock,reset,pkt_valid,fifo_full,
    rst_int_reg,detect_addr,ld_state,
    laf_state,full_state,lfd_state,data_in);

    synchronizer SYNCHRONIZER(soft_reset[0],soft_reset[1],soft_reset[2],
    valid_out_0,valid_out_1,valid_out_2,write_enb,fifo_full,
    read_enb_0,read_enb_1,read_enb_2,data_in[1:0],detect_addr,
    write_enb_reg,full[0],full[1],full[2],
    fifo_empty[0],fifo_empty[1],fifo_empty[2],clock,reset);

    router_fifo FIFO_0(data_out_0,full[0],fifo_empty[0],clock,reset,
    write_enb[0],read_enb_0,soft_reset[0],lfd_state,d_out);

    router_fifo FIFO_1(data_out_1,full[1],fifo_empty[1],clock,reset,
    write_enb[1],read_enb_1,soft_reset[1],lfd_state,d_out);

    router_fifo FIFO_2(data_out_2,full[2],fifo_empty[2],clock,reset,
    write_enb[2],read_enb_2,soft_reset[2],lfd_state,d_out);
endmodule

// FIFO Block
module router_fifo(
    output reg [7:0]data_out,
    output full,empty,
    input clk,reset,write_enb,read_enb,soft_rst,lfd_state,
    input [7:0]data_in);

    // memory declaration
    reg [8:0]mem[15:0];
    reg [4:0]wr_pntr,rd_pntr;
    reg [3:0]fifo_cntr;
    reg lfd_state_s;
    integer i;

    assign full=(wr_pntr=={~rd_pntr[4],rd_pntr[3:0]})?1'b1:1'b0;
    assign empty=(wr_pntr==rd_pntr)?1'b1:1'b0;

    // FIFO counter
    always@(posedge clk) begin
        if(!reset) fifo_cntr<=0;
        else if(soft_rst) fifo_cntr<=0;
        else if(read_enb && ~empty) begin
            if(mem[rd_pntr[3:0]][8]==1'b1) fifo_cntr<=mem[rd_pntr[3:0]][7:2]+1'b1;
            else if(fifo_cntr!=0) fifo_cntr<=fifo_cntr-1'b1;
        end
    end

    // Wr_pntr 'n' rd_pntr logic
    always@(posedge clk) begin
        if(!reset) begin
            wr_pntr<=0;
            rd_pntr<=0;
        end
        else if(soft_rst) begin
            wr_pntr<=0;
            rd_pntr<=0;
        end
        else begin
            if(write_enb && ~full) wr_pntr<=wr_pntr+1'b1;
            else wr_pntr<=wr_pntr;
            if(read_enb && ~empty) rd_pntr<=rd_pntr+1'b1;
            else rd_pntr<=rd_pntr;
        end
    end

    // dalying lfd_state by one clk pulse
    always@(posedge clk) begin
        if(!reset) lfd_state_s<=0;
        else lfd_state_s<=lfd_state;
    end

    // Write operation
    always@(posedge clk) begin
        if(!reset) begin
            for(i=0;i<16;i=i+1) begin
                mem[i]<=0;
            end
        end
        else if(soft_rst) begin
            for(i=0;i<16;i=i+1) begin
                mem[i]<=0;
            end
        end
        else if(write_enb && !full) mem[wr_pntr[3:0]]<={lfd_state_s,data_in};
    end

    // Read operation
    wire x=(fifo_cntr==0 && data_out!=0)?1'b1:1'b0;
    always@(posedge clk) begin
        if(!reset) data_out<=0;
        else if(soft_rst) data_out<=8'bzzzzzzzz;
        else begin
            if(x) data_out<=8'bzzzzzzzz;
            else if(read_enb && ~empty) data_out<=mem[rd_pntr[3:0]][7:0];
        end
    end
endmodule

// Synchronizer Block
module synchronizer(
    output reg soft_reset_0,soft_reset_1,soft_reset_2,
    output vld_out_0,vld_out_1,vld_out_2,
    output reg [2:0]write_enb,
    output reg fifo_full,
    input read_enb_0,read_enb_1,read_enb_2,
    input [1:0]data_in,
    input detect_addr,write_enb_reg,
    input full_0,full_1,full_2,
    input empty_0,empty_1,empty_2,clk,resetn);

    reg [4:0]timer_0,timer_1,timer_2;
    reg [1:0]int_reg_addr;

    wire [2:0]w;

    assign w[0]=(timer_0==5'd29)?1'b1:1'b0;
    assign w[1]=(timer_1==5'd29)?1'b1:1'b0;
    assign w[2]=(timer_2==5'd29)?1'b1:1'b0;

    // Generating soft_reset_0.
    always@(posedge clk) begin
        if(!resetn) begin
            soft_reset_0<=0;
            timer_0<=0;
        end
        else if(vld_out_0) begin
            if(!read_enb_0) begin
                if(w[0]) begin
                    timer_0<=0;
                    soft_reset_0<=1'b1;
                end
                else begin
                    timer_0<=timer_0+1'b1;
                    soft_reset_0<=0;
                end
            end
        end
    end

    // Generating soft_reset_1.
    always@(posedge clk) begin
        if(!resetn) begin
            soft_reset_1<=0;
            timer_1<=0;
        end
        else if(vld_out_1) begin
            if(!read_enb_1) begin
                if(w[1]) begin
                    timer_1<=0;
                    soft_reset_1<=1'b1;
                end
                else begin
                    timer_1<=timer_1+1'b1;
                    soft_reset_1<=0;
                end
            end
        end
    end

    // Generating soft_reset_0.
    always@(posedge clk) begin
        if(!resetn) begin
            soft_reset_2<=0;
            timer_2<=0;
        end
        else if(vld_out_2) begin
            if(!read_enb_2) begin
                if(w[2]) begin
                    timer_2<=0;
                    soft_reset_2<=1'b1;
                end
                else begin
                    timer_2<=timer_2+1'b1;
                    soft_reset_2<=0;
                end
            end
        end
    end
    
    // Internal Register Updation
    always@(posedge clk) begin
        if(!resetn) int_reg_addr<=0;
        else if(detect_addr) int_reg_addr<=data_in; 
    end

    // Logic for Write_enb
    always@(*) begin
        write_enb=3'b000;
        if(write_enb_reg) begin
            case (data_in)
                2'b00 : write_enb=3'b001;
                2'b01 : write_enb=3'b010;
                2'b10 : write_enb=3'b100; 
                default: write_enb=3'b000;
            endcase
        end
    end

    // Logic for fifo_full and vld_out
    always@(*) begin
        case (int_reg_addr)
            2'b00 : fifo_full=full_0;
            2'b01 : fifo_full=full_1;
            2'b10 : fifo_full=full_2; 
            default: fifo_full=1'b0;
        endcase
    end

    assign vld_out_0 = ~empty_0;
    assign vld_out_1 = ~empty_1;
    assign vld_out_2 = ~empty_2;
endmodule

// Router Controller
module controller(
    output detect_addr,ld_state,laf_state,
    full_state,lfd_state,write_enb_reg,rst_int_reg,busy,
    input clk,resetn,pkt_valid,parity_done,fifo_full,
    soft_reset_0,soft_reset_1,soft_reset_2,
    fifo_empty_0,fifo_empty_1,fifo_empty_2,low_pkt_valid,
    input [1:0]data_in);

    // Declaring states
    parameter Decode_addr = 3'b000,
    Load_First_Data = 3'b001,
    Load_Data = 3'b010,
    FIFO_Full_State = 3'b011,
    Load_After_Full = 3'b100,
    Load_Parity = 3'b101,
    Check_Parity_Error = 3'b110,
    Wait_Till_Empty = 3'b111;

    reg [2:0]present_state,next_state,addr;

    // Present state logic
    always@(posedge clk) begin
        if(!resetn) present_state<=Decode_addr;
        else if((soft_reset_0 && (data_in==2'b00)) || (soft_reset_1 && (data_in==2'b01))
        || (soft_reset_2 && (data_in==2'b10))) present_state<=Decode_addr;
        else present_state<=next_state;
    end

    // Next state logic
    always@(*) begin
        next_state=present_state;
        case (present_state)
            Decode_addr : begin
                if((pkt_valid && (data_in==0) && fifo_empty_0) || 
                (pkt_valid && (data_in==1) && fifo_empty_1) || 
                (pkt_valid && (data_in==2) && fifo_empty_2)) next_state=Load_First_Data;
                else if((pkt_valid && (data_in==0) && ~fifo_empty_0) || 
                    (pkt_valid && (data_in==1) && ~fifo_empty_1) || 
                    (pkt_valid && (data_in==2) && ~fifo_empty_2)) next_state=Wait_Till_Empty;
                else next_state=Decode_addr;
            end
            Load_First_Data : next_state=Load_Data;
            Load_Data : begin
                if(fifo_full) next_state=FIFO_Full_State;
                else if(!fifo_full && !pkt_valid) next_state=Load_Parity;
                else next_state=Load_Data;
            end
            FIFO_Full_State : begin
                if(!fifo_full) next_state=Load_After_Full;
                else if(fifo_full) next_state=FIFO_Full_State;
            end
            Load_After_Full : begin
                if(!parity_done && low_pkt_valid) next_state=Load_Parity;
                else if(!parity_done && !low_pkt_valid) next_state=Load_Data;
                else if(parity_done) next_state=Decode_addr;
            end
            Load_Parity : next_state=Check_Parity_Error;
            Check_Parity_Error : begin
                if(fifo_full) next_state=FIFO_Full_State;
                else if(!fifo_full) next_state=Decode_addr;
            end
            Wait_Till_Empty : begin
                if((fifo_empty_0 && (addr==0)) || (fifo_empty_1 && (addr==1)) ||
                (fifo_empty_0 && (addr==2))) next_state=Load_First_Data;
                else next_state=Wait_Till_Empty;
            end
            default: next_state=Decode_addr;
        endcase
    end

    // internal variable addr logic
    always@(posedge clk) begin
        if(~resetn) addr<=0;
        else if((soft_reset_0 && (data_in==2'b00)) || (soft_reset_1 && (data_in==2'b01))
        || (soft_reset_2 && (data_in==2'b10))) addr<=0;
        else if(detect_addr) addr<=data_in;
    end

    // Output signals
    assign detect_addr=(present_state==Decode_addr) ? 1'b1 : 1'b0;
    assign lfd_state=(present_state==Load_First_Data) ? 1'b1 : 1'b0;
    assign ld_state=(present_state==Load_Data) ? 1'b1 : 1'b0;
    assign full_state=(present_state==FIFO_Full_State) ? 1'b1 : 1'b0;
    assign laf_state=(present_state==Load_After_Full) ? 1'b1 : 1'b0;
    assign rst_int_reg=(present_state==Check_Parity_Error) ? 1'b1 : 1'b0;
    assign write_enb_reg=((present_state==Load_Data) || (present_state==Load_After_Full) || 
           (present_state==Load_Parity)) ? 1'b1 : 1'b0;
    assign busy=((present_state==Load_First_Data) || (present_state==FIFO_Full_State) ||
           (present_state==Load_After_Full) || (present_state==Load_Parity) || 
           (present_state==Check_Parity_Error) || (present_state==Wait_Till_Empty)) ? 1'b1 : 1'b0;
endmodule

// Register
module Register(
    output reg parity_done,low_pkt_valid,err,
    output reg [7:0]data_out,
    input clk,resetn,pkt_valid,
    fifo_full,rst_int_reg,detect_addr,
    ld_state,laf_state,full_state,lfd_state,
    input [7:0]data_in);

    reg [7:0] Header_byte_reg,fifo_full_state_reg;
    reg [7:0] Packet_parity,internal_parity;

    // Data_out logic
    always@(posedge clk) begin
        if(!resetn) data_out<=0;
        else if(lfd_state) data_out<=Header_byte_reg;
        else if(ld_state && !fifo_full) data_out<=data_in;
        else if(laf_state) data_out<=fifo_full_state_reg;
        else data_out<=data_out;
    end

    // Header byte and fifo_full_state_byte
    always@(posedge clk) begin
        if(!resetn) {Header_byte_reg,fifo_full_state_reg}<=0;
        else begin
            if(pkt_valid && detect_addr) Header_byte_reg<=data_in;
            else if(ld_state && fifo_full) fifo_full_state_reg<=data_in;
        end
    end

    // low_pkt_valid logic
    always@(posedge clk) begin
        if(!resetn) low_pkt_valid<=0;
        else begin
            if(rst_int_reg) low_pkt_valid<=0;
            else if(~pkt_valid && ld_state) low_pkt_valid<=1'b1;
        end
    end

    // Parity_done logic
    always@(posedge clk) begin
        if(~resetn) parity_done<=0;
        else begin
            if(ld_state && ~pkt_valid && ~fifo_full) parity_done<=1'b1;
            else if(laf_state && ~parity_done && low_pkt_valid) parity_done<=1'b1;
            else begin
                if(detect_addr) parity_done<=1'b0;
            end
        end
    end

    // Packet_parity logic
    always@(posedge clk) begin
        if(~resetn) Packet_parity<=0;
        else if((ld_state && ~pkt_valid && ~fifo_full) || 
        (laf_state && low_pkt_valid && ~parity_done)) Packet_parity<=data_in;
        else if(~pkt_valid && rst_int_reg) Packet_parity<=0; 
        else begin
            if(detect_addr) Packet_parity<=0;
        end
    end

    // Internal_parity
    always@(posedge clk) begin
        if(~resetn) internal_parity<=0;
        else if(detect_addr) internal_parity<=0;
        else if(lfd_state) internal_parity<=Header_byte_reg;
        else if(ld_state && pkt_valid && ~full_state) internal_parity<=internal_parity^data_in;
        else if(~pkt_valid && rst_int_reg) internal_parity<=0; 
    end

    // error logic
    always@(posedge clk) begin
        if(~resetn) err<=0;
        else begin
            if((parity_done==1'b1) && (internal_parity!=Packet_parity)) err<=1'b1;
            else err<=0;
        end
    end
endmodule

// Test Bench
module tb;
wire [7:0]data_out_0,data_out_1,data_out_2;
wire valid_out_0,valid_out_1,valid_out_2,error,busy;
reg clock,reset,read_enb_0,read_enb_1,read_enb_2,pkt_valid;
reg [7:0]data_in;

Router_1_3 DUT(data_out_0,data_out_1,data_out_2,valid_out_0,
valid_out_1,valid_out_2,error,busy,clock,reset,
read_enb_0,read_enb_1,read_enb_2,pkt_valid,data_in);

always #5 clock=~clock;

task rst;
begin
    @(negedge clock) reset=1'b0;
    @(negedge clock) reset=1'b1;
end
endtask

task Initial;
begin
    {clock,read_enb_0,read_enb_1,read_enb_2,pkt_valid,data_in}=0;
    reset=1'b1;
end
endtask

task pkt_gen_14;
reg [7:0]payload_data,parity,header;
reg [5:0]payload_len;
reg [1:0]addr;
integer i;

begin
    wait(~busy)
    @(negedge clock) payload_len=6'd14;
    addr=2'b00;
    header={payload_len,addr};
    parity=0;
    data_in=header;
    pkt_valid=1'b1;
    parity=header;
    @(negedge clock);
    wait(~busy)
    for(i=0;i<payload_len;i=i+1) begin
        @(negedge clock);
        wait(~busy)
        payload_data={$random}%256;
        data_in=payload_data;
        parity=parity^data_in;
    end    
    @(negedge clock);
    wait(~busy)
    pkt_valid=0;
    data_in=parity;
end
endtask

initial begin
    $dumpfile("tb.vcd");
    $dumpvars(0,tb);
    Initial;
    rst;
    repeat(3) begin
        fork
        pkt_gen_14;
        begin
            wait(valid_out_0)
            @(negedge clock) read_enb_0=1'b1;
        end
        join
    end
    #300 $finish;
end
endmodule
