//Helper Modules
module HA(x,y,S,C);
    input x,y;
    output S,C;

    xor x1 (S,x,y);
    and a1 (C,x,y);
endmodule

module FA(x,y,z,S,C);
    input x,y,z;
    output S,C;
    wire s1,c1,c2;
    
    HA  ha1(x,y,s1,c1),
        ha2(s1,z,S,c2);
    or o1 (C,c1,c2);
endmodule

//multiplexors
module Mux_2x1 (i0,i1,s,o);
    input i0,i1,s;
    output o;
    wire nS;

    not n1(nS,s);
    
    and a1 (t1,i0,nS),
        a2 (t2,i1,s);
        
    or  (o,t1,t2);
endmodule

module Quad_2x1 (i0,i1,S,O); // Same module as above but with
    input [1:0] i0,i1;
    input S;
    output [1:0] O;

    Mux_2x1 m1(i0[0],i1[0],S,O[0]),
            m2(i0[1],i1[1],S,O[1]);
endmodule

module Mux_4x1 (i0,i1,i2,i3,s,o);
    input i0,i1,i2,i3;
    input [1:0] s;
    output o;
    wire t1,t2;
    
    Mux_2x1  m0(i0,i1,s[0],t1),
             m1(i2,i3,s[0],t2),
             m2(t1,t2,s[1],o);
endmodule 

module Mux_16(i0,i1,s,O);
    input [15:0] i0,i1;
    input s;
    output [15:0] O;
    
    Mux_2x1 m1(i0[0],i1[0],s,O[0]),
            m2(i0[1],i1[1],s,O[1]),
            m3(i0[2],i1[2],s,O[2]),
            m4(i0[3],i1[3],s,O[3]),
            m5(i0[4],i1[4],s,O[4]),
            m6(i0[5],i1[5],s,O[5]),
            m7(i0[6],i1[6],s,O[6]),
            m8(i0[7],i1[7],s,O[7]),
            m9(i0[8],i1[8],s,O[8]),
            m10(i0[9],i1[9],s,O[9]),
            m11(i0[10],i1[10],s,O[10]),
            m12(i0[11],i1[11],s,O[11]),
            m13(i0[12],i1[12],s,O[12]),
            m14(i0[13],i1[13],s,O[13]),
            m15(i0[14],i1[14],s,O[14]),
            m16(i0[15],i1[15],s,O[15]);
endmodule
// End of Helper Files

module reg_file (RR1,RR2,WR,WD,RegWrite,RD1,RD2,clock);
  input [1:0] RR1,RR2,WR;
  input [15:0] WD;
  input RegWrite,clock;
  output [15:0] RD1,RD2;
  reg [15:0] Regs[0:3];
  assign RD1 = Regs[RR1];
  assign RD2 = Regs[RR2];
  initial Regs[0] = 0;
  always @(negedge clock)
    if (RegWrite==1 & WR!=0) 
	Regs[WR] <= WD;
endmodule

module alu (op,a,b,result,z);
   input [15:0] a,b;
   input [3:0] op;
   output [15:0] result;
   output z;
   wire [15:0] cout;
   aluLow   alu0 (a[0],b[0],op[3],op[2],op[1:0],set, op[2],cout[0],result[0]);
   aluLow   alu1 (a[1],b[1],op[3],op[2],op[1:0],1'b0,cout[0],   cout[1],result[1]);
   aluLow   alu2 (a[2],b[2],op[3],op[2],op[1:0],1'b0,cout[1],   cout[2],result[2]);
   aluLow   alu3 (a[3],b[3],op[3],op[2],op[1:0],1'b0,cout[2],   cout[3],result[3]);
   aluLow   alu4 (a[4],b[4],op[3],op[2],op[1:0],1'b0,cout[3],   cout[4],result[4]);
   aluLow   alu5 (a[5],b[5],op[3],op[2],op[1:0],1'b0,cout[4],   cout[5],result[5]);
   aluLow   alu6 (a[6],b[6],op[3],op[2],op[1:0],1'b0,cout[5],   cout[6],result[6]);
   aluLow   alu7 (a[7],b[7],op[3],op[2],op[1:0],1'b0,cout[6],   cout[7],result[7]);
   aluLow   alu8 (a[8],b[8],op[3],op[2],op[1:0],1'b0,cout[7],   cout[8],result[8]);
   aluLow   alu9 (a[9],b[9],op[3],op[2],op[1:0],1'b0,cout[8],   cout[9],result[9]);
   aluLow   alu10 (a[10],b[10],op[3],op[2],op[1:0],1'b0,cout[9],   cout[10],result[10]);
   aluLow   alu11 (a[11],b[11],op[3],op[2],op[1:0],1'b0,cout[10],   cout[11],result[11]);
   aluLow   alu12 (a[12],b[12],op[3],op[2],op[1:0],1'b0,cout[11],   cout[12],result[12]);
   aluLow   alu13 (a[13],b[13],op[3],op[2],op[1:0],1'b0,cout[12],   cout[13],result[13]);
   aluLow   alu14 (a[14],b[14],op[3],op[2],op[1:0],1'b0,cout[13],   cout[14],result[14]);
   aluHIGH alu15 (a[15],b[15],op[3],op[2],op[1:0],1'b0,cout[14],   cout[15],result[15],set);
   
   or (o1, result [15:0]);
   not (z, o1);
endmodule

module aluLow (a,b,ain,bin,op,less,cin,cout,result); // Alu that calculates all the lower bits
    input a,b,less,cin,ain,bin;
    input [1:0] op;
    output cout,result;
    wire nA,nB,sA,sB;
    not n1(nA, a),
        n2(nB, b);
    
    Mux_2x1 m1(a,nA,ain,sA),
           m2(b,nB,bin,sB);
    and (a1, sA, sB);
    or (o1, sA, sB);
    FA fa1(sA,sB,cin,S,cout);
    Mux_4x1 m4(a1,o1,S,less,op,result);// Outputs result based on values
endmodule

module aluHIGH (a,b,ain,bin,op,less,cin,cout,result,set); // Calculates the Highest Bit of the Alu
    input a,b,less,cin,ain,bin;
    input [1:0] op;
    output cout,result,set;
    wire nA,nB,sA,sB;
    not n1(nA, a),
        n2(nB, b);
    
    Mux_2x1 m1(a,nA,ain,sA),
           m2(b,nB,bin,sB);
    and (a1, sA, sB);
    or (o1, sA, sB);
    FA fa1(sA,sB,cin,set,cout);
    Mux_4x1 m4(a1,o1,set,less,op,result);// Outputs result based on values
  
    xor (x,cin,cout);
    and (cout,x,op[1]); 
endmodule

module MainControl (Op,Control); 
  input [3:0] Op;
  output reg [10:0] Control;
// Control bits: RegDst,ALUSrc,RegWrite _ MemtoReg,MemWrite _Branch,Branc, ALUCtl[3:0]
  always @(Op) case (Op)
    4'b0000: Control <= 11'b101_00_00_0010; // add
    4'b0001: Control <= 11'b101_00_00_0110; // sub
    4'b0010: Control <= 11'b101_00_00_0000; // and
    4'b0011: Control <= 11'b101_00_00_0001; // or
    4'b0100: Control <= 11'b101_00_00_1100; // nor
    4'b0101: Control <= 11'b101_00_00_1101; // nand
    4'b0110: Control <= 11'b101_00_00_0111; // slt
    4'b0111: Control <= 11'b011_00_00_0010; // addi
    // Progress Report 2 Added:
    4'b1000: Control <= 11'b011_10_00_0010; // lw
    4'b1001: Control <= 11'b010_01_00_0010; // sw
    4'b1010: Control <= 11'b000_00_10_0110; // beq
    4'b1011: Control <= 11'b000_00_11_0110; // bne
  endcase
endmodule

module BranchControl(PCplus4,Targ,Branch,BranchType,Zero,NextPC);
    input [15:0] PCplus4,Targ;
    input Branch,BranchType,Zero;
    output [15:0] NextPC;
    
    not n1(nBT,BranchType), 
        n2(nZ,Zero);
        
    and A1(a1,nBT,Zero), // ~BT && Zero
        A2(a2,BranchType,nZ), // BT && ~Zero
        A3(sBr,o1,Branch); // (Branch && o1)
        
    or O1(o1,a1,a2); //(a1 || a2)
  
    Mux_16 m16_0(PCplus4,Targ,sBr,NextPC); //assign NextPC = (EXMEM_Branch && EXMEM_Zero) ? EXMEM_Target: PCplus4;
endmodule

module CPU (clock,PC,IFID_IR,IDEX_IR,EXMEM_IR,MEMWB_IR,WD);
  input clock;
  output [15:0] PC,IFID_IR,IDEX_IR,EXMEM_IR,MEMWB_IR,WD;

// Test Program
  initial begin 
    // Program: swap memory cells (if needed) and compute absolute value |5-7| = 2;

    IMemory[0] = 16'h8100;  // lw $t1 0($0)              1000 0001 0000 0000 
    IMemory[1] = 16'h8204;  // lw $t2, 4($0)             1000 0010 0000 0100
    IMemory[2] = 16'h0000;  // nop
    IMemory[3] = 16'h0000;  // nop
    IMemory[4] = 16'h0000;  // nop
    
    IMemory[5] = 16'h66C0;  // slt $t3, $t1, $t2         0110 0110 1100 0000 
    IMemory[6] = 16'h0000;  // nop
    IMemory[7] = 16'h0000;  // nop
    IMemory[8] = 16'h0000;  // nop
    
    IMemory[9] = 16'hAC05;  // beq $t3, $0, IMemory[15]   1010 1100 0000 0110
    // BEQ == A || BNE = B
    IMemory[10] = 16'h0000;  // nop
    IMemory[11] = 16'h0000;  // nop
    IMemory[12] = 16'h0000;  // nop
    
    IMemory[13] = 16'h9104;  // sw $t1, 4($0)             1001 0001 0000 0100
    IMemory[14] = 16'h9200;  // sw $t2, 0($0)             1001 0010 0000 0000
    IMemory[15] = 16'h0000;  // nop
    IMemory[16] = 16'h0000;  // nop
    IMemory[17] = 16'h0000;  // nop
    
    IMemory[18] = 16'h8100;  // lw $t1, 0($0)             1000 0001 0000 0000 
    IMemory[19] = 16'h8204; // lw $t2, 4($0)              1000 0010 0000 0100
    IMemory[20] = 16'h0000;  // nop
    IMemory[21] = 16'h0000;  // nop
    IMemory[22] = 16'h0000;  // nop
    
    IMemory[23] = 16'h4A80; // nor  $t2, $t2, $t2         0100 1010 1000 0000
    // (sub $3, $1, $2 in two's complement)
    IMemory[24] = 16'h0000;  // nop
    IMemory[25] = 16'h0000;  // nop
    IMemory[26] = 16'h0000;  // nop
    
    IMemory[27] = 16'h7A01; // addi $t2,$t2,1            0111 1010 0000 0001 
    IMemory[28] = 16'h0000;  // nop
    IMemory[29] = 16'h0000;  // nop
    IMemory[30] = 16'h0000;  // nop
    IMemory[31] = 16'h06C0; // add $t3, $t1, $t2         0000 0110 1100 0000

    // Data
    DMemory[0] = 5;
    DMemory[1] = 7;
  end
  
  //Pipeline
  // == IF STAGE ==
    wire [15:0] PCplus4,NextPC;
    reg[15:0] PC,IMemory[0:1023];
  //---------------------------
    reg[15:0] IFID_IR,IFID_PCplus4;
  //---------------------------
    alu fetch (4'b0010,PC,4,PCplus4,Unused1);
    BranchControl br(PCplus4,EXMEM_Target,EXMEM_Branch,EXMEM_BranchType,EXMEM_Zero,NextPC);
    /*
    not n1(nBT,EXMEM_BranchType), 
        n2(nZ,EXMEM_Zero);
        
    and A1(a1,nBT,Zero), // ~BT && Zero
        A2(a2,EXMEM_BranchType,nZ), // BT && ~Zero
        A3(sBr,o1,EXMEM_Branch); // (Branch && o1)
        
    or O1(o1,a1,a2); //(a1 || a2)
  
    Mux_16 m16_0(PCplus4,EXMEM_Target,sBr,NextPC); //assign NextPC = (EXMEM_Branch && EXMEM_Zero) ? EXMEM_Target: PCplus4;
    */

  // == ID STAGE ==
    wire [10:0] Control;
    reg IDEX_RegWrite, IDEX_MemtoReg,IDEX_Branch,IDEX_BranchType,IDEX_MemWrite,IDEX_ALUSrc,IDEX_RegDst;
    reg [3:0] IDEX_ALUOp;
    wire [15:0] RD1,RD2,SignExtend,WD;
    reg [15:0] IDEX_PCplus4,IDEX_RD1,IDEX_RD2,IDEX_SignExt,IDEXE_IR;
    reg [15:0] IDEX_IR; // Monitor pipeline
    reg [1:0] IDEX_rt,IDEX_rd; // Registers
    
    reg MEMWB_RegWrite; //  MEM STAGE
    reg [1:0] MEMWB_rd; //  MEM STAGE
  
    reg_file rf(IFID_IR[11:10],IFID_IR[9:8],MEMWB_rd,WD,MEMWB_RegWrite,RD1,RD2,clock);
    MainControl MainCtr (IFID_IR[15:12],Control);
    // Control bits: RegDst,ALUSrc,RegWrite _ MemtoReg,MemWrite _Branch,BranchType_ ALUCtl[3:0]
    assign SignExtend = {{16{IFID_IR[7]}},IFID_IR[7:0]}; // sign extension unit
  
  // == EXE Stage ==
    reg EXMEM_RegWrite, EXMEM_MemtoReg, EXMEM_Branch, EXMEM_MemWrite,EXMEM_BranchType;
    reg EXMEM_Zero;
    reg [15:0] EXMEM_Target, EXMEM_ALUOut, EXMEM_RD2;
    reg [15:0] EXMEM_IR;// Monitor Pipeline
    reg[1:0] EXMEM_rd;
    
    wire [15:0] Target;
    wire [15:0] B,ALUOut;
    wire [3:0] ALUctl;
    wire [1:0] WR;
    
    alu branch (4'b0010,IDEX_SignExt<<2,IDEX_PCplus4,Target,Unused2);
    alu ex (IDEX_ALUOp, IDEX_RD1, B, ALUOut, Zero);
    Mux_16 m16_1(IDEX_RD2,IDEX_SignExt,IDEX_ALUSrc,B);// 16 Bit Multiplexor for Immediate or R-Type
    Quad_2x1 m2x1_1(IDEX_rt,IDEX_rd,IDEX_RegDst,WR);
    //assign WD = ALUOut;
    
  // MEM
    reg MEMWB_MemtoReg;
    reg [15:0] DMemory[0:1023], MEMWB_MemOut, MEMWB_ALUOut;
    reg [15:0] MEMWB_IR;// Monitor Pipeline
    wire [15:0] MemOut;
    
    assign MemOut = DMemory[EXMEM_ALUOut>>2];
    always @(negedge clock) if (EXMEM_MemWrite) DMemory[EXMEM_ALUOut>>2] <= EXMEM_RD2;
  // WB
    Mux_16 m16_2(MEMWB_ALUOut,MEMWB_MemOut,MEMWB_MemtoReg,WD);
  
  initial begin
    PC = 0;
// Initialize pipeline registers
    IDEX_RegWrite=0;IDEX_MemtoReg=0;IDEX_Branch=0;IDEX_MemWrite=0;IDEX_ALUSrc=0;IDEX_RegDst=0;IDEX_ALUOp=0;
    IFID_IR=0;
    EXMEM_RegWrite=0;EXMEM_MemtoReg=0;EXMEM_Branch=0;EXMEM_MemWrite=0;
    EXMEM_Target=0;
    MEMWB_RegWrite=0;MEMWB_MemtoReg=0;
   end

// Running the pipeline
   always @(negedge clock) begin 
// IF
    PC <= NextPC;
    IFID_PCplus4 <= PCplus4;
    IFID_IR <= IMemory[PC>>2];
// ID
    IDEX_IR <= IFID_IR; // For monitoring the pipeline
    {IDEX_RegDst,IDEX_ALUSrc,IDEX_RegWrite,IDEX_MemtoReg,IDEX_MemWrite,IDEX_Branch,IDEX_BranchType,IDEX_ALUOp} <= Control;   
    IDEX_PCplus4 <= IFID_PCplus4;
    IDEX_RD1 <= RD1; 
    IDEX_RD2 <= RD2;
    IDEX_SignExt <= SignExtend;
    IDEX_rt <= IFID_IR[9:8];
    IDEX_rd <= IFID_IR[7:6];
// EXE
    EXMEM_IR <= IDEX_IR; // For monitoring the pipeline
    EXMEM_RegWrite <= IDEX_RegWrite;
    EXMEM_MemtoReg <= IDEX_MemtoReg;
    EXMEM_Branch   <= IDEX_Branch;
    EXMEM_BranchType <= IDEX_BranchType;
    EXMEM_MemWrite <= IDEX_MemWrite;
    EXMEM_Target <= Target;
    EXMEM_Zero <= Zero;
    EXMEM_ALUOut <= ALUOut;
    EXMEM_RD2 <= IDEX_RD2;
    EXMEM_rd <= WR;
// MEM
    MEMWB_IR <= EXMEM_IR; // For monitoring the pipeline
    MEMWB_RegWrite <= EXMEM_RegWrite;
    MEMWB_MemtoReg <= EXMEM_MemtoReg;
    MEMWB_MemOut <= MemOut;
    MEMWB_ALUOut <= EXMEM_ALUOut;
    MEMWB_rd <= EXMEM_rd;
// WB
// Register write happens on neg edge of the clock (if MEMWB_RegWrite is asserted)
  end
endmodule

// Test module
module test ();
  reg clock;
  wire signed [15:0] PC,IFID_IR,IDEX_IR,EXMEM_IR,MEMWB_IR,WD;
  CPU test_cpu(clock,PC,IFID_IR,IDEX_IR,EXMEM_IR,MEMWB_IR,WD);
  always #1 clock = ~clock;
  initial begin
   $display ("PC   IFID_IR  IDEX_IR  EXMEM_IR MEMWB_IR  WD");
    $monitor ("%3d  %h %h %h %h %2d",PC,IFID_IR,IDEX_IR,EXMEM_IR,MEMWB_IR,WD);
    clock = 1;
    #69 $finish;
  end
endmodule