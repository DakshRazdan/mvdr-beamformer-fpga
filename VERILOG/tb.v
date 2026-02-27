module tb;
reg [3:0] A,B;
reg C0;
wire [3:0] S;
wire C4;
_4_bit_adder tba(A,B,C0,S,C4);
initial begin
A=4'b1010;B=4'b0010;C0=1'b0;
#10 
A=4'b0001;B=4'b0011;C0=1'b0;
#10 
A=4'b0010;B=4'b0110;C0=1'b0;
#10 
A=4'b0101;B=4'b0100;C0=1'b0;
#10 
$finish();
end
endmodule
