module constraints_micro(input bus_clk,input mac_clk,input rn,input data,
 output gray2_q,output event2_q);
 wire qs,qg1,qe,qe1;
 DFFR_X1 src(.CK(bus_clk),.RN(rn),.D(data),.Q(qs));
 DFFR_X1 gray1(.CK(mac_clk),.RN(rn),.D(qs),.Q(qg1));
 DFFR_X1 gray2(.CK(mac_clk),.RN(rn),.D(qg1),.Q(gray2_q));
 DFFR_X1 event_src(.CK(bus_clk),.RN(rn),.D(data),.Q(qe));
 DFFR_X1 event1(.CK(mac_clk),.RN(rn),.D(qe),.Q(qe1));
 DFFR_X1 event2(.CK(mac_clk),.RN(rn),.D(qe1),.Q(event2_q));
endmodule
