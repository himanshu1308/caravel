// SPDX-FileCopyrightText: 2020 Efabless Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// SPDX-License-Identifier: Apache-2.0

`default_nettype none
/*
 *-------------------------------------------------------------
 *
 * user_proj_example
 *
 * This is an example of a (trivially simple) user project,
 * showing how the user project can connect to the logic
 * analyzer, the wishbone bus, and the I/O pads.
 *
 * This project generates an integer count, which is output
 * on the user area GPIO pads (digital output only).  The
 * wishbone connection allows the project to be controlled
 * (start and stop) from the management SoC program.
 *
 * See the testbenches in directory "mprj_counter" for the
 * example programs that drive this user project.  The three
 * testbenches are "io_ports", "la_test1", and "la_test2".
 *
 *-------------------------------------------------------------
 */

module user_proj_example #(
    parameter BITS = 32
)(
`ifdef USE_POWER_PINS
    inout vccd1,	// User area 1 1.8V supply
    inout vssd1,	// User area 1 digital ground
`endif

    // Wishbone Slave ports (WB MI A)
    input wb_clk_i,
    input wb_rst_i,
    input wbs_stb_i,
    input wbs_cyc_i,
    input wbs_we_i,
    input [3:0] wbs_sel_i,
    input [31:0] wbs_dat_i,
    input [31:0] wbs_adr_i,
    output wbs_ack_o,
    output [31:0] wbs_dat_o,

    // Logic Analyzer Signals
    input  [127:0] la_data_in,
    output [127:0] la_data_out,
    input  [127:0] la_oenb,

    // IOs
    input  [`MPRJ_IO_PADS-1:0] io_in,
    output [`MPRJ_IO_PADS-1:0] io_out,
    output [`MPRJ_IO_PADS-1:0] io_oeb,

    // IRQ
    output [2:0] irq
);
    wire clk;
    wire rst;

    wire [`MPRJ_IO_PADS-1:0] io_in;
    wire [`MPRJ_IO_PADS-1:0] io_out;
    wire [`MPRJ_IO_PADS-1:0] io_oeb;

    

   
    // IO
    assign clk = wb_clk_i;
    assign rst = wb_rst_i;
    assign i_d = io_in[37];
    assign d_d = io_in[36];
    assign io_out[35] = pwm_out;
    assign io_oeb = 0;

    // IRQ
    assign irq = 3'b000;	// Unused
    wire i_d;
    wire d_d;
    wire pwm_out;
   
   iiitb_pwm_gen Design(.clk(clk),.reset(rst),.increase_duty(i_d),.decrease_duty(d_d),.PWM_OUT(pwm_out));
    
    


endmodule

module iiitb_pwm_gen
 (
 clk, // 100MHz clock input 
 increase_duty, // input to increase 10% duty cycle 
 decrease_duty, // input to decrease 10% duty cycle
 reset, 
 PWM_OUT // 10MHz PWM output signal 
    );
 
 input clk,reset;
 input increase_duty;
 input decrease_duty;
 output PWM_OUT ;
 wire slow_clk_enable; // slow clock enable signal for debouncing FFs(40MHz)
 reg[27:0] counter_debounce;// counter for creating slow clock enable signals 
 reg tmp1,tmp2;// temporary flip-flop signals for debouncing the increasing button
 reg tmp3,tmp4;// temporary flip-flop signals for debouncing the decreasing button
 wire duty_inc, duty_dec;
 reg[3:0] counter_PWM;
 reg[3:0] DUTY_CYCLE; 
  // Debouncing 2 buttons for inc/dec duty cycle 
  // Firstly generate slow clock enable for debouncing flip-flop 
  
 always @(posedge clk or posedge reset)
 begin
 if(reset) begin
 counter_debounce<=28'd0;
  counter_PWM<=4'd0;// counter for creating 10Mhz PWM signal
  DUTY_CYCLE<=4'd5;// initial duty cycle is 50%
  tmp1 <= 0;
  tmp2 <= 0;
  tmp3<=0;
  tmp4<=0;
  end
 else begin
 	counter_debounce <= counter_debounce>=28'd1 ? 28'd0 : counter_debounce + 28'd1;
 	if(duty_inc==1 && DUTY_CYCLE <= 9) begin
        DUTY_CYCLE <= DUTY_CYCLE + 4'd1;// increase duty cycle by 10%
        end
        else if(duty_dec==1 && DUTY_CYCLE>=1) begin
        //else begin
        DUTY_CYCLE <= DUTY_CYCLE - 4'd1;
        end//decrease duty cycle by 10%
        counter_PWM <= counter_PWM + 4'd1;
        if(counter_PWM>=9) begin
        counter_PWM <= 0;
        end
        if(slow_clk_enable==1) begin// slow clock enable signal 
  	tmp1 <= increase_duty;
  	tmp2 <= tmp1;
  	tmp3 <= decrease_duty;
  	tmp4 <= tmp3;
  	end
    end
 end
 
 assign slow_clk_enable = counter_debounce == 1 ?1:0;
  
 assign duty_inc =  tmp1 & (~ tmp2) & slow_clk_enable;
  
 assign duty_dec =  tmp3 & (~ tmp4) & slow_clk_enable;
 
 assign PWM_OUT = counter_PWM < DUTY_CYCLE ? 1:0;
 
endmodule
`default_nettype wire
