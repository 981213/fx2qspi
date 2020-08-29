/*
 * Cypress FX2 Slave FIFO to half-duplex QSPI bridge
 * FX_FULL is FLAGA configured to indicate EP6_IN full
 * and FX_EMPTY is FLAGB configured to indicate EP2_OUT empty
 * All control pins on FX2 are active low
 * FIFOADR0 should be connected to ground
 */
module fx2qspi(
	input FX_IFCLK,
	output FX_FIFOADR1,
	input FX_FULL,
	input FX_EMPTY,
	output reg FX_SLOE,
	output reg FX_SLRD,
	output reg FX_SLWR,
	output reg FX_PKTEND,
	inout [7:0]FX_DATA,
	output SPI_CS,
	output SPI_CLK,
	inout SPI_MOSI, // D/QPI IO0
	inout SPI_MISO, // D/QPI IO1
	inout SPI_WP, // QPI IO2
	inout SPI_HOLD, //QPI IO3
	output LED_R,
	output LED_G,
	output LED_B
);

// These are the first 2 bytes from FIFO for controller setup
reg op_cs;
assign SPI_CS = ~op_cs;
reg op_dir; // 0: out 1: in
localparam DIR_OUT = 1'b0, DIR_IN = 1'b1;
reg [1:0]op_mode; // 10: QPI 01: DPI 00: SPI
localparam MODE_QPI = 2'b10, MODE_DPI = 2'b01, MODE_SPI = 2'b00;
reg [11:0]op_len;

// Data buffer for FX2 FIFO
reg [7:0]fifo_data_buf;

// Values on QSPI IO3/IO2/IO1/IO0 if they are in output mode
reg [3:0]qpi_out_val;

// Combinatioal logic
// For QSPI pins
// MOSI: SPI: out D/QPI: op_dir
assign SPI_MOSI = (op_mode == MODE_SPI || op_dir == DIR_OUT) ?
	qpi_out_val[0] : 1'bz;
// MISO: SPI: in D/QPI: op_dir
assign SPI_MISO = (op_mode == MODE_SPI || op_dir == DIR_IN) ?
	1'bz : qpi_out_val[1];
// WP: SPI/DPI: z pull up QPI: op_dir
assign SPI_WP = (op_mode == MODE_QPI && op_dir == DIR_OUT) ?
	qpi_out_val[2] : 1'bz;
// HOLD: SPI_DPI: 1 QPI: op_dir
assign SPI_HOLD = (op_mode == MODE_QPI && op_dir == DIR_OUT) ?
	qpi_out_val[3] : 1'bz;

// QSPI data sampled on every rising edge
reg [7:0]qspi_in_val;
// Byte to be sent on QSPI
reg [7:0]qspi_out_data;

// QSPI sampling logic
always @(posedge SPI_CLK) begin
	case (op_mode)
		MODE_QPI: qspi_in_val <= {qspi_in_val[3:0], SPI_HOLD, SPI_WP, SPI_MISO, SPI_MOSI};
		MODE_DPI: qspi_in_val <= {qspi_in_val[5:0], SPI_MISO, SPI_MOSI};
		MODE_SPI: qspi_in_val <= {qspi_in_val[6:0], SPI_MISO};
	endcase
end

/*
 * QSPI Tx FSM:
 * transition:
 * idle: go begin if triggered
 * begin: go b0 if trigger released
 * b0: go b1
 * b1: go idle if under QPI mode. Otherwise go b2
 * b2: go b3
 * b3: go idle if under DPI mode. Otherwise go b4
 * b4: go b5
 * b5: go b6
 * b6: go b7
 * b7: go idle
 */
wire qspi_op_clk;
reg qspi_trigger;
reg [3:0]qspi_state;
reg [3:0]qspi_next_state;
wire qspi_busy;
localparam qspi_s_idle = 4'b0000,
	qspi_s_begin = 4'b0001,
	qspi_s_b0 = 4'b0011,
	qspi_s_b1 = 4'b0010,
	qspi_s_b2 = 4'b0110,
	qspi_s_b3 = 4'b0100,
	qspi_s_b4 = 4'b1100,
	qspi_s_b5 = 4'b1101,
	qspi_s_b6 = 4'b1001,
	qspi_s_b7 = 4'b1000;

// QSPI is busy if SPI_CLK is low or it's not in idle state
assign qspi_busy = (|qspi_state) | (~SPI_CLK);

// state transition on negative edge
always @(negedge qspi_op_clk, posedge qspi_trigger) begin
	if (qspi_trigger) begin
		qspi_state <= qspi_s_begin;
	end else begin
		qspi_state <= qspi_next_state;
	end
end

always @(*) begin
	case (qspi_state)
		qspi_s_begin: qspi_next_state = qspi_s_b0;
		qspi_s_b0: qspi_next_state = qspi_s_b1;
		qspi_s_b1: qspi_next_state = op_mode == MODE_QPI ? qspi_s_idle : qspi_s_b2;
		qspi_s_b2: qspi_next_state = qspi_s_b3;
		qspi_s_b3: qspi_next_state = op_mode == MODE_DPI ? qspi_s_idle : qspi_s_b4;
		qspi_s_b4: qspi_next_state = qspi_s_b5;
		qspi_s_b5: qspi_next_state = qspi_s_b6;
		qspi_s_b6: qspi_next_state = qspi_s_b7;
		default: qspi_next_state = qspi_s_idle;
	endcase
end

/*
 * clock gate:
 * we are implementing SPI mode 3 so clk should be enabled immediately
 * when clk_en goes 0 but should be gated only on posedge when clk_en = 1
 */
reg qspi_clk_en_n;
reg qspi_clk_gate; // 1: clk_output = 1 0: clk_output = qspi_op_clk
always @(posedge qspi_op_clk, negedge qspi_clk_en_n) begin
	if(qspi_clk_en_n == 0) begin
		qspi_clk_gate <= 0;
	end else begin
		qspi_clk_gate <= qspi_clk_en_n;
	end
end

assign SPI_CLK = qspi_op_clk | qspi_clk_gate;

/* clock divider 45MHz/8 for my logic analyzer :P */
reg [3:0]clkdiv_counter;
always @(negedge FX_IFCLK) begin
	clkdiv_counter <= clkdiv_counter + 1;
end
//assign qspi_op_clk = clkdiv_counter[3];
assign qspi_op_clk = clkdiv_counter[0];

/*
 * QSPI FSM Output:
 * idle/begin: 0000 clk_en_n = 1
 * b0: SPI: [0] = b7 DPI: [1:0] = b7b6 QPI: [3:0] = b7b6b5b4
 * b1: SPI: [0] = b6 DPI: [1:0] = b5b4 QPI: [3:0] = b3b2b1b0 QPI: clk_en_n = 1
 * b2: SPI: [0] = b5 DPI: [1:0] = b3b2
 * b3: SPI: [0] = b4 DPI: [1:0] = b1b0 DPI: clk_en_n = 1
 * b4: SPI: [0] = b3
 * b5: SPI: [0] = b2
 * b6: SPI: [0] = b1
 * b7: SPI: [0] = b0 clk_en_n = 1
 */
 
always @(*) begin
	qpi_out_val = 4'b0000;
	qspi_clk_en_n = 0;
	case (qspi_state)
		qspi_s_idle: qspi_clk_en_n = 1;
		qspi_s_begin: qspi_clk_en_n = 1;
		qspi_s_b0: begin
			case(op_mode)
				MODE_QPI: qpi_out_val = qspi_out_data[7:4];
				MODE_DPI: qpi_out_val[1:0] = qspi_out_data[7:6];
				MODE_SPI: qpi_out_val[0] = qspi_out_data[7];
			endcase
		end
		qspi_s_b1: begin
			case(op_mode)
				MODE_QPI: qpi_out_val = qspi_out_data[3:0];
				MODE_DPI: qpi_out_val[1:0] = qspi_out_data[5:4];
				MODE_SPI: qpi_out_val[0] = qspi_out_data[6];
			endcase
			if(op_mode == MODE_QPI) begin
				qspi_clk_en_n = 1;
			end
		end
		qspi_s_b2: begin
			case(op_mode)
				MODE_DPI: qpi_out_val[1:0] = qspi_out_data[3:2];
				MODE_SPI: qpi_out_val[0] = qspi_out_data[5];
			endcase
		end
		qspi_s_b3: begin
			case(op_mode)
				MODE_DPI: qpi_out_val[1:0] = qspi_out_data[1:0];
				MODE_SPI: qpi_out_val[0] = qspi_out_data[4];
			endcase
			if(op_mode == MODE_DPI) begin
				qspi_clk_en_n = 1;
			end
		end
		qspi_s_b4: qpi_out_val[0] = qspi_out_data[3];
		qspi_s_b5: qpi_out_val[0] = qspi_out_data[2];
		qspi_s_b6: qpi_out_val[0] = qspi_out_data[1];
		qspi_s_b7: begin
			qpi_out_val[0] = qspi_out_data[0];
			qspi_clk_en_n = 1;
		end
		default: qspi_clk_en_n = 1;
	endcase
end

// FX2 FIFO data registers
reg [7:0]fx2_read_data;
reg [7:0]fx2_write_data;
reg fx2_fpga_oe;

// Combinatioal logic for FX2
// output control. SLOE is active low and fx2_fpga_oe is active high.
// Let's check both flags to make sure pin conflict doesn't happen.
assign FX_DATA = fx2_fpga_oe ? fx2_write_data : 8'bz;
// Setup FIFOADR using SLOE level: 10:EP6IN 00:EP2OUT
assign FX_FIFOADR1 = FX_SLOE ? 1'b1 : 1'b0;

/**
 * FX2 reading FSM
 * States:
 * 0. idle: OE=1 RD=1 Transition to state 1 if triggered.
 * 1. start: OE=1 RD=1 Transition to state 2 if FX_EMPTY=1.
 * 2. data: OE=0(FIFOADR=00) RD=1 Transition to state 3 and sample data.
 * 3. incptr: OE=0 RD=0 to increase FIFOPTR and return to state 0.
 * State transitions happen on falling edge of IFCLK.
 * A start is triggered by posedge of fx2_read_trigger and should be
 * reset by driving logic.
 */
reg fx2_read_trigger;
wire fx2_read_busy;
reg [1:0]fx2_read_state;
reg [1:0]fx2_read_next_state;
localparam fx2_read_s0 = 2'b00,
	fx2_read_s1 = 2'b01,
	fx2_read_s2 = 2'b11,
	fx2_read_s3 = 2'b10;

// s0 is idle state and the rest are busy states
assign fx2_read_busy = |fx2_read_state;

// state transition
always @(negedge FX_IFCLK, posedge fx2_read_trigger) begin
	if (fx2_read_trigger) begin
		fx2_read_state <= fx2_read_s1;
	end else begin
		fx2_read_state <= fx2_read_next_state;
	end
end

// data sampling
always @(negedge FX_IFCLK) begin
	if (fx2_read_next_state == fx2_read_s3) begin
		fx2_read_data <= FX_DATA;
	end
end

// next state
always @(FX_EMPTY, fx2_read_state) begin
	case (fx2_read_state)
		fx2_read_s0: fx2_read_next_state = fx2_read_s0;
		fx2_read_s1: fx2_read_next_state = FX_EMPTY ? fx2_read_s2 : fx2_read_s1;
		fx2_read_s2: fx2_read_next_state = fx2_read_s3;
		fx2_read_s3: fx2_read_next_state = fx2_read_s0;
	endcase
end

// output
always @(FX_EMPTY, fx2_read_state) begin
	case (fx2_read_state)
		fx2_read_s0: {FX_SLOE, FX_SLRD} = 2'b11;
		fx2_read_s1: {FX_SLOE, FX_SLRD} = 2'b11;
		fx2_read_s2: {FX_SLOE, FX_SLRD} = 2'b01;
		fx2_read_s3: {FX_SLOE, FX_SLRD} = 2'b00;
	endcase
end

/*
 * FX2 writing FSM
 * FIFOADR is driven by reading block. we need to take care of
 * fx2_fpga_oe, FX_SLWR and FX_PKTEND.
 * 0: idle: fx2_fpga_oe=0 FX_SLWR=FX_PKTEND=1
 * 1: start: fx2_fpga_oe=0 FX_SLWR=FX_PKTEND=1 if FX_FULL=1 transition to 2
 * 2: sample: fx2_fpga_oe=1 FX_SLWR=0 FX_PKTEND=1
 *            if fx2_write_pktend=1 transition to 3
 *            otherwise to 0
 * 3: wait: FX2 needs one more clock before committing ZLP 
 * 4: termpkt: fx2_fpga_oe=0 FX_SLWR=1 FX_PKTEND=0 transition to 0
 *    There may be a glitch on 4->0 (4->3->0) but it doesn't matter
 *    here.
 */
 reg fx2_write_pktend;
 reg fx2_write_trigger;
 reg [2:0]fx2_write_state;
 reg [2:0]fx2_write_next_state;
 wire fx2_write_busy;
 localparam fx2_write_s0 = 3'b000,
	fx2_write_s1 = 3'b001,
	fx2_write_s2 = 3'b011,
	fx2_write_s3 = 3'b010,
	fx2_write_s4 = 3'b110;
	
assign fx2_write_busy = |fx2_write_state;

always @(negedge FX_IFCLK, posedge fx2_write_trigger) begin
	if (fx2_write_trigger) begin
		fx2_write_state <= fx2_write_s1;
	end else begin
		fx2_write_state <= fx2_write_next_state;
	end
end

always @(fx2_write_state, FX_FULL, fx2_write_pktend) begin
	case (fx2_write_state)
		// fx2_write_s0: nothing
		fx2_write_s1: fx2_write_next_state = FX_FULL ? fx2_write_s2 : fx2_write_s1;
		fx2_write_s2: fx2_write_next_state = fx2_write_pktend ? fx2_write_s3 : fx2_write_s0;
		fx2_write_s3: fx2_write_next_state = fx2_write_s4;
		default: fx2_write_next_state = fx2_write_s0;
	endcase
end

always @(fx2_write_state, FX_FULL, fx2_write_pktend) begin
	case (fx2_write_state)
		fx2_write_s2: {fx2_fpga_oe, FX_SLWR, FX_PKTEND} = 3'b101;
		fx2_write_s4: {fx2_fpga_oe, FX_SLWR, FX_PKTEND} = 3'b010;
		default: {fx2_fpga_oe, FX_SLWR, FX_PKTEND} = 3'b011;
	endcase
end

/* main stuff */

/* first stage data: load CS/mode etc. and first 4b of data length */
reg [3:0]datalen_h4b;
reg sample_stage1;
always @(posedge sample_stage1) begin
	op_cs <= fx2_read_data[7];
	op_mode <= fx2_read_data[6:5];
	op_dir <= fx2_read_data[4];
	datalen_h4b <= fx2_read_data[3:0];
end
 /* second stage data and counter */
reg [11:0]datalen;
reg sample_stage2;
reg countdown;
always @(posedge countdown, posedge sample_stage2) begin
	if (sample_stage2) begin
		datalen <= {datalen_h4b, fx2_read_data};
	end else begin
		datalen <= datalen - 1;
	end
end

wire data_left;
assign data_left = |datalen;

reg data_store;
always @(posedge data_store) begin
	fx2_write_data <= qspi_in_val;
	qspi_out_data <= fx2_read_data;
end

/*
 * main FSM
 * 0: trigger a byte read
 * 1: wait for it
 * 2: sample stage 1 data
 * 3: trigger another byte read
 * 4: wait
 * 5: sample stage 2 data
 * 6: release stage 2 clock
 * 7: trigger op1 (read a byte for writing, or trigger spi for reading)
 * 8: wait op1
 * 9: save op1 result
 * 10: trigger op12
 * 11: wait op12
 * 12: save op1 result
 * 13: trigger op2 (trigger spi for writing or write the byte back to fx2 for reading)
 *    be careful about pktend processing
 * 14: wait op2 and hold pktend
 * I can't design a glitch-free state here so glitch removal for output is mandatory.
 */
localparam main_s0 = 4'b0000,
	main_s1 = 4'b0001,
	main_s2 = 4'b0011,
	main_s3 = 4'b0010,
	main_s4 = 4'b0110,
	main_s5 = 4'b0111,
	main_s6 = 4'b0101,
	main_s7 = 4'b0100,
	main_s8 = 4'b1100,
	main_s9 = 4'b1101,
	main_s10 = 4'b1111,
	main_s11 = 4'b1110,
	main_s12 = 4'b1010,
	main_s13 = 4'b1011,
	main_s14 = 4'b1001;

reg [3:0]main_state;
reg [3:0]main_next_state;

always @(negedge FX_IFCLK) begin
	main_state <= main_next_state;
end

/*
 * state transition:
 * 0: go 1
 * 1: go 2 if byte read
 * 2: go 3 if CS=0 otherwise go 0
 * 3: go 4
 * 4: go 5 if byte read
 * 5: go 6
 * 6: go 0 if no data left otherwise go 7
 * 7: go 8
 * 8: busy wait. go 9
 * 9: go 13 if no data left otherwise go 10
 * 10: go 11
 * 11: busy wait. go 12
 * 12: go 13 if no data left otherwise go 10
 * 13: go 14
 * 14: busy wait. go 0
 */
always @(*) begin
	case (main_state)
		main_s0: main_next_state = main_s1;
		main_s1: main_next_state = fx2_read_busy ? main_s1 : main_s2;
		main_s2: main_next_state = op_cs ? main_s3 : main_s0;
		main_s3: main_next_state = main_s4;
		main_s4: main_next_state = fx2_read_busy ? main_s4 : main_s5;
		main_s5: main_next_state = main_s6;
		main_s6: main_next_state = data_left ? main_s7 : main_s0;
		main_s7: main_next_state = main_s8;
		main_s8: main_next_state = (fx2_read_busy | qspi_busy) ? main_s8 : main_s9;
		main_s9: main_next_state = data_left ? main_s10 : main_s13;
		main_s10: main_next_state = main_s11;
		main_s11: main_next_state = (fx2_read_busy | qspi_busy | fx2_write_busy) ? main_s11 : main_s12;
		main_s12: main_next_state = data_left ? main_s10 : main_s13;
		main_s13: main_next_state = main_s14;
		main_s14: main_next_state = (fx2_write_busy | qspi_busy) ? main_s14 : main_s0;
		default: main_next_state = main_s0;
	endcase
end
/*
 * Driving pins:
 * qspi_trigger fx2_write_trigger fx2_write_pktend
 * fx2_read_trigger
 * sample_stage1 sample_stage2 countdown data_store
 * here's the glitch removal part
 */
reg n_qspi_trigger;
reg n_fx2_write_trigger;
reg n_fx2_write_pktend;
reg n_fx2_read_trigger;
reg n_sample_stage1;
reg n_sample_stage2;
reg n_countdown;
reg n_data_store;
always @(posedge FX_IFCLK) begin
	qspi_trigger <= n_qspi_trigger;
	fx2_write_trigger <= n_fx2_write_trigger;
	fx2_write_pktend <= n_fx2_write_pktend;
	fx2_read_trigger <= n_fx2_read_trigger;
	sample_stage1 <= n_sample_stage1;
	sample_stage2 <= n_sample_stage2;
	countdown <= n_countdown;
	data_store <= n_data_store;
end

/* output */
always @(*) begin
	/* default values for all pins */
	n_qspi_trigger = 1'b0;
	n_fx2_write_trigger = 1'b0;
	n_fx2_write_pktend = 1'b0;
	n_fx2_read_trigger = 1'b0;
	n_sample_stage1 = 1'b0;
	n_sample_stage2 = 1'b0;
	n_countdown = 1'b0;
	n_data_store = 1'b0;
	case (main_state)
		main_s0: n_fx2_read_trigger = 1'b1;
		// main_s1: nothing
		main_s2: n_sample_stage1 = 1'b1;
		main_s3: n_fx2_read_trigger = 1'b1;
		// main_s4: nothing
		main_s5: n_sample_stage2 = 1'b1;
		// main_s6: nothing
		main_s7: begin
			if (op_dir == DIR_OUT) begin
				n_fx2_read_trigger = 1'b1;
			end else begin
				n_qspi_trigger = 1'b1;
			end
		end
		main_s8: n_countdown = 1'b1;
		main_s9: n_data_store = 1'b1;
		main_s10: begin
			if (op_dir == DIR_OUT) begin
				n_fx2_read_trigger = 1'b1;
			end else begin
				n_fx2_write_trigger = 1'b1;
			end
			n_qspi_trigger = 1'b1;
		end
		main_s11: n_countdown = 1'b1;
		main_s12: n_data_store = 1'b1;
		main_s13: begin
			if (op_dir == DIR_OUT) begin
				n_qspi_trigger = 1'b1;
			end else begin
				n_fx2_write_trigger = 1'b1;
				if (!data_left) begin
					n_fx2_write_pktend = 1;
				end
			end
		end
		main_s14: n_fx2_write_pktend = fx2_write_pktend;
	endcase
end
assign {LED_R, LED_G, LED_B} = {fx2_read_busy , qspi_busy , fx2_write_busy};
endmodule
