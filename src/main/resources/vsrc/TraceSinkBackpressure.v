// Simulation-only trace sink that discards its input and asserts `ready`
// according to a programmable pattern. Used to exercise encoder backpressure
// (stall in lossless mode, Pause/Resume in lossy mode). The byte stream itself
// is captured upstream by TraceSinkMonitor when the arbiter monitor is enabled.
//
// Pattern is chosen at run time with plusargs (defaults come from parameters):
//   +tacit_bp_mode=always                          ready every cycle
//   +tacit_bp_mode=duty   +tacit_bp_on=M  +tacit_bp_period=N   ready M of every N cycles
//   +tacit_bp_mode=burst  +tacit_bp_off=X +tacit_bp_period=N   not ready for the last X of every N cycles
//   +tacit_bp_mode=every  +tacit_bp_beats=K +tacit_bp_off=X    after K accepted beats, not ready for X cycles
//   +tacit_bp_mode=random +tacit_bp_pct=P  [+tacit_bp_seed=S]  ready with probability P/100 each cycle
//   +tacit_bp_mode=file   +tacit_bp_file=path                  one 0/1 per line, one line per cycle; holds the last value at EOF
module TraceSinkBackpressureBlackBox
#(
    parameter MODE   = "duty",
    parameter ON     = 1,
    parameter PERIOD = 4,
    parameter OFF    = 100,
    parameter BEATS  = 16,
    parameter PCT    = 25,
    parameter SEED   = 1,
    parameter FILE_NAME = "tacit_bp_ready.txt"
)
(
    input  clk,
    input  reset,
    input  in_valid,
    output in_ready
);

`ifndef SYNTHESIS

reg [8*8-1:0] mode;
integer on_cycles, period, off_cycles, beats, pct, seed;
reg [1023:0] file_name;
integer file;
integer rc;

integer phase;      // duty / burst: position in the period
integer beat_count; // every: accepted beats since the last outage
integer off_left;   // every: outage cycles remaining
reg     ready_r;
integer file_val;

initial begin
    mode = MODE; on_cycles = ON; period = PERIOD; off_cycles = OFF;
    beats = BEATS; pct = PCT; seed = SEED; file_name = FILE_NAME;
    if ($value$plusargs("tacit_bp_mode=%s", mode))        ;
    if ($value$plusargs("tacit_bp_on=%d", on_cycles))     ;
    if ($value$plusargs("tacit_bp_period=%d", period))    ;
    if ($value$plusargs("tacit_bp_off=%d", off_cycles))   ;
    if ($value$plusargs("tacit_bp_beats=%d", beats))      ;
    if ($value$plusargs("tacit_bp_pct=%d", pct))          ;
    if ($value$plusargs("tacit_bp_seed=%d", seed))        ;
    if ($value$plusargs("tacit_bp_file=%s", file_name))   ;
    phase = 0; beat_count = 0; off_left = 0; ready_r = 1'b1; file = 0; file_val = 1;
    if (mode == "file") begin
        file = $fopen(file_name, "r");
        if (file == 0) begin
            $display("TraceSinkBackpressure: failed to open %0s", file_name);
            $finish;
        end
    end
    $display("TraceSinkBackpressure: mode=%0s on=%0d period=%0d off=%0d beats=%0d pct=%0d seed=%0d",
             mode, on_cycles, period, off_cycles, beats, pct, seed);
end

// ready for the coming cycle is decided at the clock edge from registered state
always @(posedge clk) begin
    if (reset) begin
        phase <= 0; beat_count <= 0; off_left <= 0; ready_r <= 1'b1;
    end else begin
        if (mode == "always") begin
            ready_r <= 1'b1;
        end else if (mode == "duty") begin
            phase <= (phase + 1 >= period) ? 0 : phase + 1;
            ready_r <= ((phase + 1 >= period) ? 0 : phase + 1) < on_cycles;
        end else if (mode == "burst") begin
            phase <= (phase + 1 >= period) ? 0 : phase + 1;
            ready_r <= ((phase + 1 >= period) ? 0 : phase + 1) < (period - off_cycles);
        end else if (mode == "every") begin
            if (off_left > 0) begin
                off_left <= off_left - 1;
                ready_r <= (off_left - 1) == 0;
            end else begin
                if (in_valid & ready_r) begin
                    if (beat_count + 1 >= beats) begin
                        beat_count <= 0;
                        off_left <= off_cycles;
                        ready_r <= 1'b0;
                    end else begin
                        beat_count <= beat_count + 1;
                        ready_r <= 1'b1;
                    end
                end else begin
                    ready_r <= 1'b1;
                end
            end
        end else if (mode == "random") begin
            ready_r <= ($urandom(seed) % 100) < pct;
            seed <= seed + 1;
        end else if (mode == "file") begin
            if (file != 0 && !$feof(file)) begin
                rc = $fscanf(file, "%d\n", file_val);
            end
            ready_r <= file_val != 0;
        end else begin
            ready_r <= 1'b1;
        end
    end
end

assign in_ready = ready_r;

final begin
    if (file != 0) $fclose(file);
end

`else
assign in_ready = 1'b1;
`endif

endmodule
