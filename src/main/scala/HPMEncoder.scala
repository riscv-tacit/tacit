package tacit

import chisel3._
import chisel3.util._
import freechips.rocketchip.trace.{TraceEncoderControlInterface, TraceCoreParams, LazyHPMEncoder, LazyHPMEncoderModule}
import freechips.rocketchip.rocket.CSR
import org.chipsalliance.cde.config.Parameters

/* Copied from testchipip Adapter.scala */
class SerialWidthSlicer(narrowW: Int, wideW: Int) extends Module {
  require(wideW > narrowW)
  require(wideW % narrowW == 0)
  val io = IO(new Bundle {
    val wide   = Flipped(Decoupled(UInt(wideW.W)))
    val wide_last_beat = Output(Bool())
    val narrow = Decoupled(UInt(narrowW.W))
  })

  val beats = wideW / narrowW
  val wide_beats = RegInit(0.U(log2Ceil(beats).W))
  val wide_last_beat = wide_beats === (beats-1).U

  io.narrow.valid := io.wide.valid
  io.narrow.bits := io.wide.bits.asTypeOf(Vec(beats, UInt(narrowW.W)))(wide_beats)
  when (io.narrow.fire) {
    wide_beats := Mux(wide_last_beat, 0.U, wide_beats + 1.U)
  }
  io.wide.ready := wide_last_beat && io.narrow.ready
  io.wide_last_beat := wide_last_beat
}

/* instead of encoding everything at once like VarLenEncoder,
 this will encode one byte at a time */
class SerialVarLenEncoder(val maxWidth: Int) extends Module {
  val maxNumBytes = maxWidth/(8-1) + 1

  val io = IO(new Bundle {
    val in = Flipped(Decoupled(UInt(maxWidth.W)))
    val out = Decoupled(UInt(8.W))
  })

  val sIdle :: sBusy :: Nil = Enum(2)
  val state = RegInit(sIdle)
  io.in.ready := false.B
  io.out.valid := state === sBusy
  io.out.bits := 0.U
  val i = RegInit(0.U(log2Ceil(maxNumBytes).W))
  val msb_index = RegInit(0.U(log2Ceil(maxNumBytes).W))

  switch (state) {
    is (sIdle) {
      state := Mux(io.in.valid, sBusy, sIdle)
      i := 0.U
      msb_index := (maxWidth - 1).U - PriorityEncoder(Reverse(io.in.bits))
    }
    is (sBusy) {
      i := i + io.out.fire.asUInt
      val len = Mux(i*7.U + 6.U < maxWidth.U, 7.U, maxWidth.U - 1.U - i*7.U)
      val lsi = i*7.U
      val mask = ((1.U << len) - 1.U) << lsi
      io.out.bits := ((io.in.bits & mask) >> lsi) | Mux(i*7.U >= msb_index, 0x80.U, 0.U)
      when (i*7.U >= msb_index) {
        state := sIdle
        io.in.ready := true.B
      }
    }
  }
}

class TraceHPMEncoder (override val xlen: Int, val bufferDepth: Int) (implicit p: Parameters)
 extends LazyHPMEncoder(xlen)(p) {
  override lazy val module = new TraceHPMEncoderModule(this)
}

class TraceHPMEncoderModule(outer: TraceHPMEncoder) extends LazyHPMEncoderModule(outer) {

  val hpm_samples = RegInit(VecInit(Seq.fill(CSR.nHPM)(0.U(CSR.hpmWidth.W))))
  val hpm_prev_samples = RegInit(VecInit(Seq.fill(CSR.nHPM)(0.U(CSR.hpmWidth.W))))
  val prev_time = Reg(UInt(outer.xlen.W))

  // state machine
  val sDisabled :: sSync:: sIdle :: sSample :: sSequence :: Nil = Enum(5)
  val state = RegInit(sDisabled)
  val sequence_mask_reg = RegInit(0.U(log2Ceil(CSR.nHPM).W))

  val slicer = Module(new SerialWidthSlicer(8, 32)) 
  val varlen_encoder = Module(new SerialVarLenEncoder(CSR.hpmWidth))
  val byte_buffer = Module(new Queue(UInt(8.W), outer.bufferDepth * 2))

  io.out :<>= byte_buffer.io.deq

  byte_buffer.io.enq.valid := false.B
  byte_buffer.io.enq.bits := 0.U

  // default values slicer
  slicer.io.wide.bits := 0.U
  slicer.io.wide.valid := false.B
  slicer.io.narrow.ready := false.B
  // default values varlen_encoder
  varlen_encoder.io.in.valid := false.B
  varlen_encoder.io.in.bits := 0.U
  varlen_encoder.io.out.ready := false.B

  switch (state) {
    is (sDisabled) {
      state := Mux(io.control.enable, sSync, sDisabled)
      prev_time := io.time
    }
    is (sSync) {
      // send a sync packet
      state := Mux(slicer.io.wide_last_beat && byte_buffer.io.enq.fire, sIdle, sSync)
      slicer.io.wide.valid := true.B
      slicer.io.wide.bits := io.control.hpmcounter_enable 
      byte_buffer.io.enq.valid := slicer.io.narrow.valid
      byte_buffer.io.enq.bits := slicer.io.narrow.bits
      slicer.io.narrow.ready := byte_buffer.io.enq.ready
    }
    is (sIdle) {
      state := Mux(~io.control.enable, sDisabled,
        Mux(io.time > prev_time + io.control.hpmcounter_report_interval, sSample, sIdle)) // time to sample
    }
    is (sSample) {
      state := sSequence
      hpm_samples.foreach(i => 
        hpm_samples(i) := Mux(io.control.hpmcounter_enable(i+CSR.firstHPM.U), 
        io.hpmcounters(i), hpm_samples(i))) // take the delta
      hpm_prev_samples := hpm_samples
      sequence_mask_reg := io.control.hpmcounter_enable >> CSR.firstHPM.U
      prev_time := io.time
    }
    is (sSequence) {
      state := Mux(sequence_mask_reg === 0.U, sIdle, sSequence)
      // next counter to seqeunce
      val next_counter = PriorityEncoder(sequence_mask_reg)
      varlen_encoder.io.in.valid := sequence_mask_reg =/= 0.U
      val delta = hpm_samples(next_counter) - hpm_prev_samples(next_counter)
      varlen_encoder.io.in.bits := delta

      byte_buffer.io.enq.valid := varlen_encoder.io.out.valid
      byte_buffer.io.enq.bits := varlen_encoder.io.out.bits
      varlen_encoder.io.out.ready := byte_buffer.io.enq.ready

      when (varlen_encoder.io.in.fire) {
        sequence_mask_reg := sequence_mask_reg & ~(1.U << next_counter)
      }
    }
  }
}

// class TraceHPMEncoder (override val xlen: Int, val bufferDepth: Int) (implicit p: Parameters)
//  extends LazyHPMEncoder(xlen)(p) {
//   override lazy val module = new TraceHPMEncoderModule(this)
// }

// class TraceHPMEncoderModule(outer: TraceHPMEncoder) extends LazyHPMEncoderModule(outer) {

//   val hpm_samples = RegInit(VecInit(Seq.fill(CSR.nHPM)(0.U(CSR.hpmWidth.W))))
//   val prev_time = Reg(UInt(outer.xlen.W))

//   // state machine
//   val sDisabled :: sSync:: sIdle :: sSample :: sSequence :: Nil = Enum(5)
//   val state = RegInit(sDisabled)
//   val sequence_mask_reg = RegInit(0.U(log2Ceil(CSR.nCtr).W))

//   val slicer = Module(new SerialWidthSlicer(8, 32)) 
//   val varlen_encoder = Module(new SerialVarLenEncoder(CSR.hpmWidth))
//   val byte_buffer = Module(new Queue(UInt(8.W), outer.bufferDepth * 2))

//   io.out :<>= byte_buffer.io.deq

//   byte_buffer.io.enq.valid := false.B
//   byte_buffer.io.enq.bits := 0.U

//   // default values slicer
//   slicer.io.wide.bits := 0.U
//   slicer.io.wide.valid := false.B
//   slicer.io.narrow.ready := false.B
//   // default values varlen_encoder
//   varlen_encoder.io.in.valid := false.B
//   varlen_encoder.io.in.bits := 0.U
//   varlen_encoder.io.out.ready := false.B

//   switch (state) {
//     is (sDisabled) {
//       state := Mux(io.control.enable, sSync, sDisabled)
//       prev_time := io.time
//     }
//     is (sSync) {
//       // send a sync packet
//       state := Mux(slicer.io.wide_last_beat && byte_buffer.io.enq.fire, sIdle, sSync)
//       slicer.io.wide.valid := true.B
//       slicer.io.wide.bits := io.control.hpmcounter_enable 
//       byte_buffer.io.enq.valid := slicer.io.narrow.valid
//       byte_buffer.io.enq.bits := slicer.io.narrow.bits
//       slicer.io.narrow.ready := byte_buffer.io.enq.ready
//     }
//     is (sIdle) {
//       state := Mux(~io.control.enable, sDisabled,
//         Mux(prev_time + io.control.hpmcounter_report_interval > io.time, sSample, sIdle)) // time to sample
//     }
//     is (sSample) {
//       state := sSequence
//       hpm_samples := io.hpmcounters
//       sequence_mask_reg := io.control.hpmcounter_enable >> CSR.firstHPM.U
//     }
//     is (sSequence) {
//       state := Mux(sequence_mask_reg === 0.U, sIdle, sSequence)
//       // next counter to seqeunce
//       val next_counter = PriorityEncoder(sequence_mask_reg)
//       varlen_encoder.io.in.valid := true.B
//       varlen_encoder.io.in.bits := hpm_samples(next_counter)

//       byte_buffer.io.enq.valid := varlen_encoder.io.out.valid
//       byte_buffer.io.enq.bits := varlen_encoder.io.out.bits
//       varlen_encoder.io.out.ready := byte_buffer.io.enq.ready

//       when (varlen_encoder.io.in.fire) {
//         sequence_mask_reg := sequence_mask_reg & ~(1.U << next_counter)
//       }
//     }
//   }
// }