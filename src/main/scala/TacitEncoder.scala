// See LICENSE.Berkeley for license details.
// See LICENSE.SiFive for license details.

package tacit

import chisel3._
import chisel3.util._
import freechips.rocketchip.trace._

import org.chipsalliance.cde.config.Parameters

object FullHeaderType extends ChiselEnum {
  val FTakenBranch    = Value(0x0.U) // 000
  val FNotTakenBranch = Value(0x1.U) // 001
  val FUninfJump      = Value(0x2.U) // 010
  val FInfJump        = Value(0x3.U) // 011
  val FTrap           = Value(0x4.U) // 100
  val FSync           = Value(0x5.U) // 101
  val FValue          = Value(0x6.U) // 110
  val FReserved       = Value(0x7.U) // 111
}

object CompressedHeaderType extends ChiselEnum {
  val CTB = Value(0x0.U) // 00, taken branch
  val CNT = Value(0x1.U) // 01, not taken branch
  val CNA = Value(0x2.U) // 10, not a compressed packet
  val CIJ = Value(0x3.U) // 11, is a jump
}

object TrapType extends ChiselEnum {
  val TNone      = Value(0x0.U)
  val TException = Value(0x1.U)
  val TInterrupt = Value(0x2.U)
  val TReturn    = Value(0x4.U)
}

object SyncType extends ChiselEnum {
  val SyncNone = Value(0b000.U)
  val SyncStart = Value(0b001.U)
  val SyncPeriodic = Value(0b010.U)
  val SyncEnd = Value(0b011.U)
}

object HeaderByte {
  def from_trap_type(header_type: FullHeaderType.Type, trap_type: TrapType.Type): UInt = {
    Cat(
      trap_type.asUInt,
      header_type.asUInt,
      CompressedHeaderType.CNA.asUInt
    )
  }

  def from_sync_type(header_type: FullHeaderType.Type, sync_type: SyncType.Type): UInt = {
    Cat(
      sync_type.asUInt,
      header_type.asUInt,
      CompressedHeaderType.CNA.asUInt
    )
  }

  def apply(header_type: FullHeaderType.Type): UInt = {
    Cat(
      0.U(3.W),
      header_type.asUInt,
      CompressedHeaderType.CNA.asUInt
    )
  }
}

trait MetaDataWidthHelper {
  // abstract parameter
  val coreParams: TraceCoreParams
  def getMaxNumBytes(width: Int): Int = { width/(8-1) + 1 }
  lazy val maxASIdBits = coreParams.xlen match {
    case 32 => 9
    case 64 => 16
  }
  lazy val addrMaxNumBytes = getMaxNumBytes(coreParams.iaddrWidth)
  lazy val timeMaxNumBytes = getMaxNumBytes(coreParams.xlen)
  lazy val ctxMaxNumBytes = getMaxNumBytes(maxASIdBits)
}

class MetaDataBundle(val coreParams: TraceCoreParams) extends Bundle with MetaDataWidthHelper {
  val prv = UInt(1.W)
  val ctx = UInt(ctxMaxNumBytes.W)
  val target_addr = UInt(addrMaxNumBytes.W)
  val trap_addr = UInt(addrMaxNumBytes.W)
  val time = UInt(timeMaxNumBytes.W)
  val is_compressed = UInt(1.W)
}

class TacitEncoder(override val coreParams: TraceCoreParams, val bufferDepth: Int, val coreStages: Int, val bpParams: TacitBPParams)(implicit p: Parameters) 
    extends LazyTraceEncoder(coreParams)(p) {
  override lazy val module = new TacitEncoderModule(this)
}

class TacitEncoderModule(outer: TacitEncoder) extends LazyTraceEncoderModule(outer) with MetaDataWidthHelper {

  val coreParams = outer.coreParams

  val MAX_DELTA_TIME_COMP = 0x3F // 63, 6 bits
  def stallThreshold(count: UInt) = count >= (outer.bufferDepth - outer.coreStages).U

  // mode of operation
  // 0: branch target only
  // 1: branch prediction and skip jump
  // 2: branch prediction and don't skip jump
  def is_bt_mode = io.control.bp_mode === 0.U
  def is_bp_mode = io.control.bp_mode === 2.U 

  // states
  val sIdle :: sSync :: sData :: Nil = Enum(3)
  val state = RegInit(sIdle)
  val sync_type = RegInit(SyncType.SyncNone)
  val enabled = RegInit(false.B)
  val stall = Wire(Bool())
  val prev_time = Reg(UInt(coreParams.xlen.W))

  // pipeline of ingress data
  val ingress_0 = RegInit(0.U.asTypeOf(new TraceCoreInterface(coreParams)))
  val ingress_1 = RegInit(0.U.asTypeOf(new TraceCoreInterface(coreParams)))

  // shift every cycle, if not stalled
  val pipeline_advance = Wire(Bool())
  pipeline_advance := io.in.group(0).iretire === 1.U
  when (pipeline_advance) {
    ingress_0 := io.in
    ingress_1 := ingress_0
  }

  // encoders
  val trap_addr_encoder = Module(new VarLenEncoder(coreParams.iaddrWidth))
  val target_addr_encoder = Module(new VarLenEncoder(coreParams.iaddrWidth))
  val time_encoder = Module(new VarLenEncoder(coreParams.xlen))
  val prv_encoder = Module(new PrvEncoder)
  val ctx_encoder = Module(new VarLenEncoder(maxASIdBits))
  val metadataWidth = log2Ceil(trap_addr_encoder.maxNumBytes) + log2Ceil(target_addr_encoder.maxNumBytes) + log2Ceil(time_encoder.maxNumBytes) + 1

  // queue buffers
  val trap_addr_buffer = Module(new Queue(Vec(trap_addr_encoder.maxNumBytes, UInt(8.W)), outer.bufferDepth))
  val target_addr_buffer = Module(new Queue(Vec(target_addr_encoder.maxNumBytes, UInt(8.W)), outer.bufferDepth))
  val time_buffer = Module(new Queue(Vec(time_encoder.maxNumBytes, UInt(8.W)), outer.bufferDepth))
  val prv_buffer = Module(new Queue(UInt(8.W), outer.bufferDepth)) 
  val ctx_buffer = Module(new Queue(Vec(ctx_encoder.maxNumBytes, UInt(8.W)), outer.bufferDepth))
  val byte_buffer = Module(new Queue(UInt(8.W), outer.bufferDepth)) // buffer compressed packet or full header
  val metadata_buffer = Module(new Queue(new MetaDataBundle(coreParams), outer.bufferDepth))
  
  // intermediate packet signals
  val is_compressed = Wire(Bool())
  val delta_time = ingress_1.time - prev_time
  val packet_valid = Wire(Bool())
  val header_byte   = Wire(UInt(8.W)) // full header
  val comp_packet   = Wire(UInt(8.W)) // compressed packet
  val comp_header   = Wire(UInt(CompressedHeaderType.getWidth.W)) // compressed header
  
  // branch predictor
  val bp = Module(new DSCBranchPredictor(outer.bpParams))
  val bp_hit_count_next = Wire(UInt(32.W))
  val bp_inference_valid = Wire(Bool())
  val bp_hit_count_en = Wire(Bool())
  val bp_hit_count = RegEnable(bp_hit_count_next, 0.U, bp_hit_count_en)
  bp_hit_count_next := bp_hit_count // default behavior is to hold the value
  val bp_miss_flag_next = Wire(Bool())
  val bp_miss_flag_en = Wire(Bool())
  val bp_miss_flag = RegEnable(bp_miss_flag_next, false.B, bp_miss_flag_en)
  bp_miss_flag_next := false.B // default behavior is to set to false
  val bp_flush_hit = Wire(Bool())
  bp_flush_hit := false.B
  
  val bp_hit_packet = Cat(bp_hit_count(5, 0), comp_header)
  comp_packet := Cat(delta_time(5, 0), comp_header)

  // packetization of buffered message
  val trace_packetizer = Module(new TracePacketizer(coreParams))
  trace_packetizer.io.target_addr <> target_addr_buffer.io.deq
  trace_packetizer.io.trap_addr <> trap_addr_buffer.io.deq
  trace_packetizer.io.time <> time_buffer.io.deq
  trace_packetizer.io.byte <> byte_buffer.io.deq
  trace_packetizer.io.metadata <> metadata_buffer.io.deq
  trace_packetizer.io.prv <> prv_buffer.io.deq
  trace_packetizer.io.ctx <> ctx_buffer.io.deq
  trace_packetizer.io.out <> io.out

  // intermediate encoder control signals
  val encode_trap_addr_valid = Wire(Bool())
  val encode_target_addr_valid = Wire(Bool())
  val encode_prv_valid = Wire(Bool())
  val encode_ctx_valid = Wire(Bool())

  // metadata packing
  val metadata = Wire(new MetaDataBundle(coreParams))
  metadata.prv := prv_encoder.io.output_valid
  metadata.ctx := ctx_encoder.io.output_num_bytes
  metadata.trap_addr := trap_addr_encoder.io.output_num_bytes
  metadata.target_addr := target_addr_encoder.io.output_num_bytes
  metadata.time := time_encoder.io.output_num_bytes
  metadata.is_compressed := is_compressed

  metadata_buffer.io.enq.bits := metadata
  metadata_buffer.io.enq.valid := packet_valid
  // buffering compressed packet or full header depending on is_compressed
  byte_buffer.io.enq.bits := Mux(is_compressed, 
                                  Mux(bp_flush_hit, bp_hit_packet, comp_packet),
                                  header_byte)
  byte_buffer.io.enq.valid := packet_valid
  // trap address buffering
  trap_addr_buffer.io.enq.bits := trap_addr_encoder.io.output_bytes
  trap_addr_buffer.io.enq.valid := !is_compressed && packet_valid && encode_trap_addr_valid
  // target address buffering
  target_addr_buffer.io.enq.bits := target_addr_encoder.io.output_bytes
  target_addr_buffer.io.enq.valid := !is_compressed && packet_valid && encode_target_addr_valid
  // time buffering
  time_buffer.io.enq.bits := time_encoder.io.output_bytes
  time_buffer.io.enq.valid := !is_compressed && packet_valid
  // prv buffering
  prv_buffer.io.enq.bits := prv_encoder.io.output_byte
  prv_buffer.io.enq.valid := !is_compressed && packet_valid && encode_prv_valid
  // context buffering
  ctx_buffer.io.enq.bits := ctx_encoder.io.output_bytes
  ctx_buffer.io.enq.valid := !is_compressed && packet_valid && encode_ctx_valid

  // stall if any buffer is almost full 
  // technically it should always the byte buffer, but just to be safe
  stall := stallThreshold(trap_addr_buffer.io.count) || stallThreshold(target_addr_buffer.io.count) || stallThreshold(time_buffer.io.count) || stallThreshold(byte_buffer.io.count)
  io.stall := stall
  
  val sent = RegInit(false.B)
  // reset takes priority over enqueue
  when (pipeline_advance) {
    sent := false.B
  } .elsewhen (byte_buffer.io.enq.fire) {
    sent := true.B
  }

  trap_addr_encoder.io.input_valid := encode_trap_addr_valid && !is_compressed && packet_valid
  target_addr_encoder.io.input_valid := encode_target_addr_valid && !is_compressed && packet_valid
  prv_encoder.io.input_valid := encode_prv_valid && !is_compressed && packet_valid
  ctx_encoder.io.input_valid := encode_ctx_valid && !is_compressed && packet_valid
  time_encoder.io.input_valid := !is_compressed && packet_valid

  /* 
  - oldest instruction -
    ingress_1_group_0
    ingress_1_group_n
    ingress_0_group_0
    ingress_0_group_n
  - youngest instruction -
  */
  val ingress_0_has_message = ingress_0.group.map(g => g.itype =/= TraceItype.ITNothing && g.iretire === 1.U).reduce(_ || _)
  val ingress_0_has_branch = ingress_0.group.map(g => (g.itype === TraceItype.ITBrTaken || g.itype === TraceItype.ITBrNTaken) && g.iretire === 1.U).reduce(_ || _)
  val ingress_0_has_ij = ingress_0.group.map(g => (g.itype === TraceItype.ITInJump) && g.iretire === 1.U).reduce(_ || _)
  val ingress_0_has_flush = ingress_0_has_message && !ingress_0_has_branch && !ingress_0_has_ij
  val ingress_0_msg_idx = PriorityEncoder(ingress_0.group.map(g => g.itype =/= TraceItype.ITNothing && g.iretire === 1.U))
  
  val ingress_1_has_message = ingress_1.group.map(g => g.itype =/= TraceItype.ITNothing && g.iretire === 1.U).reduce(_ || _)
  val ingress_1_has_branch = ingress_1.group.map(g => (g.itype === TraceItype.ITBrTaken || g.itype === TraceItype.ITBrNTaken) && g.iretire === 1.U).reduce(_ || _)
  val ingress_1_has_packet = Mux(is_bp_mode, ingress_1_has_message && !ingress_1_has_branch, ingress_1_has_message)
  val ingress_1_msg_idx = PriorityEncoder(ingress_1.group.map(g => g.itype =/= TraceItype.ITNothing && g.iretire === 1.U))

  val ingress_1_valid_count = PopCount(ingress_1.group.map(g => g.iretire === 1.U))

  val target_addr_msg = Mux(ingress_1_msg_idx === (ingress_1_valid_count - 1.U), // am I the last message?
                            (ingress_1.group(ingress_1_msg_idx).iaddr ^ ingress_0.group(0).iaddr) >> 1.U,
                            (ingress_1.group(ingress_1_msg_idx).iaddr ^ ingress_1.group(ingress_1_msg_idx + 1.U).iaddr) >> 1.U)

  // driving branch predictor signals
  bp.io.req_pc := ingress_0.group(ingress_0_msg_idx).iaddr
  bp_inference_valid := ingress_0.group(ingress_0_msg_idx).iretire === 1.U && 
                        (ingress_0.group(ingress_0_msg_idx).itype === TraceItype.ITBrTaken || ingress_0.group(ingress_0_msg_idx).itype === TraceItype.ITBrNTaken) &&
                        pipeline_advance && io.control.enable
  bp.io.update_valid := bp_inference_valid
  bp.io.update_taken := ingress_0.group(ingress_0_msg_idx).itype === TraceItype.ITBrTaken
  bp_hit_count_en := pipeline_advance && io.control.enable
  bp_miss_flag_en := pipeline_advance && io.control.enable

  // default values
  trap_addr_encoder.io.input_value := 0.U
  target_addr_encoder.io.input_value := 0.U
  time_encoder.io.input_value := 0.U
  prv_encoder.io.from_priv := 0.U
  prv_encoder.io.to_priv := 0.U
  ctx_encoder.io.input_value := 0.U
  is_compressed := false.B
  packet_valid := false.B
  encode_target_addr_valid := false.B
  encode_trap_addr_valid := false.B
  encode_prv_valid := false.B
  encode_ctx_valid := false.B
  comp_header := CompressedHeaderType.CNA.asUInt
  header_byte := HeaderByte(FullHeaderType.FReserved)
  // state machine
  switch (state) {
    is (sIdle) {
      when (io.control.enable) { 
        state := sSync 
        sync_type := SyncType.SyncStart
      }
    }
    is (sSync) {
      header_byte := HeaderByte.from_sync_type(FullHeaderType.FSync, sync_type)
      time_encoder.io.input_value := ingress_0.time
      prev_time := ingress_0.time
      // target address
      target_addr_encoder.io.input_value := ingress_0.group(0).iaddr >> 1.U // last bit is always 0
      encode_target_addr_valid := true.B
      // prv
      prv_encoder.io.from_priv := 0b00.U
      prv_encoder.io.to_priv := ingress_0.priv
      encode_prv_valid := true.B
      // reuse trap address for runtime_cfg
      val runtime_cfg = Wire(UInt(7.W))
      // 2 bits for bp mode, 6 bits for n_entries
      runtime_cfg := Cat(log2Ceil(outer.bpParams.n_entries/64).U, io.control.bp_mode(1,0))
      trap_addr_encoder.io.input_value := runtime_cfg
      encode_trap_addr_valid := sync_type === SyncType.SyncStart
      // context
      ctx_encoder.io.input_value := ingress_0.ctx
      encode_ctx_valid := true.B
      is_compressed := false.B
      packet_valid := !sent
      // state transition: wait for message to go in
      state := Mux(pipeline_advance && (sent || byte_buffer.io.enq.fire), Mux(io.control.enable, sData, sIdle), sSync)
    }
    is (sData) {
      when (!io.control.enable) {
        state := sSync
        sync_type := SyncType.SyncEnd
      } .otherwise {
        // ingress0 logic - branch resolution
        when (ingress_0_has_branch) {
          val taken = ingress_0.group(ingress_0_msg_idx).itype === TraceItype.ITBrTaken
          bp_hit_count_next := Mux((bp.io.resp === taken) && is_bp_mode, 
                              bp_hit_count + 1.U, 
                              0.U) // reset if responded with miss
          bp_miss_flag_next := (bp.io.resp =/= taken) && is_bp_mode 
          bp_flush_hit := (bp.io.resp =/= taken) && is_bp_mode && bp_hit_count > 0.U 
        }
        .elsewhen (ingress_0_has_flush) { // these two conditions are mutually exclusive
          bp_flush_hit := is_bp_mode && bp_hit_count > 0.U
          bp_hit_count_next := 0.U
        }
        // ingress1 logic - message encoding
        when (bp_flush_hit && is_bp_mode) {
          // encode hit packet
          header_byte := HeaderByte(FullHeaderType.FTakenBranch)
          comp_header := CompressedHeaderType.CTB.asUInt
          time_encoder.io.input_value := bp_hit_count
          is_compressed := bp_hit_count <= MAX_DELTA_TIME_COMP.U
          packet_valid := !sent && is_bp_mode
        }
        .elsewhen (bp_miss_flag && is_bp_mode) {
          // encode miss packet
          header_byte := HeaderByte(FullHeaderType.FNotTakenBranch)
          comp_header := CompressedHeaderType.CNT.asUInt
          time_encoder.io.input_value := delta_time
          prev_time := Mux(byte_buffer.io.enq.fire, ingress_1.time, prev_time)
          is_compressed := delta_time <= MAX_DELTA_TIME_COMP.U
          packet_valid := !sent && is_bp_mode
        }
        .elsewhen (ingress_1_has_message) {
          switch (ingress_1.group(ingress_1_msg_idx).itype) {
            is (TraceItype.ITNothing) {
              packet_valid := false.B
            }
            is (TraceItype.ITBrTaken) {
              header_byte := HeaderByte(FullHeaderType.FTakenBranch)
              comp_header := CompressedHeaderType.CTB.asUInt
              time_encoder.io.input_value := delta_time
              prev_time := Mux(byte_buffer.io.enq.fire, ingress_1.time, prev_time)
              is_compressed := delta_time <= MAX_DELTA_TIME_COMP.U
              packet_valid := !sent && is_bt_mode
            }
            is (TraceItype.ITBrNTaken) {
              header_byte := HeaderByte(FullHeaderType.FNotTakenBranch)
              comp_header := CompressedHeaderType.CNT.asUInt
              time_encoder.io.input_value := delta_time
              prev_time := Mux(byte_buffer.io.enq.fire, ingress_1.time, prev_time)
              is_compressed := delta_time <= MAX_DELTA_TIME_COMP.U
              packet_valid := !sent && is_bt_mode
            }
            is (TraceItype.ITInJump) {
              header_byte := HeaderByte(FullHeaderType.FInfJump)
              comp_header := CompressedHeaderType.CIJ.asUInt
              time_encoder.io.input_value := delta_time
              prev_time := Mux(byte_buffer.io.enq.fire, ingress_1.time, prev_time)
              is_compressed := delta_time <= MAX_DELTA_TIME_COMP.U
              packet_valid := !sent && is_bt_mode
            }
            is (TraceItype.ITUnJump) {
              header_byte := HeaderByte(FullHeaderType.FUninfJump)
              time_encoder.io.input_value := delta_time 
              prev_time := Mux(byte_buffer.io.enq.fire, ingress_1.time, prev_time)
              target_addr_encoder.io.input_value := target_addr_msg
              encode_target_addr_valid := true.B
              is_compressed := false.B
              packet_valid := !sent
            }
            is (TraceItype.ITException) {
              header_byte := HeaderByte.from_trap_type(FullHeaderType.FTrap, TrapType.TException)
              comp_header := CompressedHeaderType.CNA.asUInt
              time_encoder.io.input_value := delta_time
              prev_time := Mux(byte_buffer.io.enq.fire, ingress_1.time, prev_time)
              target_addr_encoder.io.input_value := target_addr_msg
              encode_target_addr_valid := true.B
              trap_addr_encoder.io.input_value := ingress_1.group(ingress_1_msg_idx).iaddr >> 1.U
              encode_trap_addr_valid := true.B
              prv_encoder.io.from_priv := ingress_1.priv
              prv_encoder.io.to_priv := ingress_0.priv
              encode_prv_valid := true.B
              is_compressed := false.B
              packet_valid := !sent
            }
            is (TraceItype.ITInterrupt) {
              header_byte := HeaderByte.from_trap_type(FullHeaderType.FTrap, TrapType.TInterrupt)
              comp_header := CompressedHeaderType.CNA.asUInt
              time_encoder.io.input_value := delta_time
              prev_time := Mux(byte_buffer.io.enq.fire, ingress_1.time, prev_time)
              target_addr_encoder.io.input_value := target_addr_msg
              encode_target_addr_valid := true.B
              trap_addr_encoder.io.input_value := ingress_1.group(ingress_1_msg_idx).iaddr >> 1.U
              encode_trap_addr_valid := true.B
              prv_encoder.io.from_priv := ingress_1.priv
              prv_encoder.io.to_priv := ingress_0.priv
              encode_prv_valid := true.B
              is_compressed := false.B
              packet_valid := !sent
            }
            is (TraceItype.ITReturn) {
              header_byte := HeaderByte.from_trap_type(FullHeaderType.FTrap, TrapType.TReturn)
              comp_header := CompressedHeaderType.CNA.asUInt
              time_encoder.io.input_value := delta_time
              prev_time := Mux(byte_buffer.io.enq.fire, ingress_1.time, prev_time)
              target_addr_encoder.io.input_value := target_addr_msg
              encode_target_addr_valid := true.B
              trap_addr_encoder.io.input_value := ingress_1.group(ingress_1_msg_idx).iaddr >> 1.U
              encode_trap_addr_valid := true.B
              prv_encoder.io.from_priv := ingress_1.priv
              prv_encoder.io.to_priv := ingress_0.priv
              encode_prv_valid := true.B
              ctx_encoder.io.input_value := ingress_1.ctx
              encode_ctx_valid := ingress_0.priv === 0.U // encode ctx if returning to user mode
              is_compressed := false.B
              packet_valid := !sent
            }
          }
        }
      }
    }
  }
}