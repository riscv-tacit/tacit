// See LICENSE.Berkeley for license details.
// See LICENSE.SiFive for license details.

// message_encoder_0 -> 
// message_encoder_1 -> packet_queue -> serializer -> stream
// message_encoder_2 -> 

package tacit

import chisel3._
import chisel3.util._
import chisel3.experimental.requireIsChiselType
import freechips.rocketchip.trace._

import org.chipsalliance.cde.config.Parameters

class TacitParallelEncoder(
  override val coreParams: TraceCoreParams, 
  val bufferDepth: Int, 
  val coreStages: Int, 
)(implicit p: Parameters) 
    extends LazyTraceEncoder(coreParams)(p) {
  override lazy val module = new TacitParallelEncoderModule(this)
}

class TacitParallelEncoderModule(outer: TacitParallelEncoder) extends LazyTraceEncoderModule(outer) with MetaDataWidthHelper {

  val coreParams = outer.coreParams
  val MAX_DELTA_TIME_COMP = 0x3F // 63, 6 bits

  // states
  val sIdle :: sStall :: sSync :: sData :: Nil = Enum(4)
  val state = RegInit(sIdle)
  val sync_type = RegInit(SyncType.SyncNone)
  val encode_sync = Wire(Bool())
  val prev_time = Reg(UInt(coreParams.xlen.W))

  // pipeline of ingress data
  val ingress_0 = RegInit(0.U.asTypeOf(new TraceCoreInterface(coreParams)))
  val ingress_1 = RegInit(0.U.asTypeOf(new TraceCoreInterface(coreParams)))

  val lane_compactor = Module(new LaneCompactor(new TraceCoreGroup(coreParams), coreParams.nGroups))
  lane_compactor.io.input_mask := io.in.group.map(_.iretire === 1.U)
  lane_compactor.io.input_data := io.in.group
  val compacted_ingress_group = lane_compactor.io.output_data
  val compacted_ingress = Wire(new TraceCoreInterface(coreParams))
  compacted_ingress.group := compacted_ingress_group
  compacted_ingress.priv := io.in.priv
  compacted_ingress.ctx := io.in.ctx
  compacted_ingress.tval := io.in.tval
  compacted_ingress.cause := io.in.cause
  compacted_ingress.time := io.in.time

  val delta_time = ingress_1.time - prev_time

  val pipeline_advance = Wire(Bool())
  pipeline_advance := io.in.group.map(_.iretire === 1.U).reduce(_ || _) // at least 1 valid ingress
  when (pipeline_advance) {
    ingress_0 := compacted_ingress
    ingress_1 := ingress_0
  }

  val time_encoder = Module(new VarLenMaskEncoder(coreParams.xlen))
  
  // buffers
  val metadata_buffer = MultiPortedQueue(new MetaDataBundle(coreParams), outer.bufferDepth, coreParams.nGroups, useSramQueue = true)
  val message_packet_buffer = MultiPortedQueue(new MessagePacketBundle(coreParams), outer.bufferDepth, coreParams.nGroups, useSramQueue = true)
  val header_buffer = MultiPortedQueue(UInt(8.W), outer.bufferDepth, coreParams.nGroups, useSramQueue = true)

  val trace_packetizer = Module(new TraceMaskedPacketizer(coreParams))
  trace_packetizer.io.message <> message_packet_buffer.io.deq
  trace_packetizer.io.metadata <> metadata_buffer.io.deq
  trace_packetizer.io.byte <> header_buffer.io.deq
  io.out <> trace_packetizer.io.out

  val sent = RegInit(false.B)
  // reset takes priority over enqueue
  when (pipeline_advance) {
    sent := false.B
  } .elsewhen (metadata_buffer.io.enqs.map(_.fire).reduce(_ || _)) {
    sent := true.B
  }

  // itermediate signals
  val metadata_enq_bits = Wire(Vec(coreParams.nGroups, new MetaDataBundle(coreParams)))
  val message_packet_enq_bits = Wire(Vec(coreParams.nGroups, new MessagePacketBundle(coreParams)))

  val packet_valids = VecInit(metadata_buffer.io.enqs.map(_.valid))
  val first_valid_index = PriorityEncoder(packet_valids)
  // Create per-encoder "is first" signals
  val is_first_valid = VecInit((0 until coreParams.nGroups).map { i =>
    packet_valids(i) && (first_valid_index === i.U)
  })

  val all_buffers_ready = metadata_buffer.io.enqs(0).ready &&                                                                                                                                                                                                                                                                                                           
                          header_buffer.io.enqs(0).ready &&                                                                                                                                                                                                                                                                                                             
                          message_packet_buffer.io.enqs(0).ready 

  for (i <- 0 until coreParams.nGroups) {
    val message_encoder = Module(new MessageEncoder(coreParams, canEncodeSyncMessage = i == 0, my_index = i))
    if (i == 0) { 
      message_encoder.io.encode_sync.get := encode_sync 
      message_encoder.io.sync_type.get := sync_type
      message_encoder.io.sync_ingress.get := ingress_0 
    }
    message_encoder.io.ingress := ingress_1 // pass in all groups, irrelevant ones will be optimized out
    message_encoder.io.ingress_0_target_addr_msg := ingress_0.group(0).iaddr // backup in case this is the last valid ingress
    message_encoder.io.target_prv_msg := ingress_0.priv
    message_encoder.io.ingress_valid := ingress_1.group(i).iretire === 1.U && (if (i==0) (state === sData || state === sSync) else (state === sData))

    metadata_enq_bits(i) := message_encoder.io.metadata
    metadata_enq_bits(i).time := Mux(is_first_valid(i), time_encoder.io.output_mask, 1.U)
    val time_can_be_compressed = Mux(is_first_valid(i), delta_time < MAX_DELTA_TIME_COMP.U, true.B)
    val is_compressed = message_encoder.io.possible_to_compress && time_can_be_compressed
    metadata_enq_bits(i).is_full := ~is_compressed
    metadata_buffer.io.enqs(i).bits := metadata_enq_bits(i)
    metadata_buffer.io.enqs(i).valid := message_encoder.io.packet_valid && !sent && all_buffers_ready

    message_packet_enq_bits(i) := message_encoder.io.message
    val zero_varlen_bytes = VecInit.fill(time_encoder.maxNumBytes)(0.U(8.W))
    zero_varlen_bytes(0) := 0x80.U(8.W) // signify a varlen encoded zero
    message_packet_enq_bits(i).time := Mux(is_first_valid(i), time_encoder.io.output_bytes, zero_varlen_bytes)
    message_packet_buffer.io.enqs(i).bits := message_packet_enq_bits(i)
    message_packet_buffer.io.enqs(i).valid := message_encoder.io.packet_valid && !is_compressed && !sent && all_buffers_ready

    val compressed_packet = Cat(Mux(is_first_valid(i), delta_time(5,0), 0.U(6.W)), message_encoder.io.comp_header)
    header_buffer.io.enqs(i).bits := Mux(is_compressed, compressed_packet, message_encoder.io.full_header)
    header_buffer.io.enqs(i).valid := message_encoder.io.packet_valid && !sent && all_buffers_ready
  }

  for (i <- 0 until coreParams.nGroups) {
    metadata_buffer.io.enqs(i).bits := metadata_enq_bits(i)
    message_packet_buffer.io.enqs(i).bits := message_packet_enq_bits(i)
    assert(metadata_buffer.io.enqs(i).fire === header_buffer.io.enqs(i).fire,
      s"metadata and header buffer must fire atomically on port $i")
  }

  // at least one metadata packet has enqueued
  val do_enq = metadata_buffer.io.enqs.map(_.fire).reduce(_ || _)
  when (do_enq) { prev_time := ingress_1.time }

  // default values
  encode_sync := state === sSync
  time_encoder.io.input_valid := false.B
  time_encoder.io.input_value := DontCare
  
  switch (state) {
    is (sIdle) {
      when (io.control.enable) {
        state := sSync
        sync_type := SyncType.SyncStart
      }
    }
    is (sSync) {
      time_encoder.io.input_value := ingress_0.time
      time_encoder.io.input_valid := true.B
      prev_time := ingress_0.time
      state := Mux(pipeline_advance && (sent || metadata_buffer.io.enqs.map(_.fire).reduce(_ || _)), Mux(io.control.enable, sData, sIdle), sSync)
    }
    is (sData) {
      when (!io.control.enable) {
        state := Mux(pipeline_advance, sSync, sData)
        sync_type := SyncType.SyncEnd
      } .otherwise {
        time_encoder.io.input_value := delta_time
        time_encoder.io.input_valid := true.B
      }
    }
  }
  def stallThreshold(count: UInt) = count >= (outer.bufferDepth - outer.coreStages).U

  io.stall := stallThreshold(metadata_buffer.io.count) || 
              stallThreshold(message_packet_buffer.io.count) ||
              stallThreshold(header_buffer.io.count)
}

class MessageEncoder(
  val coreParams: TraceCoreParams, 
  val canEncodeSyncMessage: Boolean,
  val my_index: Int,
) extends Module with MetaDataWidthHelper {
  val io = IO(new Bundle {
    val encode_sync = if (canEncodeSyncMessage) Some(Input(Bool())) else None
    val sync_ingress = if (canEncodeSyncMessage) Some(Input(new TraceCoreInterface(coreParams))) else None
    val sync_type = if (canEncodeSyncMessage) Some(Input(SyncType())) else None
    val ingress = Input(new TraceCoreInterface(coreParams))
    val ingress_0_target_addr_msg = Input(UInt(coreParams.iaddrWidth.W))
    val target_prv_msg = Input(UInt(4.W))
    val ingress_valid = Input(Bool())
    val metadata = Output(new MetaDataBundle(coreParams))
    val message = Output(new MessagePacketBundle(coreParams)) 
    val comp_header = Output(UInt(CompressedHeaderType.getWidth.W))
    val full_header = Output(UInt(8.W))
    val packet_valid = Output(Bool())
    val possible_to_compress = Output(Bool())
  })

  def my_index_is_last = my_index == coreParams.nGroups - 1

  // intermediate packet signals
  val possible_to_compress = Wire(Bool())
  val header_byte   = Wire(UInt(8.W)) // full header
  io.full_header := header_byte

  val comp_header = Wire(UInt(CompressedHeaderType.getWidth.W)) // compressed header
  io.comp_header := comp_header

  // varlen encoders
  val trap_addr_encoder = Module(new VarLenMaskEncoder(coreParams.iaddrWidth))
  val target_addr_encoder = Module(new VarLenMaskEncoder(coreParams.iaddrWidth))
  val prv_encoder = Module(new PrvEncoder)
  val ctx_encoder = Module(new VarLenMaskEncoder(maxASIdBits))

  // intermediate encoder control signals
  val encode_trap_addr_valid = Wire(Bool())
  val encode_target_addr_valid = Wire(Bool())
  val encode_prv_valid = Wire(Bool())
  val encode_ctx_valid = Wire(Bool())

  trap_addr_encoder.io.input_valid := encode_trap_addr_valid && !possible_to_compress 
  target_addr_encoder.io.input_valid := encode_target_addr_valid && !possible_to_compress 
  prv_encoder.io.input_valid := encode_prv_valid && !possible_to_compress 
  ctx_encoder.io.input_valid := encode_ctx_valid && !possible_to_compress 

  // metadata packing
  val metadata = Wire(new MetaDataBundle(coreParams))
  metadata.prv := prv_encoder.io.output_valid
  metadata.ctx := ctx_encoder.io.output_mask
  metadata.trap_addr := trap_addr_encoder.io.output_mask
  metadata.target_addr := target_addr_encoder.io.output_mask
  io.possible_to_compress := possible_to_compress 
  metadata.is_full := ~possible_to_compress  
  metadata.time := DontCare
  io.metadata := metadata

  // message packing
  val message = Wire(new MessagePacketBundle(coreParams))
  message.prv := prv_encoder.io.output_byte
  message.ctx := ctx_encoder.io.output_bytes
  message.trap_addr := trap_addr_encoder.io.output_bytes
  message.target_addr := target_addr_encoder.io.output_bytes
  message.time := DontCare
  io.message := message

  // do we have a message to encode?
  // is this the last valid ingress?
  // if there are more inports, check if they are all invalid
  val no_more_ingress_valid = if (!my_index_is_last) io.ingress.group.slice(my_index+1, coreParams.nGroups).map(_.iretire === 0.U).reduce(_ && _) else true.B
  val ingress_is_last_valid = io.ingress.group(my_index).iretire === 1.U && no_more_ingress_valid // I am valid and there are no more valid inports
  val target_addr_msg = Wire(UInt(coreParams.iaddrWidth.W))
  if (my_index_is_last) {
    target_addr_msg := io.ingress_0_target_addr_msg
  } else {
    target_addr_msg := Mux(ingress_is_last_valid, io.ingress_0_target_addr_msg, io.ingress.group(my_index+1).iaddr)
  }
  val xored_addr = (target_addr_msg ^ io.ingress.group(my_index).iaddr) >> 1.U
  
  val ingress_has_message = io.ingress.group(my_index).itype =/= TraceItype.ITNothing

  io.packet_valid := io.ingress_valid && ingress_has_message

  // assign default values
  possible_to_compress := true.B
  header_byte := DontCare
  comp_header := DontCare
  target_addr_encoder.io.input_value := DontCare
  encode_target_addr_valid := false.B
  prv_encoder.io.from_priv := DontCare
  prv_encoder.io.to_priv := DontCare
  encode_prv_valid := false.B
  ctx_encoder.io.input_value := DontCare
  encode_ctx_valid := false.B
  trap_addr_encoder.io.input_value := DontCare
  encode_trap_addr_valid := false.B
  
  switch (io.ingress.group(my_index).itype) {
    is (TraceItype.ITBrTaken) {
      header_byte := HeaderByte(FullHeaderType.FTakenBranch)
      comp_header := CompressedHeaderType.CTB.asUInt
    }
    is (TraceItype.ITBrNTaken) {
      header_byte := HeaderByte(FullHeaderType.FNotTakenBranch)
      comp_header := CompressedHeaderType.CNT.asUInt
    }
    is (TraceItype.ITInJump) {
      header_byte := HeaderByte(FullHeaderType.FInfJump)
      comp_header := CompressedHeaderType.CIJ.asUInt
    }
    is (TraceItype.ITUnJump) {
      header_byte := HeaderByte(FullHeaderType.FUninfJump)
      comp_header := CompressedHeaderType.CNA.asUInt
      target_addr_encoder.io.input_value := xored_addr
      encode_target_addr_valid := true.B
      possible_to_compress := false.B
    }
    is (TraceItype.ITException) {
      header_byte := HeaderByte.from_trap_type(FullHeaderType.FTrap, TrapType.TException)
      comp_header := CompressedHeaderType.CNA.asUInt
      target_addr_encoder.io.input_value := xored_addr
      encode_target_addr_valid := true.B
      trap_addr_encoder.io.input_value := io.ingress.group(my_index).iaddr >> 1.U
      encode_trap_addr_valid := true.B
      prv_encoder.io.from_priv := io.ingress.priv
      prv_encoder.io.to_priv := io.target_prv_msg
      encode_prv_valid := true.B
      possible_to_compress := false.B
    }
    is (TraceItype.ITInterrupt) {
      header_byte := HeaderByte.from_trap_type(FullHeaderType.FTrap, TrapType.TInterrupt)
      comp_header := CompressedHeaderType.CNA.asUInt
      target_addr_encoder.io.input_value := xored_addr
      encode_target_addr_valid := true.B
      trap_addr_encoder.io.input_value := io.ingress.group(my_index).iaddr >> 1.U
      encode_trap_addr_valid := true.B
      prv_encoder.io.from_priv := io.ingress.priv
      prv_encoder.io.to_priv := io.target_prv_msg
      encode_prv_valid := true.B
      possible_to_compress := false.B
    }
    is (TraceItype.ITReturn) {
      header_byte := HeaderByte.from_trap_type(FullHeaderType.FTrap, TrapType.TReturn)
      comp_header := CompressedHeaderType.CNA.asUInt
      target_addr_encoder.io.input_value := xored_addr
      encode_target_addr_valid := true.B
      trap_addr_encoder.io.input_value := io.ingress.group(my_index).iaddr >> 1.U
      encode_trap_addr_valid := true.B
      prv_encoder.io.from_priv := io.ingress.priv
      prv_encoder.io.to_priv := io.target_prv_msg
      encode_prv_valid := true.B
      ctx_encoder.io.input_value := io.ingress.ctx
      encode_ctx_valid := io.target_prv_msg === 0.U // encode ctx if returning to user mode
      possible_to_compress := false.B
    }
  }

  if (canEncodeSyncMessage) {
    when (io.encode_sync.get) {
      io.packet_valid := true.B
      header_byte := HeaderByte.from_sync_type(FullHeaderType.FSync, io.sync_type.get)
      target_addr_encoder.io.input_value := io.sync_ingress.get.group(my_index).iaddr >> 1.U
      encode_target_addr_valid := true.B
      prv_encoder.io.from_priv := 0b00.U
      prv_encoder.io.to_priv := io.sync_ingress.get.priv
      encode_prv_valid := true.B
      // reuse trap address for runtime_cfg
      val runtime_cfg = 0.U(7.W)
      // 2 bits for bp mode, 6 bits for n_entries
      trap_addr_encoder.io.input_value := runtime_cfg
      encode_trap_addr_valid := true.B
      ctx_encoder.io.input_value := io.sync_ingress.get.ctx
      encode_ctx_valid := true.B
      possible_to_compress := false.B
    }
  }
}
