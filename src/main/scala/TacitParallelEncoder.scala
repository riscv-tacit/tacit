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
  val queueImpl: MPQueueImpl = MPQueueImpl.SRAM,
  // Cycles of ingress that can still arrive after io.stall asserts. BOOM gates
  // ROB commit on stall, so this is NOT the core pipeline depth: it is stall
  // propagation (~1 cycle) plus the encoder's own ingress_0/ingress_1 stages.
  // A fetch-stalled in-order core (e.g. Shuttle) must instead cover its full
  // drain window here. Keep it in true cycle units.
  val stallQuiescenceCycles: Int = 3,
  // Worst-case packet-producing retires per cycle (None = nGroups). A core that
  // is nGroups wide but architecturally limited (e.g. Shuttle retiring at most
  // one control-flow packet per cycle) declares the tighter bound here; the
  // queues size their stall reserve as stallQuiescenceCycles x this bound and
  // assert the bound every cycle.
  val maxPacketsPerCycle: Option[Int] = None,
  // Lossy mode hysteresis: after a Pause, Resume only once the packet queues
  // hold at most this many entries. Resuming as soon as one slot frees would
  // flap, and each Pause/Resume pair costs more bytes than the data it brackets.
  val resumeWatermark: Int = -1,
)(implicit p: Parameters)
    extends LazyTraceEncoder(coreParams)(p) {
  val resumeWatermarkEntries: Int = if (resumeWatermark < 0) bufferDepth / 4 else resumeWatermark
  require(resumeWatermarkEntries >= 0 && resumeWatermarkEntries < bufferDepth,
    s"resumeWatermark=$resumeWatermarkEntries must be in [0, bufferDepth=$bufferDepth)")
  // Occupancy above which a packet-bearing group can no longer be admitted, i.e.
  // where lossy mode starts dropping. Mirrors the queue's can_accept bound.
  // The runtime watermark is clamped below this: at or above it a gap would end
  // on the cycle it began.
  val dropThresholdEntries: Int = bufferDepth - maxPacketsPerCycle.getOrElse(coreParams.nGroups)
  require(dropThresholdEntries >= 1,
    s"bufferDepth=$bufferDepth leaves no room above the enqueue bound " +
    s"${maxPacketsPerCycle.getOrElse(coreParams.nGroups)}")
  require(resumeWatermarkEntries < dropThresholdEntries,
    s"resumeWatermark=$resumeWatermarkEntries must sit below the drop threshold " +
    s"$dropThresholdEntries, else a gap ends on the cycle it begins")
  override lazy val module = new TacitParallelEncoderModule(this)
}

class TacitParallelEncoderModule(outer: TacitParallelEncoder) extends LazyTraceEncoderModule(outer) with MetaDataWidthHelper {

  val coreParams = outer.coreParams
  val MAX_DELTA_TIME_COMP = 0x3F // 63, 6 bits

  // states
  //   sPausePending: a group was dropped; its Pause is latched and waits for queue space
  //   sPaused:       inside a gap, everything is dropped until the low watermark
  //   sSync:         emitting Start / End / Resume, bound to ingress_0 (sync_type selects)
  val sIdle :: sPausePending :: sPaused :: sSync :: sData :: Nil = Enum(5)
  val state = RegInit(sIdle)
  val sync_type = RegInit(SyncType.SyncNone)
  val encode_sync = Wire(Bool())
  val prev_time = Reg(UInt(coreParams.xlen.W))
  val lossy = io.control.lossy

  // Pause binding, latched from ingress_1 in the cycle its group is dropped
  val pause_pc = Reg(UInt(coreParams.iaddrWidth.W))
  val pause_time = Reg(UInt(coreParams.xlen.W))
  val pause_prv = Reg(UInt(4.W))
  val pause_ctx = Reg(UInt(coreParams.xlen.W))
  // packets discarded since the Pause (saturating); reported in Resume's trap_addr
  val dropped = RegInit(0.U(32.W))

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
  // Each buffer must absorb the packets still in flight after io.stall asserts:
  // stallQuiescenceCycles (stall->commit-gate propagation + ingress_0/ingress_1),
  // each cycle worth up to nGroups enqueues. The queue owns this unit conversion.
  // (coreStages is NOT used here: for BOOM it was calibrated as an element budget
  // for the legacy count threshold, not a cycle count.)
  // Legacy impls derive io.stall from count instead and never honor the reserve,
  // so don't let it constrain their elaboration (e.g. stress configs with tiny buffers).
  // +1: the queue's stall is computed from occupancy at the start of the cycle,
  // before that cycle's enqueue lands, so with exactly stallQuiescenceCycles worth
  // free it stays low one cycle too long and lets one more group commit. That group
  // is not lost (it waits in ingress_1 while the core is stalled) but the reserve
  // contract is only met if the threshold is one enqueue-cycle higher.
  val bufferReserveCycles = outer.queueImpl match {
    case MPQueueImpl.SRAM => outer.stallQuiescenceCycles + 1
    case _ => 0
  }
  val metadata_buffer = MultiPortedQueue(new MetaDataBundle(coreParams), outer.bufferDepth, coreParams.nGroups, outer.queueImpl, bufferReserveCycles, outer.maxPacketsPerCycle)
  val message_packet_buffer = MultiPortedQueue(new MessagePacketBundle(coreParams), outer.bufferDepth, coreParams.nGroups, outer.queueImpl, bufferReserveCycles, outer.maxPacketsPerCycle)
  val header_buffer = MultiPortedQueue(UInt(8.W), outer.bufferDepth, coreParams.nGroups, outer.queueImpl, bufferReserveCycles, outer.maxPacketsPerCycle)

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

  // ---------------- lossy mode ----------------
  // Which ingress_1 slots carry a packet (lane compaction packs by iretire, not by message).
  val ingress_1_msg_mask = VecInit(ingress_1.group.map(g => g.iretire === 1.U && g.itype =/= TraceItype.ITNothing))
  val ingress_1_has_message = ingress_1_msg_mask.asUInt.orR
  val ingress_1_msg_idx = PriorityEncoder(ingress_1_msg_mask)
  val ingress_1_msg_count = PopCount(ingress_1_msg_mask)
  // Every retired slot, message-carrying or not. Perf accounting only: these are
  // lost with the group but never had a packet to drop, so they are deliberately
  // absent from dropped_inc and from the Resume payload.
  val ingress_1_insn_count = PopCount(ingress_1.group.map(_.iretire === 1.U))

  // Drop decision. NOTE the enqueue valids below are gated on !lossy_drop: with a
  // programmable threshold the queue may still have physical room on the cycle we
  // decide to drop, so without that gate the group would be enqueued AND counted
  // as dropped. Before the threshold existed, !all_buffers_ready made this implicit.
  // No reserve is needed (unlike stall, which must absorb in-flight traffic):
  // the decision takes effect this cycle and the Pause itself waits for space.
  // Runtime drop threshold, absolute in queue entries. 0 selects the physical
  // bound so an unprogrammed encoder is unchanged; a lower value emulates a
  // shallower queue. Clamped to the physical bound -- it can only ever emulate
  // SMALLER, never claim entries that do not exist, and !all_buffers_ready stays
  // a hard floor below.
  val thresh_max = outer.dropThresholdEntries.U
  val thresh_req = io.control.drop_threshold
  val thresh_eff = Mux(thresh_req === 0.U || thresh_req > thresh_max, thresh_max, thresh_req)
  io.perf.drop_threshold_eff := thresh_eff
  val over_threshold = metadata_buffer.io.count > thresh_eff

  val lossy_drop = state === sData && io.control.enable && lossy &&
                   ingress_1_has_message && !sent &&
                   (!all_buffers_ready || over_threshold)
  // Runtime resume watermark, absolute in queue entries. 0 selects the
  // elaboration default so an unprogrammed encoder is unchanged; anything at or
  // above the drop threshold is clamped, since resuming there would end the gap
  // on the cycle it started and emit a Pause/Resume pair per dropped group.
  // Clamping rather than asserting: this register is software-writable, and a
  // synthesized assertion would turn a driver typo into a dead simulation.
  val wm_max = thresh_eff - 1.U
  val wm_req = io.control.resume_watermark
  val wm_sel = Mux(wm_req === 0.U, outer.resumeWatermarkEntries.U, wm_req)
  val resume_wm = Mux(wm_sel > wm_max, wm_max, wm_sel)
  val low_wm = metadata_buffer.io.count <= resume_wm

  // A group is lost when it leaves ingress_1 un-encoded: in the drop cycle, while
  // the Pause is pending, inside the gap, and while Resume is being emitted (that
  // group is older than the Resume bind point). Each group leaves exactly once.
  val resuming = state === sSync && sync_type === SyncType.SyncResume
  // Single source of truth for "the encoder is blind this cycle". drop_now and
  // io.perf.paused MUST come from the same expression: they previously carried
  // four terms and two respectively, so gap_cycles omitted both the lossy_drop
  // cycle and the entire Resume phase -- cycles in which groups were being
  // discarded -- and undercounted every gap.
  val in_gap = lossy_drop || state === sPausePending || state === sPaused || resuming
  val drop_now = pipeline_advance && in_gap
  val dropped_inc = Mux(drop_now, ingress_1_msg_count, 0.U)
  val dropped_sum = dropped +& dropped_inc
  val dropped_sat = Mux(dropped_sum(32), ~0.U(32.W), dropped_sum(31, 0))
  // Resume may enqueue before the ingress_1 group leaves; it will be dropped, count it.
  val dropped_for_packet = (dropped +& Mux(ingress_1_has_message, ingress_1_msg_count, 0.U))
  val dropped_for_packet_sat = Mux(dropped_for_packet(32), ~0.U(32.W), dropped_for_packet(31, 0))
  // Perf only: same drop_now predicate, wider mask. Feeds no packet field.
  val dropped_insns_inc = Mux(drop_now, ingress_1_insn_count, 0.U)

  // Privilege fields of trap packets. BOOM tags a retire group with RegNext(status.prv):
  // sret/mret/ecall execute in the CSR unit before they commit, so their groups arrive
  // already tagged with the NEW privilege, while faults update the CSR after commit and
  // arrive tagged with the old one. So the next group's tag (ingress_0.priv) is the
  // trap's target privilege unless that group is itself a CSR-executed trap (an
  // exception or return), in which case its tag is already past its own transition
  // and the current trap group's tag is the target instead. That fallback is right
  // only when the current trap is CSR-executed too. An interrupt next is excluded:
  // it is taken before its instruction executes, so its tag is never past its own
  // transition and ingress_0.priv stays the target. A fault or interrupt followed
  // directly by an exception can still be mislabeled.
  // from_priv is the privilege the encoder has tracked since the last sync, advanced
  // by each trap's target, so the chain of packets is self-consistent whatever the
  // core's sampling does.
  def group_has_trap(g: TraceCoreInterface): Bool = g.group.map(x => x.iretire === 1.U &&
    (x.itype === TraceItype.ITException || x.itype === TraceItype.ITInterrupt || x.itype === TraceItype.ITReturn)).reduce(_ || _)
  def group_has_csr_trap(g: TraceCoreInterface): Bool = g.group.map(x => x.iretire === 1.U &&
    (x.itype === TraceItype.ITException || x.itype === TraceItype.ITReturn)).reduce(_ || _)
  val ingress_1_has_trap = group_has_trap(ingress_1)
  val ingress_0_has_csr_trap = group_has_csr_trap(ingress_0)
  val trap_to_prv = Mux(ingress_0_has_csr_trap, ingress_1.priv, ingress_0.priv)
  val enc_prv = RegInit(3.U(4.W))   // machine mode at reset; every Start/Resume re-seeds it
  when (pipeline_advance && ingress_1_has_trap) {
    enc_prv := trap_to_prv          // the transition happened whether or not its packet was encoded
  }

  // Sync payload: Start/End/Resume bind to ingress_0 (next covered group);
  // Pause binds to the latched lost group.
  val pausing = state === sPausePending
  val sync_type_now = Mux(pausing, SyncType.SyncPause, sync_type)
  val sync_pc = Mux(pausing, pause_pc, ingress_0.group(0).iaddr)
  val sync_prv = Mux(pausing, pause_prv, ingress_0.priv)
  val sync_ctx = Mux(pausing, pause_ctx, ingress_0.ctx)
  val sync_trap_addr = Wire(UInt(coreParams.iaddrWidth.W))
  sync_trap_addr := Mux(resuming, dropped_for_packet_sat, 0.U) // Start's runtime_cfg is 0 for this encoder

  val sync_enq_fire = metadata_buffer.io.enqs(0).fire
  when (sync_enq_fire && encode_sync && (sync_type_now === SyncType.SyncStart || sync_type_now === SyncType.SyncResume)) {
    enc_prv := sync_prv             // the decoder's privilege is whatever the sync declared
  }

  for (i <- 0 until coreParams.nGroups) {
    val message_encoder = Module(new MessageEncoder(coreParams, canEncodeSyncMessage = i == 0, my_index = i))
    if (i == 0) { 
      message_encoder.io.encode_sync.get := encode_sync 
      message_encoder.io.sync_type.get := sync_type_now
      message_encoder.io.sync_pc.get := sync_pc
      message_encoder.io.sync_prv.get := sync_prv
      message_encoder.io.sync_ctx.get := sync_ctx
      message_encoder.io.sync_trap_addr.get := sync_trap_addr
    }
    message_encoder.io.ingress := ingress_1 // pass in all groups, irrelevant ones will be optimized out
    message_encoder.io.ingress_0_target_addr_msg := ingress_0.group(0).iaddr // backup in case this is the last valid ingress
    message_encoder.io.target_prv_msg := trap_to_prv
    message_encoder.io.from_prv_msg := enc_prv
    // Data packets only in sData. Lane 0 additionally carries every sync
    // (Start/End/Resume in sSync, Pause in sPausePending) via encode_sync.
    message_encoder.io.ingress_valid := ingress_1.group(i).iretire === 1.U &&
      (if (i==0) (state === sData || state === sSync || state === sPausePending) else (state === sData))

    metadata_enq_bits(i) := message_encoder.io.metadata
    metadata_enq_bits(i).time := Mux(is_first_valid(i), time_encoder.io.output_mask, 1.U)
    val time_can_be_compressed = Mux(is_first_valid(i), delta_time < MAX_DELTA_TIME_COMP.U, true.B)
    val is_compressed = message_encoder.io.possible_to_compress && time_can_be_compressed
    metadata_enq_bits(i).is_full := ~is_compressed
    metadata_buffer.io.enqs(i).bits := metadata_enq_bits(i)
    metadata_buffer.io.enqs(i).valid := message_encoder.io.packet_valid && !sent && all_buffers_ready && !lossy_drop

    message_packet_enq_bits(i) := message_encoder.io.message
    val zero_varlen_bytes = VecInit.fill(time_encoder.maxNumBytes)(0.U(8.W))
    zero_varlen_bytes(0) := 0x80.U(8.W) // signify a varlen encoded zero
    message_packet_enq_bits(i).time := Mux(is_first_valid(i), time_encoder.io.output_bytes, zero_varlen_bytes)
    message_packet_buffer.io.enqs(i).bits := message_packet_enq_bits(i)
    message_packet_buffer.io.enqs(i).valid := message_encoder.io.packet_valid && !is_compressed && !sent && all_buffers_ready && !lossy_drop

    val compressed_packet = Cat(Mux(is_first_valid(i), delta_time(5,0), 0.U(6.W)), message_encoder.io.comp_header)
    header_buffer.io.enqs(i).bits := Mux(is_compressed, compressed_packet, message_encoder.io.full_header)
    header_buffer.io.enqs(i).valid := message_encoder.io.packet_valid && !sent && all_buffers_ready && !lossy_drop
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
  encode_sync := state === sSync || pausing
  time_encoder.io.input_valid := false.B
  time_encoder.io.input_value := DontCare
  dropped := dropped_sat

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
      when (pipeline_advance && (sent || sync_enq_fire)) {
        when (io.control.enable) {
          state := sData
        } .elsewhen (sync_type === SyncType.SyncResume) {
          // disabled while paused: Resume then End, never End directly after Pause
          state := sSync
          sync_type := SyncType.SyncEnd
        } .otherwise {
          state := sIdle
        }
        when (sync_type === SyncType.SyncResume) { dropped := 0.U }
      }
    }
    is (sData) {
      // Data packets are still encoded from ingress_1 in the cycle disable is
      // observed, so the time field must be driven unconditionally here (it
      // used to sit under .otherwise, producing a time-less packet at disable).
      time_encoder.io.input_value := delta_time
      time_encoder.io.input_valid := true.B
      when (!io.control.enable) {
        // Leave for End only once the group in ingress_1 has been encoded (or has
        // nothing to encode): with a full queue at disable time it may still be
        // waiting, and sSync would otherwise discard it. In lossless mode the core
        // is stalled meanwhile, so no further group arrives.
        when (pipeline_advance && (sent || !ingress_1_has_message)) {
          state := sSync
          sync_type := SyncType.SyncEnd
        }
      } .elsewhen (lossy_drop) {
        pause_pc := ingress_1.group(ingress_1_msg_idx).iaddr
        pause_time := ingress_1.time
        pause_prv := ingress_1.priv
        pause_ctx := ingress_1.ctx
        state := sPausePending
      }
    }
    is (sPausePending) {
      time_encoder.io.input_value := pause_time
      time_encoder.io.input_valid := true.B
      when (sync_enq_fire) { state := sPaused }
    }
    is (sPaused) {
      when (low_wm) {
        state := sSync
        sync_type := SyncType.SyncResume
      }
    }
  }

  // Lossless mode must never lose a group: a packet-bearing group leaving ingress_1
  // un-encoded means the stall reserve was insufficient for the core's quiescence.
  val group_enqueued_now = metadata_buffer.io.enqs.map(_.fire).reduce(_ || _)
  assert(!(state === sData && io.control.enable && !lossy && pipeline_advance &&
           ingress_1_has_message && !sent && !group_enqueued_now && !all_buffers_ready),
    "lossless mode: retire group left ingress_1 without being encoded (stall reserve too small)")
  assert(!(state === sData && io.control.enable && !lossy && pipeline_advance &&
           ingress_1_has_message && !sent && !group_enqueued_now && all_buffers_ready),
    "lossless mode: retire group left ingress_1 without being encoded although the queues were ready")

  // software contract: lossy is written before enable and never flipped mid-run
  assert(state === sIdle || lossy === RegNext(lossy), "control.lossy changed while the encoder is active")
  assert(!(state === sPaused && metadata_buffer.io.enqs.map(_.fire).reduce(_ || _)), "enqueue inside a gap")
  assert(!(lossy && io.stall), "stall asserted to the core in lossy mode")

  val stall = Wire(Bool())
  outer.queueImpl match {
    case MPQueueImpl.SRAM =>
      stall := metadata_buffer.io.stall ||
               message_packet_buffer.io.stall ||
               header_buffer.io.stall
    case _ =>
      // legacy stall policy, preserved verbatim so existing configs/bitstreams
      // (e.g. ongoing TraceDoctor experiments) elaborate unchanged
      def stallThreshold(count: UInt) = count >= (outer.bufferDepth - outer.coreStages).U
      stall := stallThreshold(metadata_buffer.io.count) ||
               stallThreshold(message_packet_buffer.io.count) ||
               stallThreshold(header_buffer.io.count)
  }
  io.stall := stall && !lossy
  io.perf.full := stall // in lossy mode: cycles the core would have been stalled
  io.perf.paused := in_gap
  io.perf.pause_fire := pausing && sync_enq_fire
  io.perf.dropped_inc := dropped_inc
  io.perf.dropped_insns_inc := dropped_insns_inc
  io.perf.resume_watermark_eff := resume_wm
}

class MessageEncoder(
  val coreParams: TraceCoreParams, 
  val canEncodeSyncMessage: Boolean,
  val my_index: Int,
) extends Module with MetaDataWidthHelper {
  val io = IO(new Bundle {
    val encode_sync = if (canEncodeSyncMessage) Some(Input(Bool())) else None
    val sync_type = if (canEncodeSyncMessage) Some(Input(SyncType())) else None
    // sync payload, bound by the parent (ingress_0 for Start/End/Resume, latched group for Pause)
    val sync_pc = if (canEncodeSyncMessage) Some(Input(UInt(coreParams.iaddrWidth.W))) else None
    val sync_prv = if (canEncodeSyncMessage) Some(Input(UInt(4.W))) else None
    val sync_ctx = if (canEncodeSyncMessage) Some(Input(UInt(coreParams.xlen.W))) else None
    // trap_addr position: runtime_cfg (Start), 0 (End, Pause), dropped packets (Resume)
    val sync_trap_addr = if (canEncodeSyncMessage) Some(Input(UInt(coreParams.iaddrWidth.W))) else None
    val ingress = Input(new TraceCoreInterface(coreParams))
    val ingress_0_target_addr_msg = Input(UInt(coreParams.iaddrWidth.W))
    val target_prv_msg = Input(UInt(4.W))
    val from_prv_msg = Input(UInt(4.W))
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
      prv_encoder.io.from_priv := io.from_prv_msg
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
      prv_encoder.io.from_priv := io.from_prv_msg
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
      prv_encoder.io.from_priv := io.from_prv_msg
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
      target_addr_encoder.io.input_value := io.sync_pc.get >> 1.U
      encode_target_addr_valid := true.B
      prv_encoder.io.from_priv := 0b00.U
      prv_encoder.io.to_priv := io.sync_prv.get
      encode_prv_valid := true.B
      trap_addr_encoder.io.input_value := io.sync_trap_addr.get
      encode_trap_addr_valid := true.B
      ctx_encoder.io.input_value := io.sync_ctx.get
      encode_ctx_valid := true.B
      possible_to_compress := false.B
    }
  }
}
