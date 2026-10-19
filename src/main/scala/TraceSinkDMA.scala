package tacit

import chisel3._
import chisel3.util._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.diplomacy._
import org.chipsalliance.cde.config.{Parameters, Config, Field}
import freechips.rocketchip.subsystem._
import freechips.rocketchip.prci._
import freechips.rocketchip.regmapper.{RegField, RegFieldDesc}
import freechips.rocketchip.resources.{Resource, ResourceBinding, ResourceReference}
import freechips.rocketchip.tile._
import shuttle.common.{ShuttleTile, ShuttleTileAttachParams}
import freechips.rocketchip.trace._
import testchipip.soc.{SubsystemInjector, SubsystemInjectorKey}

/** Takes a Bool and forces it to deassert after pulseLength cycles by using Chisel last-connect semantics, effectively
  * "stretching" the pulse.
  *
  * @param in
  *   The wire to override
  *
  * @param pulseLength
  *   the number of cycles to stretch the pulse over.
  */
// copied from midas.widgets.Pulsify
object Pulsify {
  def apply(in: Bool, pulseLength: Int): Unit = {
    require(pulseLength > 0)
    if (pulseLength > 1) {
      val count = Counter(pulseLength)
      when(in) { count.inc() }
      when(count.value === (pulseLength - 1).U) {
        in          := false.B
        count.value := 0.U
      }
    } else {
      when(in) { in := false.B }
    }
  }
}

/** Bus the trace DMA masters attach to. MBUS writes go below the L2 and can only
  * reach off-chip memory; FBUS writes enter at the top of the SBUS and can reach
  * any SBUS slave (scratchpad, TCM) or DRAM through the L2. */
case object TraceSinkDMABusKey extends Field[TLBusWrapperLocation](MBUS)

class WithTraceSinkDMABus(where: TLBusWrapperLocation) extends Config((site, here, up) => {
  case TraceSinkDMABusKey => where
})

/** TileLink source IDs (outstanding writes) per DMA sink when not given explicitly. */
case object TraceSinkDMASourcesKey extends Field[Int](16)

case class TraceSinkDMAParams(
  regNodeBaseAddr: BigInt,
  beatBytes: Int,
  // None: use TraceSinkDMASourcesKey
  nSource: Option[Int] = None,
  // Buses the engine can master, one TileLink port each, chosen at run time by the
  // bus_select register (index into this list). None: Seq(TraceSinkDMABusKey).
  buses: Option[Seq[TLBusWrapperLocation]] = None,
  // Bytes per multi-beat Put. One header per burst instead of per beat is what
  // makes the DMA cheap on narrow links (serial TL). beatBytes disables bursting.
  burstBytes: Int = 64,
  // Flush a partial burst as single beats after this many cycles without new data,
  // so the tail of a trace window (SyncEnd) is not stranded in the buffer. Long, so
  // that ordinary mid-trace pauses (a core waiting on a miss over a slow link) do
  // not break bursts; software waits this long after disabling tracing.
  flushCycles: Int = 4096
)

object DMAMode {
  val overflow = 0
  val ringBuffer = 1
}

class TraceSinkDMA(params: TraceSinkDMAParams, hartId: Int)(implicit p: Parameters) extends LazyTraceSink {
  val numSources = params.nSource.getOrElse(p(TraceSinkDMASourcesKey))
  val burstBytes = params.burstBytes
  val flushCycles = params.flushCycles
  val buses: Seq[TLBusWrapperLocation] = params.buses.getOrElse(Seq(p(TraceSinkDMABusKey)))
  require(buses.nonEmpty && buses.distinct.size == buses.size, s"trace DMA buses must be non-empty and distinct: $buses")
  // One TileLink port per bus; they share the engine, its buffer and its source IDs.
  val dmaNodes = buses.map { _ => TLClientNode(Seq(TLMasterPortParameters.v1(Seq(TLClientParameters(
    name = "trace-sink-dma", sourceId = IdRange(0, numSources)))))) }

  val device = new SimpleDevice(s"trace-sink-dma$hartId", Seq("ucbbar,tracesinkdma"))
  val regnode = TLRegisterNode(
    address = Seq(AddressSet(params.regNodeBaseAddr, 0xFFF)),
    device = device,
    beatBytes = params.beatBytes
  )

  lazy val module = new TraceSinkDMAImpl(this)
  class TraceSinkDMAImpl(outer: TraceSinkDMA) extends LazyTraceSinkModuleImp(outer) {
    val ports = outer.dmaNodes.map(_.out(0))
    val mems = ports.map(_._1)
    val edges = ports.map(_._2)
    require(edges.map(_.bundle.dataBits).distinct.size == 1, "trace DMA ports must share a data width")
    val busWidth = edges.head.bundle.dataBits
    val nPorts = ports.size
    val selBits = log2Ceil(nPorts).max(1)
    // Which port carries writes. Software rule: soft-reset the DMA before changing it,
    // so nothing is in flight on the old port (asserted below).
    val bus_select = RegInit(0.U(selBits.W))
    def onSelected[T <: Data](xs: Seq[T]): T = if (nPorts == 1) xs.head else VecInit(xs)(bus_select)

    val beatBytes = busWidth / 8
    val burstBytes = outer.burstBytes
    require(isPow2(burstBytes) && burstBytes >= beatBytes,
      s"burstBytes=$burstBytes must be a power of two >= the bus beat ($beatBytes B)")
    val beatsPerBurst = burstBytes / beatBytes
    val bursting = burstBytes > beatBytes
    val lgBeat = log2Ceil(beatBytes)
    val lgBurst = log2Ceil(burstBytes)

    val serialWidthAggregator = Module(new SerialWidthAggregator(busWidth))
    serialWidthAggregator.io.narrow <> io.trace_in

    // Beats wait here until a whole burst is available, so a multi-beat Put never
    // holds the bus mid-message waiting for trace data.
    val beatQ = Module(new Queue(UInt(busWidth.W), 2 * beatsPerBurst, hasFlush = true))
    beatQ.io.enq <> serialWidthAggregator.io.wide
    beatQ.io.flush.get := false.B

    val mIdle :: mWrite :: mOverflow :: Nil = Enum(3)
    val mstate = RegInit(mIdle)

    // tracks how much trace data have we written in total
    val addr_counter = RegInit(0.U(64.W))
    // max permissible size of the trace data to write in bytes before we overflow/wrap
    val max_size_reg = RegInit(((BigInt(1) << 64) - 1).U(64.W))
    val addr_full = addr_counter + beatBytes.U > max_size_reg

    val dma_start_addr = RegInit(0.U(64.W))

    // control registers
    // software reset the DMA engine to clear all the status registers
    val reset_reg = RegInit(false.B)
    // mode selection register, ring buffer mode or overflow mode
    val mode_reg = RegInit(DMAMode.overflow.U) // default to overflow mode

    // status registers
    // tracks how many times the address counter wrapped around
    val wrap_count = RegInit(0.U(32.W))

    // Burst or single beat? A burst needs a full burst buffered, an aligned address,
    // room before max_size, and a destination that takes a Put of that size (the
    // edge's legality check, so a narrower slave just gets single beats). Single
    // beats realign after a partial flush, finish the buffer end (overflow/wrap
    // still step per beat) and flush a stale tail.
    val write_base = dma_start_addr + addr_counter
    val burst_aligned = if (bursting) write_base(lgBurst - 1, 0) === 0.U else true.B
    val burst_fits = addr_counter + burstBytes.U <= max_size_reg
    val burst_legal = onSelected(edges.map(_.Put(0.U, write_base, lgBurst.U, 0.U, Fill(beatBytes, 1.U(1.W)))._1))
    val have_burst = beatQ.io.count >= beatsPerBurst.U

    val idle_cycles = RegInit(0.U(log2Ceil(outer.flushCycles + 1).W))
    when (beatQ.io.enq.fire) {
      idle_cycles := 0.U
    } .elsewhen (idle_cycles =/= outer.flushCycles.U) {
      idle_cycles := idle_cycles + 1.U
    }
    val tail_stale = idle_cycles === outer.flushCycles.U

    val do_burst = bursting.B && burst_aligned && burst_fits && burst_legal && have_burst
    val do_single = beatQ.io.deq.valid && !do_burst &&
      (!bursting.B || !burst_aligned || !burst_fits || !burst_legal || tail_stale)

    val sourceBits = log2Ceil(outer.numSources).max(1)
    val sourceGen = Module(new SourceGenerator(sourceBits))
    val sourceReady = sourceGen.io.id.valid
    val src_ready_stall_count = RegInit(0.U(32.W))
    // Only count cycles the sink was actually held up: without the valid
    // qualifier this also counted idle cycles that merely happened to have no
    // free source ID, making the count an upper bound on real sink pressure.
    when (!sourceReady && beatQ.io.deq.valid) {
      src_ready_stall_count := src_ready_stall_count + 1.U
    }

    val ring_wrap = mode_reg === DMAMode.ringBuffer.U && addr_full && beatQ.io.deq.valid
    val start_write = mstate === mIdle && !addr_full && sourceReady && (do_burst || do_single)
    sourceGen.io.gen := start_write
    val d_fires = mems.map(_.d.fire)
    sourceGen.io.reclaim.valid := d_fires.reduce(_ || _)
    sourceGen.io.reclaim.bits := PriorityMux(d_fires, mems.map(_.d.bits.source))

    val latched_source = Reg(UInt(sourceBits.W))
    val latched_addr = Reg(UInt(64.W))
    val latched_burst = Reg(Bool())
    val beat_cnt = RegInit(0.U(log2Ceil(beatsPerBurst + 1).W))
    when (start_write) {
      latched_source := sourceGen.io.id.bits
      latched_addr := write_base
      latched_burst := do_burst
      beat_cnt := 0.U
    }

    // putting the buffer data on the selected TL port; address and size are
    // constant across the beats of a burst
    mems.zip(edges).zipWithIndex.foreach { case ((m, e), i) =>
      m.a.bits := e.Put(
        fromSource = latched_source,
        toAddress = latched_addr,
        lgSize = Mux(latched_burst, lgBurst.U, lgBeat.U),
        data = beatQ.io.deq.bits,
        mask = Fill(beatBytes, 1.U(1.W)))._2
      m.a.valid := mstate === mWrite && beatQ.io.deq.valid && bus_select === i.U
      m.d.ready := true.B
    }
    val a_fire = onSelected(mems.map(_.a.fire))
    beatQ.io.deq.ready := (mstate === mWrite && onSelected(mems.map(_.a.ready))) || mstate === mOverflow

    val last_beat = !latched_burst || beat_cnt === (beatsPerBurst - 1).U
    val write_bytes = Mux(latched_burst, burstBytes.U, beatBytes.U)

    // assertions
    // are we ever writing out of bounds?
    val write_end_excl = (latched_addr - dma_start_addr) + write_bytes
    assert(!(a_fire && (write_end_excl > max_size_reg)), "DMA is writing out of bounds")
    assert(!(mstate === mWrite && latched_burst && !beatQ.io.deq.valid), "DMA burst ran out of buffered beats")
    mems.foreach { m =>
      assert(!(m.d.fire && m.d.bits.denied),  "DMA got denied D response")
      assert(!(m.d.fire && m.d.bits.corrupt), "DMA got corrupt D response")
    }
    assert(PopCount(d_fires) <= 1.U, "DMA got responses on two ports at once (bus_select changed without a reset)")
    assert(!(bus_select =/= RegNext(bus_select) && (sourceGen.io.inflight || mstate === mWrite)),
      "DMA bus_select changed with writes in flight: soft-reset the DMA before switching buses")

    switch(mstate) {
      is (mIdle) {
        when (mode_reg === DMAMode.overflow.U && addr_full && beatQ.io.deq.valid) {
          mstate := mOverflow
        } .elsewhen (ring_wrap) {
          // wrap only; the next write is decided from the new address next cycle
          wrap_count := wrap_count + 1.U
          addr_counter := 0.U
        } .elsewhen (start_write) {
          mstate := mWrite
        }
      }
      is (mWrite) {
        when (a_fire) {
          beat_cnt := beat_cnt + 1.U
          when (last_beat) {
            mstate := mIdle
            addr_counter := addr_counter + write_bytes
          }
        }
      }
      is (mOverflow) {
        // always accept the upstream incoming packets
        // but never write to the memory
        // software reset is the only way to clear the overflow state
      }
    }

    // A TileLink message cannot be abandoned once started, so a software reset
    // that lands mid-write takes effect when the write completes.
    val reset_pending = RegInit(false.B)
    when (reset_reg) { reset_pending := true.B }
    when ((reset_reg || reset_pending) && mstate =/= mWrite) {
      mstate := mIdle
      addr_counter := 0.U
      src_ready_stall_count := 0.U
      reset_pending := false.B
      beatQ.io.flush.get := true.B
    }
    Pulsify(reset_reg, 1)

    val regmap = regnode.regmap(
      Seq(
        0x00 -> Seq(
          RegField(64, dma_start_addr,
            RegFieldDesc("dma_start_addr", "DMA start address"))
        ),
        0x08 -> Seq(RegField.r(64, addr_counter,
            RegFieldDesc("addr_counter", "Address counter, this is the number of bytes written to the memory to date"))
        ),
        0x10 -> Seq(RegField(64, max_size_reg, 
          RegFieldDesc("max_size_reg", "Max size register, this is the max number of bytes allowed to write before we overflow"))
        ),
        0x18 -> Seq(RegField(1, reset_reg, 
          RegFieldDesc("reset_reg", "Soft reset register")),
          ),
        0x1c -> Seq(RegField(1, mode_reg,
          RegFieldDesc("mode_reg", "Mode selection register, 0 for overflow mode, 1 for ring buffer mode")),
        ),
        0x20 -> Seq(RegField(32, wrap_count,
          RegFieldDesc("wrap_count", "Wrap count, this is the number of times the address counter wrapped around"))
        ),
        0x24 -> Seq(RegField(32, src_ready_stall_count,
          RegFieldDesc("sourceReadyStalledCount", "Source ready stalled count, this is the number of times the source was not ready"))
        )
      ) ++ (if (nPorts > 1) Seq(
        0x28 -> Seq(RegField(selBits, bus_select,
          RegFieldDesc("bus_select", s"Bus the DMA writes through: ${outer.buses.zipWithIndex.map { case (b, i) => s"$i=${b.name}" }.mkString(", ")}. Soft-reset before changing.")))
      ) else Nil):_*
    )
  }
}

/** Adds a DMA trace sink to every traced tile as sink `targetId`. With several
  * `buses` the one engine gets a TileLink port on each (e.g. FBUS, which reaches the
  * scratchpad or DRAM through the L2, and MBUS, which bypasses the L2), chosen at
  * run time by its bus_select register. */
class WithTraceSinkDMA(
  targetId: Int = 1,
  nSource: Option[Int] = None,
  buses: Option[Seq[TLBusWrapperLocation]] = None,
  regBase: BigInt = 0x3010000
) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: RocketTileAttachParams => {
      // redefine tile level constants
      val xBytes = tp.tileParams.core.xLen / 8
      tp.copy(tileParams = tp.tileParams.copy(
        traceParams = Some(tp.tileParams.traceParams.get.copy(buildSinks = 
          tp.tileParams.traceParams.get.buildSinks :+ (p => 
            (LazyModule(new TraceSinkDMA(TraceSinkDMAParams(
            regNodeBaseAddr = regBase + tp.tileParams.tileId * 0x1000,
            beatBytes = xBytes, nSource = nSource, buses = buses), hartId = tp.tileParams.tileId)(p)), targetId)))))
      )
    }
    case tp: ShuttleTileAttachParams => {
      val xBytes = tp.tileParams.core.xLen / 8
      tp.copy(tileParams = tp.tileParams.copy(
        traceParams = Some(tp.tileParams.traceParams.get.copy(buildSinks = 
          tp.tileParams.traceParams.get.buildSinks :+ (p => 
            (LazyModule(new TraceSinkDMA(TraceSinkDMAParams(
            regNodeBaseAddr = regBase + tp.tileParams.tileId * 0x1000,
            beatBytes = tp.tileParams.tileBeatBytes, nSource = nSource, buses = buses), hartId = tp.tileParams.tileId)(p)), targetId)))))
      )
    }
    case other => other
  }
  case SubsystemInjectorKey => up(SubsystemInjectorKey) + TraceSinkDMAInjector
})

case object TraceSinkDMAInjector extends SubsystemInjector((p, baseSubsystem) => {
  require(baseSubsystem.isInstanceOf[BaseSubsystem with InstantiatesHierarchicalElements])
  val hierarchicalSubsystem = baseSubsystem.asInstanceOf[BaseSubsystem with InstantiatesHierarchicalElements]
  implicit val q: Parameters = p
  val traceEncoderDmaBindings = hierarchicalSubsystem.totalTiles.values.flatMap {
    case r: RocketTile =>
      val traceSinkDmas = r.trace_sinks.collect { case dma: TraceSinkDMA => dma }
      if (r.trace_encoder_controller.isDefined || traceSinkDmas.nonEmpty) {
        require(r.trace_encoder_controller.isDefined, s"tile ${r.tileId} has trace DMA sink but no trace encoder controller")
        Some((r, r.trace_encoder_controller.get, traceSinkDmas))
      } else {
        None
      }
    case s: ShuttleTile =>
      val traceSinkDmas = s.trace_sinks.collect { case dma: TraceSinkDMA => dma }
      if (s.trace_encoder_controller.isDefined || traceSinkDmas.nonEmpty) {
        require(s.trace_encoder_controller.isDefined, s"tile ${s.tileId} has trace DMA sink but no trace encoder controller")
        Some((s, s.trace_encoder_controller.get, traceSinkDmas))
      } else {
        None
      }
    case _ => None
  }
  if (traceEncoderDmaBindings.nonEmpty) {
    traceEncoderDmaBindings.foreach { case (t, c, dmas) =>
      require(dmas.size == 1, s"tile ${t.tileId} must have exactly one trace DMA sink, found ${dmas.size}")
      dmas.foreach { s =>
        t { // in the implicit clock domain of tile
          ResourceBinding {
            Resource(c.device, "ucbbar,trace-dma").bind(ResourceReference(s.device.label))
          }
          s.buses.zip(s.dmaNodes).foreach { case (where, node) =>
            val dmaBus = baseSubsystem.locateTLBusWrapper(where)
            dmaBus.coupleFrom(t.tileParams.baseName) { bus =>
              bus := dmaBus.crossOut(node)(ValName("trace_sink_dma"))(AsynchronousCrossing())
            }
          }
          t match {
            case shuttleTile: ShuttleTile => shuttleTile.connectTLSlaveAtTileBeatBytes(s.regnode)
            case _ => t.connectTLSlave(s.regnode, t.xBytes)
          }
        }
      }
    }
  }
})
