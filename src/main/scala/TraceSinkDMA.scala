package tacit

import chisel3._
import chisel3.util._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.diplomacy._
import org.chipsalliance.cde.config.{Parameters, Config}
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

case class TraceSinkDMAParams(
  regNodeBaseAddr: BigInt,
  beatBytes: Int,
  nSource: Int = 16
)

object DMAMode {
  val overflow = 0
  val ringBuffer = 1
}

class TraceSinkDMA(params: TraceSinkDMAParams, hartId: Int)(implicit p: Parameters) extends LazyTraceSink {
  val numSources = params.nSource
  val node = TLClientNode(Seq(TLMasterPortParameters.v1(Seq(TLClientParameters(
    name = "trace-sink-dma", sourceId = IdRange(0, numSources))))))

  val device = new SimpleDevice(s"trace-sink-dma$hartId", Seq("ucbbar,tracesinkdma"))
  val regnode = TLRegisterNode(
    address = Seq(AddressSet(params.regNodeBaseAddr, 0xFF)),
    device = device,
    beatBytes = params.beatBytes
  )

  lazy val module = new TraceSinkDMAImpl(this)
  class TraceSinkDMAImpl(outer: TraceSinkDMA) extends LazyTraceSinkModuleImp(outer) {
    val (mem, edge) = outer.node.out(0)
    val busWidth = edge.bundle.dataBits

    val serialWidthAggregator = Module(new SerialWidthAggregator(busWidth))
    serialWidthAggregator.io.narrow <> io.trace_in
    
    val mIdle :: mWrite :: mOverflow :: Nil = Enum(3)
    val mstate = RegInit(mIdle)
    
    // tracks how much trace data have we written in total
    val addr_counter = RegInit(0.U(64.W))
    // max permissible size of the trace data to write in bytes before we overflow/wrap
    val max_size_reg = RegInit(((BigInt(1) << 64) - 1).U(64.W))
    val addr_step = busWidth/8
    val addr_full = addr_counter + addr_step.U > max_size_reg

    val dma_start_addr = RegInit(0.U(64.W))

    // control registers
    // software reset the DMA engine to clear all the status registers
    val reset_reg = RegInit(false.B)
    // mode selection register, ring buffer mode or overflow mode
    val mode_reg = RegInit(DMAMode.overflow.U) // default to overflow mode

    // status registers
    // tracks how many times the address counter wrapped around
    val wrap_count = RegInit(0.U(32.W))
    
    // putting the buffer data on the TL mem lane

    val put_req = edge.Put(
      fromSource = 0.U,  // to be overridden by the SourceGenerator
      toAddress = addr_counter + dma_start_addr,
      lgSize = log2Ceil(busWidth / 8).U,
      data = serialWidthAggregator.io.wide.bits,
      mask = Fill(busWidth / 8, 1.U(1.W)))._2

    val sourceGen = Module(new SourceGenerator(edge.bundle.sourceBits))
    val sourceReady = sourceGen.io.id.valid
    val src_ready_stall_count = RegInit(0.U(32.W))
    when (!sourceReady) {
      src_ready_stall_count := src_ready_stall_count + 1.U
    }
    val entering_mWrite = mstate === mIdle && serialWidthAggregator.io.wide.valid &&
                      !(mode_reg === DMAMode.overflow.U && addr_full) &&
                      sourceReady
    sourceGen.io.gen := entering_mWrite
    sourceGen.io.reclaim.valid := mem.d.fire
    sourceGen.io.reclaim.bits := mem.d.bits.source
    val latched_source = Reg(UInt(edge.bundle.sourceBits.W))
    when (entering_mWrite) {
      latched_source := sourceGen.io.id.bits
    }
    mem.a.bits := put_req
    mem.a.bits.source := latched_source
    mem.a.valid := mstate === mWrite && sourceReady && serialWidthAggregator.io.wide.valid
    serialWidthAggregator.io.wide.ready := mem.a.fire || (mstate === mOverflow)
    mem.d.ready := true.B

    // assertions
    // are we ever writing out of bounds?
    val write_addr = mem.a.bits.address
    val write_start = write_addr - dma_start_addr
    val write_end_excl = write_start + addr_step.U

    assert(!(mem.a.fire && (write_end_excl > max_size_reg)), "DMA is writing out of bounds")
    assert(!(mem.d.fire && mem.d.bits.denied),  "DMA got denied D response")
    assert(!(mem.d.fire && mem.d.bits.corrupt), "DMA got corrupt D response")

    switch(mstate) {
      is (mIdle) {
        when (serialWidthAggregator.io.wide.valid) {
          when (mode_reg === DMAMode.overflow.U && addr_full) {
            mstate := mOverflow
          } .elsewhen (sourceReady) {
            mstate := mWrite
            latched_source := sourceGen.io.id.bits
          }
        }
        when (mode_reg === DMAMode.ringBuffer.U && addr_full && serialWidthAggregator.io.wide.valid) {
          wrap_count := wrap_count + 1.U
          addr_counter := 0.U
        }
      }
      // potentially, optimize this by pipelining collect and write
      is (mWrite) {
        // we need to write the collected data to the memory
        mstate := Mux(mem.a.fire, mIdle ,mWrite)
        addr_counter := Mux(mem.a.fire, addr_counter + addr_step.U, addr_counter)
      }
      is (mOverflow) {
        // always accept the upstream incoming packets
        // but never write to the memory
        // software reset is the only way to clear the overflow state
      }
    }

    when (reset_reg) {
      mstate := mIdle
      addr_counter := 0.U
      src_ready_stall_count := 0.U
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
      ):_*
    )
  }
}

class WithTraceSinkDMA(targetId: Int = 1, nSource: Int = 16) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: RocketTileAttachParams => {
      // redefine tile level constants
      val xBytes = tp.tileParams.core.xLen / 8
      tp.copy(tileParams = tp.tileParams.copy(
        traceParams = Some(tp.tileParams.traceParams.get.copy(buildSinks = 
          tp.tileParams.traceParams.get.buildSinks :+ (p => 
            (LazyModule(new TraceSinkDMA(TraceSinkDMAParams(
            regNodeBaseAddr = 0x3010000 + tp.tileParams.tileId * 0x1000,
            beatBytes = xBytes), hartId = tp.tileParams.tileId)(p)), targetId)))))
      )
    }
    case tp: ShuttleTileAttachParams => {
      val xBytes = tp.tileParams.core.xLen / 8
      tp.copy(tileParams = tp.tileParams.copy(
        traceParams = Some(tp.tileParams.traceParams.get.copy(buildSinks = 
          tp.tileParams.traceParams.get.buildSinks :+ (p => 
            (LazyModule(new TraceSinkDMA(TraceSinkDMAParams(
            regNodeBaseAddr = 0x3010000 + tp.tileParams.tileId * 0x1000,
            beatBytes = tp.tileParams.tileBeatBytes), hartId = tp.tileParams.tileId)(p)), targetId)))))
      )
    }
    case tp: boom.v3.common.BoomTileAttachParams => {
      val xBytes = tp.tileParams.core.xLen / 8
      tp.copy(tileParams = tp.tileParams.copy(
        traceParams = Some(tp.tileParams.traceParams.get.copy(buildSinks = 
          tp.tileParams.traceParams.get.buildSinks :+ (p => 
            (LazyModule(new TraceSinkDMA(TraceSinkDMAParams(
            regNodeBaseAddr = 0x3010000 + tp.tileParams.tileId * 0x1000,
            beatBytes = xBytes, nSource = nSource), hartId = tp.tileParams.tileId)(p)), targetId)))))
      )
    }
    case tp: boom.v4.common.BoomTileAttachParams => {
      val xBytes = tp.tileParams.core.xLen / 8
      tp.copy(tileParams = tp.tileParams.copy(
        traceParams = Some(tp.tileParams.traceParams.get.copy(buildSinks = 
          tp.tileParams.traceParams.get.buildSinks :+ (p => 
            (LazyModule(new TraceSinkDMA(TraceSinkDMAParams(
            regNodeBaseAddr = 0x3010000 + tp.tileParams.tileId * 0x1000,
            beatBytes = xBytes), hartId = tp.tileParams.tileId)(p)), targetId)))))
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
        require(traceSinkDmas.size == 1, s"tile ${r.tileId} must have exactly one trace DMA sink, found ${traceSinkDmas.size}")
        Some((r, r.trace_encoder_controller.get, traceSinkDmas.head))
      } else {
        None
      }
    case s: ShuttleTile =>
      val traceSinkDmas = s.trace_sinks.collect { case dma: TraceSinkDMA => dma }
      if (s.trace_encoder_controller.isDefined || traceSinkDmas.nonEmpty) {
        require(s.trace_encoder_controller.isDefined, s"tile ${s.tileId} has trace DMA sink but no trace encoder controller")
        require(traceSinkDmas.size == 1, s"tile ${s.tileId} must have exactly one trace DMA sink, found ${traceSinkDmas.size}")
        Some((s, s.trace_encoder_controller.get, traceSinkDmas.head))
      } else {
        None
      }
    case b3: boom.v3.common.BoomTile =>
      val traceSinkDmas = b3.trace_sinks.collect { case dma: TraceSinkDMA => dma }
      if (b3.trace_encoder_controller.isDefined || traceSinkDmas.nonEmpty) {
        require(b3.trace_encoder_controller.isDefined, s"tile ${b3.tileId} has trace DMA sink but no trace encoder controller")
        require(traceSinkDmas.size == 1, s"tile ${b3.tileId} must have exactly one trace DMA sink, found ${traceSinkDmas.size}")
        Some((b3, b3.trace_encoder_controller.get, traceSinkDmas.head))
      } else {
        None
      }
    case b4: boom.v4.common.BoomTile =>
      val traceSinkDmas = b4.trace_sinks.collect { case dma: TraceSinkDMA => dma }
      if (b4.trace_encoder_controller.isDefined || traceSinkDmas.nonEmpty) {
        require(b4.trace_encoder_controller.isDefined, s"tile ${b4.tileId} has trace DMA sink but no trace encoder controller")
        require(traceSinkDmas.size == 1, s"tile ${b4.tileId} must have exactly one trace DMA sink, found ${traceSinkDmas.size}")
        Some((b4, b4.trace_encoder_controller.get, traceSinkDmas.head))
      } else {
        None
      }
    case _ => None
  }
  if (traceEncoderDmaBindings.nonEmpty) {
    val mbus = baseSubsystem.locateTLBusWrapper(MBUS)
    traceEncoderDmaBindings.foreach { case (t, c, s) =>
      t { // in the implicit clock domain of tile
        ResourceBinding {
          Resource(c.device, "ucbbar,trace-dma").bind(ResourceReference(s.device.label))
        }
        mbus.coupleFrom(t.tileParams.baseName) { bus =>
          bus := mbus.crossOut(s.node)(ValName("trace_sink_dma"))(AsynchronousCrossing())
        }
        t match {
          // Same pattern as the trace_encoder_controller connection in shuttle Tile.scala.
          // TODO: switch to shuttleTile.connectTLSlaveAtTileBeatBytes(s.regnode) once local
          // shuttle merges ucb-bar/shuttle@7f6bfe7 (iris lineage; diverged from local TLB-ASID work).
          case shuttleTile: ShuttleTile => shuttleTile.connectTLSlave(s.regnode, shuttleTile.shuttleParams.tileBeatBytes)
          case _ => t.connectTLSlave(s.regnode, t.xBytes)
        }
      }
    }
  }
})
