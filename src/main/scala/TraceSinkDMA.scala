package tacit

import chisel3._
import chisel3.util._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.diplomacy._
import org.chipsalliance.cde.config.{Parameters, Config}
import freechips.rocketchip.subsystem._
import freechips.rocketchip.prci._
import freechips.rocketchip.regmapper.{RegField, RegFieldDesc}
import freechips.rocketchip.tile._
import shuttle.common.{ShuttleTile, ShuttleTileAttachParams}
import freechips.rocketchip.trace._
import testchipip.soc.{SubsystemInjector, SubsystemInjectorKey}
import freechips.rocketchip.resources.BigIntHexContext

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
  beatBytes: Int
)

class TraceSinkDMA(params: TraceSinkDMAParams, hartId: Int)(implicit p: Parameters) extends LazyTraceSink {
  val node = TLClientNode(Seq(TLMasterPortParameters.v1(Seq(TLClientParameters(
    name = "trace-sink-dma", sourceId = IdRange(0, 16))))))

  val device = new SimpleDevice(s"trace-sink-dma$hartId", Seq("ucbbar,tracesinkdma"))
  val regnode = TLRegisterNode(
    address = Seq(AddressSet(params.regNodeBaseAddr, 0xFF)),
    device = device,
    beatBytes = params.beatBytes
  )

  lazy val module = new TraceSinkDMAImpl(this)
  class TraceSinkDMAImpl(outer: TraceSinkDMA) extends LazyTraceSinkModuleImp(outer) {
    val fifo = Module(new Queue(UInt(8.W), 32))
    fifo.io.enq <> io.trace_in
    val (mem, edge) = outer.node.out(0)
    val addrBits = edge.bundle.addressBits
    val busWidth = edge.bundle.dataBits
    val blockBytes = p(CacheBlockBytes)
    
    val mIdle :: mCollect :: mWrite :: mOverflow :: Nil = Enum(4)
    val mstate = RegInit(mIdle)
    
    // tracks how much trace data have we written in total
    val addr_counter = RegInit(x"180000000".U(64.W)) // yrh: where mbus manager scratchpad starts
    // max size of the trace data to be collected before we overflow. 
    // software shall make a guarantee on this value to be a multiple of the bus width.
    val max_size_reg = RegInit(((BigInt(1) << 10) - 1).U(64.W)) // 1KiB
    // tracks how much trace data have we collected in current transaction
    val collect_counter = RegInit(0.U(4.W))
    val msg_buffer = RegInit(VecInit(Seq.fill(busWidth / 8)(0.U(8.W))))

    val dma_start_addr = RegInit(0.U(64.W))
    val dma_addr_write_valid = Wire(Bool())

    // control registers
    // flush the stale packets in the message buffer
    val flush_reg = RegInit(false.B)
    // software reset the DMA engine to clear all the status registers
    val reset_reg = RegInit(false.B)

    // status registers
    // done flag that indicates that the DMA engine has finished processing all the packets
    val done_reg = RegInit(false.B)
    val collect_full = collect_counter === (busWidth / 8).U
    val collect_advance = Mux(flush_reg, collect_full || fifo.io.deq.valid === false.B, collect_full)
    val flush_done = (flush_reg) && (fifo.io.deq.valid === false.B) && (mstate === mIdle || mstate === mOverflow)
    done_reg := done_reg || flush_done
    
    // mask according to collect_counter
    val mask = (1.U << collect_counter) - 1.U
    // putting the buffer data on the TL mem lane

    val put_req = edge.Put(
      fromSource = 0.U,  // to be overridden by the SourceGenerator
      toAddress = addr_counter + dma_start_addr,
      lgSize = log2Ceil(busWidth / 8).U,
      data = Cat(msg_buffer.reverse),
      mask = mask)._2

    val sourceGen = Module(new SourceGenerator(edge.bundle.sourceBits))
    val sourceReady = sourceGen.io.id.valid
    sourceGen.io.gen := mstate === mCollect && collect_advance
    sourceGen.io.reclaim.valid := mem.d.fire
    sourceGen.io.reclaim.bits := mem.d.bits.source
    val latched_source = Reg(UInt(edge.bundle.sourceBits.W))
    when (sourceGen.io.gen) {
      latched_source := sourceGen.io.id.bits
    }
    mem.a.bits := put_req
    mem.a.bits.source := latched_source
    mem.a.valid := mstate === mWrite && sourceReady
    mem.d.ready := true.B
    fifo.io.deq.ready := false.B

    switch(mstate) {
      is (mIdle) {
        when (addr_counter >= max_size_reg) {
          mstate := mOverflow
        } .otherwise {
          mstate := Mux(fifo.io.deq.valid, mCollect, mIdle)
          collect_counter := 0.U
        }
      }
      is (mCollect) {
        // either we have collected enough data or that's all the messages for now
        mstate := Mux(collect_advance, mWrite, mCollect)
        collect_counter := Mux(fifo.io.deq.fire, collect_counter + 1.U, collect_counter)
        msg_buffer(collect_counter) := Mux(fifo.io.deq.fire, fifo.io.deq.bits, msg_buffer(collect_counter))
        fifo.io.deq.ready := collect_counter < (busWidth / 8).U
      }
      // potentially, optimize this by pipelining collect and write
      is (mWrite) {
        // we need to write the collected data to the memory
        mstate := Mux(mem.a.fire, mIdle, mWrite)
        addr_counter := Mux(mem.a.fire, addr_counter + collect_counter, addr_counter)
      }
      is (mOverflow) {
        fifo.io.deq.ready := true.B // dequeue all the remaining packets but do nothing with them
      }
    }

    when (reset_reg) {
      mstate := mIdle
      addr_counter := 0.U
      collect_counter := 0.U
    }
    Pulsify(reset_reg, 1)

    // regmap handler functions
    def traceSinkDMARegWrite(valid: Bool, bits: UInt): Bool = {
      dma_addr_write_valid := valid && mstate === mIdle
      when (dma_addr_write_valid) {
        dma_start_addr := bits
      }
      true.B
    }

    def traceSinkDMARegRead(ready: Bool): (Bool, UInt) = {
      (true.B, dma_start_addr)
    }

    val regmap = regnode.regmap(
      Seq(
        0x00 -> Seq(
          RegField(1, flush_reg,
            RegFieldDesc("flush_reg", "Flush register"))
        ),
        0x04 -> Seq(
          RegField.r(1, done_reg,
            RegFieldDesc("done_reg", "Done register"))
        ),
        0x08 -> Seq(
          RegField(64, traceSinkDMARegRead(_), traceSinkDMARegWrite(_, _),
            RegFieldDesc("dma_start_addr", "DMA start address"))
        ),
        0x10 -> Seq(RegField(64, addr_counter,
            RegFieldDesc("addr_counter", "Address counter, this is the number of bytes written to the memory to date"))
        ),
        0x18 -> Seq(RegField(64, max_size_reg,
          RegFieldDesc("max_size_reg", "Max size register"))
        ),
        0x20 -> Seq(RegField(1, reset_reg, 
          RegFieldDesc("reset_reg", "Soft reset register")))
      ):_*
    )
  }
}

class WithTraceSinkDMA(targetId: Int = 1) extends Config((site, here, up) => {
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
            beatBytes = xBytes), hartId = tp.tileParams.tileId)(p)), targetId)))))
      )
    }
    // case tp: boom.v3.common.BoomTileAttachParams => {
    //   val xBytes = tp.tileParams.core.xLen / 8
    //   tp.copy(tileParams = tp.tileParams.copy(
    //     traceParams = Some(tp.tileParams.traceParams.get.copy(buildSinks = 
    //       tp.tileParams.traceParams.get.buildSinks :+ (p => 
    //         (LazyModule(new TraceSinkDMA(TraceSinkDMAParams(
    //         regNodeBaseAddr = 0x3010000 + tp.tileParams.tileId * 0x1000,
    //         beatBytes = xBytes), hartId = tp.tileParams.tileId)(p)), targetId)))))
    //   )
    // }
    // case tp: boom.v4.common.BoomTileAttachParams => {
    //   val xBytes = tp.tileParams.core.xLen / 8
    //   tp.copy(tileParams = tp.tileParams.copy(
    //     traceParams = Some(tp.tileParams.traceParams.get.copy(buildSinks = 
    //       tp.tileParams.traceParams.get.buildSinks :+ (p => 
    //         (LazyModule(new TraceSinkDMA(TraceSinkDMAParams(
    //         regNodeBaseAddr = 0x3010000 + tp.tileParams.tileId * 0x1000,
    //         beatBytes = xBytes), hartId = tp.tileParams.tileId)(p)), targetId)))))
    //   )
    // }
    case other => other
  }
  case SubsystemInjectorKey => up(SubsystemInjectorKey) + TraceSinkDMAInjector
})

case object TraceSinkDMAInjector extends SubsystemInjector((p, baseSubsystem) => {
  require(baseSubsystem.isInstanceOf[BaseSubsystem with InstantiatesHierarchicalElements])
  val hierarchicalSubsystem = baseSubsystem.asInstanceOf[BaseSubsystem with InstantiatesHierarchicalElements]
  implicit val q: Parameters = p
  val traceSinkDMAs = hierarchicalSubsystem.totalTiles.values.map { t => t match {
    case r: RocketTile => r.trace_sinks.collect { case r: TraceSinkDMA => (t, r) }
    case s: ShuttleTile => s.trace_sinks.collect { case r: TraceSinkDMA => (t, r) }
    case _ => Nil
  }}.flatten
  if (traceSinkDMAs.nonEmpty) {
    val mbus = baseSubsystem.locateTLBusWrapper(MBUS)
    traceSinkDMAs.foreach { case (t, s) =>
      t { // in the implicit clock domain of tile
        mbus.coupleFrom(t.tileParams.baseName) { bus =>
          bus := mbus.crossOut(s.node)(ValName("trace_sink_dma"))(AsynchronousCrossing())
        }
        t.connectTLSlave(s.regnode, t.xBytes)
      }
    }
  }
})
