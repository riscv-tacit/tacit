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
    
    val mIdle :: mCollect :: mWrite :: Nil = Enum(3)
    val mstate = RegInit(mIdle)
    
    // tracks how much trace data have we written in total
    val addr_counter = RegInit(0.U(64.W))
    // tracks how much trace data have we collected in current transaction
    val collect_counter = RegInit(0.U(4.W))
    val msg_buffer = RegInit(VecInit(Seq.fill(busWidth / 8)(0.U(8.W))))

    val dma_start_addr = RegInit(0.U(64.W))
    val dma_addr_write_valid = Wire(Bool())

    val flush_reg = RegInit(false.B)
    val done_reg = RegInit(false.B)
    val collect_full = collect_counter === (busWidth / 8).U
    val collect_advance = Mux(flush_reg, collect_full || fifo.io.deq.valid === false.B, collect_full)
    val flush_done = (flush_reg) && (fifo.io.deq.valid === false.B) && mstate === mIdle
    done_reg := done_reg || flush_done
    
    val inflight_counter = RegInit(0.U(4.W))

    mem.a.valid := mstate === mWrite && inflight_counter < 16.U
    mem.d.ready := true.B
    fifo.io.deq.ready := false.B // default case 
    dontTouch(mem.d.valid)

    // mask according to collect_counter
    val mask = (1.U << collect_counter) - 1.U
    // putting the buffer data on the TL mem lane

    inflight_counter := Mux(mem.a.fire && !mem.d.fire, inflight_counter + 1.U, 
        Mux(mem.d.fire && !mem.a.fire, inflight_counter - 1.U, inflight_counter))

    val put_req = edge.Put(
      fromSource = 0.U,
      toAddress = addr_counter + dma_start_addr,
      lgSize = log2Ceil(busWidth / 8).U,
      data = Cat(msg_buffer.reverse),
      mask = mask)._2

    mem.a.bits := put_req

    switch(mstate) {
      is (mIdle) {
        fifo.io.deq.ready := false.B
        mstate := Mux(fifo.io.deq.valid, mCollect, mIdle)
        collect_counter := 0.U
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
        fifo.io.deq.ready := false.B
        mstate := Mux(mem.a.fire, mIdle, mWrite)
        addr_counter := Mux(mem.a.fire, addr_counter + collect_counter, addr_counter)
      }
    }

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
            RegFieldDesc("addr_counter", "Address counter"))
        )
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
    val sbus = baseSubsystem.locateTLBusWrapper(SBUS)
    traceSinkDMAs.foreach { case (t, s) =>
      t { // in the implicit clock domain of tile
        sbus.coupleFrom(t.tileParams.baseName) { bus =>
          bus := sbus.crossOut(s.node)(ValName("trace_sink_dma"))(AsynchronousCrossing())
        }
        t.connectTLSlave(s.regnode, t.xBytes)
      }
    }
  }
})
