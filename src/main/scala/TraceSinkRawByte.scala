package tacit

import chisel3._
import chisel3.util._
import chisel3.experimental.SourceInfo
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.prci._
import org.chipsalliance.cde.config.{Parameters, Config, Field}
import freechips.rocketchip.tile._
import freechips.rocketchip.subsystem._
import shuttle.common.{ShuttleTile, ShuttleTileAttachParams}

import freechips.rocketchip.trace._

class TraceSinkRawByteBundle extends Bundle {
  val out = Decoupled(UInt(8.W))
}

case class NullParams()

object TraceSinkRawByteNodeImp extends SimpleNodeImp[NullParams, NullParams, NullParams, TraceSinkRawByteBundle] {
  def bundle(x: NullParams) = new TraceSinkRawByteBundle()
  def edge(x: NullParams, y: NullParams, p: Parameters, sourceInfo: SourceInfo): NullParams = NullParams()
  def render(x: NullParams): RenderedEdge = RenderedEdge("ffffff")
}
case class TraceSinkRawByteMasterNode()(implicit valName: ValName) extends SourceNode(TraceSinkRawByteNodeImp)(Seq(NullParams()))
case class TraceSinkRawByteSlaveNode()(implicit valName: ValName) extends SinkNode(TraceSinkRawByteNodeImp)(Seq(NullParams()))

class TraceSinkRawByte()(implicit p: Parameters) extends LazyTraceSink {
  val node = new TraceSinkRawByteMasterNode()(ValName("trace_sink_raw_byte"))
  override lazy val module = new TraceSinkRawByteImpl(this)
  class TraceSinkRawByteImpl(outer: TraceSinkRawByte) extends LazyTraceSinkModuleImp(outer) {
    outer.node.out.head._1.out <> io.trace_in
  }
}

class WithTraceSinkRawByte(targetId: Int = 0) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: RocketTileAttachParams => {
      tp.copy(tileParams = tp.tileParams.copy(
        traceParams = Some(tp.tileParams.traceParams.get.copy(buildSinks = 
          tp.tileParams.traceParams.get.buildSinks :+ (p => (LazyModule(new TraceSinkRawByte()(p)), targetId)))))
      )
    }
    case tp: ShuttleTileAttachParams => {
      tp.copy(tileParams = tp.tileParams.copy(
        traceParams = Some(tp.tileParams.traceParams.get.copy(buildSinks = 
          tp.tileParams.traceParams.get.buildSinks :+ (p => (LazyModule(new TraceSinkRawByte()(p)), targetId)))))
      )
    }
    case other => other
  }
})

trait CanHaveTraceSinkRawByte { this: BaseSubsystem =>
  require(this.isInstanceOf[BaseSubsystem with InstantiatesHierarchicalElements])
  val hierarchicalSubsystem = this.asInstanceOf[BaseSubsystem with InstantiatesHierarchicalElements]
  val TraceSinkRawBytes = hierarchicalSubsystem.totalTiles.values.map { t => t match {
    case r: RocketTile => r.trace_sinks.collect { case r: TraceSinkRawByte => (t, r) }
    case s: ShuttleTile => s.trace_sinks.collect { case r: TraceSinkRawByte => (t, r) }
    case _ => Nil
  }}.flatten
  val tacit_bytes = if (TraceSinkRawBytes.nonEmpty) {
    TraceSinkRawBytes.map { case (t, s) =>
      val slavenode = new TraceSinkRawByteSlaveNode()(ValName("trace_sink_raw_byte"))
      slavenode := s.node
      InModuleBody {
        val tacit_byte = IO(new TraceSinkRawByteBundle)
        tacit_byte.out <> slavenode.in.head._1.out
        tacit_byte
      }
    }.toSeq
  } else {
    Nil
  }
  if (tacit_bytes.nonEmpty) {
    println(s"TraceSinkRawBytes: ${tacit_bytes.length}")
  }
}
