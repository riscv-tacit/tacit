package tacit

import chisel3._
import chisel3.util._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.prci._
import org.chipsalliance.cde.config.{Parameters, Config, Field}
import freechips.rocketchip.tile._
import freechips.rocketchip.subsystem._
import shuttle.common.ShuttleTileAttachParams

import freechips.rocketchip.trace._

class TraceSinkAlways()(implicit p: Parameters) extends LazyTraceSink {
  override lazy val module = new TraceSinkAlwaysImpl(this)
  class TraceSinkAlwaysImpl(outer: TraceSinkAlways) extends LazyTraceSinkModuleImp(outer) {
    io.trace_in.ready := true.B
    // Prevent FIRRTL DCE from removing this sink
    dontTouch(io.trace_in)
  }
}

class WithTraceSinkAlways(targetId: Int = 0) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: RocketTileAttachParams => {
      tp.copy(tileParams = tp.tileParams.copy(
        traceParams = Some(tp.tileParams.traceParams.get.copy(buildSinks = 
          tp.tileParams.traceParams.get.buildSinks :+ (p => (LazyModule(new TraceSinkAlways()(p)), targetId)))))
      )
    }
    case tp: ShuttleTileAttachParams => {
      tp.copy(tileParams = tp.tileParams.copy(
        traceParams = Some(tp.tileParams.traceParams.get.copy(buildSinks = 
          tp.tileParams.traceParams.get.buildSinks :+ (p => (LazyModule(new TraceSinkAlways()(p)), targetId)))))
      )
    }
    case tp: boom.v3.common.BoomTileAttachParams => {
      tp.copy(tileParams = tp.tileParams.copy(
        traceParams = Some(tp.tileParams.traceParams.get.copy(buildSinks = 
          tp.tileParams.traceParams.get.buildSinks :+ (p => (LazyModule(new TraceSinkAlways()(p)), targetId)))))
        )
      }
    case tp: boom.v4.common.BoomTileAttachParams => {
      tp.copy(tileParams = tp.tileParams.copy(
        traceParams = Some(tp.tileParams.traceParams.get.copy(buildSinks = 
          tp.tileParams.traceParams.get.buildSinks :+ (p => (LazyModule(new TraceSinkAlways()(p)), targetId)))))
      )
    }
    case other => other
  }
})

class WithHPMSinkAlways(targetId: Int = 0) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: RocketTileAttachParams => {
      tp.copy(tileParams = tp.tileParams.copy(
        traceParams = Some(tp.tileParams.traceParams.get.copy(buildSinks = 
          tp.tileParams.traceParams.get.buildSinks :+ (p => (LazyModule(new TraceSinkAlways()(p)), targetId)))))
      )
    }
    case tp: ShuttleTileAttachParams => {
      tp.copy(tileParams = tp.tileParams.copy(
        traceParams = Some(tp.tileParams.traceParams.get.copy(buildSinks = 
          tp.tileParams.traceParams.get.buildSinks :+ (p => (LazyModule(new TraceSinkAlways()(p)), targetId)))))
      )
    }
    case other => other
  }
})