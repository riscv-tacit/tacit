package tacit

import chisel3._
import chisel3.util._
import chisel3.experimental.{IntParam, StringParam}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.prci._
import org.chipsalliance.cde.config.{Parameters, Config, Field}
import freechips.rocketchip.tile._
import freechips.rocketchip.subsystem._
import shuttle.common.ShuttleTileAttachParams

import freechips.rocketchip.trace._

/** Default pattern for TraceSinkBackpressure; every field can be overridden at
  * simulation time with +tacit_bp_* plusargs (see vsrc/TraceSinkBackpressure.v). */
case class TraceSinkBackpressureParams(
  mode: String = "duty",   // always | duty | burst | every | random | file
  on: Int = 1,             // duty: ready cycles per period
  period: Int = 4,         // duty / burst: period in cycles
  off: Int = 100,          // burst: not-ready cycles per period; every: outage length
  beats: Int = 16,         // every: accepted beats between outages
  pct: Int = 25,           // random: ready probability in percent
  seed: Int = 1,           // random
  file: String = "tacit_bp_ready.txt", // file: one 0/1 per line
)

class TraceSinkBackpressureBlackBox(params: TraceSinkBackpressureParams) extends BlackBox(
  Map(
    "MODE" -> StringParam(params.mode),
    "ON" -> IntParam(params.on),
    "PERIOD" -> IntParam(params.period),
    "OFF" -> IntParam(params.off),
    "BEATS" -> IntParam(params.beats),
    "PCT" -> IntParam(params.pct),
    "SEED" -> IntParam(params.seed),
    "FILE_NAME" -> StringParam(params.file),
  )
) with HasBlackBoxResource {
  val io = IO(new Bundle {
    val clk = Input(Clock())
    val reset = Input(Reset())
    val in_valid = Input(Bool())
    val in_ready = Output(Bool())
  })
  addResource("/vsrc/TraceSinkBackpressure.v")
}

/** Simulation-only sink that discards its input and asserts ready according to
  * a programmable pattern, to exercise encoder backpressure (stall in lossless
  * mode, Pause/Resume in lossy mode). The byte stream is still observable
  * through the arbiter monitor, which sits upstream of the sinks. */
class TraceSinkBackpressure(params: TraceSinkBackpressureParams)(implicit p: Parameters) extends LazyTraceSink {
  override lazy val module = new TraceSinkBackpressureImpl(this)
  class TraceSinkBackpressureImpl(outer: TraceSinkBackpressure) extends LazyTraceSinkModuleImp(outer) {
    val bb = Module(new TraceSinkBackpressureBlackBox(params))
    bb.io.clk := clock
    bb.io.reset := reset
    bb.io.in_valid := io.trace_in.valid
    io.trace_in.ready := bb.io.in_ready
    dontTouch(io.trace_in)
  }
}

/** Attach a TraceSinkBackpressure to every traced tile as sink `targetId`
  * (0 = always, 1 = DMA, 2 = raw byte in the existing configs). */
class WithTraceSinkBackpressure(targetId: Int = 3, params: TraceSinkBackpressureParams = TraceSinkBackpressureParams())
    extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: RocketTileAttachParams => {
      tp.copy(tileParams = tp.tileParams.copy(
        traceParams = Some(tp.tileParams.traceParams.get.copy(buildSinks =
          tp.tileParams.traceParams.get.buildSinks :+ (p => (LazyModule(new TraceSinkBackpressure(params)(p)), targetId)))))
      )
    }
    case tp: ShuttleTileAttachParams => {
      tp.copy(tileParams = tp.tileParams.copy(
        traceParams = Some(tp.tileParams.traceParams.get.copy(buildSinks =
          tp.tileParams.traceParams.get.buildSinks :+ (p => (LazyModule(new TraceSinkBackpressure(params)(p)), targetId)))))
      )
    }
    case tp: boom.v3.common.BoomTileAttachParams => {
      tp.copy(tileParams = tp.tileParams.copy(
        traceParams = Some(tp.tileParams.traceParams.get.copy(buildSinks =
          tp.tileParams.traceParams.get.buildSinks :+ (p => (LazyModule(new TraceSinkBackpressure(params)(p)), targetId)))))
      )
    }
    case tp: boom.v4.common.BoomTileAttachParams => {
      tp.copy(tileParams = tp.tileParams.copy(
        traceParams = Some(tp.tileParams.traceParams.get.copy(buildSinks =
          tp.tileParams.traceParams.get.buildSinks :+ (p => (LazyModule(new TraceSinkBackpressure(params)(p)), targetId)))))
      )
    }
    case other => other
  }
})
