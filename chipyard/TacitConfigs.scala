package chipyard

import chisel3._
import org.chipsalliance.cde.config.{Config, Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.subsystem._
import freechips.rocketchip.tile._
import freechips.rocketchip.trace.{TraceCoreParams, TraceEncoderParams}

import shuttle.common.ShuttleTileAttachParams
import tacit.{TacitEncoder, TacitBPParams}

// Add a Tacit encoder to each tile
class WithTacitEncoder extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: RocketTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      traceParams = Some(TraceEncoderParams(
        encoderBaseAddr = 0x3000000 + tp.tileParams.tileId * 0x1000,
        buildEncoder = (p: Parameters) => LazyModule(new TacitEncoder(new TraceCoreParams(
          nGroups = 1,
          xlen = tp.tileParams.core.xLen,
          iaddrWidth = tp.tileParams.core.xLen
        ),
        bufferDepth = 16,
        coreStages = 5,
        bpParams = TacitBPParams(xlen = tp.tileParams.core.xLen, n_entries = 1024))(p)),
        useArbiterMonitor = false
      )),
      core = tp.tileParams.core.copy(enableTraceCoreIngress = true)))
    case tp: ShuttleTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      traceParams = Some(TraceEncoderParams(
        encoderBaseAddr = 0x3000000 + tp.tileParams.tileId * 0x1000,
        buildEncoder = (p: Parameters) => LazyModule(new TacitEncoder(new TraceCoreParams(
          nGroups = tp.tileParams.core.retireWidth,
          xlen = tp.tileParams.core.xLen,
          iaddrWidth = tp.tileParams.core.xLen
        ),
        bufferDepth = 16,
        coreStages = 7,
        bpParams = TacitBPParams(xlen = tp.tileParams.core.xLen, n_entries = 1024))(p)),
        useArbiterMonitor = false
      )),
      core = tp.tileParams.core.copy(enableTraceCoreIngress = true)))
  }
})

// Rocket with Tacit encoder and trace sinks
class TacitRocketConfig extends Config(
  new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder ++
  new chipyard.config.WithNPerfCounters(29) ++
  new freechips.rocketchip.subsystem.WithoutTLMonitors ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

// Shuttle with Tacit encoder and trace sinks
class TacitShuttleConfig extends Config(
  new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder ++
  new freechips.rocketchip.subsystem.WithoutTLMonitors ++
  new shuttle.common.WithNShuttleCores ++
  new chipyard.config.AbstractConfig)

// Rocket with Tacit encoder and raw byte sinks
class TacitRocketRawByteConfig extends Config(
  new tacit.WithTraceSinkRawByte(2) ++
  new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder ++
  new freechips.rocketchip.subsystem.WithoutTLMonitors ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)