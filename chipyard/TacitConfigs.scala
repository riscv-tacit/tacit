package chipyard

import chisel3._
import org.chipsalliance.cde.config.{Config, Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.subsystem._
import freechips.rocketchip.tile._
import freechips.rocketchip.trace.{TraceCoreParams, TraceEncoderParams}

import shuttle.common.ShuttleTileAttachParams
import tacit.{TacitEncoder, TacitParallelEncoder, TacitBPParams}
// import boom.v4.common.BoomTileAttachParams

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
  /*
    case tp: boom.v3.common.BoomTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      traceParams = Some(TraceEncoderParams(
        encoderBaseAddr = 0x3000000 + tp.tileParams.tileId * 0x1000,
        buildEncoder = (p: Parameters) => LazyModule(new TacitParallelEncoder(new TraceCoreParams(
          nGroups = tp.tileParams.core.retireWidth,
          iretireWidth = 1,
          xlen = tp.tileParams.core.xLen,
          iaddrWidth = tp.tileParams.core.xLen
        ),
        bufferDepth = 64,
        coreStages = 5, // BOOM stalls commit, not fetch
        )(p)),
        useArbiterMonitor = false
      )),
      core = tp.tileParams.core.copy(enableTraceCoreIngress = true)))
    case tp: boom.v4.common.BoomTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      traceParams = Some(TraceEncoderParams(
        encoderBaseAddr = 0x3000000 + tp.tileParams.tileId * 0x1000,
        buildEncoder = (p: Parameters) => LazyModule(new TacitParallelEncoder(new TraceCoreParams(
          nGroups = tp.tileParams.core.retireWidth,
          iretireWidth = 1,
          xlen = tp.tileParams.core.xLen,
          iaddrWidth = tp.tileParams.core.xLen
        ),
        bufferDepth = 64,
        coreStages = 5, // BOOM stalls commit, not fetch
        )(p)),
        useArbiterMonitor = false
      )),
      core = tp.tileParams.core.copy(enableTraceCoreIngress = true)))
  */
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

// Rocket with Tacit encoder and trace sinks
class TacitRocketDMAWithTLMonitorConfig extends Config(
  new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder ++
  new chipyard.config.WithNPerfCounters(29) ++
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
  // new chipyard.config.WithAsidLen(16) ++
  new freechips.rocketchip.rocket.WithL1DCacheNonblocking(2) ++     // non-blocking L1D$, L1 prefetching only works with non-blocking L1D$
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

// Rocket with no PTE cache
class TacitRocketNoPTERawByteConfig extends Config(
  // trace configs
  new tacit.WithTraceSinkRawByte(2) ++
  new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder ++
  // system configs
  new freechips.rocketchip.subsystem.WithoutTLMonitors ++
  // new chipyard.config.WithPTECacheEntries(0) ++
  // new chipyard.config.WithAsidLen(16) ++
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)

// Rocket with Tacit encoder and raw byte sinks
class TacitRocketRawBytePrefetchConfig extends Config(
  new tacit.WithTraceSinkRawByte(2) ++
  new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder ++
  new freechips.rocketchip.subsystem.WithoutTLMonitors ++
  // new chipyard.config.WithAsidLen(16) ++
  new freechips.rocketchip.rocket.WithL1DCacheNonblocking(8) ++     // non-blocking L1D$, L1 prefetching only works with non-blocking L1D$
  new freechips.rocketchip.rocket.WithNHugeCores(1) ++
  new chipyard.config.AbstractConfig)
  
class TacitDualRocketRawByteConfig extends Config(
  new tacit.WithTraceSinkRawByte(2) ++
  new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder ++
  new freechips.rocketchip.subsystem.WithoutTLMonitors ++
  // new chipyard.config.WithAsidLen(16) ++
  new freechips.rocketchip.rocket.WithNHugeCores(2) ++
  new chipyard.config.AbstractConfig)

/*
class TacitMediumBoomV3RawByteConfig extends Config(
  new tacit.WithTraceSinkRawByte(1) ++
  // new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder ++
  new freechips.rocketchip.subsystem.WithoutTLMonitors ++
  new chipyard.config.WithAsidLen(16) ++
  new boom.v3.common.WithNMediumBooms(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

class TacitLargeBoomV3RawByteConfig extends Config(
  new tacit.WithTraceSinkRawByte(1) ++
  // new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder ++
  new freechips.rocketchip.subsystem.WithoutTLMonitors ++
  new chipyard.config.WithAsidLen(16) ++
  new boom.v3.common.WithNLargeBooms(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

class TacitMegaBoomV3RawByteConfig extends Config(
  new tacit.WithTraceSinkRawByte(1) ++
  // new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder ++
  new freechips.rocketchip.subsystem.WithoutTLMonitors ++
  new chipyard.config.WithAsidLen(16) ++
  new boom.v3.common.WithNMegaBooms(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

class TacitLargeBoomV3NoASIDRawByteConfig extends Config(
  new tacit.WithTraceSinkRawByte(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder ++
  new freechips.rocketchip.subsystem.WithoutTLMonitors ++
  new boom.v3.common.WithNLargeBooms(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

class TacitLargeBoomV4Config extends Config(
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder ++
  new freechips.rocketchip.subsystem.WithoutTLMonitors ++
  // new boom.v4.common.WithBoomCommitLogPrintf ++
  new boom.v4.common.WithNLargeBooms(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

class TacitLargeBoomV4RawByteConfig extends Config(
  new tacit.WithTraceSinkRawByte(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder ++
  new chipyard.config.WithAsidLen(16) ++
  new freechips.rocketchip.subsystem.WithoutTLMonitors ++
  new boom.v4.common.WithNLargeBooms(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

class TacitLargeBoomV4RawByteNoASIDConfig extends Config(
  new tacit.WithTraceSinkRawByte(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder ++
  new freechips.rocketchip.subsystem.WithoutTLMonitors ++
  new boom.v4.common.WithNLargeBooms(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

class TacitMediumBoomV4RawByteConfig extends Config(
  new tacit.WithTraceSinkRawByte(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder ++
  new chipyard.config.WithAsidLen(16) ++
  new freechips.rocketchip.subsystem.WithoutTLMonitors ++
  new boom.v4.common.WithNMediumBooms(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)
*/
