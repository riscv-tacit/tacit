package chipyard

import chisel3._
import org.chipsalliance.cde.config.{Config, Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.subsystem._
import freechips.rocketchip.tile._
import freechips.rocketchip.trace.{TraceCoreParams, TraceEncoderParams}

import shuttle.common.ShuttleTileAttachParams
import tacit.{TacitEncoder, TacitParallelEncoder, TacitBPParams}
import boom.v4.common.BoomTileAttachParams

// Add a Tacit encoder to each tile.
// queueImpl selects the packet-buffer storage in TacitParallelEncoder (BOOM tiles
// only; the serial TacitEncoder on Rocket/Shuttle is unaffected). Defaults to the
// legacy write-first SRAM queue so existing configs and bitstreams are unchanged;
// pass tacit.MPQueueImpl.SRAM to select the credit-based queue with no
// read-under-write dependence.
// bufferDepthOverride replaces the per-core-type default (16 for the 1-wide
// Rocket/Shuttle cases, 64 for BOOM) so a depth sweep is a config change rather
// than an edit. Left as None every existing config elaborates bit-identically.
// Depth stays an elaboration parameter on purpose: it sizes real SRAM, so each
// point in a sweep is its own bitstream and its own area/timing result. Only
// the resume watermark is programmable at runtime.
class WithTacitEncoder(
  queueImpl: tacit.MPQueueImpl = tacit.MPQueueImpl.LegacyWriteFirstSRAM,
  bufferDepthOverride: Option[Int] = None) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: RocketTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      traceParams = Some(TraceEncoderParams(
        encoderBaseAddr = 0x3000000 + tp.tileParams.tileId * 0x1000,
        buildEncoder = (p: Parameters) => LazyModule(new TacitEncoder(new TraceCoreParams(
          nGroups = 1,
          xlen = tp.tileParams.core.xLen,
          iaddrWidth = tp.tileParams.core.xLen
        ),
        bufferDepth = bufferDepthOverride.getOrElse(16),
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
        bufferDepth = bufferDepthOverride.getOrElse(16),
        coreStages = 7,
        bpParams = TacitBPParams(xlen = tp.tileParams.core.xLen, n_entries = 1024))(p)),
        useArbiterMonitor = false
      )),
      core = tp.tileParams.core.copy(enableTraceCoreIngress = true)))
    case tp: boom.v3.common.BoomTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      traceParams = Some(TraceEncoderParams(
        encoderBaseAddr = 0x3000000 + tp.tileParams.tileId * 0x1000,
        buildEncoder = (p: Parameters) => LazyModule(new TacitParallelEncoder(new TraceCoreParams(
          nGroups = tp.tileParams.core.retireWidth,
          iretireWidth = 1,
          xlen = tp.tileParams.core.xLen,
          iaddrWidth = tp.tileParams.core.xLen
        ),
        bufferDepth = bufferDepthOverride.getOrElse(64),
        coreStages = 10, // BOOM stalls commit, not fetch
        queueImpl = queueImpl,
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
        bufferDepth = bufferDepthOverride.getOrElse(64),
        coreStages = 10, // BOOM stalls commit, not fetch
        queueImpl = queueImpl,
        )(p)),
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
  new chipyard.config.WithAsidLen(16) ++
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
  new chipyard.config.WithPTECacheEntries(0) ++
  new chipyard.config.WithAsidLen(16) ++
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
  new chipyard.config.WithAsidLen(16) ++
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
  new chipyard.config.WithAsidLen(16) ++
  new freechips.rocketchip.rocket.WithNHugeCores(2) ++
  new chipyard.config.AbstractConfig)

class TacitMediumBoomV3RawByteConfig extends Config(
  new tacit.WithTraceSinkRawByte(2) ++
  new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder ++
  new freechips.rocketchip.subsystem.WithoutTLMonitors ++
  new chipyard.config.WithAsidLen(16) ++
  new boom.v3.common.WithNMediumBooms(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

class TacitLargeBoomV3RawByteConfig extends Config(
  new tacit.WithTraceSinkRawByte(2) ++
  new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder ++
  // new freechips.rocketchip.subsystem.WithoutTLMonitors ++
  new chipyard.config.WithAsidLen(16) ++
  new boom.v3.common.WithNLargeBooms(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

class TacitDualLargeBoomV3RawByteConfig extends Config(
  new tacit.WithTraceSinkRawByte(2) ++
  new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder ++
  // new freechips.rocketchip.subsystem.WithoutTLMonitors ++
  new chipyard.config.WithAsidLen(16) ++
  new boom.v3.common.WithNLargeBooms(2) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

class TacitDualMegaBoomV3RawByteConfig extends Config(
  new tacit.WithTraceSinkRawByte(2) ++
  new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder ++
  // new freechips.rocketchip.subsystem.WithoutTLMonitors ++
  new chipyard.config.WithAsidLen(16) ++
  new boom.v3.common.WithNMegaBooms(2) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

class TacitMegaBoomV3RawByteConfig extends Config(
  new tacit.WithTraceSinkRawByte(2) ++
  new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder ++
  // new freechips.rocketchip.subsystem.WithoutTLMonitors ++
  new chipyard.config.WithAsidLen(16) ++
  new boom.v3.common.WithNMegaBooms(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

// Identical to TacitMegaBoomV3RawByteConfig except the encoder packet buffers use
// the credit-based SRAM queue (no write-first read-under-write dependence) and the
// reserve-based stall contract. Distinct config name so its bitstreams are
// distinguishable from ongoing experiments on the legacy queue.
class TacitMegaBoomV3SRAMQueueRawByteConfig extends Config(
  new tacit.WithTraceSinkRawByte(2) ++
  new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder(tacit.MPQueueImpl.SRAM) ++
  // new freechips.rocketchip.subsystem.WithoutTLMonitors ++
  new chipyard.config.WithAsidLen(16) ++
  new boom.v3.common.WithNMegaBooms(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

// Buffer-depth sweep. Identical to the config above apart from the packet
// buffer depth, which sizes real SRAM and so needs its own bitstream per point
// (and yields its own area/timing numbers, which a runtime knob could not).
// 32 is the floor for a 4-wide core: MultiPortedQueue requires
// reserveCycles*enqBound <= numEntries - enqBound, i.e. 16 <= N - 4, so N >= 20.
class TacitMegaBoomV3SRAMQueueDepthConfig(depth: Int) extends Config(
  new tacit.WithTraceSinkRawByte(2) ++
  new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder(tacit.MPQueueImpl.SRAM, Some(depth)) ++
  new chipyard.config.WithAsidLen(16) ++
  new boom.v3.common.WithNMegaBooms(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

class TacitMegaBoomV3SRAMQueueDepth32Config  extends TacitMegaBoomV3SRAMQueueDepthConfig(32)
// 64 is today's default; named explicitly so the sweep's three points are
// generated by one parameterised path rather than two paths that must agree.
class TacitMegaBoomV3SRAMQueueDepth64Config  extends TacitMegaBoomV3SRAMQueueDepthConfig(64)
class TacitMegaBoomV3SRAMQueueDepth128Config extends TacitMegaBoomV3SRAMQueueDepthConfig(128)

// Lossy-mode integration testing: MegaBoom with the SRAM queue and a sink that
// only accepts 1 cycle in 4 (target 3). The arbiter monitor captures the byte
// stream upstream of the sinks, so the throttled run is still decodable from
// trace_monitor_boom_tile_0.encoded.trace.
class TacitMegaBoomV3BackpressureConfig extends Config(
  new tacit.WithTraceSinkBackpressure(3) ++ // pattern via +tacit_bp_* plusargs; default: ready 1 cycle in 4
  new tacit.WithTraceSinkRawByte(2) ++
  new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder(tacit.MPQueueImpl.SRAM) ++
  new chipyard.config.WithAsidLen(16) ++
  new boom.v3.common.WithNMegaBooms(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

// Same as TacitMegaBoomV3BackpressureConfig with BOOM's commit log printf, for
// debugging sim hangs (+verbose shows every committed instruction).
class TacitMegaBoomV3BackpressurePrintfConfig extends Config(
  new boom.v3.common.WithBoomCommitLogPrintf ++
  new TacitMegaBoomV3BackpressureConfig)

class TacitMegaBoomV3PrefetchRoCCRawByteConfig extends Config(
  new tacit.WithTraceSinkRawByte(2) ++
  new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder ++
  // new freechips.rocketchip.subsystem.WithoutTLMonitors ++
  new boom.v3.common.WithSoftwarePrefetchRoCC() ++
  new chipyard.config.WithAsidLen(16) ++
  new boom.v3.common.WithNMegaBooms(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

// 4-wide BOOM with the prefetch RoCC + Tacit tracing, given the same prefetch
// support as the Medium prefetch config so the two are comparable.
// The plain TacitMegaBoomV3PrefetchRoCCRawByteConfig above inherits MegaBoom's
// defaults: prefetchCommitToL1=false (refills PARK in s_prefetch and only pay off
// via secondary merge, and parked MSHRs are stealable) and nMSHRs=8. A prefetch
// stream of distance D needs D refills in flight, so at D=16 on 8 MSHRs the
// prefetches evict each other and contend with demand misses -- measured on
// tests/prefetch-bench (2026-07-30, VCS): 4-wide baseline 15 cyc/access,
// D=16 18 cyc/access (+18.5%), D=64 +20.0%, i.e. prefetching LOST, while the
// 2-wide commit-mode/16-MSHR config gained 31.8% at D=16 on the same binary.
// This config isolates that: commit-to-L1 + 16 MSHRs on the 4-wide core, so any
// remaining difference vs Medium is core width rather than prefetch plumbing.
// WithBoomEnablePrefetching is deliberately absent -- WithNMegaBooms already sets
// enablePrefetching=true (config-mixins.scala:247), unlike Medium.
class TacitMegaBoomV3PrefetchRoCCCommitRawByteConfig extends Config(
  new tacit.WithTraceSinkRawByte(2) ++
  new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder ++
  new boom.v3.common.WithBoomPrefetchCommitToL1 ++ // commit prefetched lines to the data array (vs parking in s_prefetch)
  new boom.v3.common.WithBoomDCacheMSHRs(16) ++    // MegaBoom defaults to 8; a distance-D prefetch stream needs D in flight
  new boom.v3.common.WithSoftwarePrefetchRoCC() ++
  new chipyard.config.WithAsidLen(16) ++
  new boom.v3.common.WithNMegaBooms(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

// 3-wide BOOM with the prefetch RoCC + Tacit tracing (width-study point; modeled
// on the Mega prefetch config, defaults + SoftwarePrefetchRoCC, only the width differs)
class TacitLargeBoomV3PrefetchRoCCRawByteConfig extends Config(
  new tacit.WithTraceSinkRawByte(2) ++
  new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder ++
  new boom.v3.common.WithSoftwarePrefetchRoCC() ++
  new chipyard.config.WithAsidLen(16) ++
  new boom.v3.common.WithNLargeBooms(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

// 2-wide BOOM for fast FPGA builds, beefed up to 16 dcache MSHRs so software
// prefetches have miss capacity, with the prefetch RoCC and Tacit tracing
class TacitMediumBoomV3PrefetchRoCCRawByteConfig extends Config(
  new tacit.WithTraceSinkRawByte(2) ++
  new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder ++
  new boom.v3.common.WithBoomPrefetchCommitToL1 ++ // commit prefetched lines to the data array (vs parking in s_prefetch)
  new boom.v3.common.WithBoomEnablePrefetching ++ // Medium lacks it (NL prefetcher parity with Mega; park mode if commit knob removed)
  new boom.v3.common.WithBoomDCacheMSHRs(16) ++
  new boom.v3.common.WithSoftwarePrefetchRoCC() ++
  new chipyard.config.WithAsidLen(16) ++
  new boom.v3.common.WithNMediumBooms(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

class TacitUltraBoomV3PrefetchRoCCRawByteConfig extends Config(
  new tacit.WithTraceSinkRawByte(2) ++
  new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder ++
  // new freechips.rocketchip.subsystem.WithoutTLMonitors ++
  new boom.v3.common.WithSoftwarePrefetchRoCC() ++
  new chipyard.config.WithAsidLen(16) ++
  new boom.v3.common.WithNUltraBooms(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

class TacitMegaBoomV3RawByteShyDMAConfig extends Config(
  new tacit.WithTraceSinkRawByte(2) ++
  new tacit.WithTraceSinkDMA(1, nSource = 2) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder ++
  // new freechips.rocketchip.subsystem.WithoutTLMonitors ++
  new chipyard.config.WithAsidLen(16) ++
  new boom.v3.common.WithNMegaBooms(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

class TacitLargeBoomV3NoASIDRawByteConfig extends Config(
  new tacit.WithTraceSinkRawByte(2) ++
  new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder ++
  new freechips.rocketchip.subsystem.WithoutTLMonitors ++
  new boom.v3.common.WithNLargeBooms(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

class TacitLargeBoomV4Config extends Config(
  new tacit.WithTraceSinkRawByte(2) ++
  new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder ++
  new freechips.rocketchip.subsystem.WithoutTLMonitors ++
  // new boom.v4.common.WithBoomCommitLogPrintf ++
  new boom.v4.common.WithNLargeBooms(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

class TacitLargeBoomV4RawByteConfig extends Config(
  new tacit.WithTraceSinkRawByte(2) ++
  new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder ++
  new chipyard.config.WithAsidLen(16) ++
  new freechips.rocketchip.subsystem.WithoutTLMonitors ++
  new boom.v4.common.WithNLargeBooms(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

class TacitLargeBoomV4RawByteNoASIDConfig extends Config(
  new tacit.WithTraceSinkRawByte(2) ++
  new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder ++
  new freechips.rocketchip.subsystem.WithoutTLMonitors ++
  new boom.v4.common.WithNLargeBooms(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

class TacitMediumBoomV4RawByteConfig extends Config(
  new tacit.WithTraceSinkRawByte(2) ++
  new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitEncoder ++
  new chipyard.config.WithAsidLen(16) ++
  new freechips.rocketchip.subsystem.WithoutTLMonitors ++
  new boom.v4.common.WithNMediumBooms(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)

// Stress-test encoder: MegaBoom 4-wide with tiny buffers to trigger buffer pressure quickly
class WithTacitStressEncoder extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: boom.v3.common.BoomTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      traceParams = Some(TraceEncoderParams(
        encoderBaseAddr = 0x3000000 + tp.tileParams.tileId * 0x1000,
        buildEncoder = (p: Parameters) => LazyModule(new TacitParallelEncoder(new TraceCoreParams(
          nGroups = tp.tileParams.core.retireWidth,
          iretireWidth = 1,
          xlen = tp.tileParams.core.xLen,
          iaddrWidth = tp.tileParams.core.xLen
        ),
        bufferDepth = 16,
        coreStages = 5,
        queueImpl = tacit.MPQueueImpl.LegacyWriteFirstSRAM,
        )(p)),
        useArbiterMonitor = false
      )),
      core = tp.tileParams.core.copy(enableTraceCoreIngress = true)))
    case other => other
  }
})

class TacitMegaBoomV3StressConfig extends Config(
  new tacit.WithTraceSinkRawByte(2) ++
  new tacit.WithTraceSinkDMA(1) ++
  new tacit.WithTraceSinkAlways(0) ++
  new chipyard.config.WithTraceArbiterMonitor ++
  new chipyard.WithTacitStressEncoder ++
  new chipyard.config.WithAsidLen(16) ++
  new boom.v3.common.WithNMegaBooms(1) ++
  new chipyard.config.WithSystemBusWidth(128) ++
  new chipyard.config.AbstractConfig)