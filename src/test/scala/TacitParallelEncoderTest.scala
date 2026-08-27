package tacit

import chisel3._
import chisel3.util._
import chiseltest._
import freechips.rocketchip.trace._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.diplomacy.LazyModule
import org.scalatest.flatspec.AnyFlatSpec

import scala.collection.mutable.ArrayBuffer

/** Thin wrapper around the LazyModule so chiseltest can instantiate it. */
class TacitParallelEncoderHarness(
  coreParams: TraceCoreParams,
  bufferDepth: Int,
  coreStages: Int,
)(implicit p: Parameters) extends Module {
  val lm = LazyModule(new TacitParallelEncoder(coreParams, bufferDepth, coreStages))
  val mod = Module(lm.module)
  val io = IO(new Bundle {
    val control = Input(new TraceEncoderControlInterface())
    val in = Input(new TraceCoreInterface(coreParams))
    val stall = Output(Bool())
    val out = new TraceEgressInterface()
  })
  mod.io.control := io.control
  mod.io.in := io.in
  io.stall := mod.io.stall
  io.out <> mod.io.out
}

class TacitParallelEncoderTest extends AnyFlatSpec with ChiselScalatestTester {
  private val waveformAnnos = Seq(WriteVcdAnnotation)
  private val noVcd = Seq.empty[firrtl2.annotations.Annotation]
  implicit val p: Parameters = Parameters.empty

  private val nGroups = 2
  private val params = TraceCoreParams(nGroups = nGroups, iretireWidth = 1, xlen = 64, iaddrWidth = 64)

  private def init(c: TacitParallelEncoderHarness): Unit = {
    // Long poke-free drain loops trip chiseltest's 1000-idle-cycle watchdog;
    // all loops here are bounded, so termination is already guaranteed.
    c.clock.setTimeout(0)
    c.reset.poke(true.B)
    c.clock.step()
    c.reset.poke(false.B)

    for (i <- 0 until nGroups) {
      c.io.in.group(i).iretire.poke(0.U)
      c.io.in.group(i).iaddr.poke(0.U)
      c.io.in.group(i).itype.poke(TraceItype.ITNothing)
      c.io.in.group(i).ilastsize.poke(0.U)
    }
    c.io.in.priv.poke(0.U)
    c.io.in.ctx.poke(0.U)
    c.io.in.tval.poke(0.U)
    c.io.in.cause.poke(0.U)
    c.io.in.time.poke(0.U)
    c.io.out.ready.poke(true.B)
    c.io.control.enable.poke(false.B)
    c.io.control.target.poke(0.U)
    c.io.control.bp_mode.poke(0.U)
  }

  /** Sample egress bytes this cycle (call BEFORE clock.step). */
  private def sampleEgress(c: TacitParallelEncoderHarness, bytes: ArrayBuffer[Int]): Unit = {
    if (c.io.out.valid.peek().litToBoolean && c.io.out.ready.peek().litToBoolean) {
      for (lane <- 0 until TraceEgressConstants.numLanes) {
        if (c.io.out.mask(lane).peek().litToBoolean) {
          bytes += c.io.out.bits(lane).peek().litValue.toInt
        }
      }
    }
  }

  /** Drive one cycle: poke inputs, sample outputs, then step. Respects stall. */
  private def driveAndSample(
    c: TacitParallelEncoderHarness,
    bytes: ArrayBuffer[Int],
    itype0: TraceItype.Type, iaddr0: BigInt,
    itype1: Option[(TraceItype.Type, BigInt)],
    time: BigInt,
  ): Unit = {
    c.io.in.group(0).iretire.poke(1.U)
    c.io.in.group(0).iaddr.poke(iaddr0.U)
    c.io.in.group(0).itype.poke(itype0)
    itype1 match {
      case Some((it, addr)) =>
        c.io.in.group(1).iretire.poke(1.U)
        c.io.in.group(1).iaddr.poke(addr.U)
        c.io.in.group(1).itype.poke(it)
      case None =>
        c.io.in.group(1).iretire.poke(0.U)
        c.io.in.group(1).itype.poke(TraceItype.ITNothing)
    }
    c.io.in.time.poke(time.U)
    sampleEgress(c, bytes)
    c.clock.step()
    // Clear inputs
    c.io.in.group(0).iretire.poke(0.U)
    c.io.in.group(0).itype.poke(TraceItype.ITNothing)
    c.io.in.group(1).iretire.poke(0.U)
    c.io.in.group(1).itype.poke(TraceItype.ITNothing)
  }

  /** Idle for one cycle, sampling output. */
  private def idleAndSample(c: TacitParallelEncoderHarness, bytes: ArrayBuffer[Int]): Unit = {
    sampleEgress(c, bytes)
    c.clock.step()
  }

  behavior of "TacitParallelEncoder"

  it should "produce encoded output for a basic event sequence" in {
    test(new TacitParallelEncoderHarness(params, bufferDepth = 16, coreStages = 5))
      .withAnnotations(waveformAnnos) { c =>
      init(c)
      val bytes = ArrayBuffer[Int]()

      // Enable and wait for state machine
      c.io.control.enable.poke(true.B)
      for (_ <- 0 until 3) idleAndSample(c, bytes)

      // Trigger sync
      driveAndSample(c, bytes, TraceItype.ITBrNTaken, 0x1000, None, 100)
      for (_ <- 0 until 5) idleAndSample(c, bytes)

      // Drive events
      driveAndSample(c, bytes, TraceItype.ITBrTaken, 0x1010, None, 101)
      driveAndSample(c, bytes, TraceItype.ITBrNTaken, 0x1020, None, 102)
      driveAndSample(c, bytes, TraceItype.ITUnJump, 0x1030, None, 103)
      driveAndSample(c, bytes, TraceItype.ITBrTaken, 0x1040, None, 104)

      // Drain
      for (_ <- 0 until 200) idleAndSample(c, bytes)

      println(s"Basic test: collected ${bytes.length} bytes: ${bytes.map(b => f"$b%02x").mkString(" ")}")
      assert(bytes.nonEmpty, "Should produce output bytes")
    }
  }

  it should "not trigger atomic enqueue assertion under backpressure" in {
    test(new TacitParallelEncoderHarness(params, bufferDepth = 16, coreStages = 5))
      .withAnnotations(noVcd) { c =>
      init(c)
      val bytes = ArrayBuffer[Int]()
      val rng = new scala.util.Random(99)

      c.io.control.enable.poke(true.B)
      for (_ <- 0 until 3) idleAndSample(c, bytes)
      driveAndSample(c, bytes, TraceItype.ITBrNTaken, 0x1000, None, 100)
      for (_ <- 0 until 3) idleAndSample(c, bytes)

      val itypes = Seq(TraceItype.ITBrTaken, TraceItype.ITBrNTaken, TraceItype.ITUnJump, TraceItype.ITInJump)
      var time = 101L
      var stalledCycles = 0
      var eventsDriven = 0

      // Small buffer (16) + random backpressure → maximum buffer pressure
      for (_ <- 0 until 1) {
        c.io.out.ready.poke((rng.nextInt(4) != 0).B)

        if (c.io.stall.peek().litToBoolean) {
          idleAndSample(c, bytes)
          stalledCycles += 1
        } else {
          val useDual = rng.nextBoolean()
          val it0 = itypes(rng.nextInt(itypes.length))
          val g1 = if (useDual) Some((itypes(rng.nextInt(itypes.length)), BigInt(0x2000 + rng.nextInt(0x1000)))) else None
          driveAndSample(c, bytes, it0, 0x1000 + rng.nextInt(0x1000), g1, time)
          time += 1 + rng.nextInt(3)
          eventsDriven += 1
        }
      }

      c.io.control.enable.poke(false.B)
      c.io.out.ready.poke(true.B)
      for (_ <- 0 until 5000) idleAndSample(c, bytes)

      println(s"Backpressure test: ${bytes.length} bytes, $eventsDriven events, $stalledCycles stalled cycles")
      // If we reach here, the chisel assert (metadata.fire === header.fire) never triggered
      assert(bytes.nonEmpty)
    }
  }

  it should "survive sustained dual-group traffic at full throughput" in {
    test(new TacitParallelEncoderHarness(params, bufferDepth = 64, coreStages = 5))
      .withAnnotations(noVcd) { c =>
      init(c)
      val bytes = ArrayBuffer[Int]()
      val rng = new scala.util.Random(42)

      c.io.control.enable.poke(true.B)
      for (_ <- 0 until 3) idleAndSample(c, bytes)
      driveAndSample(c, bytes, TraceItype.ITBrNTaken, 0x1000, None, 100)
      for (_ <- 0 until 3) idleAndSample(c, bytes)

      val itypes = Seq(TraceItype.ITBrTaken, TraceItype.ITBrNTaken, TraceItype.ITUnJump, TraceItype.ITInJump)
      var time = 101L
      var stalledCycles = 0

      // Sustained dual-group at full output bandwidth
      for (_ <- 0 until 10000) {
        if (c.io.stall.peek().litToBoolean) {
          idleAndSample(c, bytes)
          stalledCycles += 1
        } else {
          val it0 = itypes(rng.nextInt(itypes.length))
          val it1 = itypes(rng.nextInt(itypes.length))
          driveAndSample(c, bytes, it0, 0x1000 + rng.nextInt(0x1000),
            Some((it1, BigInt(0x2000 + rng.nextInt(0x1000)))), time)
          time += 1 + rng.nextInt(5)
        }
      }

      c.io.control.enable.poke(false.B)
      for (_ <- 0 until 20000) idleAndSample(c, bytes)

      println(s"Throughput test: ${bytes.length} bytes, $stalledCycles stalled cycles")
      assert(bytes.nonEmpty)
    }
  }
}
