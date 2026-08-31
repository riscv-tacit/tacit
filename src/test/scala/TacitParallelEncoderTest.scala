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
    val perf = Output(new TraceEncoderPerformanceInterface())
    val out = new TraceEgressInterface()
  })
  mod.io.control := io.control
  mod.io.in := io.in
  io.stall := mod.io.stall
  io.perf := mod.io.perf
  io.out <> mod.io.out
}

class TacitParallelEncoderTest extends AnyFlatSpec with ChiselScalatestTester {
  // Verilator: the golden tests simulate tens of thousands of cycles, far too slow for treadle
  private val waveformAnnos = Seq(VerilatorBackendAnnotation, WriteVcdAnnotation)
  private val noVcd = Seq[firrtl2.annotations.Annotation](VerilatorBackendAnnotation)
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
    c.io.control.lossy.poke(false.B)
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

  /** One retired group carrying no control-flow event (advances the ingress pipeline only). */
  private def drivePlainAndSample(c: TacitParallelEncoderHarness, bytes: ArrayBuffer[Int], iaddr: BigInt, time: BigInt): Unit = {
    c.io.in.group(0).iretire.poke(1.U)
    c.io.in.group(0).iaddr.poke(iaddr.U)
    c.io.in.group(0).itype.poke(TraceItype.ITNothing)
    c.io.in.group(1).iretire.poke(0.U)
    c.io.in.group(1).itype.poke(TraceItype.ITNothing)
    c.io.in.time.poke(time.U)
    sampleEgress(c, bytes)
    c.clock.step()
    c.io.in.group(0).iretire.poke(0.U)
  }

  // ---------------------------------------------------------------------------
  // Stimulus recording and a byte-level packet parser for golden checks
  // ---------------------------------------------------------------------------

  /** One retired group as driven: retire time and its control-flow messages (slot order). */
  case class Group(time: Long, msgs: Seq[(TraceItype.Type, BigInt)], slot0Addr: BigInt)

  sealed trait Pkt { def kind: String }
  case class DataPkt(kind: String, delta: Long, targetAddr: Option[BigInt]) extends Pkt
  case class SyncPkt(syncType: Int, prvTo: Int, ctx: Long, trapAddr: Long, pc: BigInt, time: Long) extends Pkt {
    def kind = syncType match { case 1 => "Start"; case 3 => "End"; case 4 => "Pause"; case 5 => "Resume"; case t => s"Sync$t" }
  }

  /** Split a raw TACIT byte stream into packets (mirrors software/tacit_decoder/src/frontend/packet.rs). */
  def parsePackets(bytes: Seq[Int]): Seq[Pkt] = {
    var i = 0
    def varint(): Long = {
      var v = 0L; var shift = 0
      while ({ val b = bytes(i); i += 1; v |= (b & 0x7f).toLong << shift; shift += 7; (b & 0x80) == 0 }) {}
      v
    }
    val out = ArrayBuffer[Pkt]()
    while (i < bytes.length) {
      val h = bytes(i); i += 1
      val c = h & 3
      if (c != 2) {
        out += DataPkt(Seq("TB", "NT", "", "IJ")(c), (h >> 2) & 0x3f, None)
      } else {
        val f = (h >> 2) & 7
        val func3 = h >> 5
        f match {
          case 0 | 1 | 3 => out += DataPkt(Seq("TB", "NT", "", "IJ")(f), varint(), None)
          case 2 => val ta = varint(); out += DataPkt("UJ", varint(), Some(BigInt(ta)))
          case 5 =>
            val prv = bytes(i); i += 1
            assert((prv >> 6) == 2 && (prv & 7) == 0, f"bad sync prv byte $prv%02x")
            val ctx = varint(); val trap = varint(); val pc = BigInt(varint()) << 1; val time = varint()
            out += SyncPkt(func3, (prv >> 3) & 7, ctx, trap, pc, time)
          case other => fail(f"unexpected f_header $other in byte $h%02x at offset ${i - 1}")
        }
      }
    }
    out.toSeq
  }

  /** Write the raw stream next to the test run for offline inspection (TACIT_DUMP_DIR, if set). */
  private def dumpBytes(bytes: Seq[Int], name: String): Unit = {
    sys.env.get("TACIT_DUMP_DIR").foreach { dir =>
      val f = new java.io.FileOutputStream(s"$dir/$name.bin")
      f.write(bytes.map(_.toByte).toArray); f.close()
    }
  }

  private def kindOf(t: TraceItype.Type): String = t match {
    case TraceItype.ITBrTaken => "TB"; case TraceItype.ITBrNTaken => "NT"
    case TraceItype.ITInJump => "IJ"; case TraceItype.ITUnJump => "UJ"
    case other => fail(s"unsupported stimulus itype $other")
  }

  /** Check a parsed stream against the driven groups: grammar, sync bindings,
    * dropped counts, and that every covered (kind, time) event matches the stimulus
    * with the gap windows removed. Returns the number of gaps. */
  def checkGolden(pkts: Seq[Pkt], groups: Seq[Group]): Int = {
    assert(pkts.nonEmpty && pkts.head.kind == "Start", s"stream must start with Start, got ${pkts.headOption}")
    assert(pkts.last.kind == "End", s"stream must end with End, got ${pkts.last}")
    val start = pkts.head.asInstanceOf[SyncPkt]
    var ts = start.time
    var inGap = false
    var pauseTs = -1L
    var gaps = 0
    val gapWindows = ArrayBuffer[(Long, Long)]()   // [pause.time, resume.time)
    val covered = ArrayBuffer[(String, Long)]()
    for (p <- pkts.drop(1)) p match {
      case d: DataPkt =>
        assert(!inGap, s"data packet $d inside a gap")
        ts += d.delta
        covered += ((d.kind, ts))
      case sp: SyncPkt if sp.kind == "Pause" =>
        assert(!inGap, "Pause inside a gap")
        assert(sp.trapAddr == 0, s"Pause trap_addr must be 0, got ${sp.trapAddr}")
        // binds to the first message of a driven group
        val g = groups.find(g => g.time == sp.time)
        assert(g.isDefined && g.get.msgs.nonEmpty && g.get.msgs.head._2 == sp.pc,
          s"Pause $sp does not bind to the first message of a driven group (${g})")
        inGap = true; pauseTs = sp.time; gaps += 1
      case sp: SyncPkt if sp.kind == "Resume" =>
        assert(inGap, "Resume without Pause")
        assert(sp.time > pauseTs, s"Resume time ${sp.time} not after Pause time $pauseTs")
        val g = groups.find(g => g.time == sp.time)
        assert(g.isDefined && g.get.slot0Addr == sp.pc, s"Resume $sp does not bind to slot 0 of a driven group ($g)")
        val expectedDropped = groups.filter(g => g.time >= pauseTs && g.time < sp.time).map(_.msgs.length).sum
        assert(sp.trapAddr == expectedDropped, s"Resume dropped=${sp.trapAddr}, stimulus lost $expectedDropped in [$pauseTs, ${sp.time})")
        gapWindows += ((pauseTs, sp.time))
        ts = sp.time; inGap = false
      case sp: SyncPkt if sp.kind == "End" =>
        assert(!inGap, "End directly after Pause")
        assert(sp.trapAddr == 0)
      case sp => fail(s"unexpected sync $sp")
    }
    val expected = groups
      .filter(g => !gapWindows.exists { case (a, b) => g.time >= a && g.time < b })
      .flatMap(g => g.msgs.map { case (t, _) => (kindOf(t), g.time) })
    if (covered.toSeq != expected) {
      val firstBad = covered.zip(expected).indexWhere { case (a, b) => a != b }
      val at = if (firstBad < 0) math.min(covered.length, expected.length) else firstBad
      fail(s"covered events differ: got ${covered.length}, expected ${expected.length}; first mismatch at $at\n" +
        s"  got      ${covered.slice(at - 4, at + 4)}\n  expected ${expected.slice(at - 4, at + 4)}\n" +
        s"  last groups: ${groups.filter(_.msgs.nonEmpty).takeRight(4)}\n  last packets: ${pkts.takeRight(8)}")
    }
    gaps
  }

  /** Drive a random dual-lane event stream. Models the core the way BOOM attaches:
    * the commit decision at cycle T honors stall(T), but the committed group reaches
    * the encoder's io.in one cycle later (RegNext in the core), so one more group can
    * arrive after stall asserts. In lossy mode asserts stall never fires. `readyProb`
    * is the per-cycle probability that the sink accepts output. */
  private def driveRandom(c: TacitParallelEncoderHarness, bytes: ArrayBuffer[Int], groups: ArrayBuffer[Group],
                          rng: scala.util.Random, n: Int, readyProb: Double, lossy: Boolean, startTime: Long): Long = {
    val itypes = Seq(TraceItype.ITBrTaken, TraceItype.ITBrNTaken, TraceItype.ITUnJump, TraceItype.ITInJump)
    var time = startTime
    var driven = 0
    // group committed last cycle, delivered to io.in this cycle
    var pending: Option[(TraceItype.Type, BigInt, Option[(TraceItype.Type, BigInt)], Long)] = None
    def deliver(): Unit = pending match {
      case Some((it0, a0, g1, t)) => driveAndSample(c, bytes, it0, a0, g1, t); pending = None
      case None => idleAndSample(c, bytes)
    }
    while (driven < n || pending.nonEmpty) {
      c.io.out.ready.poke((rng.nextDouble() < readyProb).B)
      if (lossy) assert(!c.io.stall.peek().litToBoolean, "stall asserted in lossy mode")
      val stalled = !lossy && c.io.stall.peek().litToBoolean
      if (stalled || driven >= n || rng.nextInt(3) == 0) {
        deliver()                          // no commit this cycle (stalled or bubble)
      } else {
        // Like a real core, most retired instructions carry no control-flow event:
        // each lane is a plain instruction (ITNothing) with probability 1/2, and a
        // group may have a plain lane 0 with a message in lane 1.
        val plain0 = rng.nextBoolean()
        val it0 = if (plain0) TraceItype.ITNothing else itypes(rng.nextInt(itypes.length))
        val a0 = BigInt(0x1000 + 2 * rng.nextInt(0x800))
        val a1 = BigInt(0x2000 + 2 * rng.nextInt(0x800))
        val g1 = if (rng.nextBoolean()) Some((if (rng.nextBoolean()) TraceItype.ITNothing else itypes(rng.nextInt(itypes.length)), a1)) else None
        deliver()                          // last cycle's commit lands now
        pending = Some((it0, a0, g1, time)) // this cycle's commit lands next cycle
        val msgs = (Seq((it0, a0)) ++ g1.toSeq).filter(_._1 != TraceItype.ITNothing)
        groups += Group(time, msgs, a0)
        time += 1 + rng.nextInt(3)
        driven += 1
      }
    }
    time
  }

  /** Disable and drain: keep retiring plain groups so End (and a pending Resume) can bind. */
  private def disableAndDrain(c: TacitParallelEncoderHarness, bytes: ArrayBuffer[Int], groups: ArrayBuffer[Group],
                              startTime: Long, cycles: Int): Unit = {
    var time = startTime
    // End binds to the group in ingress_0 when it enqueues and the group then in
    // ingress_1 is discarded (spec 4.5). Retire one plain group before disabling so
    // the discarded group carries no events and the last real group is encoded.
    drivePlainAndSample(c, bytes, 0x7ffe, time)
    groups += Group(time, Seq.empty, BigInt(0x7ffe))
    time += 3
    c.io.control.enable.poke(false.B)
    c.io.out.ready.poke(true.B)
    for (k <- 0 until cycles) {
      // a real core does not retire while stalled (lossless mode with a full queue)
      if (k % 4 == 0 && !c.io.stall.peek().litToBoolean) {
        drivePlainAndSample(c, bytes, 0x7000 + 2 * (k % 64), time)
        groups += Group(time, Seq.empty, BigInt(0x7000 + 2 * (k % 64)))
        time += 3
      } else idleAndSample(c, bytes)
    }
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

  it should "decode a lossless random stream back to the driven events" in {
    test(new TacitParallelEncoderHarness(params, bufferDepth = 16, coreStages = 5))
      .withAnnotations(noVcd) { c =>
      init(c)
      val bytes = ArrayBuffer[Int](); val groups = ArrayBuffer[Group]()
      val rng = new scala.util.Random(7)
      c.io.control.enable.poke(true.B)
      for (_ <- 0 until 3) idleAndSample(c, bytes)
      val t = driveRandom(c, bytes, groups, rng, n = 2000, readyProb = 0.3, lossy = false, startTime = 100)
      disableAndDrain(c, bytes, groups, t, 3000)
      dumpBytes(bytes.toSeq, "lossless_golden")
      val pkts = parsePackets(bytes.toSeq)
      val gaps = checkGolden(pkts, groups.toSeq)
      assert(gaps == 0, "lossless stream must have no gaps")
      println(s"Lossless golden: ${bytes.length} bytes, ${pkts.length} packets, ${groups.map(_.msgs.length).sum} events")
    }
  }

  it should "not lose groups in lossless mode at full commit width under a blocked sink" in {
    // Worst case for the stall reserve: every retired lane carries a message, no
    // bubbles, the core keeps committing until it sees stall, and the committed
    // group lands one cycle later (BOOM's RegNext). The sink barely drains.
    test(new TacitParallelEncoderHarness(params, bufferDepth = 16, coreStages = 5))
      .withAnnotations(noVcd) { c =>
      init(c)
      val bytes = ArrayBuffer[Int](); val groups = ArrayBuffer[Group]()
      val itypes = Seq(TraceItype.ITBrTaken, TraceItype.ITBrNTaken, TraceItype.ITInJump)
      val rng = new scala.util.Random(3)
      c.io.control.enable.poke(true.B)
      for (_ <- 0 until 3) idleAndSample(c, bytes)
      var time = 100L
      var pending: Option[(TraceItype.Type, BigInt, Option[(TraceItype.Type, BigInt)], Long)] = None
      def deliver(): Unit = pending match {
        case Some((it0, a0, g1, t)) => driveAndSample(c, bytes, it0, a0, g1, t); pending = None
        case None => idleAndSample(c, bytes)
      }
      for (cyc <- 0 until 4000) {
        c.io.out.ready.poke((cyc % 20 == 0).B)
        val stalled = c.io.stall.peek().litToBoolean
        deliver()
        if (!stalled) {
          val it0 = itypes(rng.nextInt(itypes.length)); val it1 = itypes(rng.nextInt(itypes.length))
          val a0 = BigInt(0x1000 + 2 * rng.nextInt(0x800)); val a1 = BigInt(0x2000 + 2 * rng.nextInt(0x800))
          pending = Some((it0, a0, Some((it1, a1)), time))
          groups += Group(time, Seq((it0, a0), (it1, a1)), a0)
          time += 1
        }
      }
      deliver()
      disableAndDrain(c, bytes, groups, time, 6000)
      val pkts = parsePackets(bytes.toSeq)
      val gaps = checkGolden(pkts, groups.toSeq)
      assert(gaps == 0)
      println(s"Lossless worst case: ${groups.length} groups, ${pkts.length} packets, all events recovered")
    }
  }

  it should "pause and resume instead of stalling in lossy mode" in {
    test(new TacitParallelEncoderHarness(params, bufferDepth = 16, coreStages = 5))
      .withAnnotations(noVcd) { c =>
      init(c)
      val bytes = ArrayBuffer[Int](); val groups = ArrayBuffer[Group]()
      val rng = new scala.util.Random(11)
      c.io.control.lossy.poke(true.B)
      c.io.control.enable.poke(true.B)
      for (_ <- 0 until 3) idleAndSample(c, bytes)
      // sink accepts only ~1 cycle in 5 while the core retires at full rate: sustained overflow
      val t = driveRandom(c, bytes, groups, rng, n = 3000, readyProb = 0.2, lossy = true, startTime = 100)
      disableAndDrain(c, bytes, groups, t, 4000)
      val pkts = parsePackets(bytes.toSeq)
      val gaps = checkGolden(pkts, groups.toSeq)
      val dropped = pkts.collect { case sp: SyncPkt if sp.kind == "Resume" => sp.trapAddr }.sum
      println(s"Lossy golden: ${bytes.length} bytes, ${pkts.length} packets, $gaps gaps, $dropped of ${groups.map(_.msgs.length).sum} events dropped")
      assert(gaps > 0, "expected at least one gap under this much backpressure")
      assert(dropped > 0)
    }
  }

  it should "emit Resume then End when disabled while paused" in {
    test(new TacitParallelEncoderHarness(params, bufferDepth = 16, coreStages = 5))
      .withAnnotations(noVcd) { c =>
      init(c)
      val bytes = ArrayBuffer[Int](); val groups = ArrayBuffer[Group]()
      val rng = new scala.util.Random(5)
      c.io.control.lossy.poke(true.B)
      c.io.control.enable.poke(true.B)
      for (_ <- 0 until 3) idleAndSample(c, bytes)
      // sink completely blocked: the first groups fill the queue, then everything is dropped
      val t = driveRandom(c, bytes, groups, rng, n = 200, readyProb = 0.0, lossy = true, startTime = 100)
      assert(c.io.perf.paused.peek().litToBoolean, "encoder should be inside a gap with the sink blocked")
      disableAndDrain(c, bytes, groups, t, 4000)
      val pkts = parsePackets(bytes.toSeq)
      checkGolden(pkts, groups.toSeq)
      val tail = pkts.takeRight(2).map(_.kind)
      assert(tail == Seq("Resume", "End"), s"expected ... Resume End, got ${pkts.map(_.kind).mkString(" ")}")
      println(s"Disable-while-paused: ${pkts.map(_.kind).mkString(" ")}")
    }
  }
}
