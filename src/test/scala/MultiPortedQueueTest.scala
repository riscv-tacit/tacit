package tacit

import chisel3._
import chisel3.util.DecoupledIO
import chiseltest._
import org.scalatest.flatspec.AnyFlatSpec
import scala.collection.mutable.ArrayBuffer
import scala.util.Random

class MultiPortedQueueTest extends AnyFlatSpec with ChiselScalatestTester {
  private val waveformAnnos = Seq(WriteVcdAnnotation)

  // --- helpers ---

  /** Poke enqueue ports. Each element in `vals` corresponds to an enq port;
    * None means that port is not valid this cycle. */
  private def pokeEnqs(enqs: Vec[DecoupledIO[UInt]], vals: Seq[Option[Int]]): Unit = {
    for ((v, i) <- vals.zipWithIndex) {
      v match {
        case Some(d) =>
          enqs(i).valid.poke(true.B)
          enqs(i).bits.poke(d.U)
        case None =>
          enqs(i).valid.poke(false.B)
          enqs(i).bits.poke(0.U)
      }
    }
  }

  private def clearEnqs(enqs: Vec[DecoupledIO[UInt]], numInputs: Int): Unit = {
    pokeEnqs(enqs, Seq.fill(numInputs)(None))
  }

  /** Run a full scenario against either MultiPortedRegQueue or MultiPortedQueue.
    *
    * @param enqBatches Sequence of per-cycle enqueue batches. Each batch is
    *                   a Seq[Option[Int]] of length numInputs.
    * @param deqReady   Function from cycle number to whether deq.ready is asserted.
    * @param numInputs  Number of enqueue ports.
    * @param maxCycles  Timeout.
    * @return The sequence of dequeued values, in order.
    */
  private def runScenario(
    c: MultiPortedRegQueue[UInt],
    enqBatches: Seq[Seq[Option[Int]]],
    deqReady: Int => Boolean,
    numInputs: Int,
    maxCycles: Int = 2000
  ): Seq[Int] = {
    val pending = ArrayBuffer.from(enqBatches)
    val expectedOrder = ArrayBuffer[Int]()
    val dequeued = ArrayBuffer[Int]()
    var cycle = 0

    while ((pending.nonEmpty || expectedOrder.length > dequeued.length) && cycle < maxCycles) {
      c.io.deq.ready.poke(deqReady(cycle).B)

      // Drive enqueue ports
      if (pending.nonEmpty) pokeEnqs(c.io.enqs, pending.head) else clearEnqs(c.io.enqs, numInputs)

      // Sample fires
      val enqFires = (0 until numInputs).map(i =>
        c.io.enqs(i).valid.peek().litToBoolean && c.io.enqs(i).ready.peek().litToBoolean
      )
      val deqFire = c.io.deq.valid.peek().litToBoolean && c.io.deq.ready.peek().litToBoolean

      if (enqFires.exists(identity)) {
        val batch = if (pending.nonEmpty) pending.remove(0) else Seq.fill(numInputs)(None)
        for ((v, i) <- batch.zipWithIndex) {
          if (v.isDefined && enqFires(i)) expectedOrder += v.get
        }
      }

      if (deqFire) {
        dequeued += c.io.deq.bits.peek().litValue.toInt
      }

      c.clock.step()
      cycle += 1
    }

    // Drain remaining
    clearEnqs(c.io.enqs, numInputs)
    c.io.deq.ready.poke(true.B)
    while (expectedOrder.length > dequeued.length && cycle < maxCycles) {
      if (c.io.deq.valid.peek().litToBoolean) {
        dequeued += c.io.deq.bits.peek().litValue.toInt
      }
      c.clock.step()
      cycle += 1
    }

    assert(cycle < maxCycles, s"Scenario timed out after $maxCycles cycles")
    dequeued.toSeq
  }

  /** Overload for MultiPortedQueue (wrapper). */
  private def runScenario(
    c: MultiPortedQueue[UInt],
    enqBatches: Seq[Seq[Option[Int]]],
    deqReady: Int => Boolean,
    numInputs: Int,
    maxCycles: Int
  ): Seq[Int] = {
    val pending = ArrayBuffer.from(enqBatches)
    val expectedOrder = ArrayBuffer[Int]()
    val dequeued = ArrayBuffer[Int]()
    var cycle = 0

    while ((pending.nonEmpty || expectedOrder.length > dequeued.length) && cycle < maxCycles) {
      c.io.deq.ready.poke(deqReady(cycle).B)

      if (pending.nonEmpty) pokeEnqs(c.io.enqs, pending.head) else clearEnqs(c.io.enqs, numInputs)

      val enqFires = (0 until numInputs).map(i =>
        c.io.enqs(i).valid.peek().litToBoolean && c.io.enqs(i).ready.peek().litToBoolean
      )
      val deqFire = c.io.deq.valid.peek().litToBoolean && c.io.deq.ready.peek().litToBoolean

      if (enqFires.exists(identity)) {
        val batch = if (pending.nonEmpty) pending.remove(0) else Seq.fill(numInputs)(None)
        for ((v, i) <- batch.zipWithIndex) {
          if (v.isDefined && enqFires(i)) expectedOrder += v.get
        }
      }

      if (deqFire) {
        dequeued += c.io.deq.bits.peek().litValue.toInt
      }

      c.clock.step()
      cycle += 1
    }

    clearEnqs(c.io.enqs, numInputs)
    c.io.deq.ready.poke(true.B)
    while (expectedOrder.length > dequeued.length && cycle < maxCycles) {
      if (c.io.deq.valid.peek().litToBoolean) {
        dequeued += c.io.deq.bits.peek().litValue.toInt
      }
      c.clock.step()
      cycle += 1
    }

    assert(cycle < maxCycles, s"Scenario timed out after $maxCycles cycles")
    dequeued.toSeq
  }

  /** Build batches where all ports are valid each cycle, from a flat list of values. */
  private def fullBatches(values: Seq[Int], numInputs: Int): Seq[Seq[Option[Int]]] = {
    values.grouped(numInputs).map(_.map(Some(_)).padTo(numInputs, None)).toSeq
  }

  /** Build batches with random valid patterns per cycle.
    * Guarantees at least one valid port per batch so the scenario makes progress. */
  private def randomBatches(rng: Random, numValues: Int, numInputs: Int): (Seq[Seq[Option[Int]]], Seq[Int]) = {
    val values = (0 until numValues).map(_ => rng.nextInt(256))
    var idx = 0
    val batches = ArrayBuffer[Seq[Option[Int]]]()
    while (idx < values.length) {
      var batch = (0 until numInputs).map { _ =>
        if (idx < values.length && rng.nextBoolean()) {
          val v = values(idx); idx += 1; Some(v)
        } else None
      }
      // Ensure at least one valid port per batch
      if (batch.forall(_.isEmpty) && idx < values.length) {
        val port = rng.nextInt(numInputs)
        val v = values(idx); idx += 1
        batch = batch.updated(port, Some(v))
      }
      if (batch.exists(_.isDefined)) batches += batch
    }
    (batches.toSeq, values)
  }

  private def resetDUT(clock: Clock, reset: Reset): Unit = {
    reset.poke(true.B)
    clock.step()
    reset.poke(false.B)
  }

  // ===================== MultiPortedRegQueue Tests =====================

  behavior of "MultiPortedRegQueue"

  it should "dequeue in FIFO order with all ports valid" in {
    test(new MultiPortedRegQueue(UInt(8.W), numEntries = 8, numInputs = 2)).withAnnotations(waveformAnnos) { c =>
      resetDUT(c.clock, c.reset)
      val values = (1 to 8).toSeq
      val batches = fullBatches(values, 2)
      val result = runScenario(c, batches, _ => true, numInputs = 2)
      assert(result == values, s"Expected $values, got $result")
    }
  }

  it should "assert backpressure when full and resume after dequeue" in {
    test(new MultiPortedRegQueue(UInt(8.W), numEntries = 4, numInputs = 2)).withAnnotations(waveformAnnos) { c =>
      resetDUT(c.clock, c.reset)

      // Fill the queue (4 entries, 2 per cycle = 2 cycles)
      pokeEnqs(c.io.enqs,Seq(Some(10), Some(11)))
      c.io.deq.ready.poke(false.B)
      c.clock.step()
      pokeEnqs(c.io.enqs,Seq(Some(12), Some(13)))
      c.clock.step()

      // Queue should be full: enqs not ready
      clearEnqs(c.io.enqs,2)
      c.io.enqs(0).ready.expect(false.B)
      c.io.enqs(1).ready.expect(false.B)

      // Dequeue one element
      c.io.deq.ready.poke(true.B)
      c.io.deq.valid.expect(true.B)
      c.io.deq.bits.expect(10.U)
      c.clock.step()

      // Now there should be space
      c.io.deq.bits.expect(11.U)
      c.clock.step()
      c.io.deq.bits.expect(12.U)
      c.clock.step()
      c.io.deq.bits.expect(13.U)
      c.clock.step()

      // Queue empty
      c.io.deq.valid.expect(false.B)
    }
  }

  it should "handle partial valid patterns preserving port ordering" in {
    test(new MultiPortedRegQueue(UInt(8.W), numEntries = 8, numInputs = 3)).withAnnotations(waveformAnnos) { c =>
      resetDUT(c.clock, c.reset)

      // Only port 2 valid
      val b1: Seq[Option[Int]] = Seq(None, None, Some(99))
      // Only ports 0 and 2 valid
      val b2: Seq[Option[Int]] = Seq(Some(10), None, Some(30))
      // Only port 1 valid
      val b3: Seq[Option[Int]] = Seq(None, Some(20), None)
      // All valid
      val b4: Seq[Option[Int]] = Seq(Some(1), Some(2), Some(3))

      val batches = Seq(b1, b2, b3, b4)
      val result = runScenario(c, batches, _ => true, numInputs = 3)
      // Expected order: port ordering within each cycle, then across cycles
      // Cycle 1: port2=99
      // Cycle 2: port0=10, port2=30
      // Cycle 3: port1=20
      // Cycle 4: port0=1, port1=2, port2=3
      assert(result == Seq(99, 10, 30, 20, 1, 2, 3), s"Got $result")
    }
  }

  it should "correctly report count" in {
    test(new MultiPortedRegQueue(UInt(8.W), numEntries = 8, numInputs = 2)).withAnnotations(waveformAnnos) { c =>
      resetDUT(c.clock, c.reset)
      c.io.deq.ready.poke(false.B)

      // Enqueue 2 per cycle, check count
      pokeEnqs(c.io.enqs,Seq(Some(1), Some(2)))
      c.io.count.expect(0.U) // count updates on next cycle
      c.clock.step()
      c.io.count.expect(2.U)
      pokeEnqs(c.io.enqs,Seq(Some(3), Some(4)))
      c.clock.step()
      c.io.count.expect(4.U)
      clearEnqs(c.io.enqs,2)
      c.clock.step()
      c.io.count.expect(4.U)

      // Dequeue and check count decreases
      c.io.deq.ready.poke(true.B)
      c.clock.step()
      c.io.count.expect(3.U)
      c.clock.step()
      c.io.count.expect(2.U)
    }
  }

  it should "handle simultaneous enqueue and dequeue" in {
    test(new MultiPortedRegQueue(UInt(8.W), numEntries = 8, numInputs = 2)).withAnnotations(waveformAnnos) { c =>
      resetDUT(c.clock, c.reset)

      // Prime the queue with 2 entries
      pokeEnqs(c.io.enqs, Seq(Some(100), Some(101)))
      c.io.deq.ready.poke(false.B)
      c.clock.step()

      // Simultaneously enqueue and dequeue for several cycles
      c.io.deq.ready.poke(true.B)
      pokeEnqs(c.io.enqs, Seq(Some(102), Some(103)))
      c.io.deq.valid.expect(true.B)
      c.io.deq.bits.expect(100.U)
      c.clock.step()

      pokeEnqs(c.io.enqs, Seq(Some(104), Some(105)))
      c.io.deq.bits.expect(101.U)
      c.clock.step()

      pokeEnqs(c.io.enqs, Seq(Some(106), Some(107)))
      c.io.deq.bits.expect(102.U)
      c.clock.step()

      // Stop enqueuing, drain the rest
      clearEnqs(c.io.enqs, 2)
      c.io.deq.bits.expect(103.U)
      c.clock.step()
      c.io.deq.bits.expect(104.U)
      c.clock.step()
      c.io.deq.bits.expect(105.U)
      c.clock.step()
      c.io.deq.bits.expect(106.U)
      c.clock.step()
      c.io.deq.bits.expect(107.U)
      c.clock.step()
      c.io.deq.valid.expect(false.B)
    }
  }

  it should "survive fill-drain cycles back to back" in {
    test(new MultiPortedRegQueue(UInt(8.W), numEntries = 4, numInputs = 2)).withAnnotations(waveformAnnos) { c =>
      resetDUT(c.clock, c.reset)

      for (round <- 0 until 5) {
        val base = round * 4
        val values = (base until base + 4).toSeq
        val batches = fullBatches(values, 2)
        val result = runScenario(c, batches, _ => true, numInputs = 2)
        assert(result == values, s"Round $round: expected $values, got $result")
      }
    }
  }

  it should "handle dequeue backpressure with periodic ready" in {
    test(new MultiPortedRegQueue(UInt(8.W), numEntries = 8, numInputs = 2)).withAnnotations(waveformAnnos) { c =>
      resetDUT(c.clock, c.reset)
      val values = (0 until 8).toSeq
      val batches = fullBatches(values, 2)
      // ready only every other cycle
      val result = runScenario(c, batches, cyc => cyc % 2 == 0, numInputs = 2)
      assert(result == values, s"Expected $values, got $result")
    }
  }

  it should "pass randomized stress test" in {
    test(new MultiPortedRegQueue(UInt(8.W), numEntries = 16, numInputs = 2)).withAnnotations(waveformAnnos) { c =>
      resetDUT(c.clock, c.reset)
      val batchRng = new Random(42)
      val deqRng = new Random(43)
      val (batches, expectedValues) = randomBatches(batchRng, numValues = 100, numInputs = 2)
      val result = runScenario(c, batches, _ => deqRng.nextInt(3) != 0, numInputs = 2, maxCycles = 4000)
      assert(result == expectedValues, s"FIFO ordering violated in stress test")
    }
  }

  it should "pass randomized stress test with 3 input ports" in {
    test(new MultiPortedRegQueue(UInt(8.W), numEntries = 16, numInputs = 3)).withAnnotations(waveformAnnos) { c =>
      resetDUT(c.clock, c.reset)
      val batchRng = new Random(123)
      val deqRng = new Random(124)
      val (batches, expectedValues) = randomBatches(batchRng, numValues = 80, numInputs = 3)
      val result = runScenario(c, batches, _ => deqRng.nextInt(4) != 0, numInputs = 3, maxCycles = 4000)
      assert(result == expectedValues, s"FIFO ordering violated in 3-port stress test")
    }
  }

  it should "report correct count at full capacity and drain correctly" in {
    test(new MultiPortedRegQueue(UInt(8.W), numEntries = 4, numInputs = 2)).withAnnotations(waveformAnnos) { c =>
      resetDUT(c.clock, c.reset)
      c.io.deq.ready.poke(false.B)

      // Fill completely: 2 ports × 2 cycles = 4 entries
      pokeEnqs(c.io.enqs, Seq(Some(10), Some(11)))
      c.clock.step()
      pokeEnqs(c.io.enqs, Seq(Some(12), Some(13)))
      c.clock.step()
      clearEnqs(c.io.enqs, 2)

      // Queue is full: count must be 4, not 0
      c.io.count.expect(4.U)
      c.io.enqs(0).ready.expect(false.B)
      c.io.enqs(1).ready.expect(false.B)

      // Attempt to enqueue more — should be rejected
      pokeEnqs(c.io.enqs, Seq(Some(99), Some(100)))
      c.io.enqs(0).ready.expect(false.B)
      c.io.enqs(1).ready.expect(false.B)
      c.clock.step()
      clearEnqs(c.io.enqs, 2)
      c.io.count.expect(4.U) // still full, nothing was accepted

      // Drain one by one, verifying count and FIFO order
      c.io.deq.ready.poke(true.B)
      c.io.deq.valid.expect(true.B)
      c.io.deq.bits.expect(10.U)
      c.clock.step()
      c.io.count.expect(3.U)
      c.io.deq.bits.expect(11.U)
      c.clock.step()
      c.io.count.expect(2.U)
      c.io.deq.bits.expect(12.U)
      c.clock.step()
      c.io.count.expect(1.U)
      c.io.deq.bits.expect(13.U)
      c.clock.step()
      c.io.count.expect(0.U)
      c.io.deq.valid.expect(false.B)
    }
  }

  it should "report empty when nothing enqueued" in {
    test(new MultiPortedRegQueue(UInt(8.W), numEntries = 4, numInputs = 2)).withAnnotations(waveformAnnos) { c =>
      resetDUT(c.clock, c.reset)
      clearEnqs(c.io.enqs,2)
      c.io.deq.ready.poke(true.B)
      c.io.deq.valid.expect(false.B)
      c.io.count.expect(0.U)
      c.clock.step()
      c.io.deq.valid.expect(false.B)
      c.io.count.expect(0.U)
    }
  }

  // ===================== MultiPortedQueue (SRAM) Tests =====================

  behavior of "MultiPortedQueue (SRAM-backed)"

  it should "dequeue in FIFO order with all ports valid" in {
    test(new MultiPortedQueue(UInt(8.W), numEntries = 8, numInputs = 2, impl = MPQueueImpl.SRAM)).withAnnotations(waveformAnnos) { c =>
      resetDUT(c.clock, c.reset)
      val values = (1 to 8).toSeq
      val batches = fullBatches(values, 2)
      val result = runScenario(c, batches, _ => true, numInputs = 2, maxCycles = 2000)
      assert(result == values, s"Expected $values, got $result")
    }
  }

  it should "assert backpressure when full and resume after dequeue" in {
    test(new MultiPortedQueue(UInt(8.W), numEntries = 4, numInputs = 2, impl = MPQueueImpl.SRAM)).withAnnotations(waveformAnnos) { c =>
      resetDUT(c.clock, c.reset)

      // Fill the queue
      pokeEnqs(c.io.enqs,Seq(Some(10), Some(11)))
      c.io.deq.ready.poke(false.B)
      c.clock.step()
      pokeEnqs(c.io.enqs,Seq(Some(12), Some(13)))
      c.clock.step()

      // Should be full
      clearEnqs(c.io.enqs,2)
      c.io.enqs(0).ready.expect(false.B)
      c.io.enqs(1).ready.expect(false.B)
      c.io.stall_enq.expect(true.B)
      c.clock.step()

      // Drain and verify order
      c.io.deq.ready.poke(true.B)
      val dequeued = ArrayBuffer[Int]()
      for (_ <- 0 until 20) {
        if (c.io.deq.valid.peek().litToBoolean) {
          dequeued += c.io.deq.bits.peek().litValue.toInt
        }
        c.clock.step()
        if (dequeued.length == 4) {}
      }
      assert(dequeued.toSeq == Seq(10, 11, 12, 13), s"Got $dequeued")
    }
  }

  it should "handle partial valid patterns preserving port ordering" in {
    test(new MultiPortedQueue(UInt(8.W), numEntries = 8, numInputs = 2, impl = MPQueueImpl.SRAM)).withAnnotations(waveformAnnos) { c =>
      resetDUT(c.clock, c.reset)

      val b1: Seq[Option[Int]] = Seq(Some(10), None)
      val b2: Seq[Option[Int]] = Seq(None, Some(20))
      val b3: Seq[Option[Int]] = Seq(Some(30), Some(40))
      val b4: Seq[Option[Int]] = Seq(Some(50), None)

      val batches = Seq(b1, b2, b3, b4)
      val result = runScenario(c, batches, _ => true, numInputs = 2, maxCycles = 2000)
      assert(result == Seq(10, 20, 30, 40, 50), s"Got $result")
    }
  }

  it should "handle simultaneous enqueue and dequeue" in {
    test(new MultiPortedQueue(UInt(8.W), numEntries = 4, numInputs = 2, impl = MPQueueImpl.SRAM)).withAnnotations(waveformAnnos) { c =>
      resetDUT(c.clock, c.reset)

      // Prime the queue
      pokeEnqs(c.io.enqs,Seq(Some(100), Some(101)))
      c.io.deq.ready.poke(false.B)
      c.clock.step()
      clearEnqs(c.io.enqs,2)

      // Allow SRAM read latency
      c.clock.step()
      c.clock.step()

      // Now enqueue while dequeuing
      c.io.deq.ready.poke(true.B)
      pokeEnqs(c.io.enqs,Seq(Some(102), Some(103)))

      val dequeued = ArrayBuffer[Int]()
      for (_ <- 0 until 20) {
        if (c.io.deq.valid.peek().litToBoolean && c.io.deq.ready.peek().litToBoolean) {
          dequeued += c.io.deq.bits.peek().litValue.toInt
        }
        c.clock.step()
        // Stop enqueuing after first cycle
        clearEnqs(c.io.enqs,2)
      }
      assert(dequeued.toSeq == Seq(100, 101, 102, 103), s"Got $dequeued")
    }
  }

  it should "survive fill-drain cycles back to back" in {
    test(new MultiPortedQueue(UInt(8.W), numEntries = 4, numInputs = 2, impl = MPQueueImpl.SRAM)).withAnnotations(waveformAnnos) { c =>
      resetDUT(c.clock, c.reset)

      for (round <- 0 until 5) {
        val base = round * 4
        val values = (base until base + 4).map(_ % 256).toSeq
        val batches = fullBatches(values, 2)
        val result = runScenario(c, batches, _ => true, numInputs = 2, maxCycles = 2000)
        assert(result == values, s"Round $round: expected $values, got $result")
      }
    }
  }

  it should "handle dequeue backpressure with periodic ready" in {
    test(new MultiPortedQueue(UInt(8.W), numEntries = 8, numInputs = 2, impl = MPQueueImpl.SRAM)).withAnnotations(waveformAnnos) { c =>
      resetDUT(c.clock, c.reset)
      val values = (0 until 8).toSeq
      val batches = fullBatches(values, 2)
      val result = runScenario(c, batches, cyc => cyc % 2 == 0, numInputs = 2, maxCycles = 2000)
      assert(result == values, s"Expected $values, got $result")
    }
  }

  it should "pass randomized stress test" in {
    test(new MultiPortedQueue(UInt(8.W), numEntries = 16, numInputs = 2, impl = MPQueueImpl.SRAM)).withAnnotations(waveformAnnos) { c =>
      resetDUT(c.clock, c.reset)
      val batchRng = new Random(42)
      val deqRng = new Random(43)
      val (batches, expectedValues) = randomBatches(batchRng, numValues = 100, numInputs = 2)
      val result = runScenario(c, batches, _ => deqRng.nextInt(3) != 0, numInputs = 2, maxCycles = 4000)
      assert(result == expectedValues, s"FIFO ordering violated in SRAM stress test")
    }
  }

  it should "pass randomized stress test with 4 input ports" in {
    test(new MultiPortedQueue(UInt(8.W), numEntries = 16, numInputs = 4, impl = MPQueueImpl.SRAM)).withAnnotations(waveformAnnos) { c =>
      resetDUT(c.clock, c.reset)
      val batchRng = new Random(999)
      val deqRng = new Random(1000)
      val (batches, expectedValues) = randomBatches(batchRng, numValues = 80, numInputs = 4)
      val result = runScenario(c, batches, _ => deqRng.nextInt(4) != 0, numInputs = 4, maxCycles = 4000)
      assert(result == expectedValues, s"FIFO ordering violated in 4-port SRAM stress test")
    }
  }

  it should "report correct count at full capacity and drain correctly" in {
    test(new MultiPortedQueue(UInt(8.W), numEntries = 4, numInputs = 2, impl = MPQueueImpl.SRAM)).withAnnotations(waveformAnnos) { c =>
      resetDUT(c.clock, c.reset)
      c.io.deq.ready.poke(false.B)

      // Fill completely: 2 ports × 2 cycles = 4 entries
      pokeEnqs(c.io.enqs, Seq(Some(10), Some(11)))
      c.clock.step()
      pokeEnqs(c.io.enqs, Seq(Some(12), Some(13)))
      c.clock.step()
      clearEnqs(c.io.enqs, 2)

      // Queue is full: count must be 4, not 0
      c.io.count.expect(4.U)
      c.io.enqs(0).ready.expect(false.B)
      c.io.enqs(1).ready.expect(false.B)
      c.io.stall_enq.expect(true.B)

      // Attempt to enqueue more — should be rejected
      pokeEnqs(c.io.enqs, Seq(Some(99), Some(100)))
      c.io.enqs(0).ready.expect(false.B)
      c.clock.step()
      clearEnqs(c.io.enqs, 2)
      c.io.count.expect(4.U) // still full

      // Drain all entries, collecting values and checking count
      c.io.deq.ready.poke(true.B)
      val dequeued = ArrayBuffer[Int]()
      val expectedCounts = ArrayBuffer[Int]()
      for (_ <- 0 until 20) {
        if (c.io.deq.valid.peek().litToBoolean) {
          dequeued += c.io.deq.bits.peek().litValue.toInt
        }
        c.clock.step()
        expectedCounts += c.io.count.peek().litValue.toInt
        if (dequeued.length >= 4) {
          // Verify final count is 0
          c.io.count.expect(0.U)
          c.io.deq.valid.expect(false.B)
        }
      }
      assert(dequeued.toSeq == Seq(10, 11, 12, 13), s"Got $dequeued")
    }
  }

  it should "report empty when nothing enqueued" in {
    test(new MultiPortedQueue(UInt(8.W), numEntries = 4, numInputs = 2, impl = MPQueueImpl.SRAM)).withAnnotations(waveformAnnos) { c =>
      resetDUT(c.clock, c.reset)
      clearEnqs(c.io.enqs,2)
      c.io.deq.ready.poke(true.B)
      c.io.deq.valid.expect(false.B)
      c.clock.step()
      c.io.deq.valid.expect(false.B)
    }
  }

  // ===================== MultiPortedQueue (Reg-backed via wrapper) Tests =====================

  behavior of "MultiPortedQueue (Reg-backed via wrapper)"

  it should "match reg queue behavior through the wrapper" in {
    test(new MultiPortedQueue(UInt(8.W), numEntries = 8, numInputs = 2, impl = MPQueueImpl.Reg)).withAnnotations(waveformAnnos) { c =>
      resetDUT(c.clock, c.reset)
      val values = (10 to 17).toSeq
      val batches = fullBatches(values, 2)
      val result = runScenario(c, batches, _ => true, numInputs = 2, maxCycles = 2000)
      assert(result == values, s"Expected $values, got $result")
    }
  }

  it should "pass randomized stress test through the wrapper" in {
    test(new MultiPortedQueue(UInt(8.W), numEntries = 16, numInputs = 2, impl = MPQueueImpl.Reg)).withAnnotations(waveformAnnos) { c =>
      resetDUT(c.clock, c.reset)
      val batchRng = new Random(77)
      val deqRng = new Random(78)
      val (batches, expectedValues) = randomBatches(batchRng, numValues = 100, numInputs = 2)
      val result = runScenario(c, batches, _ => deqRng.nextInt(3) != 0, numInputs = 2, maxCycles = 4000)
      assert(result == expectedValues, s"FIFO ordering violated in reg wrapper stress test")
    }
  }

  it should "preserve FIFO order in the SRAM queue when driven to full occupancy" in {
    // Random enqueue at full width whenever the queue accepts, slow random drain, so
    // occupancy sits at numEntries most of the time. Every dequeued value must match
    // the enqueue sequence exactly (no loss, no duplication, no reordering).
    test(new MultiPortedSRAMQueue(UInt(16.W), numEntries = 16, numInputs = 2, reserveCycles = 0))
      .withAnnotations(Seq(VerilatorBackendAnnotation)) { c =>
      c.clock.setTimeout(0)
      val rng = new scala.util.Random(17)
      val sent = ArrayBuffer[Int]()
      val got = ArrayBuffer[Int]()
      var next = 1
      var maxCount = 0
      for (i <- 0 until 2) { c.io.enqs(i).valid.poke(false.B) }
      c.io.deq.ready.poke(false.B)
      for (cyc <- 0 until 6000) {
        val deqReady = rng.nextInt(5) == 0
        c.io.deq.ready.poke(deqReady.B)
        val n = if (cyc < 5000) rng.nextInt(3) else 0   // 0..2 lanes, then drain
        for (i <- 0 until 2) {
          c.io.enqs(i).valid.poke((i < n).B)
          c.io.enqs(i).bits.poke((next + i).U)
        }
        // sample fires before the edge
        val ready = c.io.enqs(0).ready.peek().litToBoolean
        if (ready) { for (i <- 0 until n) sent += next + i; next += n }
        if (deqReady && c.io.deq.valid.peek().litToBoolean) got += c.io.deq.bits.peek().litValue.toInt
        maxCount = math.max(maxCount, c.io.count.peek().litValue.toInt)
        c.clock.step()
      }
      println(s"SRAM queue full-occupancy test: sent ${sent.length}, got ${got.length}, max count $maxCount")
      assert(maxCount == 16, s"test did not reach full occupancy (max $maxCount)")
      assert(got.toSeq == sent.take(got.length).toSeq,
        s"FIFO order violated at index ${got.zip(sent).indexWhere { case (a, b) => a != b }}")
    }
  }
}
