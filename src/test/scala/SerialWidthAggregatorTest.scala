package tacit

import chisel3._
import chiseltest._
import freechips.rocketchip.trace.TraceEgressConstants
import org.scalatest.flatspec.AnyFlatSpec
import scala.collection.mutable.ArrayBuffer
import scala.util.Random

class SerialWidthAggregatorTest extends AnyFlatSpec with ChiselScalatestTester {
  private val waveformAnnos = Seq(WriteVcdAnnotation)
  private val numLanes = TraceEgressConstants.numLanes
  private val bytesPerWide = 8

  private def pokeNarrow(c: SerialWidthAggregator, bytes: Seq[Int]): Unit = {
    require(bytes.length <= numLanes)
    c.io.narrow.valid.poke(true.B)
    for (i <- 0 until numLanes) {
      if (i < bytes.length) {
        c.io.narrow.mask(i).poke(true.B)
        c.io.narrow.bits(i).poke(bytes(i).U)
      } else {
        c.io.narrow.mask(i).poke(false.B)
        c.io.narrow.bits(i).poke(0.U)
      }
    }
  }

  private def clearNarrow(c: SerialWidthAggregator): Unit = {
    c.io.narrow.valid.poke(false.B)
    for (i <- 0 until numLanes) {
      c.io.narrow.mask(i).poke(false.B)
      c.io.narrow.bits(i).poke(0.U)
    }
  }

  private def wordFromBytesLE(bytes: Seq[Int]): BigInt = {
    require(bytes.length == bytesPerWide)
    bytes.zipWithIndex.foldLeft(BigInt(0)) { case (acc, (b, i)) =>
      acc | (BigInt(b & 0xff) << (8 * i))
    }
  }

  private def runScenario(
    c: SerialWidthAggregator,
    chunksIn: Seq[Seq[Int]],
    readyAt: Int => Boolean,
    maxCycles: Int = 2000
  ): Unit = {
    val pendingChunks = ArrayBuffer.from(chunksIn)
    val acceptedBytes = ArrayBuffer[Int]()
    var cycle = 0

    while ((pendingChunks.nonEmpty || acceptedBytes.length >= bytesPerWide) && cycle < maxCycles) {
      c.io.wide.ready.poke(readyAt(cycle).B)

      if (pendingChunks.nonEmpty) pokeNarrow(c, pendingChunks.head) else clearNarrow(c)

      // Check presented wide word (when any full word is buffered).
      if (acceptedBytes.length >= bytesPerWide) {
        c.io.wide.valid.expect(true.B)
        val expected = wordFromBytesLE(acceptedBytes.take(bytesPerWide).toSeq)
        c.io.wide.bits.expect(expected.U(64.W))
      }

      val narrowFire = c.io.narrow.valid.peek().litToBoolean && c.io.narrow.ready.peek().litToBoolean
      val wideFire = c.io.wide.valid.peek().litToBoolean && c.io.wide.ready.peek().litToBoolean

      if (narrowFire) {
        acceptedBytes ++= pendingChunks.remove(0)
      }
      if (wideFire) {
        acceptedBytes.remove(0, bytesPerWide)
      }

      c.clock.step()
      cycle += 1
    }

    // Drain remaining full words.
    c.io.wide.ready.poke(true.B)
    clearNarrow(c)
    while (acceptedBytes.length >= bytesPerWide && cycle < maxCycles) {
      c.io.wide.valid.expect(true.B)
      val expected = wordFromBytesLE(acceptedBytes.take(bytesPerWide).toSeq)
      c.io.wide.bits.expect(expected.U(64.W))
      c.clock.step()
      acceptedBytes.remove(0, bytesPerWide)
      cycle += 1
    }

    c.io.wide.valid.expect(false.B)
    assert(cycle < maxCycles, s"Scenario timed out after $maxCycles cycles")
  }

  behavior of "SerialWidthAggregator"

  it should "pack bytes with first accepted byte in the least-significant byte of wide output" in {
    test(new SerialWidthAggregator(64)).withAnnotations(waveformAnnos) { c =>
      c.reset.poke(true.B)
      c.clock.step()
      c.reset.poke(false.B)

      c.io.wide.ready.poke(true.B)
      clearNarrow(c)

      // 8 bytes in-order: 11 22 33 44 55 66 77 88
      pokeNarrow(c, Seq(0x11, 0x22, 0x33, 0x44))
      c.clock.step()
      pokeNarrow(c, Seq(0x55, 0x66, 0x77, 0x88))
      c.clock.step()
      clearNarrow(c)

      // If the byte order is correct for little-endian packing, wide should be:
      // 0x8877665544332211
      c.io.wide.valid.expect(true.B)
      c.io.wide.bits.expect("h8877665544332211".U)
    }
  }

  it should "handle repeated 3+3+3+... chunks across wide-word boundaries" in {
    test(new SerialWidthAggregator(64)).withAnnotations(waveformAnnos) { c =>
      c.reset.poke(true.B)
      c.clock.step()
      c.reset.poke(false.B)
      clearNarrow(c)
      c.io.wide.ready.poke(false.B)

      val bytes = (0 until 48).map(i => (0x40 + i) & 0xff)
      val chunks = bytes.grouped(3).map(_.toSeq).toSeq
      runScenario(c, chunks, _ => true)
    }
  }

  it should "preserve stream ordering with mixed chunk sizes and periodic backpressure" in {
    test(new SerialWidthAggregator(64)).withAnnotations(waveformAnnos) { c =>
      c.reset.poke(true.B)
      c.clock.step()
      c.reset.poke(false.B)
      clearNarrow(c)
      c.io.wide.ready.poke(false.B)

      val sizes = Seq(1, 4, 2, 3, 1, 4, 4, 2, 3, 1, 4, 2, 3, 3, 1, 4)
      val bytes = (0 until sizes.sum).map(i => (0x80 + i) & 0xff)
      var idx = 0
      val chunks = sizes.map { s =>
        val out = bytes.slice(idx, idx + s)
        idx += s
        out
      }
      runScenario(c, chunks, cyc => (cyc % 5) != 2)
    }
  }

  it should "pass randomized stress with valid/ready stalls and varying chunk sizes" in {
    test(new SerialWidthAggregator(64)).withAnnotations(waveformAnnos) { c =>
      c.reset.poke(true.B)
      c.clock.step()
      c.reset.poke(false.B)
      clearNarrow(c)
      c.io.wide.ready.poke(false.B)

      val rng = new Random(7)
      val chunks = (0 until 120).map { _ =>
        val n = rng.nextInt(numLanes) + 1
        Seq.fill(n)(rng.nextInt(256))
      }
      runScenario(c, chunks, _ => rng.nextBoolean(), maxCycles = 4000)
    }
  }
}
