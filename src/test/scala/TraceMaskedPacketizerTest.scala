package tacit

import chisel3._
import chiseltest._
import freechips.rocketchip.trace.TraceCoreParams
import org.scalatest.flatspec.AnyFlatSpec

class TraceMaskedPacketizerTest extends AnyFlatSpec with ChiselScalatestTester {
  private val waveformAnnos = Seq(WriteVcdAnnotation)
  private val params = TraceCoreParams(xlen = 32, iaddrWidth = 32)

  private def init(c: TraceMaskedPacketizer): Unit = {
    c.reset.poke(true.B)
    c.clock.step()
    c.reset.poke(false.B)

    c.io.metadata.valid.poke(false.B)
    c.io.byte.valid.poke(false.B)
    c.io.message.valid.poke(false.B)
    c.io.out.ready.poke(false.B)
  }

  private def pokeMessage(c: TraceMaskedPacketizer): Unit = {
    c.io.message.bits.prv.poke("hB1".U)
    c.io.message.bits.ctx(0).poke("hC0".U)
    c.io.message.bits.ctx(1).poke("hC1".U)

    for (i <- 0 until 5) {
      c.io.message.bits.trap_addr(i).poke((0xD0 + i).U)
      c.io.message.bits.target_addr(i).poke((0xE0 + i).U)
      c.io.message.bits.time(i).poke((0xF0 + i).U)
    }
  }

  behavior of "TraceMaskedPacketizer"

  it should "emit compressed packets on lane 0 only" in {
    test(new TraceMaskedPacketizer(params)).withAnnotations(waveformAnnos) { c =>
      init(c)

      c.io.out.ready.poke(true.B)
      c.io.byte.valid.poke(true.B)
      c.io.byte.bits.poke("h5A".U)
      c.io.message.valid.poke(true.B)
      pokeMessage(c)

      c.io.metadata.valid.poke(true.B)
      c.io.metadata.bits.is_full.poke(0.U)
      c.io.metadata.bits.prv.poke(0.U)
      c.io.metadata.bits.ctx.poke(0.U)
      c.io.metadata.bits.trap_addr.poke(0.U)
      c.io.metadata.bits.target_addr.poke(0.U)
      c.io.metadata.bits.time.poke(0.U)

      c.clock.step() // pIdle -> pComp

      c.io.out.valid.expect(true.B)
      c.io.out.mask(0).expect(true.B)
      c.io.out.mask(1).expect(false.B)
      c.io.out.mask(2).expect(false.B)
      c.io.out.mask(3).expect(false.B)
      c.io.out.bits(0).expect("h5A".U)

      c.io.metadata.valid.poke(false.B)
      c.clock.step()
    }
  }

  it should "map sparse full-packet metadata bits to payload bytes in priority order" in {
    test(new TraceMaskedPacketizer(params)).withAnnotations(waveformAnnos) { c =>
      init(c)
      c.io.out.ready.poke(true.B)
      c.io.byte.valid.poke(true.B)
      c.io.byte.bits.poke("hAA".U)
      c.io.message.valid.poke(true.B)
      pokeMessage(c)

      // Selected metadata bit indices: 0, 3, 5, 9, 14, 18
      // Expected bytes in order:
      // cycle 1 -> header, ctx(1), trap_addr(1), target_addr(0)
      // cycle 2 -> time(0), time(4)
      c.io.metadata.valid.poke(true.B)
      c.io.metadata.bits.is_full.poke(1.U)
      c.io.metadata.bits.prv.poke(0.U)
      c.io.metadata.bits.ctx.poke("b10".U)         // idx 3
      c.io.metadata.bits.trap_addr.poke("b00010".U)   // idx 5
      c.io.metadata.bits.target_addr.poke("b00001".U) // idx 9
      c.io.metadata.bits.time.poke("b10001".U)        // idx 14 and 18

      c.clock.step() // pIdle -> pFull

      c.io.out.valid.expect(true.B)
      c.io.out.mask(0).expect(true.B)
      c.io.out.mask(1).expect(true.B)
      c.io.out.mask(2).expect(true.B)
      c.io.out.mask(3).expect(true.B)
      c.io.out.bits(0).expect("hAA".U)
      c.io.out.bits(1).expect("hC1".U)
      c.io.out.bits(2).expect("hD1".U)
      c.io.out.bits(3).expect("hE0".U)

      c.clock.step()

      c.io.out.valid.expect(true.B)
      c.io.out.mask(0).expect(true.B)
      c.io.out.mask(1).expect(true.B)
      c.io.out.mask(2).expect(false.B)
      c.io.out.mask(3).expect(false.B)
      c.io.out.bits(0).expect("hF0".U)
      c.io.out.bits(1).expect("hF4".U)

      c.clock.step()
      c.io.out.valid.expect(false.B)
      c.io.byte.ready.expect(true.B)
      c.io.message.ready.expect(true.B)
    }
  }

  it should "hold lane outputs stable when backpressured" in {
    test(new TraceMaskedPacketizer(params)).withAnnotations(waveformAnnos) { c =>
      init(c)
      c.io.out.ready.poke(false.B)
      c.io.byte.valid.poke(true.B)
      c.io.byte.bits.poke("hAB".U)
      c.io.message.valid.poke(true.B)
      pokeMessage(c)

      // Five selected bytes: idx 0,1,2,3,4
      c.io.metadata.valid.poke(true.B)
      c.io.metadata.bits.is_full.poke(1.U)
      c.io.metadata.bits.prv.poke(1.U)
      c.io.metadata.bits.ctx.poke("b11".U)
      c.io.metadata.bits.trap_addr.poke("b00001".U)
      c.io.metadata.bits.target_addr.poke(0.U)
      c.io.metadata.bits.time.poke(0.U)

      c.clock.step() // pIdle -> pFull

      // First beat should stay constant while ready is low.
      c.io.out.valid.expect(true.B)
      c.io.out.mask(0).expect(true.B)
      c.io.out.mask(1).expect(true.B)
      c.io.out.mask(2).expect(true.B)
      c.io.out.mask(3).expect(true.B)
      c.io.out.bits(0).expect("hAB".U)
      c.io.out.bits(1).expect("hB1".U)
      c.io.out.bits(2).expect("hC0".U)
      c.io.out.bits(3).expect("hC1".U)

      c.clock.step()
      c.io.out.valid.expect(true.B)
      c.io.out.bits(0).expect("hAB".U)
      c.io.out.bits(1).expect("hB1".U)
      c.io.out.bits(2).expect("hC0".U)
      c.io.out.bits(3).expect("hC1".U)

      // Release backpressure and verify the next beat advances.
      c.io.out.ready.poke(true.B)
      c.clock.step()
      c.io.out.valid.expect(true.B)
      c.io.out.mask(0).expect(true.B)
      c.io.out.mask(1).expect(false.B)
      c.io.out.mask(2).expect(false.B)
      c.io.out.mask(3).expect(false.B)
      c.io.out.bits(0).expect("hD0".U)
    }
  }
}
