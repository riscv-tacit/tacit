package tacit

import chisel3._
import chisel3.util._
import freechips.rocketchip.tilelink._

/** Copied and adapted from Randiance SourceGenerator.scala **/ 
class SourceGenerator[T <: Data](
    sourceWidth: Int,
    metadata: Option[T] = None,
    ignoreInUse: Boolean = false
) extends Module {
  val io = IO(new Bundle {
    val gen = Input(Bool())
    val reclaim = Input(Valid(UInt(sourceWidth.W)))
    val id = Output(Valid(UInt(sourceWidth.W)))
    val meta = metadata.map(Input(_))
    val peek = metadata.map(Output(_))
    val inflight = Output(Bool())
  })
  val head = RegInit(UInt(sourceWidth.W), 0.U)
  head := Mux(io.gen, head + 1.U, head)

  val outstanding = RegInit(UInt((sourceWidth + 1).W), 0.U)
  io.inflight := (outstanding > 0.U) || io.gen

  val numSourceId = 1 << sourceWidth
  val occupancyTable = Mem(numSourceId, Bool())
  val metadataTable = metadata.map(Mem(numSourceId, _))
  
  when(reset.asBool) {
    (0 until numSourceId).foreach { occupancyTable(_) := false.B }
  }
  
  val frees = (0 until numSourceId).map(!occupancyTable(_))
  val lowestFree = PriorityEncoder(frees)
  val lowestFreeValid = occupancyTable(lowestFree)

  io.id.valid := (if (ignoreInUse) true.B else !lowestFreeValid)
  io.id.bits := lowestFree
  
  when(io.gen && io.id.valid) {
    when (!io.reclaim.valid || io.reclaim.bits =/= io.id.bits) {
      occupancyTable(io.id.bits) := true.B
      metadataTable.foreach { table =>
        table(io.id.bits) := io.meta.get
      }
    }
  }
  
  when(io.reclaim.valid) {
    occupancyTable(io.reclaim.bits) := false.B
  }

  metadataTable.foreach { table =>
    io.peek.get := table(io.reclaim.bits)
  }

  when(io.gen && io.id.valid) {
    when(!io.reclaim.valid) {
      assert(outstanding < (1 << sourceWidth).U)
      outstanding := outstanding + 1.U
    }
  }.elsewhen(io.reclaim.valid) {
    assert(outstanding > 0.U,
           "Over-reclaim. Did some responses get dropped?")
    outstanding := outstanding - 1.U
  }
  dontTouch(outstanding)
}

object SourceGenerator {
  def apply(node: TLBundle) = {
    val sourceGen = Module(new SourceGenerator(node.params.sourceBits))
    sourceGen.io.gen := node.a.fire
    node.a.bits.source := sourceGen.io.id.bits
    sourceGen.io.reclaim.valid := node.d.fire
    sourceGen.io.reclaim.bits := node.d.bits.source

    (sourceGen.io.id.valid, sourceGen.io.inflight, sourceGen)
  }
}