package tacit

import chisel3._
import chisel3.util._
import chisel3.experimental.requireIsChiselType

/**
 Given Input mask and data, output the compacted data and new mask.
 e.g. input mask: 0b10101010, data: [1,2,3,4,5,6,7,8]
      output mask: 0b11110000, data: [1,3,5,7,DC,DC,DC,DC]
 */
class LaneCompactor[T <: Data](
  val gen: T,
  val numLanes: Int,
) extends Module {
  requireIsChiselType(gen)
  val io = IO(new Bundle {
    val input_mask = Input(Vec(numLanes, Bool()))
    val input_data = Input(Vec(numLanes, gen))
    val output_mask = Output(Vec(numLanes, Bool()))
    val output_data = Output(Vec(numLanes, gen))
  })

  val num_valid_lanes = io.input_mask.map(_.asUInt).reduce(_ +& _)
  val out = Wire(Vec(numLanes, gen))
  out := VecInit(Seq.fill(numLanes)(0.U.asTypeOf(gen)))

  for (i <- 0 until numLanes) {
    // how many valid lanes are there before me?
    val dst = PopCount(io.input_mask.take(i))
    when(io.input_mask(i)) { out(dst) := io.input_data(i) }
  }
  io.output_mask := DontCare
  io.output_data := out
}