package tacit

import chisel3._
import chisel3.util._
import scala.math.min

// Variable-length encoding helper module
class VarLenEncoder(val maxWidth: Int) extends Module {
  val maxNumBytes = maxWidth/(8-1) + 1
  
  val io = IO(new Bundle {
    val input_value = Input(UInt(maxWidth.W))
    val input_valid = Input(Bool())
    val output_num_bytes = Output(UInt(log2Ceil(maxNumBytes).W))
    val output_bytes = Output(Vec(maxNumBytes, UInt(8.W)))
  })

  // 0-indexed MSB index 
  val msb_index = (maxWidth - 1).U - PriorityEncoder(Reverse(io.input_value))
  io.output_num_bytes := Mux(io.input_valid, (msb_index / 7.U) + 1.U, 0.U)

  for (i <- 0 until maxNumBytes) {
    val is_last_byte = (i.U === (io.output_num_bytes - 1.U))
    io.output_bytes(i) := Mux(i.U < io.output_num_bytes,
      io.input_value(min(i*7+6, maxWidth-1), i*7) | Mux(is_last_byte, 0x80.U, 0.U),
      0.U
    )
  }
}

// Processor privilege level encoder
class PrvEncoder extends Module {
  val io = IO(new Bundle {
    val from_priv = Input(UInt(3.W))
    val to_priv = Input(UInt(3.W))
    val input_valid = Input(Bool())
    val output_byte = Output(UInt(8.W))
    val output_valid = Output(Bool())
  })
  io.output_byte := Cat(0b10.U, io.to_priv, io.from_priv)
  io.output_valid := io.input_valid
}
