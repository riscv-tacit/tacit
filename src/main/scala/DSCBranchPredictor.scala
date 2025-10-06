// See LICENSE.Berkeley for license details.
// See LICENSE.SiFive for license details.

package tacit

import chisel3._
import chisel3.util._

object CounterState {
  val STRONG_NOT_TAKEN = 0.U(2.W)
  val WEAK_NOT_TAKEN = 1.U(2.W)
  val WEAK_TAKEN = 2.U(2.W)
  val STRONG_TAKEN = 3.U(2.W)
}

case class TacitBPParams(xlen: Int, n_entries: Int)

class DSCBranchPredictor(params: TacitBPParams) extends Module {
  val io = IO(new Bundle {
    // synchronous request
    val req_pc = Input(UInt(params.xlen.W))
    // same-cycle response
    val resp = Output(Bool())
    // asynchronous update
    val update_valid = Input(Bool())
    val update_taken = Input(Bool())
  })
  require(isPow2(params.n_entries), "n_entries must be a power of 2")
  require(params.n_entries >= 64, "n_entries must be at least 64")
  require(params.n_entries <= 1024, "n_entries must be at most 1024")
  
  def hash(pc: UInt): UInt = { (pc >> 1.U) % params.n_entries.U }
  def judge(counter: UInt): Bool = { counter === CounterState.STRONG_TAKEN || counter === CounterState.WEAK_TAKEN}
  def update(counter: UInt, taken: Bool): Unit = {
    counter := Mux(taken, 
      Mux(counter === CounterState.STRONG_TAKEN, CounterState.STRONG_TAKEN, counter + 1.U),
      Mux(counter === CounterState.STRONG_NOT_TAKEN, CounterState.STRONG_NOT_TAKEN, counter - 1.U)
    )
  }
  val counters = RegInit(VecInit(Seq.fill(params.n_entries)(CounterState.WEAK_NOT_TAKEN)))
  io.resp := judge(counters(hash(io.req_pc)))
  when (io.update_valid) {update(counters(hash(io.req_pc)), io.update_taken)}
}