package tacit

import chisel3._
import chisel3.util._
import chisel3.experimental.requireIsChiselType
import freechips.rocketchip.trace._

import org.chipsalliance.cde.config.Parameters

/** Selects the storage implementation behind MultiPortedQueue. */
sealed trait MPQueueImpl
object MPQueueImpl {
  /** Flop-based storage (MultiPortedRegQueue). */
  case object Reg extends MPQueueImpl
  /** Banked SRAM relying on SyncReadMem.WriteFirst read-under-write semantics.
   *  Deprecated: the write-first guarantee is dropped by SRAM macro replacement
   *  (--repl-seq-mem emits a conf with no read-under-write field). Kept only so
   *  existing configs/bitstreams elaborate unchanged for ongoing experiments. */
  case object LegacyWriteFirstSRAM extends MPQueueImpl
  /** Banked SRAM with default read-under-write policy: reads are scheduled
   *  against registered occupancy, so a same-cycle collision is impossible by
   *  construction and any memory implementation is correct. */
  case object SRAM extends MPQueueImpl
}

object MultiPortedQueue {
  def apply[T <: Data](gen: T, numEntries: Int, numInputs: Int): MultiPortedQueue[T] = {
    Module(new MultiPortedQueue(gen, numEntries, numInputs))
  }

  def apply[T <: Data](gen: T, numEntries: Int, numInputs: Int, impl: MPQueueImpl): MultiPortedQueue[T] = {
    Module(new MultiPortedQueue(gen, numEntries, numInputs, impl))
  }

  def apply[T <: Data](gen: T, numEntries: Int, numInputs: Int, impl: MPQueueImpl, reserveCycles: Int): MultiPortedQueue[T] = {
    Module(new MultiPortedQueue(gen, numEntries, numInputs, impl, reserveCycles))
  }

  def apply[T <: Data](gen: T, numEntries: Int, numInputs: Int, impl: MPQueueImpl, reserveCycles: Int, maxEnqPerCycle: Option[Int]): MultiPortedQueue[T] = {
    Module(new MultiPortedQueue(gen, numEntries, numInputs, impl, reserveCycles, maxEnqPerCycle))
  }
}

/**
 * A queue that allows multiple inputs to be enqueued per cycle and a multiple outputs to be dequeued.
 * Does not need banking as this is flip-flop based.
 */
class MultiPortedRegQueue[T <: Data](
  val gen: T,
  val numEntries: Int,
  val numInputs: Int
) extends Module {
  requireIsChiselType(gen)
  val io = IO(new Bundle {
    val enqs = Flipped(Vec(numInputs, Decoupled(gen)))
    val deq = Decoupled(gen)
    val stall_enq = Output(Bool())
    val count = Output(UInt(log2Ceil(numEntries + 1).W))
  })

  requireIsChiselType(gen)

  val ram = Reg(Vec(numEntries, gen))

  // head and tail are masks, not pointers
  val head = RegInit(1.U(numEntries.W)) // deq
  val tail = RegInit(1.U(numEntries.W)) // enq
  val maybe_full = RegInit(false.B)

  val enq_mask = io.enqs.map(_.valid)

  val enq_count = io.enqs.map(_.fire.asUInt).reduce(_ +& _)

  def rotateLeft(in: UInt, k: Int) = {
    val n = in.getWidth
    Cat(in(n-k-1,0), in(n-1, n-k))
  }

  // explicitly excluding the tail case, as the final element hitting head is ok
  val might_hit_head = (1 until numInputs).map(k => rotateLeft(tail, k) & head).reduce(_|_).orR
  // is the tail at the head?
  val ptr_match = (tail & head).orR

  val do_enq = !(ptr_match && maybe_full || might_hit_head)
  io.enqs.map(_.ready := do_enq)
  io.stall_enq := !do_enq

  // Generate one-hot write indices
  val enq_idxs = Wire(Vec(numInputs, UInt(numEntries.W)))

  def inc_mask(mask: UInt): UInt = {
    val n = mask.getWidth
    Cat(mask(n-2,0), mask(n-1))
  }

  var enq_idx = tail // notice that this is mutable
  for (i <- 0 until numInputs) {
    enq_idxs(i) := enq_idx
    enq_idx = Mux(enq_mask(i), inc_mask(enq_idx), enq_idx)
  }

  // write to the RAM
  for (i <- 0 until numInputs) {
    for (j <- 0 until numEntries) {
      when (do_enq && enq_mask(i) && enq_idxs(i)(j)) {
        ram(j) := io.enqs(i).bits
      }
    }
  }

  // dequeue logic
  val head_hit_tail = (head & tail).orR
  val empty = head_hit_tail && !maybe_full

  val do_deq = io.deq.ready && !empty

  when (do_enq) {
    tail := enq_idx
    when (enq_mask.reduce(_||_)) {
      maybe_full := true.B
    }
  }

  when (do_deq) {
    head := inc_mask(head)
    maybe_full := false.B
  }

  io.deq.bits := Mux1H(head, ram)
  io.deq.valid := !empty

  // Convert one-hot masks to binary indices for count calculation
  val tail_idx = OHToUInt(tail)
  val head_idx = OHToUInt(head)
  val ptr_diff = tail_idx - head_idx

  if (isPow2(numEntries)) {
    io.count := Mux(maybe_full && ptr_match, numEntries.U, 0.U) | ptr_diff
  } else {
    io.count := Mux(
      ptr_match,
      Mux(maybe_full, numEntries.asUInt, 0.U),
      Mux(head_idx > tail_idx, numEntries.asUInt + ptr_diff, ptr_diff)
    )
  }
}

class MultiPortedQueue[T <: Data](
  val gen: T,
  val numEntries: Int,
  val numInputs: Int,
  val impl: MPQueueImpl = MPQueueImpl.Reg,
  val reserveCycles: Int = 0,
  // Producer-declared worst-case fires per cycle (None = numInputs). A producer
  // that is physically numInputs wide but architecturally rate-limited (e.g. an
  // in-order core that retires at most one control-flow packet per cycle) can
  // declare a tighter bound so the reserve and capacity reservation don't
  // overprovision. The declared bound is asserted every cycle.
  val maxEnqPerCycle: Option[Int] = None
) extends Module {
  requireIsChiselType(gen)
  val enqBound = maxEnqPerCycle.getOrElse(numInputs)
  require(enqBound >= 1 && enqBound <= numInputs,
    s"maxEnqPerCycle=$enqBound must be in [1, numInputs=$numInputs]")
  require(reserveCycles * enqBound <= numEntries - enqBound,
    s"reserveCycles=$reserveCycles cannot be honored: needs ${reserveCycles * enqBound} of ${numEntries - enqBound} usable entries")
  val io = IO(new Bundle {
    val enqs = Flipped(Vec(numInputs, Decoupled(gen)))
    val deq = Decoupled(gen)
    val stall_enq = Output(Bool())
    // asserted when fewer than reserveCycles worst-case (enqBound-wide) enqueue cycles of space remain
    val stall = Output(Bool())
    val count = Output(UInt(log2Ceil(numEntries + 1).W))
  })

  // the producer's rate-bound declaration is a checked contract, not folklore
  assert(PopCount(io.enqs.map(_.fire)) <= enqBound.U,
    s"enqueue rate exceeded declared maxEnqPerCycle=$enqBound")

  def stallFromCount(count: UInt): Bool =
    (numEntries.U -& count) < (reserveCycles * enqBound).U

  impl match {
    case MPQueueImpl.Reg =>
      val reg_queue = Module(new MultiPortedRegQueue(gen, numEntries, numInputs))
      reg_queue.io.enqs <> io.enqs
      io.deq <> reg_queue.io.deq
      io.stall_enq := reg_queue.io.stall_enq
      io.stall := stallFromCount(reg_queue.io.count)
      io.count := reg_queue.io.count
    case MPQueueImpl.LegacyWriteFirstSRAM =>
      val sram_queue = Module(new MultiPortedWriteFirstSRAMQueue(gen, numEntries, numInputs))
      sram_queue.io.enqs <> io.enqs
      io.deq <> sram_queue.io.deq
      io.stall_enq := sram_queue.io.full
      io.stall := stallFromCount(sram_queue.io.count)
      io.count := sram_queue.io.count
    case MPQueueImpl.SRAM =>
      val sram_queue = Module(new MultiPortedSRAMQueue(gen, numEntries, numInputs, reserveCycles, maxEnqPerCycle))
      sram_queue.io.enqs <> io.enqs
      io.deq <> sram_queue.io.deq
      io.stall_enq := sram_queue.io.stall_enq
      io.stall := sram_queue.io.stall
      io.count := sram_queue.io.count
  }
}

/* A queue accepting up to numInputs enqueues per cycle (shared all-or-nothing
 * ready, lane order preserved) and one dequeue per cycle, backed by banked SRAM.
 *
 * Storage: lanes are padded internally to physInputs = pow2(numInputs); element k
 * lives in bank (k % physInputs), row (k / physInputs), so any set of same-cycle
 * enqueues (consecutive logical slots) lands in distinct banks and the bank/row
 * split is pure bit slicing.
 *
 * Collision-freedom by construction: reads are gated on `sram_avail`, a REGISTER
 * counting elements committed at a previous clock edge. A read therefore never
 * targets a slot written in the same cycle, and the banks need no read-under-write
 * guarantee (default policy; any macro/FPGA/synflops implementation is correct).
 * The cost is one extra cycle of empty-to-deq.valid latency (2 instead of 1);
 * sustained throughput is unaffected.
 *
 * Read responses are un-stallable (data valid one cycle after issue). The output
 * queue's capacity equals the read round-trip (2), which lets the credit rule use
 * registered state only: no combinational path from io.deq.ready into the SRAM.
 */
class MultiPortedSRAMQueue[T <: Data](
  val gen: T,
  val numEntries: Int,
  val numInputs: Int,
  val reserveCycles: Int = 0,
  val maxEnqPerCycle: Option[Int] = None
) extends Module {
  requireIsChiselType(gen)
  require(numInputs >= 1)
  require(isPow2(numEntries) && numEntries >= 2 * numInputs)
  val enqBound = maxEnqPerCycle.getOrElse(numInputs)
  require(enqBound >= 1 && enqBound <= numInputs)
  require(reserveCycles * enqBound <= numEntries - enqBound)

  private val physInputs = 1 << log2Ceil(numInputs) // pad lanes to a power of two
  val depth    = numEntries / physInputs            // exact: physInputs <= numEntries, both pow2
  val ptrBits  = log2Ceil(numEntries)
  val bankBits = log2Ceil(physInputs) max 1
  val cntBits  = log2Ceil(numEntries + 1)

  def bankOf(p: UInt): UInt = if (physInputs == 1) 0.U(1.W) else p(log2Ceil(physInputs) - 1, 0)
  def rowOf(p: UInt):  UInt = if (physInputs == 1) p else p(ptrBits - 1, log2Ceil(physInputs))

  val io = IO(new Bundle {
    val enqs      = Flipped(Vec(numInputs, Decoupled(gen)))
    val deq       = Decoupled(gen)
    val stall_enq = Output(Bool())          // immediate: cannot accept a full-width enqueue
    val stall     = Output(Bool())          // early-warning per the reserveCycles contract
    val count     = Output(UInt(cntBits.W)) // exact element occupancy
  })

  // ---------------- state (complete inventory) ----------------
  val banks        = Seq.fill(physInputs)(SyncReadMem(depth, gen))
  val wr_ptr       = RegInit(0.U(ptrBits.W))  // next slot to write
  val rd_ptr       = RegInit(0.U(ptrBits.W))  // next slot to fetch from SRAM
  val sram_avail   = RegInit(0.U(cntBits.W))  // committed-before-this-edge, not yet fetched
  val resp_pending = RegInit(false.B)         // a read was issued last edge
  val resp_bank    = Reg(UInt(bankBits.W))    // which bank that read targeted
  val out_q        = Module(new Queue(gen, 2, flow = true))

  // ---------------- derived occupancy (never registered) ----------------
  val total = sram_avail +& resp_pending.asUInt +& out_q.io.count
  // reserve capacity for one declared-worst-case enqueue cycle (enqBound <= numInputs)
  val can_accept = total <= (numEntries - enqBound).U
  io.enqs.foreach(_.ready := can_accept)
  io.stall_enq := !can_accept
  // Early warning from registered occupancy only, so the path into the core's
  // commit gate stays register-derived. Note the snapshot excludes this cycle's
  // enqueue: the producer must size reserveCycles as (cycles of traffic still in
  // flight after stall) + 1, see TacitParallelEncoder.
  io.stall     := (numEntries.U -& total) < (reserveCycles * enqBound).U
  io.count     := total(cntBits - 1, 0)

  // ---------------- enqueue stage ----------------
  val enq_fires = io.enqs.map(_.fire)
  val enq_count = PopCount(enq_fires)

  val write_en   = WireDefault(VecInit(Seq.fill(physInputs)(false.B)))
  val write_row  = Wire(Vec(physInputs, UInt(log2Ceil(depth).W)))
  val write_data = Wire(Vec(physInputs, gen))
  write_row  := DontCare
  write_data := DontCare

  for (i <- 0 until numInputs) {
    // firing lane i takes logical slot wr_ptr + (number of firing lanes below i);
    // slots are consecutive, so banks are distinct (consecutive mod pow2)
    val slot = wr_ptr + PopCount(enq_fires.take(i))
    when (io.enqs(i).fire) {
      write_en(bankOf(slot))   := true.B
      write_row(bankOf(slot))  := rowOf(slot)
      write_data(bankOf(slot)) := io.enqs(i).bits
    }
  }
  for (b <- 0 until physInputs) {
    when (write_en(b)) { banks(b).write(write_row(b), write_data(b)) }
  }
  wr_ptr := wr_ptr + enq_count

  // ---------------- fetch stage ----------------
  // capacity-2 output queue = read round-trip, so the credit check needs no io.deq.fire term
  val do_read   = sram_avail =/= 0.U && (out_q.io.count +& resp_pending.asUInt) < 2.U
  val read_bank = bankOf(rd_ptr)
  val read_row  = rowOf(rd_ptr)

  val bank_rdata = VecInit(banks.zipWithIndex.map { case (m, b) =>
    m.read(read_row, do_read && read_bank === b.U)
  })

  when (do_read) {
    rd_ptr    := rd_ptr + 1.U
    resp_bank := read_bank
  }
  resp_pending := do_read
  sram_avail   := sram_avail + enq_count - do_read.asUInt

  // ---------------- delivery stage ----------------
  val resp_data =
    if (physInputs == 1) bank_rdata(0)
    else Mux1H(UIntToOH(resp_bank, physInputs), bank_rdata)

  out_q.io.enq.valid := resp_pending
  out_q.io.enq.bits  := resp_data
  io.deq <> out_q.io.deq

  // ---------------- invariants ----------------
  assert(!resp_pending || out_q.io.enq.ready,
    "response arrived with no landing space: credit rule violated")
  assert(!do_read || !(write_en(read_bank) && write_row(read_bank) === read_row),
    "read collided with a same-cycle write: sram_avail gating violated")
  assert(sram_avail === wr_ptr - rd_ptr, "occupancy counter out of sync with pointers")
  assert(total <= numEntries.U, "conservation violated")
  assert(enq_count <= enqBound.U, s"enqueue rate exceeded declared maxEnqPerCycle=$enqBound")
  for (b <- 0 until physInputs) {
    val hits = (0 until numInputs).map { i =>
      io.enqs(i).fire && bankOf(wr_ptr + PopCount(enq_fires.take(i))) === b.U
    }
    assert(PopCount(hits) <= 1.U, s"two lanes wrote bank $b in one cycle")
  }
}

/* A queue that allows multiple inputs to be enqueued per cycle and a single output to be dequeued.
  All or nothing enqueue, ordered dequeue
*/
class MultiPortedWriteFirstSRAMQueue[T <: Data](
  val gen: T,
  val numEntries: Int,
  val numInputs: Int,
) extends Module {

  requireIsChiselType(gen)

  val io = IO(new Bundle {
    val enqs = Flipped(Vec(numInputs, Decoupled(gen)))
    val deq = Decoupled(gen)
    val full = Output(Bool()) // essentially enq ready, but deduplicated
    val count = Output(UInt(log2Ceil(numEntries + 1).W))
  })

  val depth = numEntries / numInputs

  require(depth > 0)
  require(numInputs > 0)

  val max_entries = depth * numInputs
  val ptr_width = log2Ceil(max_entries)
  val count_width = log2Ceil(max_entries + 1)

  def wrapAdd(ptr: UInt, inc: UInt): UInt = {
    val sum = ptr +& inc
    Mux(sum >= max_entries.U, (sum - max_entries.U)(ptr_width - 1, 0), sum(ptr_width - 1, 0))
  }

  private val bank_idx_width = log2Ceil(numInputs)
  def bankOf(ptr: UInt): UInt = {
    if (numInputs == 1) 0.U(1.W) else (ptr % numInputs.U)(bank_idx_width - 1, 0)
  }
  def rowOf(ptr: UInt): UInt = ptr / numInputs.U

  val banks = Seq.fill(numInputs)(SyncReadMem(depth, gen, SyncReadMem.WriteFirst))

  val enq_ptr = RegInit(0.U(ptr_width.W))
  val deq_ptr = RegInit(0.U(ptr_width.W))
  val entries = RegInit(0.U(count_width.W))

  val front_data = Reg(gen)
  val front_valid = RegInit(false.B)

  val read_pending = RegInit(false.B)
  val read_pending_bank = RegInit(0.U(bank_idx_width.W))

  // Match the register queue policy: all inputs share one ready bit and enqueue is all-or-nothing
  // with capacity reserved for a full numInputs-wide enqueue.
  val can_accept = entries <= (max_entries - numInputs).U
  io.enqs.foreach(_.ready := can_accept)
  io.full := !can_accept

  val enq_fire_vec = io.enqs.map(enq => enq.valid && can_accept)
  val enq_count = PopCount(enq_fire_vec)
  val do_enq = enq_count =/= 0.U
  val deq_valid = front_valid || read_pending
  val do_deq = io.deq.ready && deq_valid

  val write_enable = WireInit(VecInit(Seq.fill(numInputs)(false.B)))
  val write_addr = Wire(Vec(numInputs, UInt(log2Ceil(depth).W)))
  val write_data = Wire(Vec(numInputs, gen))
  for (i <- 0 until numInputs) {
    write_addr(i) := 0.U
    write_data(i) := DontCare
  }

  for (i <- 0 until numInputs) {
    when (enq_fire_vec(i)) {
      val enq_rank = PopCount(enq_fire_vec.take(i))
      val write_ptr = wrapAdd(enq_ptr, enq_rank)
      val write_bank = bankOf(write_ptr)
      write_enable(write_bank) := true.B
      write_addr(write_bank) := rowOf(write_ptr)
      write_data(write_bank) := io.enqs(i).bits
    }
  }

  val next_deq_ptr = Mux(do_deq, wrapAdd(deq_ptr, 1.U), deq_ptr)
  val entries_after = entries +& enq_count -& do_deq.asUInt
  val front_valid_after_deq = front_valid && !do_deq
  val consumed_from_read_pending = do_deq && !front_valid && read_pending
  val front_valid_next = (front_valid && !do_deq) || (read_pending && !consumed_from_read_pending)
  front_valid := front_valid_next
  val issue_front_read = entries_after =/= 0.U && !front_valid_next

  val read_req_bank = bankOf(next_deq_ptr)
  val read_req_row = rowOf(next_deq_ptr)

  val bank_read_data = Wire(Vec(numInputs, gen))
  for (i <- 0 until numInputs) {
    when (write_enable(i)) {
      banks(i).write(write_addr(i), write_data(i))
    }
    val do_read = issue_front_read && read_req_bank === i.U
    bank_read_data(i) := banks(i).read(read_req_row, do_read)
  }

  val read_resp_data = Mux1H(UIntToOH(read_pending_bank, numInputs), bank_read_data)
  val deq_bits = Mux(front_valid, front_data, read_resp_data)

  when (read_pending && !consumed_from_read_pending) {
    front_data := read_resp_data
  }

  when (do_deq) {
    deq_ptr := wrapAdd(deq_ptr, 1.U)
  }

  when (do_enq) {
    enq_ptr := wrapAdd(enq_ptr, enq_count)
  }

  entries := entries_after

  when (issue_front_read) {
    read_pending := true.B
    read_pending_bank := read_req_bank
  } .otherwise {
    read_pending := false.B
  }

  io.deq.bits := deq_bits
  io.deq.valid := deq_valid

  io.count := entries
}
