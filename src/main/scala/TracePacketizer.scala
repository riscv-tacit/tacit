package tacit

import chisel3._
import chisel3.util._
import freechips.rocketchip.trace.{TraceCoreParams, TraceEgressConstants, TraceEgressInterface}

// slice packets into bytes TODO: is this efficient?
class TracePacketizer(val coreParams: TraceCoreParams) extends Module with MetaDataWidthHelper {

  val io = IO(new Bundle {
    val target_addr = Flipped(Decoupled(Vec(addrMaxNumBytes, UInt(8.W))))
    val trap_addr = Flipped(Decoupled(Vec(addrMaxNumBytes, UInt(8.W))))
    val time = Flipped(Decoupled(Vec(timeMaxNumBytes, UInt(8.W))))
    val byte = Flipped(Decoupled(UInt(8.W)))
    val prv = Flipped(Decoupled(UInt(8.W)))
    val ctx = Flipped(Decoupled(Vec(ctxMaxNumBytes, UInt(8.W))))
    val metadata = Flipped(Decoupled(new MetaDataBundle(coreParams)))
    val out = Decoupled(UInt(8.W))
  })

  val pIdle :: pComp :: pFull :: Nil = Enum(3)
  val state = RegInit(pIdle)

  val trap_addr_num_bytes = Reg(UInt(log2Ceil(addrMaxNumBytes).W))
  val trap_addr_index = Reg(UInt(log2Ceil(addrMaxNumBytes).W))
  val target_addr_num_bytes = Reg(UInt(log2Ceil(addrMaxNumBytes).W))
  val target_addr_index = Reg(UInt(log2Ceil(addrMaxNumBytes).W))
  val time_num_bytes = Reg(UInt(log2Ceil(timeMaxNumBytes).W))
  val time_index = Reg(UInt(log2Ceil(timeMaxNumBytes).W))
  val prv_num_bytes = Reg(UInt(1.W))
  val prv_index = Reg(UInt(1.W))
  val ctx_num_bytes = Reg(UInt(log2Ceil(ctxMaxNumBytes).W))
  val ctx_index = Reg(UInt(log2Ceil(ctxMaxNumBytes).W))
  val header_num_bytes = Reg(UInt(1.W))
  val header_index = Reg(UInt(1.W))
  
  // default values
  io.out.valid := false.B
  io.metadata.ready := false.B
  io.target_addr.ready := false.B
  io.trap_addr.ready := false.B
  io.time.ready := false.B
  io.prv.ready := false.B
  io.ctx.ready := false.B
  io.byte.ready := false.B
  io.out.bits := 0.U

  def prep_next_state(): Unit = {
    trap_addr_index := 0.U
    trap_addr_num_bytes := Mux(io.metadata.fire, io.metadata.bits.trap_addr, 0.U)
    target_addr_index := 0.U
    target_addr_num_bytes := Mux(io.metadata.fire, io.metadata.bits.target_addr, 0.U)
    time_index := 0.U
    time_num_bytes := Mux(io.metadata.fire, io.metadata.bits.time, 0.U)
    prv_index := 0.U
    prv_num_bytes := Mux(io.metadata.fire, io.metadata.bits.prv, 0.U)
    header_index := 0.U
    header_num_bytes := Mux(io.metadata.fire, io.metadata.bits.is_full, 0.U)
    state := Mux(io.metadata.fire, 
      Mux(io.metadata.bits.is_full.asBool, pFull, pComp),
      pIdle
    )
  }
  
  switch (state) {
    is (pIdle) {
      io.metadata.ready := true.B
      when (io.metadata.fire) {
        trap_addr_num_bytes := io.metadata.bits.trap_addr
        trap_addr_index := 0.U
        target_addr_num_bytes := io.metadata.bits.target_addr
        target_addr_index := 0.U
        time_num_bytes := io.metadata.bits.time
        time_index := 0.U
        header_num_bytes := io.metadata.bits.is_full
        header_index := 0.U
        prv_num_bytes := io.metadata.bits.prv
        prv_index := 0.U
        ctx_num_bytes := io.metadata.bits.ctx
        ctx_index := 0.U
        state := Mux(io.metadata.bits.is_full.asBool, pFull, pComp)
      }
    }
    is (pComp) {
      // transmit a byte from byte buffer
      io.byte.ready := io.out.ready
      io.out.valid := io.byte.valid
      io.out.bits := io.byte.bits
      when (io.byte.fire) {
        // metadata runs ahead by 1 cycle for performance optimization
        io.metadata.ready := true.B
        prep_next_state()
      }
    }
    is (pFull) {
      // header, addr, time
      io.out.valid := true.B
      when (header_num_bytes > 0.U && header_index < header_num_bytes) {
        io.out.bits := io.byte.bits
        io.out.valid := io.byte.valid
        header_index := header_index + io.out.fire
      } .elsewhen (prv_num_bytes > 0.U && prv_index < prv_num_bytes) {
        io.out.bits := io.prv.bits
        io.out.valid := io.prv.valid
        prv_index := prv_index + io.out.fire
      } .elsewhen (ctx_num_bytes > 0.U && ctx_index < ctx_num_bytes) {
        io.out.bits := io.ctx.bits(ctx_index)
        io.out.valid := io.ctx.valid
        ctx_index := ctx_index + io.out.fire
      } .elsewhen (trap_addr_num_bytes > 0.U && trap_addr_index < trap_addr_num_bytes) {
        io.out.bits := io.trap_addr.bits(trap_addr_index)
        io.out.valid := io.trap_addr.valid
        trap_addr_index := trap_addr_index + io.out.fire
      } .elsewhen (target_addr_num_bytes > 0.U && target_addr_index < target_addr_num_bytes) {
        io.out.bits := io.target_addr.bits(target_addr_index)
        io.out.valid := io.target_addr.valid
        target_addr_index := target_addr_index + io.out.fire
      } .elsewhen (time_num_bytes > 0.U && time_index < time_num_bytes) {
        io.out.bits := io.time.bits(time_index)
        io.out.valid := io.time.valid
        time_index := time_index + io.out.fire
      } .otherwise {
        // FIXME: delay for 1 cycle
        io.out.valid := false.B
        // release buffers
        io.byte.ready := true.B
        // conditional depletion of buffers, based on whether we included it in the packet
        io.target_addr.ready := target_addr_num_bytes =/= 0.U
        io.trap_addr.ready := trap_addr_num_bytes =/= 0.U
        io.time.ready := time_num_bytes =/= 0.U
        io.prv.ready := prv_num_bytes =/= 0.U
        io.ctx.ready := ctx_num_bytes =/= 0.U
        io.metadata.ready := true.B
        prep_next_state()
      }
    }
  }
}

class TraceMaskedPacketizer(val coreParams: TraceCoreParams) extends Module with MetaDataWidthHelper {
  private val numLanes = TraceEgressConstants.numLanes
  val io = IO(new Bundle {
    val message = Flipped(Decoupled(new MessagePacketBundle(coreParams)))
    val byte = Flipped(Decoupled(UInt(8.W)))
    val metadata = Flipped(Decoupled(new MetaDataBundle(coreParams)))
    val out = new TraceEgressInterface()
  })

  val pIdle :: pComp :: pFull :: Nil = Enum(3)
  val state = RegInit(pIdle)

  val metadata_mask_reg = Reg(UInt(metaDataBundleWidth.W))

  io.out.valid := false.B
  io.metadata.ready := false.B
  io.message.ready := false.B
  io.byte.ready := false.B
  io.out.bits := VecInit.fill(numLanes)(0.U(8.W))
  io.out.mask := VecInit.fill(numLanes)(false.B)

  def prep_next_state(): Unit = {
    io.metadata.ready := true.B
    state := Mux(io.metadata.fire, 
      Mux(io.metadata.bits.is_full.asBool, pFull, pComp),
      pIdle
    )
  }

  io.byte.ready := false.B
  io.message.ready := false.B
  io.metadata.ready := false.B
  when (io.metadata.fire) { metadata_mask_reg := io.metadata.bits.asUInt }

  switch (state) {
    is (pIdle) {
      io.metadata.ready := true.B
      when (io.metadata.fire) {
        state := Mux(io.metadata.bits.is_full.asBool, pFull, pComp)
      }
    }
    is (pComp) {
      io.byte.ready := io.out.ready
      io.out.valid := io.byte.valid
      // only use the first byte for now
      io.out.bits(0) := io.byte.bits
      io.out.mask(0) := io.byte.valid
      when (io.byte.fire) {
        io.metadata.ready := true.B
        prep_next_state()
      }
    }
    is (pFull) {
      when (metadata_mask_reg =/= 0.U) {
        val payloadBytes = Wire(Vec(metaDataBundleWidth, UInt(8.W)))
        payloadBytes := VecInit(Seq.fill(metaDataBundleWidth)(0.U(8.W)))
        payloadBytes(0) := io.byte.bits
        payloadBytes(1) := io.message.bits.prv
        for (i <- 0 until ctxMaxNumBytes) {
          payloadBytes(2 + i) := io.message.bits.ctx(i)
        }
        for (i <- 0 until addrMaxNumBytes) {
          payloadBytes(2 + ctxMaxNumBytes + i) := io.message.bits.trap_addr(i)
        }
        for (i <- 0 until addrMaxNumBytes) {
          payloadBytes(2 + ctxMaxNumBytes + addrMaxNumBytes + i) := io.message.bits.target_addr(i)
        }
        for (i <- 0 until timeMaxNumBytes) {
          payloadBytes(2 + ctxMaxNumBytes + 2*addrMaxNumBytes + i) := io.message.bits.time(i)
        }

        val laneValid = Wire(Vec(numLanes, Bool()))
        val laneIdx = Wire(Vec(numLanes, UInt(log2Ceil(metaDataBundleWidth).W)))
        val laneMask = Wire(Vec(numLanes + 1, UInt(metaDataBundleWidth.W)))

        laneMask(0) := metadata_mask_reg
        for (i <- 0 until numLanes) {
          laneValid(i) := laneMask(i) =/= 0.U
          laneIdx(i) := PriorityEncoder(laneMask(i))
          laneMask(i + 1) := Mux(laneValid(i), laneMask(i) & ~(1.U << laneIdx(i)), laneMask(i))
        }

        val needByte = laneValid.zip(laneIdx).map { case (v, idx) => v && idx === 0.U }.reduce(_||_)
        val needMessage = laneValid.zip(laneIdx).map { case (v, idx) => v && idx =/= 0.U }.reduce(_||_)
        val canSend = laneValid(0) && (!needByte || io.byte.valid) && (!needMessage || io.message.valid)

        io.out.valid := canSend
        io.out.mask := VecInit(laneValid.map(_ && canSend))

        for (i <- 0 until numLanes) {
          io.out.bits(i) := payloadBytes(laneIdx(i))
        }

        metadata_mask_reg := Mux(io.out.fire, laneMask(numLanes), metadata_mask_reg)
      } .otherwise {
        io.out.valid := false.B // FIXME: there's always a hiccup here?
        io.byte.ready := true.B // dequeue the header byte
        io.message.ready := true.B // dequeue the message
        prep_next_state()
      }
    }
  }

}
