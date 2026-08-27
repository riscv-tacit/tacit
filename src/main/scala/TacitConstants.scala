package tacit

import chisel3._
import chisel3.util._
import freechips.rocketchip.trace._

import org.chipsalliance.cde.config.Parameters

object FullHeaderType extends ChiselEnum {
  val FTakenBranch    = Value(0x0.U) // 000
  val FNotTakenBranch = Value(0x1.U) // 001
  val FUninfJump      = Value(0x2.U) // 010
  val FInfJump        = Value(0x3.U) // 011
  val FTrap           = Value(0x4.U) // 100
  val FSync           = Value(0x5.U) // 101
  val FValue          = Value(0x6.U) // 110
  val FReserved       = Value(0x7.U) // 111
}

object CompressedHeaderType extends ChiselEnum {
  val CTB = Value(0x0.U) // 00, taken branch
  val CNT = Value(0x1.U) // 01, not taken branch
  val CNA = Value(0x2.U) // 10, not a compressed packet
  val CIJ = Value(0x3.U) // 11, is a jump
}

object TrapType extends ChiselEnum {
  val TNone      = Value(0x0.U)
  val TException = Value(0x1.U)
  val TInterrupt = Value(0x2.U)
  val TReturn    = Value(0x4.U)
}

object SyncType extends ChiselEnum {
  val SyncNone = Value(0b000.U)
  val SyncStart = Value(0b001.U)
  val SyncPeriodic = Value(0b010.U)
  val SyncEnd = Value(0b011.U)
}

object HeaderByte {
  def from_trap_type(header_type: FullHeaderType.Type, trap_type: TrapType.Type): UInt = {
    Cat(
      trap_type.asUInt,
      header_type.asUInt,
      CompressedHeaderType.CNA.asUInt
    )
  }

  def from_sync_type(header_type: FullHeaderType.Type, sync_type: SyncType.Type): UInt = {
    Cat(
      sync_type.asUInt,
      header_type.asUInt,
      CompressedHeaderType.CNA.asUInt
    )
  }

  def apply(header_type: FullHeaderType.Type): UInt = {
    Cat(
      0.U(3.W),
      header_type.asUInt,
      CompressedHeaderType.CNA.asUInt
    )
  }
}

trait MetaDataWidthHelper {
  // abstract parameter
  val coreParams: TraceCoreParams
  def getMaxNumBytes(width: Int): Int = { width/(8-1) + 1 }
  lazy val maxASIdBits = coreParams.xlen match {
    case 32 => 9
    case 64 => 16
  }
  lazy val addrMaxNumBytes = getMaxNumBytes(coreParams.iaddrWidth)
  lazy val timeMaxNumBytes = getMaxNumBytes(coreParams.xlen)
  lazy val ctxMaxNumBytes = getMaxNumBytes(maxASIdBits)
  def metaDataBundleWidth = timeMaxNumBytes + 2*addrMaxNumBytes + ctxMaxNumBytes + 2
}

class MetaDataBundle(val coreParams: TraceCoreParams) extends Bundle with MetaDataWidthHelper {
  val time = UInt(timeMaxNumBytes.W) // 10
  val target_addr = UInt(addrMaxNumBytes.W) // 6
  val trap_addr = UInt(addrMaxNumBytes.W) // 6
  val ctx = UInt(ctxMaxNumBytes.W) // 3
  val prv = UInt(1.W) // 1
  val is_full = UInt(1.W) // 1
}

class MessagePacketBundle(val coreParams: TraceCoreParams) extends Bundle with MetaDataWidthHelper {
  val time = Vec(timeMaxNumBytes, UInt(8.W))
  val target_addr = Vec(addrMaxNumBytes, UInt(8.W))
  val trap_addr = Vec(addrMaxNumBytes, UInt(8.W))
  val ctx = Vec(ctxMaxNumBytes, UInt(8.W))
  val prv = UInt(8.W)
}
