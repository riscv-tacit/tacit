package tacit

import chisel3._
import chisel3.util._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.prci._
import org.chipsalliance.cde.config.{Parameters, Config, Field}
import freechips.rocketchip.tile._
import freechips.rocketchip.subsystem._
import shuttle.common.{ShuttleTile, ShuttleTileAttachParams}
import sifive.blocks.devices.uart.{UARTTx, UARTParams}
import testchipip.soc.{SubsystemInjector, SubsystemInjectorKey}

import freechips.rocketchip.trace._

class TraceSinkUart()(implicit p: Parameters) extends LazyTraceSink {
  override lazy val module = new TraceSinkUartImpl(this)
  class TraceSinkUartImpl(outer: TraceSinkUart) extends LazyTraceSinkModuleImp(outer) {
    val uart_io = IO(new Bundle {
      val tx = Output(Bool())
    })
    val uart = Module(new UARTTx(UARTParams(
      address = 0x10000000,
      stopBits = 1,
    )))
    uart.io.en := true.B
    uart.io.in <> io.trace_in
    uart.io.div := 868.U
    uart.io.nstop := 1.U
    uart.io.tx_busy := DontCare
    uart_io.tx := uart.io.out
  }
}

class WithTraceSinkUart(targetId: Int = 0) extends Config((site, here, up) => {
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: RocketTileAttachParams => {
      tp.copy(tileParams = tp.tileParams.copy(
        traceParams = Some(tp.tileParams.traceParams.get.copy(buildSinks = 
          tp.tileParams.traceParams.get.buildSinks :+ (p => (LazyModule(new TraceSinkUart()(p)), targetId)))))
      )
    }
    case tp: ShuttleTileAttachParams => {
      tp.copy(tileParams = tp.tileParams.copy(
        traceParams = Some(tp.tileParams.traceParams.get.copy(buildSinks = 
          tp.tileParams.traceParams.get.buildSinks :+ (p => (LazyModule(new TraceSinkUart()(p)), targetId)))))
      )
    }
    case other => other
  }
  case SubsystemInjectorKey => up(SubsystemInjectorKey) + TraceSinkUartInjector
})

case object TraceSinkUartInjector extends SubsystemInjector((p, baseSubsystem) => {
  require(baseSubsystem.isInstanceOf[BaseSubsystem with InstantiatesHierarchicalElements])
  val hierarchicalSubsystem = baseSubsystem.asInstanceOf[BaseSubsystem with InstantiatesHierarchicalElements]
  implicit val q: Parameters = p
  val traceSinkUarts = hierarchicalSubsystem.totalTiles.values.map { t => t match {
    case r: RocketTile => r.trace_sinks.collect { case r: TraceSinkUart => (t, r) }
    case s: ShuttleTile => s.trace_sinks.collect { case r: TraceSinkUart => (t, r) }
    case _ => Nil
  }}.flatten
  if (traceSinkUarts.nonEmpty) {
    traceSinkUarts.foreach { case (t, s) =>
        val tile_uart_io = t { InModuleBody {
          val tile_uart_io = IO(Output(Bool())).suggestName("tacit_uart_io")
          tile_uart_io := s.module.uart_io.tx
          tile_uart_io
        }
      }
      val uart_io = InModuleBody {
        val uart_io = IO(Output(Bool())).suggestName("tacit_uart_io")
        uart_io := tile_uart_io
        uart_io
      }
    }
  }
})