package gemmini

import chisel3._
import chisel3.util._

object Activation {
  val NONE = 0.U
  val RELU = 1.U
  val LAYERNORM = 2.U
  val IGELU = 3.U
  val SOFTMAX = 4.U
  val SILU = 5.U
  // Exact YOLO residual writeback: identity except that the signed INT8
  // minimum is -127 rather than the architectural -128.
  val EXACT_RESADD = 6.U

  val bitwidth = 3
}

object SiLULut {
  val entries = 256
  val entryBits = 8
  val entriesPerWrite = 8
  val chunks = entries / entriesPerWrite
  val chunkBits = log2Ceil(chunks)

  /**
    * The LUT is addressed by the raw two's-complement bit pattern of q_mid.
    * The table value is sign-extended so that it can travel through the
    * accumulator-scale pipeline without changing that pipeline's data type.
    */
  def lookup[T <: Data](lut: Vec[Vec[UInt]], qMid: T): T = {
    require(qMid.getWidth >= entryBits)
    val index = qMid.asUInt
    val result = lut(index(7, 3))(index(2, 0))
    result.asSInt.pad(qMid.getWidth).asTypeOf(qMid)
  }
}

class SiLULutWrite extends Bundle {
  val chunk = UInt(SiLULut.chunkBits.W)
  val data = UInt((SiLULut.entriesPerWrite * SiLULut.entryBits).W)
}
