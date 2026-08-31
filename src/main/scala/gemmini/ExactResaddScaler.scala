package gemmini

import chisel3._
import chisel3.util._

/**
  * Fixed-point INT8 -> INT32 scaling used only by the opt-in exact ResAdd
  * command. The 32-bit scale payload is intentionally not a Float:
  *
  *   [31:26] unsigned right shift (0..63)
  *   [25: 0] signed two's-complement multiplier
  *
  * Rounding is symmetric half-away-from-zero, matching yolov8_cpu_ops.c.
  * Unlike the legacy mvin scaler, the result is not narrowed to INT8 before
  * it reaches the accumulator.
  */
class ExactResaddScaleReq[Tag <: Data](
  blockCols: Int, inputWidth: Int, tagT: Tag) extends Bundle {
  val in = Vec(blockCols, SInt(inputWidth.W))
  val scale = UInt(32.W)
  val last = Bool()
  val tag = tagT.cloneType
}

class ExactResaddScaleResp[Tag <: Data](
  blockCols: Int, accWidth: Int, tagT: Tag) extends Bundle {
  val out = Vec(blockCols, SInt(accWidth.W))
  val last = Bool()
  val tag = tagT.cloneType
}

class ExactResaddScaler[Tag <: Data](
  blockCols: Int, inputWidth: Int, accWidth: Int, tagT: Tag) extends Module {
  val io = IO(new Bundle {
    val req = Flipped(Decoupled(
      new ExactResaddScaleReq(blockCols, inputWidth, tagT)))
    val resp = Decoupled(
      new ExactResaddScaleResp(blockCols, accWidth, tagT))
  })

  val outputQ = Module(new Queue(
    new ExactResaddScaleResp(blockCols, accWidth, tagT),
    entries = 2, pipe = true))

  outputQ.io.enq.valid := io.req.valid
  io.req.ready := outputQ.io.enq.ready
  outputQ.io.enq.bits.last := io.req.bits.last
  outputQ.io.enq.bits.tag := io.req.bits.tag
  io.resp <> outputQ.io.deq

  val shift = io.req.bits.scale(31, 26)
  val multiplier = io.req.bits.scale(25, 0).asSInt
  val maxAcc = ((BigInt(1) << (accWidth - 1)) - 1).S(66.W)
  val minAcc = (-(BigInt(1) << (accWidth - 1))).S(66.W)

  for (lane <- 0 until blockCols) {
    val product = io.req.bits.in(lane) * multiplier
    val productWide = product.pad(64)
    val negative = productWide < 0.S
    val magnitude = Mux(negative, (-productWide).asUInt,
      productWide.asUInt)
    val roundBias = Mux(shift === 0.U, 0.U(64.W),
      (1.U(64.W) << (shift - 1.U))(63, 0))
    val roundedMagnitude = Mux(shift === 0.U, magnitude,
      (magnitude +& roundBias) >> shift)
    val roundedSigned = Mux(negative,
      -roundedMagnitude.asSInt, roundedMagnitude.asSInt).pad(66)
    val saturated = Mux(roundedSigned > maxAcc, maxAcc,
      Mux(roundedSigned < minAcc, minAcc, roundedSigned))

    outputQ.io.enq.bits.out(lane) := saturated(accWidth - 1, 0).asSInt
  }
}
