package gemmini

import chisel3._
import chisel3.util._

/** Seven elastic stages; same 64/65/66-bit intermediate widths and symmetric
  * half-away-from-zero rounding as ExactResaddScaler. Every stage carries its
  * shift/sign, DMA tag and last bit with the data, including under backpressure.
  * Saturation is registered, not left on the output queue's input path.
  */
class ExactResaddScalePipeline[Tag <: Data](
  blockCols: Int, inputWidth: Int, accWidth: Int, tagT: Tag) extends Module {
  require(inputWidth == 8 && accWidth == 32,
    "Pipelined exact ResAdd currently supports INT8 to INT32")
  val io = IO(new Bundle {
    val req = Flipped(Decoupled(new ExactResaddScaleReq(blockCols, inputWidth, tagT)))
    val resp = Decoupled(new ExactResaddScaleResp(blockCols, accWidth, tagT))
  })
  class Lane extends Bundle {
    val in = SInt(inputWidth.W)
    val product = SInt((inputWidth + 26).W)
    val negative = Bool()
    val magnitude = UInt(64.W)
    val biased = UInt(65.W)
    val shifted = UInt(65.W)
    val signed = SInt(66.W)
    val out = SInt(accWidth.W)
  }
  class Token extends Bundle {
    val multiplier = SInt(26.W)
    val shift = UInt(6.W)
    val bias = UInt(64.W)
    val lanes = Vec(blockCols, new Lane)
    val last = Bool()
    val tag = tagT.cloneType
  }
  val input = Wire(Decoupled(new Token))
  input.valid := io.req.valid
  io.req.ready := input.ready
  input.bits := 0.U.asTypeOf(input.bits)
  input.bits.multiplier := io.req.bits.scale(25, 0).asSInt
  input.bits.shift := io.req.bits.scale(31, 26)
  input.bits.last := io.req.bits.last
  input.bits.tag := io.req.bits.tag
  (input.bits.lanes zip io.req.bits.in).foreach { case (l, d) => l.in := d }
  def transform(f: (Token, Token) => Unit): Token => Token = (x: Token) => {
    val y = WireInit(x)
    f(x, y)
    y
  }
  val capture: Token => Token = (x: Token) => x
  val multiply = transform { (x, y) =>
    (x.lanes zip y.lanes).foreach { case (a, b) => b.product := a.in * x.multiplier }
  }
  val magnitude = transform { (x, y) =>
    y.bias := Mux(x.shift === 0.U, 0.U(64.W),
      (1.U(64.W) << (x.shift - 1.U))(63, 0))
    (x.lanes zip y.lanes).foreach { case (a, b) =>
      val wide = a.product.pad(64)
      b.negative := wide < 0.S
      b.magnitude := Mux(wide < 0.S, (-wide).asUInt, wide.asUInt)
    }
  }
  val rounding = transform { (x, y) =>
    (x.lanes zip y.lanes).foreach { case (a, b) => b.biased := a.magnitude +& x.bias }
  }
  val shift = transform { (x, y) =>
    (x.lanes zip y.lanes).foreach { case (a, b) => b.shifted := a.biased >> x.shift }
  }
  val sign = transform { (x, y) =>
    (x.lanes zip y.lanes).foreach { case (a, b) =>
      b.signed := Mux(a.negative, -a.shifted.asSInt, a.shifted.asSInt).pad(66)
    }
  }
  val saturate = transform { (x, y) =>
    val maxAcc = ((BigInt(1) << (accWidth - 1)) - 1).S(66.W)
    val minAcc = (-(BigInt(1) << (accWidth - 1))).S(66.W)
    (x.lanes zip y.lanes).foreach { case (a, b) =>
      b.out := Mux(a.signed > maxAcc, maxAcc,
        Mux(a.signed < minAcc, minAcc, a.signed))(accWidth - 1, 0).asSInt
    }
  }
  val result = Pipeline(input, 7,
    Seq(capture, multiply, magnitude, rounding, shift, sign, saturate, capture))
  io.resp.valid := result.valid
  result.ready := io.resp.ready
  io.resp.bits.last := result.bits.last
  io.resp.bits.tag := result.bits.tag
  (io.resp.bits.out zip result.bits.lanes).foreach { case (o, l) => o := l.out }
}
