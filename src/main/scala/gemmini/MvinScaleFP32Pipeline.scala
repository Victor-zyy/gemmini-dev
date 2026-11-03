package gemmini

import chisel3._
import chisel3.util._
import hardfloat._

/** Default signed INT8 * FP32 MVIN semantics, eight fixed-latency stages.
  * Unlike the accumulator scaler, RecFNToIN is deliberately eight bits:
  * converting to INT32 then clipping would change exceptional-value semantics.
  * Valid-only pipeline: the VSM reserves an entry/lane before issuing a token.
  */
class MvinScaleFP32Pipeline extends Module {
  val io = IO(new Bundle {
    val in = Input(Valid(new Bundle {
      val data = SInt(8.W)
      val scale = UInt(32.W)
    }))
    val out = Output(Valid(SInt(8.W)))
  })
  class Token extends Bundle {
    val data = SInt(8.W)
    val scale = UInt(33.W)
    val recoded = UInt(33.W)
    val a = UInt(24.W)
    val b = UInt(24.W)
    val c = UInt(48.W)
    val inter = new MulAddRecFN_interIo(8, 24)
    val product = UInt(49.W)
    val raw = new RawFloat(8, 26)
    val invalid = Bool()
    val rounded = UInt(33.W)
  }
  val input = Wire(Valid(new Token))
  input.valid := io.in.valid
  input.bits := 0.U.asTypeOf(input.bits)
  input.bits.data := io.in.bits.data
  input.bits.scale := recFNFromFN(8, 24, io.in.bits.scale)
  def stage(x: ValidIO[Token])(f: (Token, Token) => Unit): ValidIO[Token] = {
    val y = WireInit(x)
    f(x.bits, y.bits)
    Pipe(y, 1)
  }
  val s0 = Pipe(input, 1) // capture DMA/arbitration output before arithmetic
  val s1 = stage(s0) { (a, b) =>
    val m = Module(new INToRecFN(8, 8, 24))
    m.io.signedIn := true.B
    m.io.in := a.data.asUInt
    m.io.roundingMode := consts.round_near_even
    m.io.detectTininess := consts.tininess_afterRounding
    b.recoded := m.io.out
  }
  val s2 = stage(s1) { (a, b) =>
    val m = Module(new MulAddRecFNToRaw_preMul(8, 24))
    m.io.op := 0.U
    m.io.a := a.recoded
    m.io.b := a.scale
    m.io.c := 0.U
    b.a := m.io.mulAddA
    b.b := m.io.mulAddB
    b.c := m.io.mulAddC
    b.inter := m.io.toPostMul
  }
  val s3 = stage(s2) { (a, b) => b.product := (a.a * a.b) +& a.c }
  val s4 = stage(s3) { (a, b) =>
    val m = Module(new MulAddRecFNToRaw_postMul(8, 24))
    m.io.fromPreMul := a.inter
    m.io.mulAddResult := a.product
    m.io.roundingMode := consts.round_near_even
    b.raw := m.io.rawOut
    b.invalid := m.io.invalidExc
  }
  val s5 = stage(s4) { (a, b) =>
    val m = Module(new RoundRawFNToRecFN(8, 24, 0))
    m.io.in := a.raw
    m.io.invalidExc := a.invalid
    m.io.infiniteExc := false.B
    m.io.roundingMode := consts.round_near_even
    m.io.detectTininess := consts.tininess_afterRounding
    b.rounded := m.io.out
  }
  val s6 = stage(s5) { (a, b) =>
    val m = Module(new RecFNToIN(8, 24, 8))
    m.io.in := a.rounded
    m.io.roundingMode := consts.round_near_even
    m.io.signedOut := true.B
    val sign = rawFloatFromRecFN(8, 24, a.rounded).sign
    b.data := Mux(m.io.intExceptionFlags(1),
      Mux(sign, (-128).S(8.W), 127.S(8.W)), m.io.out.asSInt)
  }
  val s7 = Pipe(s6, 1)
  io.out.valid := s7.valid
  io.out.bits := s7.bits.data
}
