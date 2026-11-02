package gemmini

import chisel3._
import chisel3.util._
import hardfloat._
import gemmini.Arithmetic.SIntArithmetic

/** Registered intermediate values for the default FP32 accumulator scale.
  * Dead fields between stages are removed by synthesis.
  */
class AccScaleFP32Lane extends Bundle {
  val integer = SInt(32.W)
  val recoded = UInt(33.W)
  val a = UInt(24.W)
  val b = UInt(24.W)
  val c = UInt(48.W)
  val inter = new MulAddRecFN_interIo(8, 24)
  val product = UInt(49.W)
  val raw = new RawFloat(8, 26)
  val invalid = Bool()
  val rounded = UInt(33.W)
  val scaled = SInt(32.W)
}
class AccScaleFP32Token(rows: Int, cols: Int) extends Bundle {
  val tagged = new AccumulatorScaleTaggedResp(Vec(rows, Vec(cols, SInt(32.W))),
    Vec(rows, Vec(cols, SInt(8.W))))
  val scale = UInt(33.W)
  val lanes = Vec(rows * cols, new AccScaleFP32Lane)
}

/** Eight genuinely separated arithmetic stages, with the same round-near-even
  * operations as GemminiConfigs.defaultConfig.acc_scale_args.
  * Pipeline provides per-stage backpressure, no dropped/reordered transactions.
  * Full-width data, activation and DMA/bank tags travel with their INT8 result.
  * No SiLU/ResAdd changes: those remain in AccumulatorPostProcess.
  */
class AccumulatorScaleFP32Pipeline(rows: Int, cols: Int, nonlinear: Boolean) extends Module {
  val full = Vec(rows, Vec(cols, SInt(32.W)))
  val small = Vec(rows, Vec(cols, SInt(8.W)))
  val io = IO(new Bundle {
    val in = Flipped(Decoupled(new NormalizedOutput(full, Float(8, 24))))
    val out = Decoupled(new AccumulatorScaleTaggedResp(full, small))
  })
  val input = Wire(Decoupled(new AccScaleFP32Token(rows, cols)))
  input.valid := io.in.valid
  io.in.ready := input.ready
  input.bits := 0.U.asTypeOf(input.bits)
  val resp = io.in.bits.acc_read_resp
  input.bits.tagged.resp.full_data := resp.data
  input.bits.tagged.resp.fromDMA := resp.fromDMA
  input.bits.tagged.resp.acc_bank_id := resp.acc_bank_id
  input.bits.tagged.act := resp.act
  input.bits.scale := recFNFromFN(8, 24, resp.scale.bits)
  for (r <- 0 until rows; c <- 0 until cols) {
    val e = resp.data(r)(c)
    input.bits.lanes(r * cols + c).integer :=
      Mux(nonlinear.B && resp.act === Activation.RELU && e < 0.S, 0.S, e)
  }

  type Token = AccScaleFP32Token
  def transform(f: (AccScaleFP32Lane, AccScaleFP32Lane, Token) => Unit): Token => Token =
    (x: Token) => {
      val y = WireInit(x)
      (x.lanes zip y.lanes).foreach { case (a, b) => f(a, b, x) }
      y
    }

  // Stage 0 captures queue output and control before the arithmetic fanout.
  val capture: Token => Token = (x: Token) => x
  val convert = transform { (a, b, _) =>
    val m = Module(new INToRecFN(32, 8, 24))
    m.io.signedIn := true.B
    m.io.in := a.integer.asUInt
    m.io.roundingMode := consts.round_near_even
    m.io.detectTininess := consts.tininess_afterRounding
    b.recoded := m.io.out
  }
  val preMultiply = transform { (a, b, x) =>
    val m = Module(new MulAddRecFNToRaw_preMul(8, 24))
    m.io.op := 0.U
    m.io.a := a.recoded
    m.io.b := x.scale
    m.io.c := 0.U
    b.a := m.io.mulAddA
    b.b := m.io.mulAddB
    b.c := m.io.mulAddC
    b.inter := m.io.toPostMul
  }
  val multiply = transform { (a, b, _) =>
    b.product := (a.a * a.b) +& a.c
  }
  val postMultiply = transform { (a, b, _) =>
    val m = Module(new MulAddRecFNToRaw_postMul(8, 24))
    m.io.fromPreMul := a.inter
    m.io.mulAddResult := a.product
    m.io.roundingMode := consts.round_near_even
    b.raw := m.io.rawOut
    b.invalid := m.io.invalidExc
  }
  val round = transform { (a, b, _) =>
    val m = Module(new RoundRawFNToRecFN(8, 24, 0))
    m.io.in := a.raw
    m.io.invalidExc := a.invalid
    m.io.infiniteExc := false.B
    m.io.roundingMode := consts.round_near_even
    m.io.detectTininess := consts.tininess_afterRounding
    b.rounded := m.io.out
  }
  val toInteger = transform { (a, b, _) =>
    val m = Module(new RecFNToIN(8, 24, 32))
    m.io.in := a.rounded
    m.io.roundingMode := consts.round_near_even
    m.io.signedOut := true.B
    val sign = rawFloatFromRecFN(8, 24, a.rounded).sign
    val saturated = Mux(sign, (-BigInt(1) << 31).S(32.W),
      ((BigInt(1) << 31) - 1).S(32.W))
    b.scaled := Mux(m.io.intExceptionFlags(1), saturated, m.io.out.asSInt)
  }
  val clip: Token => Token = (x: Token) => {
    val y = WireInit(x)
    for (r <- 0 until rows; c <- 0 until cols) {
      val v = x.lanes(r * cols + c).scaled
      y.tagged.resp.data(r)(c) := Mux(v > 127.S, 127.S,
        Mux(v < (-128).S, (-128).S, v)).asTypeOf(SInt(8.W))
    }
    y
  }
  val result = Pipeline(input, 8, Seq(capture, convert, preMultiply, multiply,
    postMultiply, round, toInteger, clip, capture))
  io.out.valid := result.valid
  result.ready := io.out.ready
  io.out.bits := result.bits.tagged
}
