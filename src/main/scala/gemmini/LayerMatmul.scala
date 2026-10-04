package gemmini

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import GemminiISA._

/** Whole-layer NHWC 1x1 frontend. No new DMA, SRAM or arithmetic datapath.
  * A[M,K] times B[N,K]^T; full K per tile, repeating INT32 bias, INT8/INT32 C.
  * 30: M | K<<32|N; 31: tile rows | tile cols (elements, DIM multiples);
  * 32: A | B; 33: D | C; 34: strideA | strideC<<32|strideB (elements);
  * 35: LOOP_WS rs1 flags (accumulate/fullC/act only) | 0.
  * All five config words must be supplied for EACH launch. Status is copied
  * from launch, retaining Linux privilege/translation semantics. Completion
  * means downstream DMA has drained, NOT just that all tiles were submitted.
  */
class LayerMatmul(dim: Int, robEntries: Int, spRows: Int, accRows: Int)
    (implicit p: Parameters) extends Module {
  require(isPow2(dim))
  private val shift = log2Ceil(dim)
  val io = IO(new Bundle {
    val in = Flipped(Decoupled(new GemminiCmd(robEntries)))
    val out = Decoupled(new GemminiCmd(robEntries))
    val downstreamIdle = Input(Bool())
    val busy = Output(Bool())
  })
  val m = Reg(UInt(32.W)); val n = Reg(UInt(16.W)); val k = Reg(UInt(16.W))
  val ti = Reg(UInt(16.W)); val tj = Reg(UInt(16.W))
  val a = Reg(UInt(64.W)); val b = Reg(UInt(64.W))
  val d = Reg(UInt(64.W)); val c = Reg(UInt(64.W))
  val sa = Reg(UInt(32.W)); val sb = Reg(UInt(32.W)); val sc = Reg(UInt(32.W))
  val configured = RegInit(0.U(5.W))
  val saved = Reg(new GemminiCmd(robEntries))
  val row = RegInit(0.U(32.W)); val col = RegInit(0.U(16.W))
  val step = RegInit(0.U(3.W))
  val idle :: drainBefore :: emit :: drainAfter :: Nil = Enum(4)
  val state = RegInit(idle)
  val fn = io.in.bits.cmd.inst.funct
  val own = fn >= 30.U && fn <= 35.U
  // The preceding ExactGather queue may hold our launch while raw_cmd is
  // already empty. We consume it locally (out.valid is false), and state
  // does not become drainBefore until the next edge. Include the presented
  // command NOW, otherwise global RoCC busy can drop for one cycle and a
  // CPU fence can retire before this layer starts. Also cover backpressured
  // passthrough commands; do not include this signal in downstreamIdle.
  io.busy := state =/= idle || io.in.valid
  io.in.ready := state === idle && Mux(own, true.B, io.out.ready)
  io.out.valid := state === emit || (state === idle && io.in.valid && !own)
  io.out.bits := Mux(state === emit, saved, io.in.bits)

  def blocks(x: UInt): UInt = (x +& (dim - 1).U) >> shift
  val rows = Mux(m - row > ti, ti, m - row)
  val cols = Mux(n - col > tj, tj, n - col)
  val ib = blocks(rows); val jb = blocks(cols); val kb = blocks(k)
  val flags = saved.cmd.rs1
  val full = flags(1)
  // Retain the standard LoopMatmul <=2-B-tile reuse rule. The launch capacity
  // checks reserve worst-case A+B in either half, so automatic A ping-pong
  // cannot overwrite either fixed B slot. Reload on row zero of EACH layer.
  val reuseB = n <= (tj << 1)
  val bSlot = Mux(reuseB, Mux(col === 0.U, 1.U, 2.U), 0.U)
  when(state === emit) {
    io.out.bits.cmd.inst.xd := false.B
    io.out.bits.cmd.rs1 := 0.U
    io.out.bits.cmd.rs2 := 0.U
    switch(step) {
      is(0.U) {
        io.out.bits.cmd.inst.funct := LOOP_WS_CONFIG_BOUNDS
        io.out.bits.cmd.rs1 := ((kb * dim.U - k) << 32) |
          ((jb * dim.U - cols) << 16) | (ib * dim.U - rows)
        io.out.bits.cmd.rs2 := (kb << 32) | (jb << 16) | ib
      }
      is(1.U) {
        io.out.bits.cmd.inst.funct := LOOP_WS_CONFIG_ADDRS_AB
        io.out.bits.cmd.rs1 := a + row * sa
        io.out.bits.cmd.rs2 := Mux(reuseB && row =/= 0.U, 0.U, b + col * sb)
      }
      is(2.U) {
        io.out.bits.cmd.inst.funct := LOOP_WS_CONFIG_ADDRS_DC
        io.out.bits.cmd.rs1 := Mux(d === 0.U, 0.U, d + col * 4.U)
        io.out.bits.cmd.rs2 := c + (row * sc +& col) * Mux(full, 4.U, 1.U)
      }
      is(3.U) {
        io.out.bits.cmd.inst.funct := LOOP_WS_CONFIG_STRIDES_AB
        io.out.bits.cmd.rs1 := sa; io.out.bits.cmd.rs2 := sb
      }
      is(4.U) {
        io.out.bits.cmd.inst.funct := LOOP_WS_CONFIG_STRIDES_DC
        io.out.bits.cmd.rs1 := 0.U; io.out.bits.cmd.rs2 := sc
      }
      is(5.U) {
        io.out.bits.cmd.inst.funct := LOOP_WS
        io.out.bits.cmd.rs1 := flags | (bSlot << 16)
        io.out.bits.cmd.rs2 := 2.U // B transpose; no skipped stages
      }
    }
  }
  when(io.in.fire && own) {
    switch(fn) {
      is(30.U) { m := io.in.bits.cmd.rs1(31,0)
        n := io.in.bits.cmd.rs2(15,0); k := io.in.bits.cmd.rs2(47,32)
        configured := configured | 1.U }
      is(31.U) { ti := io.in.bits.cmd.rs1(15,0); tj := io.in.bits.cmd.rs2(15,0)
        configured := configured | 2.U }
      is(32.U) { a := io.in.bits.cmd.rs1; b := io.in.bits.cmd.rs2
        configured := configured | 4.U }
      is(33.U) { d := io.in.bits.cmd.rs1; c := io.in.bits.cmd.rs2
        configured := configured | 8.U }
      is(34.U) { sa := io.in.bits.cmd.rs1(31,0)
        sb := io.in.bits.cmd.rs2(31,0); sc := io.in.bits.cmd.rs2(63,32)
        configured := configured | 16.U }
      is(35.U) {
        assert(configured.andR, "LayerMatmul incomplete descriptor")
        assert(m =/= 0.U && n =/= 0.U && k =/= 0.U && ti =/= 0.U && tj =/= 0.U)
        assert(ti(shift-1,0) === 0.U && tj(shift-1,0) === 0.U)
        assert(sa >= k && sb >= k && sc >= n)
        assert(a =/= 0.U && b =/= 0.U && c =/= 0.U)
        assert((blocks(ti) +& blocks(tj)) * blocks(k) * dim.U <= (spRows/2).U,
          "LayerMatmul SPAD overflow")
        assert(blocks(ti) * blocks(tj) * dim.U <= (accRows/2).U,
          "LayerMatmul accumulator overflow")
        assert((io.in.bits.cmd.rs1 & ~"h703".U(64.W)) === 0.U &&
          io.in.bits.cmd.rs2 === 0.U, "LayerMatmul unsupported launch flags")
        assert(io.in.bits.cmd.rs1(0) === (d =/= 0.U), "LayerMatmul bias mismatch")
        saved := io.in.bits; configured := 0.U
        row := 0.U; col := 0.U; step := 0.U; state := drainBefore
      }
    }
  }
  when(state === drainBefore && io.downstreamIdle) { state := emit }
  when(state === emit && io.out.fire) {
    when(step =/= 5.U) { step := step + 1.U }.otherwise {
      step := 0.U
      when(col +& tj >= n) {
        col := 0.U
        when(row +& ti >= m) { state := drainAfter }
          .otherwise { row := row + ti }
      }.otherwise { col := col + tj }
    }
  }
  when(state === drainAfter && io.downstreamIdle) { state := idle }
}
