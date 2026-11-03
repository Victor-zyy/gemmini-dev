package gemmini

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.{Field, Parameters}
import freechips.rocketchip.diplomacy.{LazyModule, LazyModuleImp}
import freechips.rocketchip.tilelink._

/** Simulation diagnostics only; default designs elaborate neither option. */
case class OrderingDiagnosticParams(trace: Boolean = false, readDelayCycles: Int = 0)
case object GemminiOrderingDiagnosticKey
  extends Field[OrderingDiagnosticParams](OrderingDiagnosticParams())

class OrderingReadBeat(width: Int) extends Bundle {
  val payload = UInt(width.W)
  val hold = Bool()
  val last = Bool()
}

/** Buffer one selected complete D packet, letting other sources pass.
  * Preserve beat order; never interleave packets. Delay starts at capture.
  * This is a diagnostic, not a production memory-system implementation.
  */
class OrderingReadSkew(width: Int, maxBeats: Int, cycles: Int) extends Module {
  require(maxBeats > 0 && cycles > 0)
  val io = IO(new Bundle {
    val in = Flipped(Decoupled(new OrderingReadBeat(width)))
    val out = Decoupled(new OrderingReadBeat(width))
    val captured = Output(Bool())
    val released = Output(Bool())
    val busy = Output(Bool())
  })
  val indexBits = log2Ceil(maxBeats + 1) max 1
  val data = Reg(Vec(maxBeats, new OrderingReadBeat(width)))
  val written = RegInit(0.U(indexBits.W))
  val read = RegInit(0.U(indexBits.W))
  val full = RegInit(false.B)
  val releaseActive = RegInit(false.B)
  val passActive = RegInit(false.B)
  val timer = RegInit(0.U((log2Ceil(cycles + 1) max 1).W))
  val release = releaseActive || (full && timer === 0.U && !passActive)
  val capture = io.in.bits.hold && !release
  io.in.ready := !release && Mux(capture, !full && written < maxBeats.U, io.out.ready)
  io.out.valid := Mux(release, full, io.in.valid && !capture)
  io.out.bits := Mux(release, data(read), io.in.bits)
  io.captured := io.in.fire && capture && io.in.bits.last
  io.released := release && io.out.fire && io.out.bits.last
  io.busy := written =/= 0.U || full || releaseActive
  when (full && timer =/= 0.U) { timer := timer - 1.U }
  when (io.in.fire && capture) {
    assert(written < maxBeats.U && !full, "ordering diagnostic D packet overflow")
    data(written) := io.in.bits
    written := written + 1.U
    when (io.in.bits.last) {
      full := true.B
      timer := cycles.U
    }
  }
  // Lock even a stalled single-beat offer; never withdraw an offered packet.
  when (!release && io.in.valid && !capture) {
    passActive := !(io.out.fire && io.in.bits.last)
  }
  when (release) {
    releaseActive := true.B
    when (io.out.fire) {
      read := read + 1.U
      when (io.out.bits.last) {
        assert(read + 1.U === written, "ordering diagnostic packet length mismatch")
        full := false.B
        releaseActive := false.B
        written := 0.U
        read := 0.U
      }
    }
  }
}

/** Hold the first Get response; later sources may overtake it. */
class OrderingTLReadSkew(maxBytes: Int, cycles: Int)(implicit p: Parameters)
    extends LazyModule {
  val node = TLAdapterNode()
  lazy val module = new LazyModuleImp(this) {
    (node.in zip node.out).foreach { case ((in, _), (out, edge)) =>
      require(edge.client.clients.forall(!_.requestFifo),
        "Reordering must not violate a client's requested FIFO domain")
      out.a <> in.a
      in.b <> out.b
      out.c <> in.c
      out.e <> in.e
      val selected = RegInit(false.B)
      val active = RegInit(false.B)
      val source = Reg(UInt(out.d.bits.source.getWidth.W))
      val skew = Module(new OrderingReadSkew(out.d.bits.getWidth,
        maxBytes / (edge.bundle.dataBits / 8), cycles))
      skew.io.in.valid := out.d.valid
      out.d.ready := skew.io.in.ready
      skew.io.in.bits.payload := out.d.bits.asUInt
      skew.io.in.bits.hold := active && out.d.bits.source === source
      skew.io.in.bits.last := edge.last(out.d)
      in.d.valid := skew.io.out.valid
      skew.io.out.ready := in.d.ready
      in.d.bits := skew.io.out.bits.payload.asTypeOf(in.d.bits)
      when (in.a.fire && !selected) {
        assert(in.a.bits.opcode === TLMessages.Get, "ordering diagnostic expects read-only DMA")
        selected := true.B
        active := true.B
        source := in.a.bits.source
        printf(p"[ORDER-SKEW] select source=${in.a.bits.source} addr=0x${Hexadecimal(in.a.bits.address)} delay=${cycles.U}\n")
      }
      when (out.d.fire) {
        assert(out.d.bits.size <= log2Ceil(maxBytes).U,
          "ordering diagnostic received oversized packet")
      }
      when (skew.io.captured) {
        printf(p"[ORDER-SKEW] captured source=${source}\n")
      }
      when (skew.io.released) {
        active := false.B
        printf(p"[ORDER-SKEW] released source=${source}\n")
      }
    }
  }
}
