// See README.md for license details.
package gemmini

import chisel3._
import chisel3.util._

/**
  * Combinational signed MAC carrying a Xilinx USE_DSP synthesis attribute.
  * There are deliberately no registers here, so enabling this implementation
  * does not change Gemmini's PE latency or software-visible behavior.
  */
class XilinxDspMac(aWidth: Int, bWidth: Int, cWidth: Int, dWidth: Int)
    extends BlackBox with HasBlackBoxInline {
  require(dWidth >= aWidth + bWidth,
    "XilinxDspMac output must contain the full multiplication result")
  require(cWidth >= dWidth,
    "XilinxDspMac accumulator input must be at least as wide as its output")

  val io = IO(new Bundle {
    val in_a = Input(SInt(aWidth.W))
    val in_b = Input(SInt(bWidth.W))
    val in_c = Input(SInt(cWidth.W))
    val out_d = Output(SInt(dWidth.W))
  })

  private val productWidth = aWidth + bWidth
  private val extensionWidth = dWidth - productWidth

  override def desiredName: String =
    s"XilinxDspMac_a${aWidth}_b${bWidth}_c${cWidth}_d${dWidth}"

  setInline(s"$desiredName.sv",
    s"""
       |(* use_dsp = "yes" *)
       |module $desiredName (
       |  input  wire signed [${aWidth - 1}:0] in_a,
       |  input  wire signed [${bWidth - 1}:0] in_b,
       |  input  wire signed [${cWidth - 1}:0] in_c,
       |  output wire signed [${dWidth - 1}:0] out_d
       |);
       |  wire signed [${productWidth - 1}:0] product = in_a * in_b;
       |  (* use_dsp = "yes" *)
       |  wire signed [${dWidth - 1}:0] mac_result =
       |    $$signed({{${extensionWidth}{product[${productWidth - 1}]}}, product}) +
       |    $$signed(in_c[${dWidth - 1}:0]);
       |
       |  assign out_d = mac_result;
       |endmodule
       |""".stripMargin)
}

class PEControl[T <: Data : Arithmetic](accType: T) extends Bundle {
  val dataflow = UInt(1.W) // TODO make this an Enum
  val propagate = UInt(1.W) // Which register should be propagated (and which should be accumulated)?
  val shift = UInt(log2Up(accType.getWidth).W) // TODO this isn't correct for Floats

}

class MacUnit[T <: Data](inputType: T, weightType: T, cType: T, dType: T,
                        use_dsp_for_mac: Boolean = false)
                       (implicit ev: Arithmetic[T]) extends Module {
  import ev._
  val io = IO(new Bundle {
    val in_a  = Input(inputType)
    val in_b  = Input(weightType)
    val in_c  = Input(cType)
    val out_d = Output(dType)
  })

  if (use_dsp_for_mac) {
    require(inputType.isInstanceOf[SInt] && weightType.isInstanceOf[SInt] &&
      cType.isInstanceOf[SInt] && dType.isInstanceOf[SInt],
      "Xilinx DSP MAC mapping currently supports signed-integer Gemmini configurations only")

    val dspMac = Module(new XilinxDspMac(
      inputType.getWidth, weightType.getWidth, cType.getWidth, dType.getWidth))
    dspMac.io.in_a := io.in_a.asUInt.asSInt
    dspMac.io.in_b := io.in_b.asUInt.asSInt
    dspMac.io.in_c := io.in_c.asUInt.asSInt
    io.out_d := dspMac.io.out_d.asTypeOf(dType)
  } else {
    io.out_d := io.in_c.mac(io.in_a, io.in_b)
  }
}

// TODO update documentation
/**
  * A PE implementing a MAC operation. Configured as fully combinational when integrated into a Mesh.
  * @param width Data width of operands
  */
class PE[T <: Data](inputType: T, weightType: T, outputType: T, accType: T,
                   df: Dataflow.Value, max_simultaneous_matmuls: Int,
                   use_dsp_for_mac: Boolean = false)
                   (implicit ev: Arithmetic[T]) extends Module { // Debugging variables
  import ev._

  val io = IO(new Bundle {
    val in_a = Input(inputType)
    val in_b = Input(outputType)
    val in_d = Input(outputType)
    val out_a = Output(inputType)
    val out_b = Output(outputType)
    val out_c = Output(outputType)

    val in_control = Input(new PEControl(accType))
    val out_control = Output(new PEControl(accType))

    val in_id = Input(UInt(log2Up(max_simultaneous_matmuls).W))
    val out_id = Output(UInt(log2Up(max_simultaneous_matmuls).W))

    val in_last = Input(Bool())
    val out_last = Output(Bool())

    val in_valid = Input(Bool())
    val out_valid = Output(Bool())

    val bad_dataflow = Output(Bool())
  })

  val cType = if (df == Dataflow.WS) inputType else accType

  // When creating PEs that support multiple dataflows, the
  // elaboration/synthesis tools often fail to consolidate and de-duplicate
  // MAC units. To force mac circuitry to be re-used, we create a "mac_unit"
  // module here which just performs a single MAC operation
  val mac_unit = Module(new MacUnit(inputType, weightType,
    if (df == Dataflow.WS) outputType else accType, outputType,
    use_dsp_for_mac))

  val a  = io.in_a
  val b  = io.in_b
  val d  = io.in_d
  val c1 = Reg(cType)
  val c2 = Reg(cType)
  val dataflow = io.in_control.dataflow
  val prop  = io.in_control.propagate
  val shift = io.in_control.shift
  val id = io.in_id
  val last = io.in_last
  val valid = io.in_valid

  io.out_a := a
  io.out_control.dataflow := dataflow
  io.out_control.propagate := prop
  io.out_control.shift := shift
  io.out_id := id
  io.out_last := last
  io.out_valid := valid

  mac_unit.io.in_a := a

  val last_s = RegEnable(prop, valid)
  val flip = last_s =/= prop
  val shift_offset = Mux(flip, shift, 0.U)

  // Which dataflow are we using?
  val OUTPUT_STATIONARY = Dataflow.OS.id.U(1.W)
  val WEIGHT_STATIONARY = Dataflow.WS.id.U(1.W)

  // Is c1 being computed on, or propagated forward (in the output-stationary dataflow)?
  val COMPUTE = 0.U(1.W)
  val PROPAGATE = 1.U(1.W)

  io.bad_dataflow := false.B
  when ((df == Dataflow.OS).B || ((df == Dataflow.BOTH).B && dataflow === OUTPUT_STATIONARY)) {
    when(prop === PROPAGATE) {
      io.out_c := (c1 >> shift_offset).clippedToWidthOf(outputType)
      io.out_b := b
      mac_unit.io.in_b := b.asTypeOf(weightType)
      mac_unit.io.in_c := c2
      c2 := mac_unit.io.out_d
      c1 := d.withWidthOf(cType)
    }.otherwise {
      io.out_c := (c2 >> shift_offset).clippedToWidthOf(outputType)
      io.out_b := b
      mac_unit.io.in_b := b.asTypeOf(weightType)
      mac_unit.io.in_c := c1
      c1 := mac_unit.io.out_d
      c2 := d.withWidthOf(cType)
    }
  }.elsewhen ((df == Dataflow.WS).B || ((df == Dataflow.BOTH).B && dataflow === WEIGHT_STATIONARY)) {
    when(prop === PROPAGATE) {
      io.out_c := c1
      mac_unit.io.in_b := c2.asTypeOf(weightType)
      mac_unit.io.in_c := b
      io.out_b := mac_unit.io.out_d
      c1 := d
    }.otherwise {
      io.out_c := c2
      mac_unit.io.in_b := c1.asTypeOf(weightType)
      mac_unit.io.in_c := b
      io.out_b := mac_unit.io.out_d
      c2 := d
    }
  }.otherwise {
    io.bad_dataflow := true.B
    //assert(false.B, "unknown dataflow")
    io.out_c := DontCare
    io.out_b := DontCare
    mac_unit.io.in_b := b.asTypeOf(weightType)
    mac_unit.io.in_c := c2
  }

  when (!valid) {
    c1 := c1
    c2 := c2
    mac_unit.io.in_b := DontCare
    mac_unit.io.in_c := DontCare
  }
}
