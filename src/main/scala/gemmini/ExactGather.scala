package gemmini

import chisel3._
import chisel3.util._
import freechips.rocketchip.rocket.MStatus
import org.chipsalliance.cde.config.Parameters
import GemminiISA._

/** One contiguous row fragment handled by the native Exact Gather DMA path. */
class ExactGatherDMARequest(coreMaxAddrBits: Int, scaleBits: Int)
    extends Bundle {
  val src = UInt(coreMaxAddrBits.W)
  val dst = UInt(coreMaxAddrBits.W)
  val cols = UInt(16.W)
  val scale = UInt(scaleBits.W)
  val status = new MStatus
}

/**
  * Controller-to-scratchpad interface for native Exact Gather.
  *
  * The scratchpad accepts row fragments into a bounded multi-transaction
  * pipeline and pulses completed only after every write DMA has retired.
  */
class ExactGatherDMAIO(coreMaxAddrBits: Int, scaleBits: Int) extends Bundle {
  val req = Decoupled(new ExactGatherDMARequest(coreMaxAddrBits, scaleBits))
  val completed = Input(Bool())
  val busy = Input(Bool())
}

class ExactGatherBranch(coreMaxAddrBits: Int) extends Bundle {
  val src = UInt(coreMaxAddrBits.W)
  val srcStride = UInt(16.W)
  val channels = UInt(16.W)
  val dstOffset = UInt(16.W)
  val scale = UInt(32.W)
  val geometryValid = Bool()
  val scaleValid = Bool()
}

/**
  * Native exact INT8 Gather-Requant controller.
  *
  * Funct 26--28 retain the existing four-branch configuration protocol.
  * Funct 29 walks each branch, row, and block-sized channel fragment and sends
  * it directly to the scratchpad's dedicated read/scale/write engine. Unlike
  * the former implementation, this module never stages values in accumulator
  * row zero and never expands one operation into CONFIG/MVIN/MVOUT commands.
  */
class ExactGather(
    blockSize: Int,
    coreMaxAddrBits: Int,
    robEntries: Int,
    scaleBits: Int)(implicit p: Parameters) extends Module {

  require(blockSize >= 2)
  require(scaleBits == 32,
    "Exact Gather requires the 32-bit fixed-point Exact ResAdd scale payload")

  val io = IO(new Bundle {
    val in = Flipped(Decoupled(new GemminiCmd(robEntries)))
    val out = Decoupled(new GemminiCmd(robEntries))
    val dma = new ExactGatherDMAIO(coreMaxAddrBits, scaleBits)
    val busy = Output(Bool())
  })

  val inputQ = Queue(io.in)

  val dst = Reg(UInt(coreMaxAddrBits.W))
  val rows = Reg(UInt(16.W))
  val cols = Reg(UInt(16.W))
  val dstStride = Reg(UInt(16.W))
  val branchCount = Reg(UInt(3.W))
  val dstSpad = Reg(Bool())
  val branches = RegInit(VecInit(Seq.fill(4)(0.U.asTypeOf(
    new ExactGatherBranch(coreMaxAddrBits)))))
  val savedStatus = Reg(chiselTypeOf(inputQ.bits.cmd.status))

  object State extends ChiselEnum {
    val idle, issueDMA, waitDMA = Value
  }
  import State._
  val state = RegInit(idle)
  val branchId = RegInit(0.U(2.W))
  val row = RegInit(0.U(16.W))
  val channel = RegInit(0.U(16.W))

  val funct = inputQ.bits.cmd.inst.funct
  val isConfigBounds = funct === EXACT_GATHER_CONFIG_BOUNDS
  val isConfigBranch = funct === EXACT_GATHER_CONFIG_BRANCH
  val isConfigScale = funct === EXACT_GATHER_CONFIG_SCALE
  val isExecute = funct === EXACT_GATHER
  val isGatherCommand = isConfigBounds || isConfigBranch ||
    isConfigScale || isExecute

  val currentBranch = branches(branchId)
  val remainingChannels = currentBranch.channels - channel
  val fragmentCols = Mux(remainingChannels > blockSize.U,
    blockSize.U, remainingChannels)
  val srcAddress = currentBranch.src + row * currentBranch.srcStride + channel
  val dstAddress = dst + row * dstStride + currentBranch.dstOffset + channel

  // Non-gather commands pass through unchanged. Gather configuration and
  // execution commands are consumed locally and never enter the ROB.
  io.out.bits := inputQ.bits
  io.out.valid := state === idle && inputQ.valid && !isGatherCommand
  inputQ.ready := state === idle && Mux(isGatherCommand, true.B, io.out.ready)

  io.dma.req.valid := state === issueDMA
  io.dma.req.bits.src := srcAddress
  io.dma.req.bits.dst := dstAddress
  io.dma.req.bits.cols := fragmentCols
  io.dma.req.bits.scale := currentBranch.scale
  io.dma.req.bits.status := savedStatus

  io.busy := state =/= idle || io.dma.busy ||
    (inputQ.valid && isGatherCommand)

  when (state === idle && inputQ.fire) {
    when (isConfigBounds) {
      dst := inputQ.bits.cmd.rs1
      rows := inputQ.bits.cmd.rs2(15, 0)
      cols := inputQ.bits.cmd.rs2(31, 16)
      dstStride := inputQ.bits.cmd.rs2(47, 32)
      branchCount := inputQ.bits.cmd.rs2(50, 48)
      dstSpad := inputQ.bits.cmd.rs2(51)
      branches.foreach { branch =>
        branch.geometryValid := false.B
        branch.scaleValid := false.B
      }
    }.elsewhen (isConfigBranch) {
      val id = inputQ.bits.cmd.rs2(49, 48)
      branches(id).src := inputQ.bits.cmd.rs1
      branches(id).srcStride := inputQ.bits.cmd.rs2(15, 0)
      branches(id).channels := inputQ.bits.cmd.rs2(31, 16)
      branches(id).dstOffset := inputQ.bits.cmd.rs2(47, 32)
      branches(id).geometryValid := true.B
    }.elsewhen (isConfigScale) {
      val id = inputQ.bits.cmd.rs2(1, 0)
      branches(id).scale := inputQ.bits.cmd.rs1(31, 0)
      branches(id).scaleValid := true.B
    }.elsewhen (isExecute) {
      assert(rows =/= 0.U && cols =/= 0.U,
        "Exact Gather rows and columns must be non-zero")
      assert(branchCount =/= 0.U && branchCount <= 4.U,
        "Exact Gather requires one to four branches")
      assert(dstStride >= cols,
        "Exact Gather DRAM stride must cover every output column")
      assert(!dstSpad,
        "Native Exact Gather supports the YOLO DRAM destination only")
      for (id <- 0 until 4) {
        when (id.U < branchCount) {
          assert(branches(id).geometryValid && branches(id).scaleValid,
            "Exact Gather branch configuration is incomplete")
          assert(branches(id).channels =/= 0.U &&
            branches(id).srcStride >= branches(id).channels,
            "Exact Gather branch geometry is invalid")
          assert(branches(id).dstOffset +& branches(id).channels <= cols,
            "Exact Gather branch exceeds the destination columns")
        }
      }
      savedStatus := inputQ.bits.cmd.status
      branchId := 0.U
      row := 0.U
      channel := 0.U
      state := issueDMA
    }
  }

  when (state === issueDMA && io.dma.req.fire) {
    when (channel + fragmentCols < currentBranch.channels) {
      channel := channel + fragmentCols
    }.otherwise {
      channel := 0.U
      when (row + 1.U < rows) {
        row := row + 1.U
      }.otherwise {
        row := 0.U
        // branchId is two bits because valid IDs are 0..3. Use widening
        // addition here; ordinary `+` would wrap 3 + 1 back to zero and make
        // a four-branch Gather loop forever.
        when (branchId +& 1.U < branchCount) {
          branchId := branchId + 1.U
        }.otherwise {
          // Every fragment has been accepted by the native DMA. Keep the
          // architectural command busy until all outstanding writes retire.
          state := waitDMA
        }
      }
    }
  }

  when (state === waitDMA && io.dma.completed) {
    state := idle
  }
}

object ExactGather {
  def apply(in: DecoupledIO[GemminiCmd], blockSize: Int,
      coreMaxAddrBits: Int, robEntries: Int, scaleBits: Int)
      (implicit p: Parameters): ExactGather = {
    val mod = Module(new ExactGather(blockSize, coreMaxAddrBits, robEntries,
      scaleBits))
    mod.io.in <> in
    mod
  }
}
