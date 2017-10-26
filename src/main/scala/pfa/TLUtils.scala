package freechips.rocketchip.pfa

import chisel3._
import chisel3.util._
import freechips.rocketchip.config.{Field, Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.rocket.PAddrBits
import freechips.rocketchip.tilelink._
import freechips.rocketchip.pfa._

// TLWriter writes 64bits to memory at specified addr
class TLWriter(name: String)(implicit p: Parameters) extends LazyModule {
  val node = TLClientNode(TLClientParameters(
    name = name, sourceId = IdRange(0, 1)))
  lazy val module = new TLWriterModule(this)
}

class TLWriterIO extends Bundle {
  val req = Decoupled(new Bundle {
    val addr = UInt(64.W) // memory address to write to
    val data = UInt(64.W)
  })
  val resp = Flipped(Decoupled(Bool()))
}

class TLWriterModule(outer: TLWriter) extends LazyModuleImp(outer) {
  val io = IO(new Bundle {
    val tl = outer.node.bundleOut
    val write = Flipped(new TLWriterIO)
  })

  val tl = io.tl(0)
  val edge = outer.node.edgesOut(0)
  val beatBytes = tl.params.dataBits / 8
  val byteAddrBits = log2Ceil(beatBytes)
  val addrBits = p(PAddrBits) - byteAddrBits

  val s_idle :: s_writing :: s_comp :: Nil = Enum(3)
  val state = RegInit(s_idle)
  val baseAddr = RegInit(0.U(addrBits.W))
  val data = RegInit(0.U(64.W))

  val xact_busy = RegInit(0.U(1.W))
  val xact_onehot = PriorityEncoderOH(~xact_busy)
  val can_send = !xact_busy.andR

  xact_busy := (xact_busy | Mux(tl.a.fire(), xact_onehot, 0.U)) &
                  ~Mux(tl.d.fire(), UIntToOH(tl.d.bits.source), 0.U)


  tl.a.valid := state === s_writing && !xact_busy.andR
  tl.a.bits := edge.Put(
    fromSource = OHToUInt(xact_onehot),
    toAddress = baseAddr << byteAddrBits.U,
    lgSize = byteAddrBits.U,
    data = data)._2
  tl.d.ready := xact_busy.orR

  io.write.req.ready := state === s_idle
  io.write.resp.valid := state === s_comp && !xact_busy.orR
  io.write.resp.bits := true.B

  when (io.write.req.fire()) {
    baseAddr := io.write.req.bits.addr >> byteAddrBits.U
    data := io.write.req.bits.data
    state := s_writing
  }

  when (tl.a.fire()) {
    state := s_comp
  }

  when (io.write.resp.fire()) {
    state := s_idle
  }
}

class TLReader(name: String)(implicit p: Parameters) extends LazyModule {
  val node = TLClientNode(TLClientParameters(
    name = name, sourceId = IdRange(0, 1)))
  lazy val module = new TLReaderModule(this)
}

class TLReaderIO extends Bundle {
  val req = Decoupled(new Bundle {
    val addr = UInt(64.W) // memory address to read from
  })
  val resp = Flipped(Decoupled(new Bundle {
    val data = UInt(64.W)
  }))
}

class TLReaderModule(outer: TLReader) extends LazyModuleImp(outer) {
  val io = IO(new Bundle {
    val tl = outer.node.bundleOut
    val read = Flipped(new TLReaderIO)
  })

  val tl = io.tl(0)
  val edge = outer.node.edgesOut(0)
  //val grantqueue = Queue(tl.d, 1)
  val beatBytes = tl.params.dataBits / 8
  val byteAddrBits = log2Ceil(beatBytes)
  val addrBits = p(PAddrBits) - byteAddrBits

  val inflight = RegInit(false.B)

  tl.a.valid := !inflight && io.read.req.valid
  io.read.req.ready := !inflight && tl.a.ready
  tl.a.bits := edge.Get(
    fromSource = 0.U,
    toAddress = io.read.req.bits.addr,
    lgSize = 1.U)._2

  io.read.resp.valid := tl.d.valid && inflight
  tl.d.ready := io.read.resp.ready && inflight
  io.read.resp.bits.data := tl.d.bits.data

  when (tl.a.fire()) {
    inflight := true.B
  }

  when (io.read.resp.fire()) {
    inflight := false.B
  }
}
