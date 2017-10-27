package freechips.rocketchip.pfa

import chisel3._
import chisel3.util._
import freechips.rocketchip.coreplex.HasSystemBus
import freechips.rocketchip.config.{Field, Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.regmapper.{HasRegMap, RegField}
import freechips.rocketchip.rocket.PAddrBits
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util.TwoWayCounter
import freechips.rocketchip.pfa._
import scala.util.Random

class EvictIO extends Bundle {
  // requests are pfns and they are handled in PFAEvictPath
  val req = Decoupled(UInt(64.W))
  // high when a req has completed
  val resp = Flipped(Decoupled(Bool()))
}

class PFAIO extends Bundle {
  // the ptw drives the requests
  val req = Decoupled(new Bundle {
    val pageid = UInt(28.W)
    val protbits = UInt(10.W)
  })
  val resp = Flipped(Decoupled(UInt(64.W))) // pfa's replies
}

case class PFAControllerParams(addr: BigInt, beatBytes: Int)

class PFAFetchPath(implicit p: Parameters) extends LazyModule {
  lazy val module = new PFAFetchPathModule(this)
}

class PFAFetchPathModule(outer: PFAFetchPath) extends LazyModuleImp(outer) {
  val io = IO(new Bundle {
    val remoteFault = Flipped(new PFAIO)
  })

  val s_idle :: s_frame1 :: s_frame2 :: s_frame3 :: s_pte :: s_comp :: Nil = Enum(6)

  val s = RegInit(s_idle)

  io.remoteFault.req.ready := s === s_idle

  when (io.remoteFault.req.fire()) {
    assert(false.B === true.B)
  }
}

class PFAEvictPath(nicaddr: BigInt)(implicit p: Parameters) extends LazyModule {
  lazy val module = new PFAEvictPathModule(this, nicaddr)
}

class PFAEvictPathModule(outer: PFAEvictPath, nicaddr: BigInt) extends LazyModuleImp(outer) {
  val io = IO(new Bundle {
    val evict = Flipped(new EvictIO)
    val sendpacket = new SendPacketIO
  })

  val s_idle :: s_frame1 :: s_frame2 :: s_frame3 :: s_comp :: Nil = Enum(5)
  val send = io.sendpacket
  val packetReq = io.sendpacket.req.bits
  val s = RegInit(s_idle)
  val frameaddr = RegInit(0.U(64.W))
  val sendCompFired = RegNext(send.resp.fire(), false.B)
  val evictFired = RegNext(io.evict.req.fire(), false.B)
  val pageid = RegInit(0.U(32.W))

  io.evict.req.ready := s === s_idle
  io.evict.resp.valid := s === s_comp
  io.evict.resp.bits := true.B

  send.req.valid := MuxCase(0.U, Array(
            (s === s_frame1) -> evictFired,
            (s === s_frame2 || s === s_frame3) -> sendCompFired))
  packetReq.payload.addr := MuxCase(0.U, Array(
            (s === s_frame1) -> (frameaddr),
            (s === s_frame2) -> (frameaddr + 1368.U),
            (s === s_frame3) -> (frameaddr + 2736.U)))
  packetReq.payload.len := MuxCase(0.U, Array(
            (s === s_frame1) -> 1368.U,
            (s === s_frame2) -> 1368.U,
            (s === s_frame3) -> 1360.U))
  packetReq.header.partid := MuxCase(0.U, Array(
            (s === s_frame1) -> 0.U,
            (s === s_frame2) -> 1.U,
            (s === s_frame3) -> 2.U))
  packetReq.header.pageid := pageid
  send.resp.ready := s === s_frame1 || s === s_frame2 || s === s_frame3

  when (io.evict.req.fire()) {
    pageid := io.evict.req.bits(63,36)
    frameaddr := io.evict.req.bits(35,0) << 12
    s := s_frame1
  }

  when (send.resp.fire()) {
    switch (s) {
      is (s_frame1) { s := s_frame2 }
      is (s_frame2) { s := s_frame3 }
      is (s_frame3) { s := s_comp }
    }
  }

  when (io.evict.resp.fire()) {
    s := s_idle
  }
}

trait PFAControllerBundle extends Bundle {
  val evict = new EvictIO
  val free = Decoupled(UInt(64.W))
  val workbuf = Decoupled(UInt(64.W))
}

trait PFAControllerModule extends Module with HasRegMap {
  val io: PFAControllerBundle
  val qDepth = 10

  val evictQueue = Module(new Queue(UInt(64.W), qDepth))
  val evictsInProg = TwoWayCounter(io.evict.req.fire(), io.evict.resp.fire(), qDepth)
  val evictStat = RegInit(0.U(64.W))
  io.evict.req <> evictQueue.io.deq
  io.evict.resp.ready := true.B // always ready to evict?
  evictStat := Mux(evictsInProg > evictQueue.io.count, evictsInProg, evictQueue.io.count)

  val workbufQueue = Module(new Queue(UInt(64.W), 1))
  io.workbuf <> workbufQueue.io.deq

  val freeQueue = Module(new Queue(UInt(64.W), qDepth))
  io.free <> freeQueue.io.deq

  regmap(
    0x0 -> Seq(RegField.w(64, freeQueue.io.enq)),
    0x8 -> Seq(RegField.r(64, qDepth.U - freeQueue.io.count)),
    0x10 -> Seq(RegField.w(64, evictQueue.io.enq)),
    0x18 -> Seq(RegField.r(64, qDepth.U - evictStat)),
    0x38 -> Seq(RegField.w(64, workbufQueue.io.enq)))
}

class PFAController(c: PFAControllerParams)(implicit p: Parameters)
  extends TLRegisterRouter(c.addr, "pfa",  Seq("ucbbar,pfa"),
                           beatBytes = c.beatBytes)(
                            new TLRegBundle(c, _) with PFAControllerBundle)(
                            new TLRegModule(c, _, _) with PFAControllerModule)

class PFA(addr: BigInt, nicaddr: BigInt, beatBytes: Int = 8)(implicit p: Parameters)
    extends LazyModule {
  val control = LazyModule(new PFAController(
      PFAControllerParams(addr, beatBytes)))
  val fetchPath = LazyModule(new PFAFetchPath)
  val evictPath = LazyModule(new PFAEvictPath(nicaddr))
  val sendframePkt1 = LazyModule(new SendPacket(nicaddr, "pfa-sendframe1"))
  val sendframePkt2 = LazyModule(new SendPacket(nicaddr, "pfa-sendframe2")) // TODO: use arb instead

  val mmionode = TLInputNode()
  val dmanode = TLOutputNode()

  control.node := mmionode;
  dmanode := sendframePkt1.writenode
  dmanode := sendframePkt1.readnode
  dmanode := sendframePkt2.writenode
  dmanode := sendframePkt2.readnode

  lazy val module = new LazyModuleImp(this) {
    val io = IO(new Bundle {
      val tlout = dmanode.bundleOut
      val tlin = mmionode.bundleIn
      val remoteFault = Flipped(new PFAIO)
    })

    io.remoteFault <> fetchPath.module.io.remoteFault
    evictPath.module.io.evict <> control.module.io.evict
    sendframePkt1.module.io.workbuf <> control.module.io.workbuf
    sendframePkt2.module.io.workbuf <> control.module.io.workbuf
    evictPath.module.io.sendpacket <> sendframePkt1.module.io.sendpacket
  }
}

trait HasPeripheryPFA extends HasSystemBus {
  private val nicaddr = BigInt(0x10016000)
  private val pfaaddr = BigInt(0x10017000)

  val pfa = LazyModule(new PFA(pfaaddr, nicaddr, sbus.beatBytes))
  pfa.mmionode := sbus.toVariableWidthSlaves
  sbus.fromSyncPorts() :=* pfa.dmanode
}

trait HasPeripheryPFAModuleImp extends LazyMultiIOModuleImp {
  val outer: HasPeripheryPFA
}
