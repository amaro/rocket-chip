package freechips.rocketchip.pfa

import chisel3._
import chisel3.util._
import freechips.rocketchip.coreplex.HasSystemBus
import freechips.rocketchip.config.{Field, Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.regmapper.{HasRegMap, RegField}
import freechips.rocketchip.tilelink._
import freechips.rocketchip.util.TwoWayCounter
import freechips.rocketchip.pfa._

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
    val faultvpn = UInt(27.W)
    val pteppn = UInt(54.W)
  })
  val resp = Flipped(Decoupled(UInt(64.W))) // pfa's replies TODO: whats this for?
}

case class PFAControllerParams(addr: BigInt, beatBytes: Int)

class PFAFetchPath(implicit p: Parameters) extends LazyModule {
  val tlwriter = LazyModule(new TLWriter("pfa-fetch-modpte"))
  val writenode = TLIdentityNode()
  writenode := tlwriter.node
  lazy val module = new PFAFetchPathModule(this)
}

class PFAFetchPathModule(outer: PFAFetchPath) extends LazyModuleImp(outer) {
  val io = IO(new Bundle {
    val fetch = Flipped(new PFAIO)
    val free = Flipped(Decoupled(UInt(64.W)))
    val sendpacket = new SendPacketIO
    val recvpacket = new RecvPacketIO
  })

  val write = outer.tlwriter.module.io.write
  // s_sendreq: sends a request packet with required pageid
  // s_nicrecv: writes recv addr (from free queue) to nic recv register
  // s_modpte: update pte, point to new paddr and mark as not remote and valid
  val s_idle :: s_sendreq :: s_nicrecv :: s_modpte :: s_comp :: Nil = Enum(5)
  val send = io.sendpacket
  val recv = io.recvpacket
  val sendPktReq = send.req.bits
  val sendPktHeader = sendPktReq.header
  val sendPktPayload = sendPktReq.payload

  val s = RegInit(s_idle)
  val targetaddr = RegInit(0.U(64.W))
  val pageid = RegInit(0.U(28.W))
  val protbits = RegInit(0.U(10.W))
  val pteppn = RegInit(0.U(54.W))
  val faultvpn = RegInit(0.U(27.W))
  val fetchFired = RegNext(io.fetch.req.fire(), false.B)
  val sendRespFired = RegNext(send.resp.fire(), false.B)
  val recvRespFired = RegNext(recv.resp.fire(), false.B)
  val xactid = Counter(io.fetch.req.fire(), (1 << 16) - 1)._1
  val frameFrag = Counter(recv.resp.fire(), 3)._1
  val newpte = RegInit(0.U(64.W))

  newpte := ((targetaddr >> 12.U) << 10.U) | protbits

  io.fetch.req.ready := s === s_idle && targetaddr != 0.U
  io.fetch.resp.valid := s === s_comp
  io.fetch.resp.bits := newpte

  io.free.ready := s === s_idle

  send.req.valid := s === s_sendreq && fetchFired
  send.resp.ready := s === s_sendreq
  // we don't need a payload
  sendPktPayload.addr := 0.U
  sendPktPayload.len := 0.U
  sendPktHeader.opcode := 0.U // read
  sendPktHeader.partid := 0.U
  sendPktHeader.pageid := pageid
  sendPktHeader.xactid := xactid

  recv.req.valid := MuxCase(false.B, Array(
              (s === s_nicrecv && frameFrag === 0.U) -> sendRespFired,
              (s === s_nicrecv && frameFrag === 1.U) -> recvRespFired,
              (s === s_nicrecv && frameFrag === 2.U) -> recvRespFired))
  recv.resp.ready := s === s_nicrecv
  recv.req.bits.taddr := MuxCase(targetaddr, Array(
              (frameFrag === 1.U) -> (targetaddr + 1368.U),
              (frameFrag === 2.U) -> (targetaddr + 2736.U)))

  write.req.valid := s === s_modpte && recvRespFired
  write.req.bits.data := newpte
  write.req.bits.addr := pteppn
  write.resp.ready := s === s_modpte

  when (io.free.fire()) {
    targetaddr := io.free.bits
  }

  when (io.fetch.req.fire()) {
    pageid := io.fetch.req.bits.pageid
    protbits := io.fetch.req.bits.protbits
    pteppn := io.fetch.req.bits.pteppn
    faultvpn := io.fetch.req.bits.faultvpn
    s := s_sendreq
  }

  when (send.resp.fire()) {
    s := s_nicrecv
  }

  // move to s_modpte when we are done receiving the frame completely
  when (recv.resp.fire()) {
    s := Mux(frameFrag === 2.U, s_modpte, s_nicrecv)
  }

  // write request to update pte is done
  when (write.resp.fire()) {
    s := s_comp
  }

  when (io.fetch.resp.fire()) {
    s := s_idle
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
  val pktRequest = io.sendpacket.req.bits
  val pktHeader = pktRequest.header
  val pktPayload = pktRequest.payload

  val s = RegInit(s_idle)
  val frameaddr = RegInit(0.U(64.W))
  val sendCompFired = RegNext(send.resp.fire(), false.B)
  val evictFired = RegNext(io.evict.req.fire(), false.B)
  val pageid = RegInit(0.U(32.W))
  val xactid = Counter(io.evict.req.fire(), (1 << 16) - 1)._1

  io.evict.req.ready := s === s_idle
  io.evict.resp.valid := s === s_comp
  io.evict.resp.bits := true.B

  send.req.valid := MuxCase(0.U, Array(
            (s === s_frame1) -> evictFired,
            (s === s_frame2 || s === s_frame3) -> sendCompFired))

  pktPayload.addr := MuxCase(0.U, Array(
            (s === s_frame1) -> (frameaddr),
            (s === s_frame2) -> (frameaddr + 1368.U),
            (s === s_frame3) -> (frameaddr + 2736.U)))
  pktPayload.len := MuxCase(0.U, Array(
            (s === s_frame1) -> 1368.U,
            (s === s_frame2) -> 1368.U,
            (s === s_frame3) -> 1360.U))

  pktHeader.opcode := 1.U // write
  pktHeader.partid := MuxCase(0.U, Array(
            (s === s_frame1) -> 0.U,
            (s === s_frame2) -> 1.U,
            (s === s_frame3) -> 2.U))
  pktHeader.pageid := pageid
  pktHeader.xactid := xactid

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
  val workbuf = Valid(UInt(39.W))
}

trait PFAControllerModule extends HasRegMap {
  val io: PFAControllerBundle
  val qDepth = 10

  val evictQueue = Module(new Queue(UInt(64.W), qDepth))
  val evictsInProg = TwoWayCounter(io.evict.req.fire(), io.evict.resp.fire(), qDepth)
  val evictStat = RegInit(0.U(64.W))
  io.evict.req <> evictQueue.io.deq
  io.evict.resp.ready := true.B // always ready to evict?
  evictStat := Mux(evictsInProg > evictQueue.io.count, evictsInProg, evictQueue.io.count)

  val workbufQueue = Module(new Queue(UInt(64.W), 1))
  io.workbuf.bits <> workbufQueue.io.deq.bits(38, 0)
  io.workbuf.valid <> workbufQueue.io.deq.valid
  //workbufQueue.io.deq.ready := true.B

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
  val recvframePkt = LazyModule(new RecvPacket(nicaddr, "pfa-recvframe1"))

  val mmionode = TLIdentityNode()
  val dmanode = TLIdentityNode()

  control.node := mmionode;
  dmanode := sendframePkt1.writenode
  dmanode := sendframePkt1.readnode
  dmanode := sendframePkt2.writenode
  dmanode := sendframePkt2.readnode
  dmanode := recvframePkt.writenode
  dmanode := recvframePkt.readnode
  dmanode := fetchPath.writenode

  lazy val module = new LazyModuleImp(this) {
    val io = IO(new Bundle {
      val remoteFault = Flipped(new PFAIO)
    })

    sendframePkt1.module.io.workbuf <> control.module.io.workbuf
    sendframePkt2.module.io.workbuf <> control.module.io.workbuf

    evictPath.module.io.evict <> control.module.io.evict
    evictPath.module.io.sendpacket <> sendframePkt1.module.io.sendpacket

    io.remoteFault <> fetchPath.module.io.fetch
    fetchPath.module.io.sendpacket <> sendframePkt2.module.io.sendpacket
    fetchPath.module.io.recvpacket <> recvframePkt.module.io.recvpacket
    fetchPath.module.io.free <> control.module.io.free
  }
}

trait HasPeripheryPFA extends HasSystemBus {
  private val nicaddr = BigInt(0x10016000)
  private val pfaaddr = BigInt(0x10017000)

  val pfa = LazyModule(new PFA(pfaaddr, nicaddr, sbus.beatBytes))
  pfa.mmionode := sbus.toVariableWidthSlaves
  sbus.fromSyncPorts() :=* pfa.dmanode
}
