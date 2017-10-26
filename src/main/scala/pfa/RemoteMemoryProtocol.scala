package freechips.rocketchip.pfa

import chisel3._
import chisel3.util._
import freechips.rocketchip.config.{Field, Parameters}
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.rocket.PAddrBits
import freechips.rocketchip.tilelink._
import freechips.rocketchip.pfa._

// Sends subsets of frames in network packets based on remote memory protocol
// Waits for send completion
class SendFramePacket(nicaddr: BigInt)(implicit p: Parameters) extends LazyModule {
  val tlwriter = LazyModule(new TLWriter("pfa-sendframe-write"))
  val writenode = TLOutputNode()
  writenode := tlwriter.node

  val tlreader = LazyModule(new TLReader("pfa-sendframe-read"))
  val readnode = TLOutputNode()
  readnode := tlreader.node

  lazy val module = new SendFramePacketModule(this, nicaddr)
}

class SendFramePacketIO extends Bundle {
  val req = Decoupled(new Bundle {
    val addr = UInt(39.W)
    val len = UInt(11.W)
    val part_id = UInt(8.W)
    val pageid = UInt(8.W)
  })
  val resp = Flipped(Decoupled(Bool()))
}

class SendFramePacketModule(outer: SendFramePacket, nicaddr: BigInt)
    extends LazyModuleImp(outer) {
  val io = IO(new Bundle {
    val tlwrite = outer.writenode.bundleOut
    val tlread = outer.readnode.bundleOut
    val sendframe = Flipped(new SendFramePacketIO)
    val workbuf = Flipped(Decoupled(UInt(64.W)))
  })

  val write = outer.tlwriter.module.io.write
  val read = outer.tlreader.module.io.read
  val addr = io.sendframe.req.bits.addr
  val len = io.sendframe.req.bits.len
  val nicSendReq = nicaddr
  val nicSendCompAddr = nicaddr + 20 // how many completions are available
  val nicSendAckCompAddr = nicaddr + 16 // ack the completions by reading
  val s_idle :: s_send :: s_wait :: s_ack :: s_comp :: Nil = Enum(5)

  val s = RegInit(s_idle)
  val sendReqFired = RegNext(io.sendframe.req.fire(), false.B)
  val writeCompFired = RegNext(write.resp.fire(), false.B)
  val readCompFired = RegNext(read.resp.fire(), false.B)
  val nicSentPackets = Wire(0.U(4.W))

  write.req.valid := s === s_send && sendReqFired
  write.req.bits.data := Cat(0.U(5.W), len, 0.U(9.W), addr)
  write.req.bits.addr := nicSendReq.U
  write.resp.ready := s === s_send

  read.req.valid := MuxCase(false.B, Array(
                      (s === s_wait) -> (writeCompFired || readCompFired),
                      (s === s_ack) -> readCompFired))
  read.req.bits.addr := MuxCase(0.U, Array(
                      (s === s_wait) -> nicSendCompAddr.U,
                      (s === s_ack) -> nicSendAckCompAddr.U))
  read.resp.ready := s === s_wait || s === s_ack

  io.sendframe.req.ready := s === s_idle
  io.sendframe.resp.valid := s === s_comp
  io.sendframe.resp.bits := true.B

  io.workbuf.ready := s === s_idle

  nicSentPackets := (read.resp.bits.data >> 40) & 0xF.U

  when (io.sendframe.req.fire()) {
    s := s_send
  }
  when (write.resp.fire()) {
    s := s_wait
  }
  when (read.resp.fire()) {
    switch (s) {
      is (s_wait) {
        s := Mux(nicSentPackets > 0.U, s_ack, s_wait)
      }
      is (s_ack) {
        s := s_comp
      }
    }
  }
  when (io.sendframe.resp.fire()) {
    s := s_idle
  }
}
