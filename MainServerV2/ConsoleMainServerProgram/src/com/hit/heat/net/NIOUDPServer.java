package com.hit.heat.net;

import java.io.IOException;
import java.io.UnsupportedEncodingException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.SocketAddress;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.nio.channels.Selector;
import java.util.Arrays;
import java.util.Queue;

import javax.lang.model.type.PrimitiveType;

import org.jfree.util.Log;

import io.netty.bootstrap.Bootstrap;
import io.netty.buffer.ByteBuf;
import io.netty.buffer.Unpooled;
import io.netty.channel.Channel;
import io.netty.channel.ChannelHandlerContext;
import io.netty.channel.ChannelOption;
import io.netty.channel.EventLoopGroup;
import io.netty.channel.SimpleChannelInboundHandler;
import io.netty.channel.nio.NioEventLoopGroup;
import io.netty.channel.socket.DatagramPacket;
import io.netty.channel.socket.nio.NioDatagramChannel;
import io.netty.util.CharsetUtil;

public class NIOUDPServer {
	private Channel chanel = null;

	// 消息处理函数
	private NIOUDPServerMsgHandler msgHandler;
	// 块大小
	private int BLOCK = 4096;
	// 服务器绑定的端口
	private String addr;
	// 服务器绑定的端口
	private int port;
	private Bootstrap b;

	/**
	 * 初始化非阻塞UDP服务器
	 * 
	 * @param addr
	 *            服务器绑定的IP地址
	 * @param port
	 *            服务器绑定的端口
	 * @throws UnknownHostException
	 * @throws IOException
	 */
	public NIOUDPServer(String addr, int port) throws UnknownHostException {// udp服务端，接受客户端发送的广播
		this.addr = addr;
		this.port = port;
		b = new Bootstrap();
		EventLoopGroup group = new NioEventLoopGroup();
		b.group(group).channel(NioDatagramChannel.class)
				.handler(new UdpServerHandler());
	}

	public NIOUDPServer(int port) throws IOException {
		this("0.0.0.0", port);
	}

	public void start() throws InterruptedException, IllegalStateException,
			Exception {

		if (msgHandler == null) {
			System.out.println("2222222222222222222222222222222222222222222");
			throw new IllegalStateException(
					"Do not register message handler yet.");
		}

		
		chanel = b
				.bind(new InetSocketAddress(InetAddress.getByName(this.addr),
						this.port)).sync().channel();
		chanel.closeFuture();
		
	}

	public void stop() {
		chanel.close();
	}

	public boolean sendto(String message, SocketAddress remoteAddr) {

		DatagramPacket data = new DatagramPacket(Unpooled.copiedBuffer(message,
				CharsetUtil.UTF_8), (InetSocketAddress) remoteAddr);
		chanel.writeAndFlush(data);// 向客户端发送消息
		return true;
	}

	public boolean sendto(byte[] message, SocketAddress remoteAddr) {
		DatagramPacket data = new DatagramPacket(
				Unpooled.copiedBuffer(message), (InetSocketAddress) remoteAddr);
		//System.out.println(Arrays.toString(message) + (InetSocketAddress) remoteAddr );
		//System.out.println(data);
		if(chanel == null) {
			System.out.println("chanel is empty&!@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@222");
		}
		chanel.writeAndFlush(data);// 向客户端发送消息
		return true;
	}

	public void registerHandler(NIOUDPServerMsgHandler handler) {
		msgHandler = handler;
	}

	private class UdpServerHandler extends
			SimpleChannelInboundHandler<DatagramPacket> {

		@Override
		protected void channelRead0(ChannelHandlerContext ctx,
				DatagramPacket packet) throws Exception {
			// TODO Auto-generated method stub
			ByteBuf buf = packet.copy().content();
			try{
			byte[] req = new byte[buf.readableBytes()];
			buf.readBytes(req);
			System.out.println(packet.sender().getHostString()
					+ packet.sender().getPort());
			msgHandler.messageHandler(packet.sender().getHostString(), req);
			}catch (Exception e){
				e.printStackTrace();
				buf.release();
			}finally{
				buf.release();
				buf.clear();
			}
			
		}
	}

//	public static void main(String[] arg) {
//		try {
//			NIOUDPServer nettyUDPServer = new NIOUDPServer("127.0.0.1",
//					9999);
//			nettyUDPServer.registerHandler(new NIOUdpNetDataHandl());
//			try {
//				nettyUDPServer.start();
//			} catch (IllegalStateException e) {
//				// TODO Auto-generated catch block
//				e.printStackTrace();
//			} catch (InterruptedException e) {
//				// TODO Auto-generated catch block
//				e.printStackTrace();
//			} catch (Exception e) {
//				// TODO Auto-generated catch block
//				e.printStackTrace();
//			}
//		} catch (UnknownHostException e) {
//			// TODO Auto-generated catch block
//			e.printStackTrace();
//		}
//	}
}

//class NIOUdpNetDataHandl implements NIOUDPServerMsgHandler {
//
//	@Override
//	public byte[] messageHandler(String addr, byte[] message) {
//		// TODO Auto-generated method stub
//		System.out.println(addr + "  " + Arrays.toString(message));
//		return null;
//	}
//
//}