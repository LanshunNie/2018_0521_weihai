package com.hit.heat.net;

import io.netty.bootstrap.ServerBootstrap;
import io.netty.channel.ChannelHandlerContext;
import io.netty.channel.ChannelInitializer;
import io.netty.channel.ChannelOption;
import io.netty.channel.EventLoopGroup;
import io.netty.channel.SimpleChannelInboundHandler;
import io.netty.channel.nio.NioEventLoopGroup;
import io.netty.channel.socket.SocketChannel;
import io.netty.channel.socket.nio.NioServerSocketChannel;
import io.netty.handler.codec.string.StringDecoder;
import io.netty.handler.codec.string.StringEncoder;

/**
 * 
 * @lhy Lhy tcp Netty服务器
 *
 */

public class NettyServer {
	private String host;
	private int port;
	private EventLoopGroup bossGroup;//处理连接的线程
	private EventLoopGroup workerGroup;//工作线程
	private ServerBootstrap bootstrap;
	private NettyMsgHandler nettyMsgHandler;
	private boolean startFlag;
	public NettyServer(String host,int port,NettyMsgHandler nettyMsgHandler){
		this.host = host;
		this.port = port;
		this.nettyMsgHandler = nettyMsgHandler;
		startFlag = false;
	}
	public void start() throws InterruptedException{
		start(host, port);
	}
	public void start(String host,int port) throws InterruptedException{
		if(startFlag){
			throw new IllegalStateException("Netty server is running.");
		}
		bossGroup = new NioEventLoopGroup(Runtime.getRuntime().availableProcessors());
		workerGroup = new NioEventLoopGroup(Runtime.getRuntime().availableProcessors() * 2);
		bootstrap = new ServerBootstrap();//启动Netty服务器
		bootstrap.group(bossGroup, workerGroup)
				.channel(NioServerSocketChannel.class)
				.childHandler(new ChannelInitializer<SocketChannel>() {

					@Override
					protected void initChannel(SocketChannel arg0)
							throws Exception {
						// TODO 自动生成的方法存根
						// 字符串解码 和 编码
						arg0.pipeline().addLast("decoder", new StringDecoder());
						arg0.pipeline().addLast("encoder", new StringEncoder());
						arg0.pipeline().addLast("handler",
								new NettyServerHandler());
					}
				}).option(ChannelOption.SO_KEEPALIVE, true);
		bootstrap.bind(host, port).sync();
		startFlag = true;
	}
	public void stop(){
		workerGroup.shutdownGracefully();
		bossGroup.shutdownGracefully();
		startFlag = false;
	}
	
	private class NettyServerHandler extends SimpleChannelInboundHandler<Object> {
		
		@Override
		public void channelActive(ChannelHandlerContext ctx) throws Exception {
			// TODO 自动生成的方法存根
		}
		
		@Override
		protected void channelRead0(ChannelHandlerContext arg0, Object arg1)
				throws Exception {
			String response = nettyMsgHandler.messageHandler((String)arg1);
			if(response != null){
				arg0.writeAndFlush(response);
				response = null;
			}
		}
		@Override
		public void channelInactive(ChannelHandlerContext ctx) throws Exception {
			// TODO 自动生成的方法存根
		}
		
		@Override
		public void exceptionCaught(ChannelHandlerContext ctx, Throwable cause)
				throws Exception {
			// TODO 自动生成的方法存根
			//System.out.println("except");
		}
	}
//	public static void main(String[] args) {
//		NettyServer server = new NettyServer("0.0.0.0",8888,new NettyMsgHandler() {
//			
//			@Override
//			public String messageHandler(String message) {
//				// TODO 自动生成的方法存根
//				System.out.println(message);
//				try {
//					Thread.sleep(1000);
//				} catch (Exception e) {
//					// TODO: handle exception
//				}
//				return message;
//			}
//		});
//		try {
//			System.out.println("开启");
//			server.start();
////			server.start();
////			//Thread.sleep(1000);
////			System.out.println("关闭");
////			server.stop();
////			//Thread.sleep(1000);
////			server.start();
////			System.out.println("开启");
//		} catch (InterruptedException e) {
//			// TODO 自动生成的 catch 块
//			e.printStackTrace();
//		}
//	}
}
