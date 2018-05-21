package com.hit.heat.net;

import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import io.netty.bootstrap.Bootstrap;
import io.netty.channel.Channel;
import io.netty.channel.ChannelInitializer;
import io.netty.channel.nio.NioEventLoopGroup;
import io.netty.channel.socket.SocketChannel;
import io.netty.channel.socket.nio.NioSocketChannel;
import io.netty.handler.codec.string.StringEncoder;

/**
 * @lhy Lhy
 * @time	  于2015年7月31日上午10:53:35
 *
 * @description 
 **/

public class NettyClient{
	
	private final int BlockQueueSize = 128;
	//初始化Client服务
	private Bootstrap bootstrap;
	//客户端Channel
	private Channel channel;
	private String host;
	private int port;
	private BlockingQueue<String> cacheQueue;
	private RemoteServerOfflineHandler handler;
	boolean startListenFlag;
	private boolean remoteHostOnlineFlag;
	public NettyClient(String host,int port){
		
		// TODO 自动生成的构造函数存根
		startListenFlag =false;
		this.host = host;
		this.port = port;
		bootstrap = new Bootstrap();
		cacheQueue = new ArrayBlockingQueue<String>(BlockQueueSize);
		
		//指定channel类型
		bootstrap.channel(NioSocketChannel.class);
		//bootstrap.option(ChannelOption.TCP_NODELAY, true);
		//bootstrap.option(, value)
		// 指定Handler
		bootstrap.handler(new ChannelInitializer<SocketChannel>() {
			@Override
			protected void initChannel(SocketChannel arg0) throws Exception {
				arg0.pipeline().addLast("encoder", new StringEncoder());
			}
		});
		//指定EventLoopGroup
		bootstrap.group(new NioEventLoopGroup());
		
		new NettyClientTask().start();
	}
	
	public boolean remoteHostIsOnline(){
		Socket socket = new Socket();
		try{
			socket.connect(new InetSocketAddress(host,port), 100);
		}catch(Exception e){
			try {
				socket.close();
			} catch (IOException e1) {
				// TODO 自动生成的 catch 块
			}
			return false;
		}
		try {
			socket.close();
		} catch (IOException e1) {
			// TODO 自动生成的 catch 块
		}
		return true;
	}
	
	public void startListenHeart(){
		startListenFlag =true;
		if(remoteHostIsOnline()){
			remoteHostOnlineFlag = true;
		}else{
			remoteHostOnlineFlag = false;
		}
		//System.out.println(remoteHostIsOnline());
		handler.actionPerformed(remoteHostOnlineFlag);
	}
	
	public void stopListenHeart(){
		startListenFlag =false;
	}
	
	
	public void registerRemoteServerOfflineHandler(RemoteServerOfflineHandler handler){
		this.handler = handler;
	}
	
	public void setRemoteAddress(String host,int port){
		this.host = host;
		this.port = port;
		channel.close();
		channel = null;
	}
	
	//发送
	public boolean asyncWriteAndFlush(String message){
		System.out.println("send");
		if(message == null || message.isEmpty()){
			return false;
		}
		if(BlockQueueSize == cacheQueue.size()){
			return false;
		}
		if(null == channel || !channel.isActive()){
			//连接到服务器
			try {
				channel = bootstrap.connect(new InetSocketAddress(host,port)).sync().channel();
			} catch (Exception e) {
				if(startListenFlag){
					handler.actionPerformed(false);
				}
				return false;
			}
		}
		cacheQueue.add(message);
		return true;
	}
	
	public void closeChannel(){
		channel.close();
		bootstrap.group().shutdownGracefully();
	}
	
	class NettyClientTask extends Thread{
		
		@Override
		public void run() {
			// TODO 自动生成的方法存根
			while(true){
				if(!cacheQueue.isEmpty()){//队列不为空，说明里面有数据
					if(channel.isWritable()){//判断channel是否可写，可写的话将数据从队列中拿出，发送给控制中枢
						channel.writeAndFlush(cacheQueue.poll());
					}
				}//队列空，等待100毫秒，再检查队列是否为空。轮询
				try {
					Thread.sleep(100);//等待10毫秒
				} catch (Exception e) {
					// TODO: handle exception
				}
			}
		}
	}
//	public static void main(String[] args) throws InterruptedException {
//		NettyClient nettyClient = new NettyClient("192.168.1.142", 7969);
//		nettyClient.registerRemoteServerOfflineHandler(new RemoteServerOfflineHandler() {
//			
//			@Override
//			public void actionPerformed(boolean flag) {
//				// TODO 自动生成的方法存根
//				System.out.println("远程服务器不在线");
//			}
//		});
//		String msg="68363668080272000000008F410322020000000F0700F108042302000000000000000002"
//				+ "000000003523101908230A15200500020000000000009F16";
//		JSONObject obj;
//		//数据保存至JSON数组中
//		obj = new JSONObject();
//		JSONArray array = new JSONArray();
//		try {
//			obj.put("addr", "aaaa:0:0:0:12:7400:1:1a");
//			obj.put("data", msg);
//			array.put(obj);
//		} catch (JSONException e) {
//			// TODO 自动生成的 catch 块
//			e.printStackTrace();	
//		}
//		System.out.println(array.toString());
//		nettyClient.asyncWriteAndFlush(array.toString());
//		//System.out.println("lll");
////		for(int i=0;i <100; i++){
////			nettyClient.asyncWriteAndFlush("hello world" + i);
////		}
////		Thread.sleep(15000);
////		nettyClient.setRemoteAddress("192.168.1.92", 8888);
//		
////		for(int j=0;j <100; j++){
////			nettyClient.asyncWriteAndFlush("hello world" + j);
////		}
//		
//		
//	}
}
