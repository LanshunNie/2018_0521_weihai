package com.hit.heat.net;

import java.io.IOException;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.SocketAddress;
import java.nio.ByteBuffer;
import java.nio.channels.DatagramChannel;
import java.nio.channels.SelectionKey;
import java.nio.channels.Selector;
import java.nio.charset.Charset;
import java.util.Iterator;
import java.util.Queue;
import java.util.Set;
import java.util.concurrent.ArrayBlockingQueue;



/**
 * @author 	 lhy
 * @time	  于2015年7月31日下午10:01:32
 *
 * @description 
 **/

public class NIOUDPServer1 implements Runnable {

	//块大小
	private int BLOCK = 4096;
	//接收缓存区
	private ByteBuffer buffer;
	//服务器绑定的端口
	private String addr;
	//服务器绑定的端口
	private int port;
	//服务器通道
	private DatagramChannel channel;
	//选择器
	private Selector selector;
	//运行标志
	private boolean runFlag =  false;
	//消息处理函数
	private NIOUDPServerMsgHandler msgHandler = null;
	//客户端
	private DatagramChannel client;
	//发送远程字符串队列
	private Queue<byte[]> sendMessageQueue;
	//发送远程套接字地址队列，两者一起使用主动向远程主机发送数据
	private Queue<SocketAddress> sendRemoteAddrQueue;
	//队列大小
	private int QUEUE_CAP = 16;
	
	private int count = 0;
	/**
	 * 初始化非阻塞UDP服务器
	 * @param addr 服务器绑定的IP地址
	 * @param port 服务器绑定的端口
	 * @throws IOException
	 */
	public NIOUDPServer1(String addr,int port) throws IOException{
		this.addr = addr;
		this.port = port;
		channel = DatagramChannel.open();
		channel.configureBlocking(false);
		channel.socket().bind(new InetSocketAddress(InetAddress.getByName(addr),port));
		buffer = ByteBuffer.allocate(BLOCK);
		selector = Selector.open();
		channel.register(selector, SelectionKey.OP_READ | SelectionKey.OP_WRITE);
		sendMessageQueue = new ArrayBlockingQueue<byte[]>(QUEUE_CAP);
		sendRemoteAddrQueue = new ArrayBlockingQueue<SocketAddress>(QUEUE_CAP);
	}
	
	/**
	 * 初始化非阻塞UDP服务器
	 * @param port 服务器绑定的端口,(IP地址默认为 0.0.0.0)
	 * @throws IOException
	 */
	public NIOUDPServer1(int port) throws IOException{
		this("0.0.0.0", port);
	}
	
	public void start() throws InterruptedException,IllegalStateException{
		if(runFlag){
			throw new IllegalStateException("Thread can only start once.");
		}
		if(msgHandler == null){
			throw new IllegalStateException("Do not register message handler yet.");
		}
		runFlag = true;
		Thread thread = new Thread(this);
		thread.start();
	}
	
	public void stop(){
		runFlag = false;
	}
	
	public void registerHandler(NIOUDPServerMsgHandler handler) {
		msgHandler = handler;
	}
	
	/**
	 * 
	 * @description 将字符串转化为ByteBuffer
	 * @param str
	 * @return ByteBuffer
	 *
	 */
	public ByteBuffer encodeString(String str) {
		if (str == null) {
			return null;
		}
		return Charset.forName("utf8").encode(str);
	}
	
	/**
	 * 
	 * @description 使用通道主动向远程主机发送消息
	 * @param message 消息字符串
	 * @param remoteAddr 远程主机的地址
	 * @return boolean	添加任务是否成功
	 *
	 */
	public boolean sendto(String message,SocketAddress remoteAddr){
		if(sendMessageQueue.size() == QUEUE_CAP){
			return false;
		}
		sendMessageQueue.add(message.getBytes());
		sendRemoteAddrQueue.add(remoteAddr);
		return true;
	}
	public boolean sendto(byte[] message,SocketAddress remoteAddr){
		if(sendMessageQueue.size() == QUEUE_CAP){
			return false;
		}
		sendMessageQueue.add(message);
		sendRemoteAddrQueue.add(remoteAddr);
		return true;
	}
	
	/**
	 * 
	 * @des 
	 * @param key
	 */
	private void selectionKeyHandler(SelectionKey key){
		try {
			if(key.isReadable()){
				/* 接收字符串长度 */
				int recvLen;
				/* 接收的字符串 */
				byte[] message = null;
				/* 远程地址 */
				SocketAddress remoteAddress;
				client = (DatagramChannel) key.channel();
				client.configureBlocking(false);
				remoteAddress = client.receive(buffer);
				recvLen = buffer.position();
				message = new byte[recvLen];
				System.arraycopy(buffer.array(), 0, message, 0, recvLen);
				buffer.clear();
				// 如果不需要写数据到客户端，则返回null
				//可以获取对方的ip
				message = msgHandler.messageHandler(((InetSocketAddress)remoteAddress).getHostString(),message);
				if (message != null) {
					buffer.limit(buffer.capacity());
					buffer.put(message);
					buffer.flip();
					client.send(buffer, remoteAddress);
					buffer.clear();
				}
			}else if(key.isWritable()){
				if(!sendMessageQueue.isEmpty()){
					client = (DatagramChannel) key.channel();
					client.configureBlocking(false);
					//从发送队列获取第一个发送信息即时发送
					byte[] taskMessage = sendMessageQueue.poll();
					//远程地址
					SocketAddress taskRemoteAddr = sendRemoteAddrQueue.poll();
					buffer.limit(buffer.capacity());
					buffer.put(taskMessage);
					buffer.flip();
					client.send(buffer, taskRemoteAddr);
					if(count < 3){
						count++;
						buffer.clear();
						taskMessage = null;
						taskRemoteAddr = null;
						throw new Exception("异常" + count);
					}
					buffer.clear();
					taskMessage = null;
					taskRemoteAddr = null;
				}
			}
		} catch (Exception e) {
			// TODO: handle exception
		}
	}
	
	@Override
	public void run() {
		// TODO 自动生成的方法存根
		while(runFlag){
			try {
				selector.select();
				Set<SelectionKey> selectionKeys = selector.selectedKeys();
				Iterator<SelectionKey> iterator = selectionKeys.iterator();
				while (iterator.hasNext()) {
					SelectionKey selectionKey = iterator.next();
					iterator.remove();
					selectionKeyHandler(selectionKey); 
					//System.out.println("select");
				}
			} catch (IOException e) {
				// TODO: handle exception
				System.out.println("IOEx");
				e.printStackTrace();
			}
		}
		try {
			selector.close();
			channel.socket().close();
			channel.close();
			System.out.println("close");
		} catch (Exception e) {
			// TODO: handle exception
			e.printStackTrace();
		}
	}
	
	
	public static String formatBytesToStr(byte[] bytes){
		StringBuilder sb = new StringBuilder();
		int index;
		for(index = 0;index < bytes.length - 1; index++){
			sb.append(String.format("0x%02X", bytes[index]) + " ");
		}
		sb.append(String.format("0x%02X", bytes[index]));
		return sb.toString();
	}
	
//	public static void main(String[] args) throws IOException,InterruptedException,IllegalStateException{
//		NIOUDPServer udpServer = new NIOUDPServer("0.0.0.0",9999);
//		udpServer.registerHandler(new NIOUDPServerMsgHandler() {
//			
//			@Override
//			public byte[] messageHandler(String addr,byte[] message) {
//				// TODO 自动生成的方法存根
//				System.out.println(addr +" say :" + formatBytesToStr(message));
//				return message;
//			}
//		});
//		udpServer.start();
//		SocketAddress remoteAddr = new InetSocketAddress(InetAddress.getByName("192.168.1.141"), 10000);
//		for (int i = 0; i < 1000; ++i) {
//			udpServer.sendto("remote message" + i, remoteAddr);
//			System.out.println("send to");
//			Thread.sleep(5000);
//		}
//	}
}
