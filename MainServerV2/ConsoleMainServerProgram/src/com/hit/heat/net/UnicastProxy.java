package com.hit.heat.net;

import java.util.HashMap;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

/**
 * @lhy Lhy
 * @date 2015年11月2日
 * @des
 */
public class UnicastProxy {

	private List<String> addrList;
	private HashMap<String, Integer> ackHashMap;
	private byte[] content;
	private long delay;//多个节点发送的延时
	private long ack_delay;//单个节点等待ack上报延时
	private final int DELAY_TICK = 100;
	private int ticker;
	private int counter;
	private Timer timer;
	private boolean isRunning;
	private boolean stopFlag;
	private int curNo;
	private ProxyInvoke method;
	private Timer ack_timer;
	private int retransmit_count;
	private int port;
	
	public UnicastProxy(){
		isRunning = false;
		stopFlag = false;
		curNo = 0;
		delay = 1000;
		ack_delay = 3100;
		ticker =(int) delay / DELAY_TICK;
		retransmit_count = 4;
		ack_timer = new Timer(false);
	}
	
	public void registerMethod(ProxyInvoke method){
		this.method = method;
	}
	
	public void setProxyTask(List<String> addrList,int port,byte[] content){
		this.addrList = addrList;
		this.port = port;
		ackHashMap = new HashMap<String, Integer>(addrList.size());
		this.content = content;
		//初始化，设置所有的地址未收到ack
		for(String addr: addrList){
			System.out.println(addr);
			ackHashMap.put(addr, retransmit_count);
		}
	}
	
	public void setDelay(long delay){
		delay = delay >= 100 ? delay : 100;
		this.delay = delay;
		ticker = (int) delay / DELAY_TICK;
	}
	
	public void setReTransmitCount(int count){
		retransmit_count = count >= 1 ? count : 1;
	}
	
	public void setAckWaitDelay(long delay){
		ack_delay = delay;
	}
	
	public void setAck(String addr){
		synchronized (ackHashMap) {
			ackHashMap.put(addr, 0);
		}
	}
	
	public void start() throws Exception{
		
		if(addrList == null || method == null){
			throw new Exception("异常：未注册回调函数或未设置地址列表");
		}
		if(isRunning){
			timer.cancel();
		}
		counter = ticker;
		timer = new Timer(false);
		timer.schedule(new TimerTask() {
			
			@Override
			public void run() {
				// TODO 自动生成的方法存根
				//关闭，优先级最高
				if(stopFlag){
					exit();
					return;
				}
				//计时器触发
				if(counter == ticker){
					//真正的定时任务
					method.invoke(addrList.get(curNo),port,content);
//					try {
//						System.out.println(InetAddress.getByName(addrList.get(curNo)));
//					} catch (UnknownHostException e) {
//						// TODO 自动生成的 catch 块
//						e.printStackTrace();
//					}
					//printMap();
					ack_timer.schedule(new TimerTaskImpl(addrList.get(curNo)),ack_delay);
					curNo++;
					if(curNo == addrList.size()){
						exit();
						return;
					}
					counter = 0;
					return;
				}
				counter++;
			}
		}, 100, DELAY_TICK);
		isRunning = true;
	}
	public void printMap(){
		for(String key : ackHashMap.keySet()){
			System.out.println(key +" " + ackHashMap.get(key));
		}
	}
	class TimerTaskImpl extends TimerTask{

		private String addr;

		public TimerTaskImpl(String addr) {
			// TODO 自动生成的构造函数存根
			this.addr = addr;
		}
		@Override
		public void run() {
			// TODO 自动生成的方法存根
			int c = 0;
			synchronized (ackHashMap) {
				c = ackHashMap.get(addr);
				if(c > 0){
					method.invoke(addr,port, content);
					ackHashMap.put(addr, c - 1);
					ack_timer.schedule(new TimerTaskImpl(addr), ack_delay);
					//printMap();
				}
			}
		}
	}
	
	/**
	 * 
	 * @description void
	 * 退出，取消定时器，设置运行状态为false，停止状态为false
	 *
	 */
	private void exit(){
		timer.cancel();
		isRunning = false;
		stopFlag = false;
	}
	public void stop(){
		stopFlag = true;
	}
	
	static class MethodInvoke implements ProxyInvoke{

		/* （非 Javadoc）
		 * @see com.hit.hvac.net.ProxyInvoke#invoke(java.lang.String, int, byte[])
		 */
		@Override
		public void invoke(String addr, int port, byte[] message) {
			// TODO 自动生成的方法存根
			System.out.println(addr +" "+" /"+port+ new String(message));
		}
	}
//	public static void main(String[] args) {
//		UnicastProxy unicastProxy = new UnicastProxy();
//		List<String> list = new ArrayList<String>();
////		for(int i=0;i<3;i++){
////			list.add("192.168.1.141" + (i+1));
////		}
//		list.add("192.168.1.141");
//		unicastProxy.registerMethod(new MethodInvoke());
//		unicastProxy.setProxyTask(list,7788, "hello world".getBytes());
//		unicastProxy.setDelay(1000);
//		
//		try {
//			unicastProxy.start();
//			
//		} catch (Exception e) {
//			// TODO 自动生成的 catch 块
//			e.printStackTrace();
//		}
////		try {
////			Thread.sleep(3000);
////		} catch (Exception e) {
////			// TODO: handle exception
////		}
////		unicastProxy.setAck("aaaa:2");
//	}
}
