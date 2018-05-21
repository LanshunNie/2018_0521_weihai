package com.hit.heat.net;

import java.util.Hashtable;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

/* @lhy Lhy
 * @date 2015年12月8日
 * @des  
 */
public class HeartProxy {
	private Hashtable<String, Integer> hashTable;//线程安全的HashMap
	private Timer timer;
	private boolean isRunning;
	private HeartOffLineHandler offLineHandler;
	private HeartOnLineHandler onLineHandler;
	private int inverval;
	public HeartProxy(List<String> list,int inverval){
		this.inverval = inverval;
		hashTable = new Hashtable<String, Integer>();
		for(String ip : list){
			hashTable.put(ip, 3);
		}
		isRunning = false;
	}
	
	
	public void registerOffLineHandler(HeartOffLineHandler handler){
		this.offLineHandler = handler;
	}
	
	public void registerOnLineHandler(HeartOnLineHandler handler){
		this.onLineHandler = handler;
	}
	
	public void start() throws IllegalStateException{
		
		if(offLineHandler == null){
			throw new IllegalStateException("尚未注册掉线处理函数");
		}
		
		if(onLineHandler == null){
			throw new IllegalStateException("尚未注册掉线重连处理函数");
		}
		
		if(isRunning){
			return;
		}
		timer = new Timer(false);
		System.out.println("启动心跳代理");
		for(String addr : hashTable.keySet()){
			//System.out.println(inverval);
			timer.schedule(new TimerTaskImpl(addr), inverval);
		}
		isRunning = true;
	}
	public void stop(){
		timer.cancel();
		isRunning=false;
	}
	
	public void setInverval(int inverval){
		if(this.inverval == inverval){
			return;
		}
		this.inverval = inverval;
		if(timer == null){
			return;
		}
		timer.cancel();
		timer = new Timer(false);
		for(String addr : hashTable.keySet()){
			//System.out.println(addr);
			timer.schedule(new TimerTaskImpl(addr), inverval);
		}
	}
	
	public void setRevcAddr(String addr){
		
		if(hashTable.get(addr) == 0){
			//断线重连
			onLineHandler.actionPerformed(addr);
		}
		hashTable.put(addr, 3);
	}
	
	public boolean isOnline(String addr){
		int c = hashTable.get(addr);
		if(c == 0){
			return false;
		}
		return true;
 	}
	
	class TimerTaskImpl extends TimerTask{

		String addr;
		public TimerTaskImpl(String addr) {
			// TODO 自动生成的构造函数存根
			this.addr = addr;
		}
		@Override
		public void run() {
			// TODO 自动生成的方法存根
			int count = hashTable.get(addr);
			if(count > 0){
				hashTable.put(addr, count - 1);
				timer.schedule(new TimerTaskImpl(addr), inverval);
			}else{
				offLineHandler.actionPerformed(addr);
				timer.schedule(new TimerTaskImpl(addr), inverval);//1000 * 60
			}
		}
	}
	
//	public static void main(String[] args) {
//		List<String> list = new ArrayList<String>();
//		list.add("A");
//		list.add("B");
//		list.add("C");
//		list.add("D");
//		HeartProxy heartProxy = new HeartProxy(list,5000);
//		heartProxy.registerOffLineHandler(new HeartOffLineHandler() {
//			
//			@Override
//			public void actionPerformed(String addr) {
//				// TODO 自动生成的方法存根
//				System.out.println(addr + " 掉线了");
//			}
//		});
//		
//		heartProxy.registerOnLineHandler(new HeartOnLineHandler() {
//			
//			@Override
//			public void actionPerformed(String addr) {
//				// TODO 自动生成的方法存根
//				System.out.println(addr +"重连");
//			}
//		});
//		heartProxy.start();
//		try {
//			Thread.sleep(31 * 1000);
//			heartProxy.setRevcAddr("A");
//		} catch (Exception e) {
//			// TODO: handle exception
//		}
//		heartProxy.setInverval(10 * 1000);
//		
//	}
}
