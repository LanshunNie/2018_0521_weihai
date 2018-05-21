package com.hit.heat.net;

import java.util.ArrayList;
import java.util.Hashtable;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;

/* @lhy Lhy
 * @date 2015年12月21日
 * @des  参数配置代理
 */
public class ParamConfigProxy {
	private ProxyInvoke proxyInvoke;
	private Hashtable<String, Integer> hashTable;
	private List<String> ipList;
	//private HashSet<String> hashSet;
	private int totalNodeSize;
	//private boolean mcastModel;
	private Timer timer;
	private String rootAddr;
	private int curCastAckNodeCount;
	private int mcastDelay;
	private int ucastDelay;
	private int singleDelay;
	private boolean singleModel;
	private float castValue;
	private int curCastCount;//多播重复次数
	private ParamConfigResult configResult;
	private byte[] buffer;
	private int curNo;
	public ParamConfigProxy(String rootAddr,ProxyInvoke proxyInvoke) throws IllegalArgumentException{

		if(rootAddr == null){
			throw new IllegalArgumentException("root节点不能为null");
		}
		if(proxyInvoke == null){
			throw new IllegalArgumentException("代理不能为null");
		}
		hashTable = new Hashtable<String, Integer>();
		this.rootAddr = rootAddr;
		this.proxyInvoke = proxyInvoke;
		mcastDelay = 1000 *130;//多播配置参数等待时间
		ucastDelay = 1000;//节点间延迟
		singleDelay = 1000 * 5;//单播配置参数等待时间
		castValue = 1.0f;//可靠性
		singleModel = false;
	}
	
	public ParamConfigProxy registerConfigResultHandler(ParamConfigResult configResult){
		this.configResult = configResult;
		return this;
	}
	
	public void mcastConfig(byte[] message,List<String> list,int mPort,int uPort){
		System.out.println("do");
		if(message == null || list == null){
			return;
		}
		singleModel = false;
		buffer = message;
		curCastCount = 0;
		curCastAckNodeCount = 0;
		totalNodeSize = list.size();
		ipList = list;
		hashTable.clear();
		for(String ip : ipList){
			hashTable.put(ip, 0);
		}
		proxyInvoke.invoke(rootAddr,mPort, message);
		timer = new Timer();
		timer.schedule(new MCastTimeOutImpl(mPort,uPort), mcastDelay);
	}
	//ip列表
	public void ucastConfig(byte[] message,List<String> list,int port){
		if(message == null || list == null){
			return;
		}
		ucastConfigExec(message,list,port,false);
	}
	
	private void ucastConfigExec(byte[] message,List<String> list,int port,boolean mcastFlag){
		singleModel = false;
		curCastCount = 0;
		curCastAckNodeCount = 0;
		curNo = 0;
		buffer = message;
		totalNodeSize = list.size();
		ipList = list;
		hashTable.clear();
		for(String ip : ipList){
			hashTable.put(ip, 0);
		}
		proxyInvoke.invoke(list.get(curNo),port, message);
		if(!mcastFlag){
			timer = new Timer();
		}
		timer.schedule(new UCastTimeOutImpl(port), ucastDelay);
	}
	//ip单节点
	public void ucastConfigSingleNode(byte[] message,String addr,int port){
		if(message == null || addr ==null){
			return;
		}
		buffer = message;
		singleModel = true;
		curCastCount = 0;
		proxyInvoke.invoke(addr, port, message);
		timer = new Timer();
		timer.schedule(new SingleNodeUcastTimeOutImpl(addr, port) , singleDelay);
	}
	
	public void setConfigAck(String ip){
		if(!singleModel){
			if(hashTable.keySet().contains(ip) && hashTable.get(ip) == 0){
				curCastAckNodeCount++;
				hashTable.put(ip, 1);
				System.out.println(ip +" come ");
			}
			return;
		}
		timer.cancel();
		if(configResult != null){
			configResult.actionPerformed(null);
			//计数清零
			curCastAckNodeCount = 0;
			return;
		}
	}
	
	private class MCastTimeOutImpl extends TimerTask{

		private int mPort;
		private int uPort;
		public MCastTimeOutImpl(int mPort,int uPort){
			this.mPort = mPort;
			this.uPort = uPort;
		}
		@Override
		public void run() {
			// TODO 自动生成的方法存根
			if(curCastAckNodeCount == totalNodeSize){
				//配置成功
				timer.cancel();
				if(configResult != null){
					configResult.actionPerformed(null);
					//计数清零
					curCastAckNodeCount = 0;
				}
				return;
			}
			float rate = 1.0f * curCastAckNodeCount / totalNodeSize;
			if(rate < castValue){
				curCastCount++;
				//只發一次
				if(curCastCount >=1){
					
					List<String> newList = new ArrayList<String>();
					for(String key : hashTable.keySet()){
						if(hashTable.get(key) == 0){
							newList.add(key);
						}
					}
					configResult.actionPerformed(newList);
					//计数清零
					curCastAckNodeCount = 0;
					newList = null;
					return;
				}
				proxyInvoke.invoke(rootAddr,mPort, buffer);
				timer.schedule(new MCastTimeOutImpl(mPort,uPort), mcastDelay);
			}
			
		}
	}
	
	private class SingleNodeUcastTimeOutImpl extends TimerTask{
		private int port;
		private String addr;
		public SingleNodeUcastTimeOutImpl(String addr,int port) {
			// TODO 自动生成的构造函数存根
			this.port = port;
			this.addr = addr;
		}
		@Override
		public void run() {
			// TODO 自动生成的方法存根
			curCastCount++;
			if(curCastCount == 3){
				timer.cancel();
				List<String> faiList = new ArrayList<String>();
				faiList.add(addr);
				configResult.actionPerformed(faiList);
				//计数清零
				curCastAckNodeCount = 0;
				return;
			}
			proxyInvoke.invoke(addr, port, buffer);
			timer.schedule(new SingleNodeUcastTimeOutImpl(addr, port) , singleDelay);
		}
	}
	private class UCastTimeOutImpl extends TimerTask{

		private int port;
		public UCastTimeOutImpl(int port){
			this.port = port;
		}
		@Override
		public void run() {
			// TODO 自动生成的方法存根
			if(curCastAckNodeCount == totalNodeSize){
				//配置成功
				timer.cancel();
				if(configResult != null){
					configResult.actionPerformed(null);
					//计数清零
					curCastAckNodeCount = 0;
				}
				return;
			}
			curNo++;
			if(curNo == hashTable.size()){
				curCastCount++;
				curNo =0;
			}
			if(curCastCount == 3){
				timer.cancel();
				if(configResult == null){
					return;
				}
				List<String> failList = new ArrayList<String>();
				for(String ip : hashTable.keySet()){
					if(hashTable.get(ip) == 0){
						failList.add(ip);
					}
				}
				configResult.actionPerformed(failList);
				//计数清零
				curCastAckNodeCount = 0;
				failList = null;
				return;
			}
			for(int i = curNo;i < hashTable.size();i++){
				if(hashTable.get(ipList.get(i)) == 0){
					curNo = i;
					break;
				}
			}
			proxyInvoke.invoke(ipList.get(curNo),port,buffer);
			timer.schedule(new UCastTimeOutImpl(port), ucastDelay);
		}
	}
	
	
//	public static void main(String[] args) throws InterruptedException {
//		List<String> addrList = new ArrayList<String>();
//		for(int i=0;i< 5;i++){
//			addrList.add("aaaa::" + (i + 1));
//		}
//		ParamConfigProxy proxy = new ParamConfigProxy("127.0.0.1", new ProxyInvoke() {
//
//			@Override
//			public void invoke(String addr, int port, byte[] message) {
//				// TODO 自动生成的方法存根
//				System.out.println("发送配置信息  to " + addr +":" + port);
//			}
//		});
//		//proxy.ucastConfig("hello".getBytes(), addrList,5656);
//		proxy.mcastConfig("hello".getBytes(), addrList,5678,6565);
//		//proxy.ucastConfigSingleNode("hello".getBytes(), "aaaa::1", 5678);
//		proxy.registerConfigResultHandler(new ParamConfigResult() {
//			
//			@Override
//			public void actionPerformed(List<String> failIpList) {
//				// TODO 自动生成的方法存根
//				if(failIpList == null){
//					JOptionPane.showMessageDialog(null, "配置成功！", "警告",
//							JOptionPane.OK_OPTION);
//					return;
//				}
//				StringBuilder sb = new StringBuilder();
//				sb.append("节点参数配置失败，配置失败节点ip为\r\n");
//				for(String ip : failIpList){
//					sb.append("  " + ip + "\r\n");
//				}
//				JOptionPane.showMessageDialog(null, sb.toString(), "警告",
//						JOptionPane.OK_OPTION);
//				sb = null;
//				
//			}
//		});
//		Thread.sleep(4000);
//		proxy.setConfigAck("aaaa::2");
//		//Thread.sleep(3000);
//		//proxy.ucastConfigSingleNode("hello".getBytes(), "aaaa::1", 5678);
//		//proxy.setConfigAck("aaaa::2");
//	}
}
