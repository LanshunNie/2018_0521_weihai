package com.hit.heat.control;

import java.io.BufferedReader;
import java.io.InputStreamReader;


/* @lhy Lhy
 * @date 2015年11月15日
 * @des  
 */
public class TunslipExecutor {
	private String encode;
	private MessageHandler handler;
	private String subnetPerfix;
	private boolean runTag = true;
	public TunslipExecutor(String subnetPerfix,MessageHandler hanler,String encode) {
		this.subnetPerfix = subnetPerfix;
		this.handler = hanler;
		this.encode = encode;
	}

	public TunslipExecutor(String subnetPerfix,MessageHandler hanler){
		this(subnetPerfix, hanler, "utf8");
	}
	 
	
	public void executor() {
		Thread thread = new Thread(new TunslipExecutorThread());
		thread.start();
		runTag= true;
	}
	public void stop() {
		runTag= false;
	}
	class TunslipExecutorThread implements Runnable {

		@Override
		public void run() {
			if(runTag){
			// TODO 自动生成的方法存根
			try {
				String []cmd ={"/bin/sh","-c","./tunslip6 -B9600 " + subnetPerfix +"::1/64"};
				Process pro = Runtime.getRuntime().exec(cmd);
				//Process pro = Runtime.getRuntime().exec("sudo ./tunslip6 -B9600" + subnetPerfix +"::1/64");
				BufferedReader buf = new BufferedReader(new InputStreamReader(
						pro.getInputStream(), encode));
				
				String message;
				while ((message = buf.readLine()) != null) {
					handler.messageHandler(message);
				}
			} catch (Exception ex) {
				System.out.println(ex.getMessage());
			}
		}
		}
	}
	public void close(){
		Runtime.getRuntime().exit(0);
	}
//	public static void main(String[] args) {
//		TunslipExecutor tunslipExecutor = new TunslipExecutor("aaaa", new MessageHandler(){
//
//			@Override
//			public void messageHandler(String message) {
//				// TODO 自动生成的方法存根
//				System.out.println(message);
//			}
//		}, "GBK");
//		tunslipExecutor.executor();
//	}
//	
}
