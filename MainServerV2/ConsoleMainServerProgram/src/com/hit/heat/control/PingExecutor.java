/*PingExecThread.java
 * 2015年8月3日
 * Lhy
*/
package com.hit.heat.control;

import java.io.BufferedReader;
import java.io.InputStreamReader;

/**
 * @lhy Lhy
 *
 */
public class PingExecutor {
	
	private String encode;
	private MessageHandler handler;
	private String addr;
	private boolean runTag = true;
	public PingExecutor(String addr,MessageHandler hanler,String encode) {
		this.addr = addr;
		this.handler = hanler;
		this.encode = encode;
	}

	public PingExecutor(String addr,MessageHandler hanler){
		this(addr, hanler, "utf8");
	}
	 
	
	public void executor() {
		Thread thread = new Thread(new PingExecutorThread());
		thread.start();
		runTag= true;
	}
	public void stop() {
		runTag= false;
	}
	class PingExecutorThread implements Runnable {

		@Override
		public void run() {
			if(runTag){
			// TODO 自动生成的方法存根
			try {
				Process pro = Runtime.getRuntime().exec("ping " + addr +" -n 4");//windows下-n ubuntu下-c
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
//	public static void main(String[] args) {
//		PingExecutor pingExecutor = new PingExecutor("192.168.1.141", new MessageHandler(){
//
//			@Override
//			public void messageHandler(String message) {
//				// TODO 自动生成的方法存根
////				Console console =  System.console();
////				System.out.println(console.readLine());
//				
//				System.out.println(message);
//				
//			}
//		}, "gbk");
//		pingExecutor.executor();
//	}
}
