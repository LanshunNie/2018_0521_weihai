package com.hit.heat.model;

import java.util.ArrayList;
import java.util.List;

import org.json.JSONException;
import org.json.JSONObject;

import com.hit.heat.util.Util;



/* @lhy Lhy
 * @date 2015年11月19日
 * @des  
 */
public class Command {

	private byte[] cmd_content;
	private byte type;
	private List<Integer> nodeList;
	
	private Command setContent(byte[] content){
		this.cmd_content = content;
		return this;
	}
	
	private Command setType(byte type){
		this.type = type;
		return this;
	}
	
	private Command setNodeList(List<Integer> nodeList){
		this.nodeList = nodeList;
		return this;
	}
	
	public byte getType(){
		return type;
	}
	
	public byte[] getContent(){
		return cmd_content;
	}
	
	public List<Integer> getNodeList(){
		return nodeList;
	}
	/**
	 * 
	 * @des 将json格式的字符串解析出来
	 * @param content
	 * @return
	 * @throws JSONException
	 */
	public static Command parseCmdFromStream(String content) throws JSONException{
	//	System.out.println(new String(content));
		JSONObject json = new JSONObject(content);
		String cmd_bytes = json.getString("cont"); //cmd内容
		String[] splits = cmd_bytes.split(",");
		byte[] cmd_content = new byte[splits.length];
		for(int i=0;i < splits.length;i++){
			//System.out.println(splits[i]);
			cmd_content[i] = (byte)Integer.parseInt(splits[i]);//将整数形式的字节cmd转换成byte数组cmd_content
			//System.out.println(cmd_content[i]);
		}
		byte type = (byte)json.getString("type").charAt(0);
		List<Integer> list = new ArrayList<Integer>();//解析单播消息中需要发送命令的地址列表
		if(type == 'u'){//类型为单播
			splits = json.getString("list").split(",");
			for(String sp : splits){
				list.add(Integer.parseInt(sp));//存储地址列表
			}
		}
		return new Command().setContent(cmd_content).setType(type).setNodeList(list);//返回cmd+type+addrlist的command对象
	}
	
	public static JSONObject jsonFromStr(String str) throws JSONException{
		JSONObject json = new JSONObject(str);
		System.out.println(json.toString());
		return json;
	}
	public static Command parseCmdFromStream(byte [] content){
		//System.out.println(new String(content));
		byte cmd_length = content[0];
		byte[] cmd_content = new byte[cmd_length];
		System.arraycopy(content, 1, cmd_content, 0, cmd_length);
		byte type = content[cmd_length + 1];
		List<Integer> list = new ArrayList<Integer>();
		
		if(type == (byte)'u'){
			for(int i= cmd_length + 2; i < content.length;i++){
				list.add((int) content[i]);
			}
		}
		return new Command().setContent(cmd_content).setType(type).setNodeList(list);
	}
	
	@Override
	public String toString() {
		// TODO 自动生成的方法存根
		return "指令长度 ： " + String.valueOf(cmd_content.length) + "\r\n" +
				"指令内容： " + Util.formatBytesToStr(cmd_content)+ "\r\n" +
				"后缀：" + String.valueOf((char)type) + "\r\n" +
				"Ip列表： " + nodeList.toString();
	}

	
//	public static void main(String[] args) throws JSONException {
//		
//		//String dest = "{\"cont\":\"16,91,21,89,22\",\"type\":\"u\",\"list\":\"2,255\"}";
//		String dest = "{\"cont\":\"100\",\"type\":\"u\",\"list\":\"2,255\"}";
//		Command command = parseCmdFromStream(dest);
//		System.out.println(command.toString());
//		//String cmd ={"type":"cmd","data":"da"};
//		//String m =" {"pama_data":"60","type":"pama_send"}";
//		
//		
//	}
}
