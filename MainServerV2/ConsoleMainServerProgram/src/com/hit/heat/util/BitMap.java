package com.hit.heat.util;

import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
//import java.util.Spliterator;
import java.util.regex.Pattern;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import com.sun.org.apache.xerces.internal.impl.xs.identity.Selector.Matcher;

/**
 * @lhy Lhy 
 * @time	  于2015年10月2日下午7:58:39
 *
 * @description 统计所有节点在一个上报周期的上报情况，使用bitmap思想，初始位为1，节点上报后置位0
 *				假设节点编号（对应节点IP地址的最后一个字节）由1开始，最大节点数为255个，总共需要32个字节
 *				在实验中，节点IP地址不连续，需要手动设置节点总数以统计上报率
 *				在一个上报周期，需要缓存节点上报的数据，使用HashMap存储，key为节点IP地址，value为上报内容（字符串存储）
 **/

public class BitMap {

	//最大节点数(节点编号由1开始)
	private final int MAX_NODE_SIZE = 255;
	//实际节点总数
	private int TOTAL_NODE_SIZE = 12;
	//bitmap
	private byte[] bitMap;
	//已经上报的节点总数
	private int total_node;
	//缓存HashMap
	private HashMap<String, String> dataHashMap;
	public BitMap(){
		bitMap = new byte[getBitMapLength(MAX_NODE_SIZE)];
		dataHashMap = new HashMap<String, String>();
		initParam();
	}

	/**
	* @description 初始化参数，设置已经上报的节点数为0，设置所有的bit为1
	*/
	public void initParam(){
		total_node = 0;
		for(int i = 0;i < bitMap.length;i++){
			bitMap[i] = (byte) 255;
		}
	}
	
	/**
	*@description 根据节点总数获取bit字节的个数
	*/
	private int getBitMapLength(int nodeSize){
		nodeSize = (nodeSize & 0x07) == 0 ? nodeSize : nodeSize + 8;
		return nodeSize / 8;
	}
	
	/**
	 * 
	 * @description 根据节点IP地址最后一个字节设置相应的bit位，并缓存节点上报的内容
	 * @param addr IP地址，为16进制的字符串（如aaaa::1）
	 * @param content节点上报的内容
	 *
	 */
	public void setBit(String addr,String content){
		if(addr == null || addr.length() < 2){
			return;
		}
		String lastSec = addr.trim().substring(addr.lastIndexOf(":") + 1);
		int nodeId = -1;
		//获取IP地址最后一个字节转换为十进制数即为节点编号
		if(lastSec.length() > 2){
			nodeId = Integer.parseInt(lastSec.trim().substring(lastSec.length() -2), 16);
		}else{
			nodeId = Integer.parseInt(lastSec,16);
		}
		//节点编号为大于1的整数
		if (nodeId < 1) {
			return;
		}
		nodeId--;
		//节点编号除以8即为所在字节位置
		int byte_offset = nodeId >> 3;
		//节点编号模8即为对应的bit位
		int bit_offset = nodeId & 0x07;
		//对应的bit为0，其余全为1
		byte bit = (byte)(~(0x01 << bit_offset));
		//位与运算即设置对应位为0
		bitMap[byte_offset] &= bit;
		dataHashMap.put(addr, content);
		total_node++;
	}
	
	
	public void setBit(String addr,byte[] content){
		if(addr == null || addr.length() < 2){
			return;
		}
		String lastSec = addr.trim().substring(addr.lastIndexOf(":") + 1);
		int nodeId = -1;
		//获取IP地址最后一个字节转换为十进制数即为节点编号
		if(lastSec.length() > 2){
			nodeId = Integer.parseInt(lastSec.trim().substring(lastSec.length() -2), 16);
		}else{
			nodeId = Integer.parseInt(lastSec,16);
		}
		//节点编号为大于1的整数
//		if (nodeId < 1) {
//			return;
//		}
//		nodeId--;
		//节点编号除以8即为所在字节位置
		int byte_offset = nodeId >> 3;
		//节点编号模8即为对应的bit位
		int bit_offset = nodeId & 0x07;
		//对应的bit为0，其余全为1
		byte bit = (byte)(~(0x01 << bit_offset));
		//位与运算即设置对应位为0
		bitMap[byte_offset] &= bit;
		total_node++;
		dataHashMap.put(addr, Util.formatByteToByteStr(content));
	}
	
	
	/**
	 * 
	 * @des	获取所有数据的json形式的字节数组
	 * @return
	 * @throws JSONException
	 */
	public String getTotalUploadData() throws JSONException{
		JSONArray json = new JSONArray();
		JSONObject obj;
		//防止多线程访问冲突，进行线程同步
		synchronized (dataHashMap) {
			//数据保存至JSON数组中
			for(String addr : dataHashMap.keySet()){
				obj = new JSONObject();
				obj.put("addr", addr);
				obj.put("data", dataHashMap.get(addr));
				json.put(obj);
			}
			dataHashMap.clear();
		}
		return json.toString();
	}
	
	/**
	 * 
	 * @des	获取所有数据的json形式的字节数组
	 * @return
	 * @throws JSONException
	 */
	public String getTotalUploadData(String str) throws JSONException{
		JSONArray json = new JSONArray();
		JSONObject obj;
		//防止多线程访问冲突，进行线程同步
		synchronized (dataHashMap) {
			//数据保存至JSON数组中
			for(String addr : dataHashMap.keySet()){
				obj = new JSONObject();
				obj.put("type", str);
				obj.put("addr", addr);
				obj.put("data", dataHashMap.get(addr));
				json.put(obj);
			}
			dataHashMap.clear();
		}
		return json.toString();
	}
	/**
	* @description 获取当前节点上报率 = 实际上报数 / 实际节点数 （结果保留四位有效数字）
	*/
	public double getUploadRate(){
		NumberFormat nbf = NumberFormat.getInstance();
		nbf.setMinimumFractionDigits(4);
		return Double.parseDouble(nbf.format(1.0 * total_node / TOTAL_NODE_SIZE));
	}
	
	/**
	*	@description 根据节点IP地址判断节点是否需要上报，根据地址最后一个字节表示的节点编号
	*/
	public boolean reUpload(String addr){
		if(addr == null || addr.length() < 2){
			return false;
		}
		String lastSec = addr.trim().substring(addr.lastIndexOf(":") + 1);
		int nodeId = -1;
		if(lastSec.length() > 2){
			nodeId = Integer.parseInt(lastSec.trim().substring(lastSec.length() -2), 16);
		}else{
			nodeId = Integer.parseInt(lastSec,16);
		}
		return reUpload(nodeId);
	}
	
	/**
	*	@description 根据节点编号查看对应位置bit位，若为0，则不需要重传，否则需要重传
	*/
	public boolean reUpload(int nodeId){
		if(nodeId < 0 || nodeId > MAX_NODE_SIZE){
			return false;
		}
		nodeId--;
		int byte_offset = nodeId >> 3;
		int bit_offset = nodeId & 0x07;
		//对应位为1，其余位为0
		byte bit = (byte)(0x01 << bit_offset);
		//位与运算，若为0，则结果为0，否则不为0
		bit &=  bitMap[byte_offset];
		return bit == 0 ? false : true;
	}
	
	//返回bitmap数组
	public byte[] getBitMap(){
		return bitMap;
	}
	
	/**
	 * 
	 * @des 设置当前需要上报的节点总数 
	 * @param size
	 */
	public void setTotalNodeSize(int size){
		TOTAL_NODE_SIZE = size;
	}
	
	/**
	 * 
	 * @des 设置局部需要上报的节点的bitmap，其他均设置为不需要上报 
	 * @param list
	 */
	public void setPartReUploadList(List<String> list){
		TOTAL_NODE_SIZE = list.size();
		for(int i = 0;i < bitMap.length;i++){
			bitMap[i] = (byte)0;
		}
		//节点编号除以8即为所在字节位置
		int byte_offset;
		//节点编号模8即为对应的bit位
		int bit_offset ;
		//对应的bit为0，其余全为1
		byte bit;
		String lastSec;
		int nodeId = -1;
		for(String addr: list){
			lastSec = addr.trim().substring(addr.lastIndexOf(":") + 1);
			//获取IP地址最后一个字节转换为十进制数即为节点编号
			if(lastSec.length() > 2){
				nodeId = Integer.parseInt(lastSec.trim().substring(lastSec.length() -2), 16);
			}else{
				nodeId = Integer.parseInt(lastSec,16);
			}
//			nodeId--;
			byte_offset = nodeId >> 3;
			bit_offset = nodeId & 0x07;
			bit = (byte)(0x01 << bit_offset);
			bitMap[byte_offset] |= bit;
		}
	}
	
	/**
	 * 
	 * @des 设置局部上报的节点的地址列表，并将其转换成字节形式的地址列表，下发给根节点。
	 * 每个地址长度固定8字节，根节点实现单点控制的地址列表下行
	 * @param list
	 */
	public byte[] setPartUploadAddrList(List<String> list){
		if(list==null){
			return null;
		}
		TOTAL_NODE_SIZE = list.size();
		ArrayList<Byte> al=new ArrayList<Byte>();
		byte[] addrlist =null;
		//每个地址的长度固定8字节
		String regEx="^([a-fA-F0-9][a-fA-F0-9][:]){7}[a-fA-F0-9][a-fA-F0-9]$";
		Pattern pattern= Pattern.compile(regEx);
		
		for(String addr:list){
			//正则表达式判定addr xx:xx:xx:xx:xx:xx:xx:xx
			java.util.regex.Matcher matcher = pattern.matcher(addr);
			if(!matcher.matches()){//addr xx:xx:xx:xx:xx:xx:xx:xx
				System.err.println("error unicast addr input format");
				continue;
			}
			for(int i=0;i<addr.length()-1;i++){
				
				char a = addr.charAt(i);
				char b = addr.charAt(i+1);
				
				if(a!=':'&&b!=':'){
					int high =0;
					int low = 0;
					//get high 4 bits
					if(a>='0'&&a<='9'){
						high = a-'0';
					}else if(a>='A'&&a<='F'){
						high = a-'A'+0x0A;
					}
					//get low 4 bits
					if(b>='0'&&b<='9'){
						low = b-'0';
					}else if(b>='A'&&b<='F'){
						low = b-'A'+0x0A;
					}	
					
					byte ch=(byte)(high*16+low);
					al.add(ch);//add right address
				}
			
			}
		}
		addrlist = new byte[al.size()];
		for(int i=0;i<al.size();i++){
			addrlist[i]=al.get(i);
		}
		
		al = null;
		return addrlist;
	}
//	public static void main(String[] args) throws JSONException {
//		BitMap bitMap = new BitMap();
//		byte[] bytes = new byte[6];
//		bytes[0] = (byte)0x01;
//		bytes[1] = (byte)0x5B;
//		bytes[2] = (byte)0x10;
//		bytes[3] = (byte)0xFE;
//		bytes[4] = (byte)0xFF;
//		bytes[5] = (byte)'o';
//		bitMap.setBit("aaaa:0:0:0:12:7400:1:28",bytes);
//		bitMap.setBit("aaaa:0:0:0:12:7400:1:12","hello world2");
//		bitMap.setBit("aaaa:0:0:0:12:7400:1:11","hello world3");
//		
//		String content = bitMap.getTotalUploadData();
//		System.out.println(content);
//		//bitMap.setBit("aaaa:0:0:0:12:7400:1:29");
//		//bitMap.setBit("aaaa:0:0:0:12:7400:1:30");
//		
////		for(int i=1;i< 255;i+=2){
////			bitMap.setBit(i);
////		}
////		List<Integer> list = new ArrayList<Integer>();
////		list.add(2);
////		list.add(4);
////		list.add(5);
////		list.add(6);
////		bitMap.setPartReUploadList(list);
////		for(int i = 1; i < 255;i++){
////			if(!bitMap.reUpload(i)){
////				System.out.println("节点" + i + "不需要重传");
////			}
////		}
////		byte[] bitmap = bitMap.getBitMap();
////		System.out.println(new String(bitmap));
////		System.out.println(bitMap.getUploadRate());
////		JSONArray jsonArray = new JSONArray(content);
////		JSONObject obj;
////		Object data;
////		for(int i=0;i< jsonArray.length();i++){
////			obj = jsonArray.getJSONObject(i);
////			data = obj.get("data");
////			System.out.println("addr :" + obj.getString("addr") + " data : " + data);
////			System.out.println(Util.formatBytesToStr(data.toString().getBytes()));
////		}
//		//byte [] byte1 = {(byte) 0xF1,(byte) 0xFF, (byte)0xF0};
//		//System.out.println(Util.formatBytesToStr(byte1));
//	}
}
