
package com.hit.heat.util;

import java.awt.Color;

import java.awt.Dimension;
import java.awt.Font;
import java.awt.Point;
import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.ByteBuffer;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Timer;
import java.util.TimerTask;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import javax.swing.JLabel;
import javax.swing.JPanel;

import com.hit.heat.model.Energy;
import com.hit.heat.model.GlobalDefines;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
import com.hit.heat.model.Command;
import com.hit.heat.model.CurrentTime;
import com.hit.heat.model.FloorInfor;
import com.hit.heat.model.Location;
import com.hit.heat.model.NetParameter;
import com.hit.heat.model.Node;
import com.hit.heat.model.SynParameter;
import com.hit.heat.model.SystemParam;
import com.hit.heat.net.NettyClient;

/**
 * @lhy Lhy
 *
 */
public class Util {
	/**
	 *
	 * @description 设置JLabel的属性
	 * @param parent
	 *            父控件
	 * @param comp
	 *            子控件
	 * @param d
	 *            大小
	 * @param p
	 *            位置
	 * @param font
	 *            字体
	 * @param c
	 *            颜色
	 *
	 */
	public static void setLabelProperty(JPanel parent, JLabel comp, Dimension d, Point p, Font font, Color c,
			int alignment) {
		if (comp == null || parent == null) {
			return;
		}
		comp.setSize(d);
		comp.setLocation(p);
		comp.setFont(font);
		comp.setForeground(c);
		comp.setHorizontalAlignment(alignment);
		parent.add(comp);
	}

	/**
	 *
	 * @des
	 * @param parent
	 * @param comp
	 * @param d
	 * @param p
	 * @param font
	 * @param text
	 * @param name
	 */

	public static boolean verifyIPv4(String host) {

		Pattern pattern = Pattern.compile(
				"\\b((?!\\d\\d\\d)\\d+|1\\d\\d|2[0-4]\\d|25[0-5])\\.((?!\\d\\d\\d)\\d+|1\\d\\d|2[0-4]\\d|25[0-5])\\.((?!\\d\\d\\d)\\d+|1\\d\\d|2[0-4]\\d|25[0-5])\\.((?!\\d\\d\\d)\\d+|1\\d\\d|2[0-4]\\d|25[0-5])\\b");
		Matcher matcher = pattern.matcher(host);
		return matcher.matches();
	}

	public static boolean verifyIPv6(String host) {

		String std_ipv6 = "^(?:[0-9a-fA-F]{1,4}:){7}[0-9a-fA-F]{1,4}$";
		String comp_ipv6 = "^((?:[0-9A-Fa-f]{1,4}(?::[0-9A-Fa-f]{1,4})*)?)::((?:[0-9A-Fa-f]{1,4}(?::[0-9A-Fa-f]{1,4})*)?)$";
		Pattern pattern = Pattern.compile(std_ipv6);
		Matcher matcher = pattern.matcher(host);
		if (matcher.matches()) {
			return true;
		}
		pattern = Pattern.compile(comp_ipv6);
		matcher = pattern.matcher(host);
		return matcher.matches();
	}

	public static boolean verifyHost(String host) {

		return verifyIPv4(host) | verifyIPv6(host);
	}

	public static boolean verifyPort(String port) {
		try {
			int iPort = Integer.valueOf(port);
			if (iPort < 0 || iPort > 65535) {
				return false;
			}
		} catch (Exception e) {
			// TODO: handle exception
			return false;
		}
		return true;
	}

	public static String formatTime(long secs) {
		int hour;
		int minute;
		int second;
		hour = (int) secs / 3600;
		secs = secs % 3600;
		minute = (int) secs / 60;
		second = (int) secs % 60;
		return String.format("%02d:%02d:%02d", hour, minute, second);
	}

	public static String getCurrentTime() {
		return new SimpleDateFormat("yyyy-MM-dd HH:mm:ss").format(new Date());
	}

	public static String getCurrentDateTime() {
		return new SimpleDateFormat("HH:mm:ss").format(new Date());
	}

	public static Long getUtcTime() {
		return new Date().getTime();
	}

	public static CurrentTime getCurrentDateTime(String date) {
		if (date == null || date.equals("")) {
			return null;
		}
		String[] splits = date.split(":");
		if (splits.length != 3) {
			return null;
		}
		return new CurrentTime(Integer.parseInt(splits[0]), Integer.parseInt(splits[1]), Integer.parseInt(splits[2]));
	}

	public static String getCurrentTime(String format) {
		if (format == null) {
			return new SimpleDateFormat("yyyy-MM-dd HH:mm:ss").format(new Date());
		}
		return new SimpleDateFormat(format).format(new Date());
	}

	public static boolean messageIsJSONType(String message) {
		if (message == null || message.length() < 2) {
			return false;
		}
		if (message.charAt(0) == '{' && message.charAt(message.length() - 1) == '}') {
			return true;
		}
		return false;
	}

	public static int Ceiling(int x, int y) {
		if (x % y == 0) {
			return x / y;
		}
		return x / y + 1;
	}

	public static List<String> parseIpListFromFile(String path) throws IOException {
		List<String> list = new ArrayList<String>();
		File file = new File(path);
		FileReader fileReader = new FileReader(file);
		BufferedReader reader = new BufferedReader(fileReader);
		String line;
		while ((line = reader.readLine()) != null) {
			list.add(line);
		}
		fileReader.close();
		reader.close();
		return list;
	}

	public static Map<Integer, Location> parseLocationsFromFile(String path) throws IOException {
		Map<Integer, Location> locations = new HashMap<Integer, Location>();
		// String parentPath = GetPath.getProjectPath();
		// System.out.println((parentPath+"\\"+path));
		// File file = new File(parentPath+"/"+path);
		File file = new File(path);
		FileReader fileReader = new FileReader(file);
		BufferedReader bufferedReader = new BufferedReader(fileReader);
		String line;
		Location loc;
		while ((line = bufferedReader.readLine()) != null) {
			if ((loc = parseLocationFromLine(line)) != null) {
				locations.put(loc.getId(), loc);
			}
		}
		fileReader.close();
		bufferedReader.close();
		return locations;
	}

	public static void writeLocationToFile(List<Location> locations, String path) throws IOException {
		File file = new File(path);
		FileWriter fileWriter = new FileWriter(file);
		BufferedWriter bufferedWriter = new BufferedWriter(fileWriter);
		int i;
		for (i = 0; i < locations.size() - 1; i++) {
			bufferedWriter.write(locations.get(i).toFormatString() + "\r\n");
		}
		bufferedWriter.write(locations.get(i).toFormatString());
		bufferedWriter.flush();
		fileWriter.flush();
		bufferedWriter.close();
		fileWriter.close();
	}

	private static Location parseLocationFromLine(String line) {
		if (line == null || line.isEmpty()) {
			return null;
		}
		String[] splits = line.split(",");
		if (splits.length != 5) {
			return null;
		}
		return new Location(Integer.parseInt(splits[0]), splits[1], splits[2],
				new Point(Integer.parseInt(splits[3]), Integer.parseInt(splits[4])));
	}

	/**
	 *
	 * @des 将完整的Ip地址转换为缩略形式 ff00:0:0:0:1:2:1:1 to ff00::1:2:1:1
	 * @param ip
	 * @return
	 */
	public static String formatToSimpIpv6(String ip) {
		int index = ip.indexOf("::");
		if (index >= 0) {
			return ip;
		}
		String[] splits = ip.split(":");
		if (splits.length != 8) {
			return null;
		}
		// boolean flag = false;
		StringBuilder sb = new StringBuilder();
		int state = 0;
		for (int i = 0; i < 7; i++) {
			switch (state) {
			case 0:
				if ("0".equals(splits[i])) {
					state = 1;
				} else {
					sb.append(splits[i] + ":");
				}
				break;
			case 1:
				break;
			}
		}
		sb.append(splits[7]);
		return null;
	}

	/**
	 *
	 * @des 将缩略形式的IP地址转换为完整形式
	 * @param ip
	 * @return
	 */
	public static String formatToCompIpv6(String ip) {
		int index = ip.indexOf("::");
		if (index < 0) {
			return ip;
		}
		String leftStr = ip.trim().substring(0, index);
		String rigthStr = ip.trim().substring(index + 2);
		int countOflsp = countChInStr(leftStr, ':');
		int countOfrsp = countChInStr(rigthStr, ':');
		StringBuilder sb = new StringBuilder();
		sb.append(leftStr);
		for (int i = 0; i < (6 - countOflsp - countOfrsp); i++) {
			sb.append(":0");
		}
		sb.append(":" + rigthStr);
		return sb.toString();
	}

	public static boolean ipv6Equals(String ip1, String ip2) {
		String cip1 = formatToCompIpv6(ip1);
		String cip2 = formatToCompIpv6(ip2);
		return cip1.equals(cip2);
	}

	public static int countChInStr(String str, char ch) {
		int count = 0;
		for (int i = 0; i < str.length(); i++) {
			if (str.charAt(i) == ch) {
				count++;
			}
		}
		return count;
	}
	public static void removeFile(String fileName) {
		String cmd = "rm "+fileName;
		//System.out.println(cmd);
		Process commandProcess;
		try {
			commandProcess = Runtime.getRuntime().exec(cmd);
			final BufferedReader input = new BufferedReader(new InputStreamReader(commandProcess.getInputStream()));
			final BufferedReader err = new BufferedReader(new InputStreamReader(commandProcess.getErrorStream()));
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	public static String formatByteToByteStr(byte[] bytes) {
		StringBuilder sb = new StringBuilder(2 * bytes.length);
		for (byte b : bytes) {
			sb.append(String.format("%02X", b));
		}
		return sb.toString();
	}

	public static String formatBytesToStr(byte[] bytes) {
		StringBuilder sb = new StringBuilder();
		int index;
		for (index = 0; index < bytes.length - 1; index++) {
			sb.append(String.format("%d", bytes[index]) + " ");
		}
		sb.append(String.format("%d", bytes[index]));
		return sb.toString();
	}

	// 去掉检测数据中用于去重复的前三个字节
	public static String formatBytesToStrGxn(byte[] bytes) {
		StringBuilder sb = new StringBuilder();
		int index;
		for (index = 0; index < bytes.length - 1; index++) {
			sb.append(String.format("%d", bytes[index]) + " ");
		}
		sb.append(String.format("%d", bytes[index]));
		return sb.toString().substring(3);
	}

	public static byte[] formatByteStrBitmapToBytes(String st) {
		byte[] result = new byte[18];
		st = (String) st.subSequence(1, st.length() - 1);
		String[] strArray = st.split(",");
		for (int i = 0; i < strArray.length; i++) {
			result[i] = (byte) ((int) Integer.valueOf(strArray[i].trim()));
		}
		return result;
	}

	public static byte[] formatByteStrToByte(String str) {
		int length = str.length();
		// System.out.println(length);
		if (length % 2 != 0 || str.charAt(0) == '{') {
			return null;
		}
		byte[] bytes = new byte[length / 2];
		int hitBit;
		int lowBit;
		for (int i = 0; i < str.length(); i += 2) {
			hitBit = Integer.valueOf(str.charAt(i) + "", 16);
			lowBit = Integer.valueOf(str.charAt(i + 1) + "", 16);
			bytes[i / 2] = (byte) (hitBit * 16 + lowBit);
		}
		return bytes;
	}

	public static byte[] formatNetStrToByte(String str) {
		String[] buf = str.split(" ");
		int i = buf.length;
		// System.out.println(i);
		byte[] by = new byte[i];
		for (int j = 0; j < i; j++) {
			by[j] = Byte.parseByte(buf[j]);
		}
		return by;
	}
	
	public static byte[] formatStringToByte(String str){
		if(str!=null){
			byte[] by=new byte[str.length()];
			for(int i=0;i<str.length();i++){
				
			}
			
		}
		return null;
	}
	public static int RandomInt(int n) {
		Random random = new Random();
		return random.nextInt(n);
	}

	public static boolean fileExist(String path) {
		File file = new File(path);
		if (file.isFile() && file.exists()) {
			return true;
		}
		return false;
	}

	/**
	 *
	 * @des 从文件中读取json格式的数据，解析成楼层链表
	 * @param path
	 * @return
	 * @throws IOException
	 * @throws JSONException
	 */
	public static List<FloorInfor> readRoomInforsForFile(String path) throws IOException, JSONException {
		List<FloorInfor> floorInfors = new ArrayList<FloorInfor>();
		StringBuilder sb = new StringBuilder(1024 * 32);
		FileInputStream fileInputStream = new FileInputStream(new File(path));
		BufferedInputStream bufferedInputStream = new BufferedInputStream(fileInputStream);
		byte[] buffer = new byte[1024];
		int readLen;
		while ((readLen = bufferedInputStream.read(buffer, 0, 1024)) > 0) {
			byte[] read = new byte[readLen];
			System.arraycopy(buffer, 0, read, 0, readLen);
			sb.append(new String(read));
		}
		bufferedInputStream.close();
		fileInputStream.close();
		JSONArray array = new JSONArray(sb.toString());
		for (int i = 0; i < array.length(); ++i) {
			floorInfors.add(FloorInfor.fromJsonObject(array.getJSONObject(i)));
		}
		return floorInfors;
	}

	/**
	 *
	 * @des 将楼层信息以json的形式写入到文件中
	 * @param path
	 * @param floors
	 * @throws IOException
	 * @throws JSONException
	 */
	public static void writeRoomInforToFile(String path, List<FloorInfor> floors) throws IOException, JSONException {
		BufferedWriter bufferedWriter = new BufferedWriter(new FileWriter(new File(path)));
		JSONArray floorArray = new JSONArray();
		for (FloorInfor floorInfor : floors) {
			floorArray.put(floorInfor.toJsonObject());
		}
		bufferedWriter.write(floorArray.toString());
		bufferedWriter.flush();
		bufferedWriter.close();
	}

	/***
	 *
	 * @des 将同步消息以json的形式写入文件中
	 * @param message
	 * @return
	 * @throws JSONException
	 */
	public static void writeSynConfigParamToFile(SynParameter synParameter, String path) {

		// System.out.println("000000000000000000");

		JSONObject object = new JSONObject();
		try {

			object.put("seqNum", synParameter.getSeqNum());
			object.put("level", synParameter.getLevel());
			object.put("hour", synParameter.getHour());
			object.put("minute", synParameter.getMinute());
			object.put("second", synParameter.getSecond());
			object.put("period", synParameter.getPeriod());

			object.put("bitmap", synParameter.getBitmap());
			object.put("state", synParameter.isFlag());

			try {
				BufferedWriter bufferedWriter;
				bufferedWriter = new BufferedWriter(new FileWriter(new File(path)));
				bufferedWriter.write(object.toString());
				bufferedWriter.flush();
				bufferedWriter.close();
				// System.out.println("aaaaaaaaaaaaaaaaaaaaaaa");
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

			// System.out.println("ffffffffffffffffff");
		} catch (JSONException e) {
			// TODO 自动生成的 catch 块
			// System.out.println("bbbbbbbbbbbbbbbbbbbb");
			e.printStackTrace();
		}
	}

	
	public static void writeConfigParamToFile(NetParameter parameter, String path) {

		// System.out.println("000000000000000000");

		JSONObject object = new JSONObject();
		try {
//			"id"
			object.put("id", parameter.getId());
//			"HeartIntSec"
			object.put("HeartIntSec", parameter.getHeartIntSec());
//			"AckHeartInt"
			object.put("AckHeartInt", parameter.getAckHeartInt());
//			"MaxAckFail"
			object.put("MaxAckFail", parameter.getMaxAckFail());
//			"tcpAddr"
			object.put("tcpAddr", parameter.getTcpAddr());
//			"tcpPort"
			object.put("tcpPort", parameter.getTcpPort());
//			"tcpByWebPort"
			object.put("tcpByWebPort", parameter.getTcpByWebPort());
//			"tcpRemoteConfigPort"
			object.put("tcpRemoteConfigPort", parameter.getTcpRemoteConfigPort());
//			"udpAddr"
			object.put("udpAddr", parameter.getUdpAddr());
//			"udpPort"
			object.put("udpPort", parameter.getUdpPort());
//			"rootAddr"
			object.put("rootAddr", parameter.getRootAddr());
//			"rootPort"
			object.put("rootPort", parameter.getRootPort());
//			"rootRoomId"
			object.put("rootRoomId", parameter.getRootRoomId());
//			"rootX"
			object.put("rootX", parameter.getRootX());
//			"rootY"
			object.put("rootY", parameter.getRootY());
//			"tcpWebServerAddr"
			object.put("tcpWebServerAddr", parameter.getTcpWebServerAddr());
//			"tcpWebServerPort"
			object.put("tcpWebServerPort", parameter.getTcpWebServerPort());
//			"remoteAddr"
			object.put("remoteAddr", parameter.getRemoteAddr());
//			"remotePort"
			object.put("remotePort", parameter.getRemotePort());
//			"netPort"
			object.put("netPort", parameter.getNetPort());
//			"dayLength"
			object.put("dayLength", parameter.getdayLength());
//			"appSendLength"
			object.put("appSendLength", parameter.getappSendLength());
//			"upperAddr"
			object.put("upperAddr", parameter.getupperAddr());
//			"upperPort"
			object.put("upperPort", parameter.getupperPort());
//			"ftpuser"     
			object.put("ftpuser", parameter.getftpuser());
//		    "ftphost"
			object.put("ftphost", parameter.getftphost());
//			"ftpPwd":"xiaoming",
			object.put("ftpPwd", parameter.getftpPwd());
//			"ftpPort":21,
			object.put("ftpPort", parameter.getftpPort());
//			"serverIp":"127.0.0.1"
			object.put("serverIp", parameter.getserverIp());

			try {
				BufferedWriter bufferedWriter;
				bufferedWriter = new BufferedWriter(new FileWriter(new File(path)));
				bufferedWriter.write(object.toString());
				bufferedWriter.flush();
				bufferedWriter.close();
				// System.out.println("aaaaaaaaaaaaaaaaaaaaaaa");
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}

			// System.out.println("ffffffffffffffffff");
		} catch (JSONException e) {
			// TODO 自动生成的 catch 块
			// System.out.println("bbbbbbbbbbbbbbbbbbbb");
			e.printStackTrace();
		}
	}
	public static Object parseMetaData(String message) throws JSONException {
		if (message == null) {
			return null;
		}
		// System.out.println("gg"+message);
		JSONObject json = new JSONObject(message);
		// Object arrayData = json.get("pama_data");
		String type = json.getString("type");
		switch (type) {
		case "cmd":
			// System.out.println(json.getString("data"));
			return Command.parseCmdFromStream(json.getString("data"));
		case "sync":
			JSONArray array = json.getJSONArray("data");
			List<Location> ret = new ArrayList<Location>();
			for (int i = 0; i < array.length(); ++i) {
				// System.out.println(array.get(i).toString());
				ret.add(Location.parseFromJsonObject(array.getJSONObject(i), (i + 1)));
			}
			return ret;
		case "data":
			// 将不在线时期的数据重新上报一下

			return new Integer(1);
		case "pama_send":
			// System.out.println(json.getJSONObject("pama_data"));
			// JSONObject msgData = new
			// JSONObject(json.getJSONObject("pama_data"));
			// System.out.println(msgData.toString());
			// System.out.println(new JSONObject(msgData.get("type")));
			// System.out.println();
			// System.out.println(json.get("pama_data").toString());
			// return json.getString("pama_data");
			return jsonFromStr(json.getString("pama_data"));
		case "root_send":
			return jsonFromStr(json.getString("pama_data"));
		case "ping_ok":
			return jsonFromStr(json.getString("pama_data"));
		case "test_send":
			return jsonFromStr(json.getString("pama_data"));
		case "cmd_send":
		case "cmd_mcast":
			return jsonFromStr(json.getString("pama_data"));
		// 配置上报周期 而不再是心跳周期
		case "config_heart":
			return jsonFromStr(json.getString("pama_data"));
		case "schedule":
			return jsonFromStr(json.getString("pama_data"));
		case "pama_corr":
			return jsonFromStr(json.getString("pama_data"));
		case "pama_syn":
			return jsonFromStr(json.getString("pama_data"));
		// 添加节点重启，初始化，系统监测，网络监测和网络参数配置
		case "reboot_send":
			return jsonFromStr(json.getString("pama_data"));
		case "reboot_macst":
			return jsonFromStr(json.getString("pama_data"));
		case "restart_send":
			return jsonFromStr(json.getString("pama_data"));
		case "restart_macst":
			return jsonFromStr(json.getString("pama_data"));
		case "reportSys":
			return jsonFromStr(json.getString("pama_data"));
		case "reportNet":
			return jsonFromStr(json.getString("pama_data"));
		case "sendNet":
			return jsonFromStr(json.getString("pama_data"));
		case "mcast":
			return jsonFromStr(json.getString("pama_data"));
		case "unicast":
			return jsonFromStr(json.getString("pama_data"));
		case "mcast_ack":
			return jsonFromStr(json.getString("pama_data"));
		case "unicast_ack":
			return jsonFromStr(json.getString("pama_data"));
		default:
			return null;
		}
	}

	public static JSONObject jsonFromStr(String str) throws JSONException {
		JSONObject json = new JSONObject(str);
		// System.out.println(json.toString());
		return json;
	}

	public static boolean verifyIntegerStr(String str) {
		if (str.isEmpty()) {
			return false;
		}
		Pattern pattern = Pattern.compile("[0-9]*");
		Matcher m = pattern.matcher(str);
		return m.matches();
	}

	public static String getIpv6LastByte(String ipv6) {
		String lastSector = ipv6.trim().substring(ipv6.lastIndexOf(":") + 1);
		// System.out.println(lastSector);
		if (lastSector.length() >= 2) {
			return lastSector.substring(lastSector.length() - 2);
		}
		return "0" + lastSector;
	}

	public static String getIpv6LastByte2(String ipv6) {
		String lastSector = ipv6.trim().substring(ipv6.lastIndexOf(":") + 1);
		// System.out.println(lastSector);
		if (lastSector.length() >= 2) {
			return lastSector.substring(lastSector.length() - 2);
		}
		return lastSector;
	}

	public static int byteToInt(byte high, byte low) {
		int highBit = high;
		highBit = highBit < 0 ? highBit + 256 : highBit;
		int lowBit = low;
		lowBit = lowBit < 0 ? lowBit + 256 : lowBit;
		return (highBit << 8) + lowBit;
	}

	public static int byteToInt(byte b) {
		int i = b;
		if (i < 0) {
			return i + 256;
		}
		return i;
	}

	public static String intToHexStr(int i) {
		return String.format("%02x", i);
	}

	public static boolean verifyDoubleStr(String str) {
		if (str.isEmpty()) {
			return false;
		}
		Pattern pattern = Pattern.compile("\\+{0,1}[0]\\.[1-9]*|\\+{0,1}[1-9]\\d*\\.\\d*");
		Matcher m = pattern.matcher(str);
		return m.matches();
	}

	/**
	 * @des 绘制网络拓扑结构时将节点以（key=节点id，value=节点）形式存入Map中
	 * @param map
	 *            不能为空
	 * @param selfId
	 *            节点自身id
	 * @param parentId
	 *            节点父亲id（如果是br节点，为null即可）
	 */
	public static void updateTopNodesMap(Map<String, Node> map, String selfId, String parentId) {
		if (map == null) {
			throw new RuntimeException("参数map未初始化");
		}
		Node node;
		if (map.containsKey(selfId)) {
			node = map.get(selfId);
			node.setParentId(parentId);
			map.put(selfId, node);
		} else {
			node = new Node(selfId, parentId);
			map.put(selfId, node);
		}
	}

	/********************** 关于同步消息设置保存和读取 ***************************/
	public static void writeSynParamToFile() {

	}

	public static void writeSystemParamToFile(SystemParam systemParam, String path) throws IOException {
		BufferedWriter bufferedWriter = new BufferedWriter(new FileWriter(new File(path)));
		JSONObject object = new JSONObject();
		try {
			object.put("nodeHeartDelay", systemParam.getSysNodeHeartDelay());
			object.put("mcastReliValue", systemParam.getSysMcastReliValue());
			object.put("mcastRetransTimes", systemParam.getSysMcastRetransTimes());
			object.put("nodeTransDelay", systemParam.getSysNodeTransDelay());
			object.put("ucastRetransTimes", systemParam.getSysUcastRetransTimes());
			object.put("nodeWaitAckDelay", systemParam.getSysNodeWaitAckDelay());
			object.put("ucastToMcastValue", systemParam.getSysUcastToMcastValue());
			object.put("waitRetransDelay", systemParam.getSysWaitRetransDelay());
			bufferedWriter.write(object.toString());
			bufferedWriter.flush();
			bufferedWriter.close();
		} catch (Exception e) {
			// TODO: handle exception
		}
	}

	public static SystemParam parseSystemParamFromFile(String path) throws IOException, JSONException {
		StringBuilder sb = new StringBuilder(1024 * 4);
		// String parentPath = GetPath.getProjectPath();
		// System.out.println((parentPath+"\\"+path));
		// File fp = new File(path);
		// FileInputStream fileInputStream = new FileInputStream(new
		// File(parentPath+"/"+path));
		FileInputStream fileInputStream = new FileInputStream(new File(path));
		BufferedInputStream bufferedInputStream = new BufferedInputStream(fileInputStream);
		byte[] buffer = new byte[1024];
		int readLen;
		byte[] read;
		while ((readLen = bufferedInputStream.read(buffer, 0, buffer.length)) > 0) {
			read = new byte[readLen];
			System.arraycopy(buffer, 0, read, 0, readLen);
			sb.append(new String(read));
		}
		bufferedInputStream.close();
		fileInputStream.close();
		JSONObject object = new JSONObject(sb.toString());
		SystemParam systemParam = new SystemParam();
		systemParam.setSysNodeHeartDelay(object.getInt("nodeHeartDelay"));
		systemParam.setSysMcastReliValue(object.getDouble("mcastReliValue"));
		systemParam.setSysMcastRetransTimes(object.getInt("mcastRetransTimes"));
		systemParam.setSysUcastToMcastValue(object.getDouble("ucastToMcastValue"));
		systemParam.setSysNodeTransDelay(object.getInt("nodeTransDelay"));
		systemParam.setSysNodeWaitAckDelay(object.getInt("nodeWaitAckDelay"));
		systemParam.setSysUcastRetransTimes(object.getInt("ucastRetransTimes"));
		systemParam.setSysWaitRetransDelay(object.getInt("waitRetransDelay"));
		return systemParam;
	}

	public static double getDistance(Point p1, Point p2) {
		return p1.distance(p2);
	}

	public static int celling(double f) {
		int integer = (int) f;
		double decimal = f - integer;
		if (decimal == 0) {
			return integer;
		}
		return integer + 1;
	}

	public static Point getPointAwayFromRadius(Point sp, Point ep, int radius) {
		double dis = sp.distance(ep);
		int x = celling(radius / dis * (sp.x - ep.x) + ep.x);
		int y = celling(radius / dis * (sp.y - ep.y) + ep.y);
		return new Point(x, y);
	}

	// 集中器后台与上位机通讯的CCA检验函数
	public static String Check(String message) {
		int i = 0;
		// System.out.println(str.length());
		// for(i = 0;i<2;i++){
		// buff[i] = str.substring(i*2, i+2);
		// }
		// buff[i] = "sl";
		int sum = 0;
		for (i = 0; i < message.length() / 2; i++) {
			int temp = Integer.parseInt(message.substring(2 * i, 2 * i + 2), 16);
			sum += temp;
		}
		sum = sum % 256;// FF
		message = Integer.toHexString(sum).toUpperCase();
		if (message.length() == 1) {
			return "0" + message;
		} else {
			return message;
		}
	}

	public void sendInitHeart(boolean receive_frag, final boolean receiving_frag, final NetParameter parameter,
			final NettyClient remoteClient) {
		int HeartIntSec = parameter.getHeartIntSec();
		if (!receive_frag) {
			Timer initHeart = new Timer();
			initHeart.schedule(new TimerTask() {
				int seq0 = 0;
				int seq1 = 0;

				@Override
				public void run() {
					String id = parameter.getId();
					// int AckHeartInt = parameter.getAckHeartInt();
					// int MaxAckFail = parameter.getMaxAckFail();
					String message = "F800000D";
					message = message.concat(id);
					message = message.concat("0000010000000000");
					String str0 = Integer.toHexString(seq0).toUpperCase();
					if (str0.length() == 1) {
						str0 = "0" + str0;
					}
					message = message.concat(str0);
					String str1 = Integer.toHexString(seq1).toUpperCase();
					if (str1.length() == 1) {
						str1 = "0" + str1;
					}
					message = message.concat(str1);
					message = message.concat(Util.Check(message));
					message = message.concat("16");
					// System.out.println(message);
					remoteClient.asyncWriteAndFlush(message);
					seq0++;
					if (seq0 == 256) {// FF
						seq0 = 0;
						seq1++;
						if (seq1 == 256) {
							seq1 = 0;
						}
					}
				}
			}, 0, 1000 * HeartIntSec);

		} else {
			Timer initHeart = new Timer();
			initHeart.schedule(new TimerTask() {

				int seq0 = 0;
				int seq1 = 0;

				@Override
				public void run() {
					String id = parameter.getId();
					String message = "F800010A";
					message = message.concat(id);
					message = message.concat("00000000");
					String str0 = Integer.toHexString(seq0).toUpperCase();
					if (str0.length() == 1) {
						str0 = "0" + str0;
					}
					message = message.concat(str0);
					String str1 = Integer.toHexString(seq1).toUpperCase();
					if (str1.length() == 1) {
						str1 = "0" + str1;
					}
					message = message.concat(str1);
					message = message.concat(Util.Check(message));
					message = message.concat("16");
					// System.out.println(message);
					if (receiving_frag) {
					} else {
						remoteClient.asyncWriteAndFlush(message);
					}
					seq0++;
					if (seq0 == 256) {// FF
						seq0 = 0;
						seq1++;
						if (seq1 == 256) {
							seq1 = 0;
						}
					}
				}
			}, 0, 1000 * HeartIntSec);
		}
	}

	public static byte[] getSynMessage(int seq, int level, int h, int m, int s, int p, byte[] content) {// 配置同步信息
		// content
		// 为bitmap
		if (content == null || content.length == 0) {
			return null;
		}
		int length = content.length;
		byte[] cmd = new byte[length + 6];
		// cmd[0] = (byte) (length + 6);
		// cmd[1] = GlobalDefines.GlobalCmd.G_SYN_CONFIG_MSG;
		cmd[0] = (byte) seq;
		cmd[1] = (byte) level;
		cmd[2] = (byte) h;
		cmd[3] = (byte) m;
		cmd[4] = (byte) s;
		cmd[5] = (byte) p;

		System.arraycopy(content, 0, cmd, 6, length);
		return cmd;
	}

	public static byte[] getCorrectTimeMessage2(int flag, int h, int m, int s) {// 配置同步信息
																				// content
		// 为bitmap

		byte[] cmd = new byte[4];
		// cmd[0] = (byte) (length + 6);
		// cmd[1] = GlobalDefines.GlobalCmd.G_SYN_CONFIG_MSG;
		cmd[0] = (byte) flag;
		cmd[1] = (byte) h;
		cmd[2] = (byte) m;
		cmd[3] = (byte) s;
		return cmd;
	}

	public static byte[] getCorrectTimeMessage(int h, int m, int s) {// 配置同步信息
																		// content
		// 为bitmap

		byte[] cmd = new byte[3];
		// cmd[0] = (byte) (length + 6);
		// cmd[1] = GlobalDefines.GlobalCmd.G_SYN_CONFIG_MSG;
		cmd[1] = (byte) h;
		cmd[2] = (byte) m;
		cmd[3] = (byte) s;
		return cmd;
	}

	public static byte[] packetMcastSend(byte[] content) {
		if (content == null || content.length == 0) {
			// System.out.print("aaa");
			return null;
		}
		// System.out.print("aaa");
		byte[] cmd = new byte[content.length + 2];
		cmd[0] = (byte) (content.length + 1);
		cmd[1] = (byte) (1); // 1 表示 多播
		System.arraycopy(content, 0, cmd, 2, content.length);
		// System.out.print("bbb");
		return cmd;
	}

	// 局部多播
	public static byte[] packageUnicastSend(byte[] content, byte[] bitmap) {
		// 指令加bitmap
		// 类型2
		if (content == null || content.length == 0) {
			return null;
		}
		if (bitmap == null || bitmap.length == 0) {
			return null;
		}
		
		byte[] cmd = new byte[content.length + bitmap.length + 3];
		cmd[0] = (byte) (content.length + bitmap.length + 2);
		cmd[1] = (byte) (2); // 2 表示 局部单播
		cmd[2] = (byte) content.length;// 指令长度
		System.arraycopy(bitmap, 0, cmd, 3, bitmap.length);
		System.arraycopy(content, 0, cmd, bitmap.length + 3, content.length);
		return cmd;
	}
	/*
	 * @def 单播下行，携带地址列表，携带仪表命令
	 * @param content:仪表命令 addr:地址列表
	 */
	public static byte[] packetUnicastCmd(byte[]content,byte[]addr){
		if (content == null || content.length == 0) {
			return null;
		}
		if (addr == null || addr.length == 0) {
			return null;
		}
		byte[] cmd = new byte[content.length + addr.length + 3];
		cmd[0] = (byte) (content.length + addr.length + 2);
		cmd[1] = (byte) (2); // 2 表示 局部单播
		cmd[2] = (byte) (addr.length/8);// 指令长度
		System.arraycopy(addr, 0, cmd, 3, addr.length);
		System.arraycopy(content, 0, cmd, addr.length + 3, content.length);
		return cmd;
	}

	public String[] addrFilter(String addrList) {
		String[] first_filter = addrList.split(",");
		String[] second_filter;
		String[] result = new String[1];
		if (first_filter.length == 1) {
			second_filter = first_filter[0].split("\"");
			result[0] = second_filter[1];
		} else {

		}
		return result;
	}

	public String parseCmdToStr(String content) {
		String cmd = content.replace(",", "");
		System.out.println("test " + cmd);
		return cmd;
	}

	// 通过CMD获得当前时间系统的IPv4地址
	public String getLocalIPForCMD() {
		StringBuilder sb = new StringBuilder();
		// String command = "cmd.exe /c ipconfig | findstr IPv4";
		String command = "cmd.exe /c ipconfig | findstr IPv4";
		try {
			Process p = Runtime.getRuntime().exec(command);
			BufferedReader br = new BufferedReader(new InputStreamReader(p.getInputStream()));
			String line = null;
			while ((line = br.readLine()) != null) {
				line = line.substring(line.lastIndexOf(":") + 2, line.length());
				sb.append(line);
			}
			br.close();
			p.destroy();
		} catch (IOException e) {
			e.printStackTrace();
		}
		return sb.toString();
	}

	public byte[] parseCmdFromStr(String content) {
		// String cmd_bytes = content; //cmd内容
		// System.out.println(content);
		String[] splits = content.split(",");
		// System.out.println(splits.length);
		byte[] cmd_content = new byte[splits.length];
		for (int i = 0; i < splits.length; i++) {
			// System.out.println(splits[i]);
			cmd_content[i] = (byte) Integer.parseInt(splits[i]);// 将整数形式的字节cmd转换成byte数组cmd_content
			// System.out.println(cmd_content[i]);
		}

		return cmd_content;//
	}

	public static byte[] int2byteArray(int num) {
		byte[] result = new byte[4];
		result[0] = (byte) (num >>> 24);// 取最高8位放到0下标
		result[1] = (byte) (num >>> 16);// 取次高8为放到1下标
		result[2] = (byte) (num >>> 8); // 取次低8位放到2下标
		result[3] = (byte) (num); // 取最低8位放到3下标
		return result;
	}

	public static byte[] getRdcControlMessage(byte flag, int budget, int guadr) {
		byte[] result = new byte[10];
		result[0] = 10;
		result[1] = 1;

		result[2] = (byte) (budget >> 24);// 取最高8位放到0下标
		result[3] = (byte) (budget >> 16);// 取次高8为放到1下标
		result[4] = (byte) (budget >> 8); // 取次低8位放到2下标
		result[5] = (byte) (budget); // 取最低8位放到3下标

		result[6] = (byte) (guadr >> 24);// 取最高8位放到0下标
		result[7] = (byte) (guadr >> 16);// 取次高8为放到1下标
		result[8] = (byte) (guadr >> 8); // 取次低8位放到2下标
		result[9] = (byte) (guadr); // 取最低8位放到3下标
		//
		// for (byte b : result) {
		// System.out.print(b + ",");
		// }
		// System.out.println();
		return result;
	}

	public static long getLong(byte[] bb) {
		return (((long) (bb[0] & 0x00ff) << 40) | ((long) (bb[1] & 0x00ff) << 32) | ((long) (bb[2] & 0x00ff) << 24)
				| ((long) (bb[3] & 0x00ff) << 16) | ((long) (bb[4] & 0x00ff) << 8) | ((long) (bb[5] & 0x00ff) << 0));
	}

	public static int getInt(byte[] bb) {
		return ((bb[1] & 0xff) | ((bb[0] << 8) & 0xff00));
	}

	public static Energy Create_Energy(String selfId, byte[] others) {
		byte[] message = others;

		int length0 = GlobalDefines.GlobalCollectView.G_DEF_ENERGY_LENGTH0;
		byte[] node_time = new byte[length0];
		System.arraycopy(message, GlobalDefines.GlobalCollectView.G_DEF_CRO_TIME_HOUR, node_time, 0, length0);

		// CPU
		int length1 = GlobalDefines.GlobalCollectView.G_DEF_ENERGY_LENGTH1;
		byte[] CPU_time = new byte[length1];
		System.arraycopy(message, GlobalDefines.GlobalCollectView.G_DEF_ENERGY_CPU, CPU_time, 0, length1);
		// System.out.println(CPU_time[0]+"!");
		// LPM
		byte[] LPM_time = new byte[length1];
		System.arraycopy(message, GlobalDefines.GlobalCollectView.G_DEF_ENERGY_LPM, LPM_time, 0, length1);
		// System.out.println(LPM_time[0]);

		// cycleTime
		byte[] cycleTime = new byte[2];
		System.arraycopy(message, GlobalDefines.GlobalCollectView.G_DEF_ENERGY_CYCLETIME, cycleTime, 0, 2);
		// cycleTimeDirection
		byte cycleTimeDirection = message[GlobalDefines.GlobalCollectView.G_DEF_ENERGY_CYCLETIMEDIRECTION];
		// Send time
		byte[] Send_time = new byte[length1];
		System.arraycopy(message, GlobalDefines.GlobalCollectView.G_DEF_ENERGY_SEND, Send_time, 0, length1);
		// System.out.println(Send_time[0]);
		// Receiver time
		byte[] Receive_time = new byte[length1];
		System.arraycopy(message, GlobalDefines.GlobalCollectView.G_DEF_ENERGY_RECEIVE, Receive_time, 0, length1);
		// System.out.println(Receive_time[0]);

		// 電壓 2個字節
		byte[] Voltage = new byte[2];
		System.arraycopy(message, GlobalDefines.GlobalCollectView.G_DEF_ENERGY_VOLTAGE, Voltage, 0, 2);
		// ParentID
		byte[] ParentID = new byte[2];
		System.arraycopy(message, GlobalDefines.GlobalCollectView.G_DEF_PARENTID_HIGH, ParentID, 0, 2);
		// beacon 2个 字节
		byte[] Beacon = new byte[2];
		System.arraycopy(message, GlobalDefines.GlobalCollectView.G_DEF_ENERGY_BEACON, Beacon, 0, 2);
		// num_neighbors 2个 字节
		byte[] Num_neighbors = new byte[2];
		System.arraycopy(message, GlobalDefines.GlobalCollectView.G_DEF_ENERGY_NUM_NEGINBORS, Num_neighbors, 0, 2);
		// rtmetric 2 个 字节
		byte[] Rtmetric = new byte[2];
		System.arraycopy(message, GlobalDefines.GlobalCollectView.G_DEF_ENERGY_RTMERTIC, Rtmetric, 0, 2);

		byte timeh = message[GlobalDefines.GlobalCollectView.G_DEF_CRO_TIME_HOUR];
		byte timem = message[GlobalDefines.GlobalCollectView.G_DEF_CRO_TIME_MINUTE];
		byte times = message[GlobalDefines.GlobalCollectView.G_DEF_CRO_TIME_SECOND];
		String currenttime = timeh + ":" + timem + ":" + times;
		
		
		// byte[] currenttime = new byte[1];
		// System.arraycopy(message,
		// GlobalDefines.GlobalCollectView.G_DEF_CRO_TIME_HOUR,
		// currenttime, 0, 1);
		// System.out.println(currenttime + "!!!");
		// 時間同步 1字節
		byte SynTime = message[GlobalDefines.GlobalCollectView.G_DEF_ENERGY_SYNTIME];
		byte[] Current = new byte[4];
		System.arraycopy(message, GlobalDefines.GlobalCollectView.G_DEF_ENERGY_CURRENT, Current, 0, 4);
		// 重启次数 1字節
		int ReBoot = message[GlobalDefines.GlobalCollectView.G_DEF_ENERGY_REBOOT];

		String NodeID = selfId.split(":")[selfId.split(":").length - 1];
		int i = 0;
		int lengthing = NodeID.length();
		for (i = 0; i < 4 - lengthing; i++) {
			NodeID = "0" + NodeID;
		}
//		Energy ene = new Energy(NodeID, getLong(CPU_time), getLong(LPM_time), getLong(Send_time), getLong(Receive_time),
//		Util.getInt(Voltage), Util.formatByteToByteStr(ParentID), SynTime, "" + Util.getInt(Beacon),
//		Util.getInt(Num_neighbors), Util.getInt(Rtmetric), ReBoot, 0, getInt(cycleTime),
//		"" + cycleTimeDirection, "" + currenttime,bytetofloat(Current));
		Energy ene = new Energy(NodeID, getLong(CPU_time), getLong(LPM_time), getLong(Send_time), getLong(Receive_time),
				Util.getInt(Voltage), Util.formatByteToByteStr(ParentID), SynTime, "" + Util.getInt(Beacon),
				Util.getInt(Num_neighbors), Util.getInt(Rtmetric), ReBoot, 0, getInt(cycleTime),
				"" + cycleTimeDirection, "" + currenttime,(float)(byteArrayToInt(Current)*1.0/1000));
		System.out.println("new mode:\t" +NodeID+"\t"+Send_time+'\t'+(float)(byteArrayToInt(Current)*1.0/1000));
		return ene;
	}

	public static boolean Online_Judge(byte[] bitmap) {
		String datetime = getCurrentDateTime();
		String[] times = datetime.split(":");
		int hour = Integer.parseInt(times[0]);
		int minute = Integer.parseInt(times[1]);
		int count = hour * 6 + minute / 10;
		int index = count / 8;
		int inbyte = count % 8;
		//System.out.println(hour+","+minute+","+count+","+index+","+inbyte);
		//System.out.println("bitmap[index]:"+bitmap[index]);
		//System.out.println("bitmap[index] >> inbyte:"+(bitmap[index] >> inbyte));
		if ((bitmap[index] >> (7-inbyte)) % 2 == 1 || (bitmap[index] >> (7-inbyte)) % 2 == -1) {
			//System.out.println("true");
			return true;
		} else{
			//System.out.println("false");
			return false;
		}
	}

	public static int StatusJuage(boolean online) {
		String datetime = getCurrentDateTime();
		System.out.println(datetime);
		String[] times = datetime.split(":");
		// int hour = Integer.parseInt(times[0]);
		int minute = Integer.parseInt(times[1]);
		int second = Integer.parseInt(times[2]);;
		int time = second + (minute % 10) * 60;
		//System.out.println("time:"+time);
		int flag = 0;
		if (!online) {
			if (time < 20)
				flag = 1;
			else if (time > 20 && time < 600)
				flag = 2;
		} else {
			if (time < 330)
				flag = 3;
			else if (time >= 330 && time < 480)
				flag = 4;
			else if (time >= 480 && time < 600)
				flag = 5;
		}
		return flag;
	}
	//byte 数组与 long 的相互转换  
    public static byte[] longToBytes(long x) {  
    	 ByteBuffer buffer = ByteBuffer.allocate(8);
        buffer.putLong(0, x);  
        return buffer.array();  
    } 
    
    public static byte[] intToByteArray(int a) {  
        return new byte[] {  
            (byte) ((a >> 24) & 0xFF),  
            (byte) ((a >> 16) & 0xFF),     
            (byte) ((a >> 8) & 0xFF),     
            (byte) (a & 0xFF)  
        };  
    }  
    
    public static float bytetofloat(byte[] b) {    
        int l;                                             
        l = b[0];                                  
        l &= 0xff;                                         
        l |= ((long) b[1] << 8);                   
        l &= 0xffff;                                       
        l |= ((long) b[2] << 16);                  
        l &= 0xffffff;                                     
        l |= ((long) b[3] << 24);                  
        return Float.intBitsToFloat(l);                    
    } 
    public static int byteArrayToInt(byte[] bytes) {  
        int value=0;  
        //由高位到低位  
        for(int i = 0; i < 4; i++) {  
            int shift= (4-1-i) * 8;  
            value +=(bytes[i] & 0x000000FF) << shift;//往高位游  
        }  
        return value;  
    }  
	public static void main(String[] args) throws IOException, JSONException {
		byte[] bitmap = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 16, 1, 1, 1, 1, 1, 1 };
		Online_Judge(bitmap);
	}
}