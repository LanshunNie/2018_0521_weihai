package com.hit.heat.util;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.HashMap;

/* @lhy Lhy
 * @date 2016年3月4日
 * @des  
 */
public class Test {

	public HashMap<String, Integer> getTxtTime(String filepath,
			String startTime, String endTime) throws IOException,
			ParseException {
		File fp = new File(filepath);
		if (!fp.exists()) {
			throw new IOException("File not exists");
		}
		FileReader fileReader = new FileReader(fp);
		BufferedReader bufferedReader = new BufferedReader(fileReader);
		String line;
		int counter = 0;
		int start;
		int stop;
		//String middleTime;
		String ip;
		SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd hh:MM:ss");
		Date bt = sdf.parse(startTime);
		Date et = sdf.parse(endTime);
		//System.out.println(bt.before(et));//<
		HashMap<String, Integer> map = new HashMap<String, Integer>();
		//while ((line = bufferedReader.readLine()) != null &&(bt.before(sdf.parse(line.substring(0, line.indexOf("[")-1)) ))&&((sdf.parse(line.substring(0, line.indexOf("[")-1))).before(et)) ) {
		while ((line = bufferedReader.readLine()) != null ) {
			start = line.indexOf("[") + 1;
			stop = line.indexOf("]");
			//System.out.println("当前时间"+line.substring(0, start - 2));
			if(bt.before(sdf.parse(line.substring(0, line.indexOf("[")-1))) && (sdf.parse(line.substring(0, line.indexOf("[")-1))).before(et)){
				System.out.println("当前时间"+line.substring(0, start - 2));
				ip = line.substring(start, stop);
				if (map.keySet().contains(ip)) {
					map.put(ip, map.get(ip) + 1);
				} else {
					map.put(ip, 1);
				}
			}
			counter++;
		}
		for(String ip1 : map.keySet()){
			System.out.println(ip1 + " " + map.get(ip1));
		}
		bufferedReader.close();
		fileReader.close();
		return map;
	}

	public HashMap<String, Integer> getCmdTime(String filepath,
			String startTime, String endTime) throws IOException,
			ParseException {
		File fp = new File(filepath);
		if (!fp.exists()) {
			throw new IOException("File not exists");
		}
		FileReader fileReader = new FileReader(fp);
		BufferedReader bufferedReader = new BufferedReader(fileReader);
		String line;
		int counter = 0;
		int cmdCount=0;
		int start;
		int stop;
		//String middleTime;
		String ip;
		SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
		Date bt = sdf.parse(startTime);
		Date et = sdf.parse(endTime);
		long btTime=bt.getTime()-15000; //这是毫秒数
		long etTime=et.getTime()-15000;
		//System.out.println(sdf.format(btTime)+"/"+sdf.format(etTime));
		HashMap<String, Integer> map = new HashMap<String, Integer>();
		while ((line = bufferedReader.readLine()) != null ) {
			start = line.indexOf("[") + 1;
			stop = line.indexOf("]");
			if(btTime<(sdf.parse(line.substring(0, line.indexOf("[")-1))).getTime() &&
					(sdf.parse(line.substring(0, line.indexOf("[")-1))).getTime()<etTime){
				System.out.println("当前时间"+line.substring(0, start - 2));
				cmdCount++;
				System.out.println(cmdCount);
				ip = line.substring(start, stop);
				if (map.keySet().contains(ip)) {
					map.put(ip, map.get(ip) + 1);
					
				} else {
					map.put(ip, 1);
				}
			}
			counter++;
		}
		for(String ip1 : map.keySet()){
			System.out.println(ip1 + " " + map.get(ip1));
		}
		bufferedReader.close();
		fileReader.close();
		return map;
	}
	public int  getCmdCount(String filepath,
			String startTime, String endTime) throws IOException,
			ParseException {
		File fp = new File(filepath);
		if (!fp.exists()) {
			throw new IOException("File not exists");
		}
		FileReader fileReader = new FileReader(fp);
		BufferedReader bufferedReader = new BufferedReader(fileReader);
		String line;
		int counter = 0;
		int cmdCount=0;
		int start;
		int stop;
		//String middleTime;
		String ip;
		SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
		Date bt = sdf.parse(startTime);
		Date et = sdf.parse(endTime);
		long btTime=bt.getTime()-15000; //这是毫秒数
		long etTime=et.getTime()-15000;
		//System.out.println(sdf.format(btTime)+"/"+sdf.format(etTime));
		HashMap<String, Integer> map = new HashMap<String, Integer>();
		while ((line = bufferedReader.readLine()) != null ) {
			start = line.indexOf("[") + 1;
			stop = line.indexOf("]");
			if(btTime<(sdf.parse(line.substring(0, line.indexOf("[")-1))).getTime() &&
					(sdf.parse(line.substring(0, line.indexOf("[")-1))).getTime()<etTime){
				System.out.println("当前时间"+line.substring(0, start - 2));
				cmdCount++;
				System.out.println(cmdCount);
				ip = line.substring(start, stop);
				if (map.keySet().contains(ip)) {
					map.put(ip, map.get(ip) + 1);
					
				} else {
					map.put(ip, 1);
				}
			}
			counter++;
		}
		for(String ip1 : map.keySet()){
			System.out.println(ip1 + " " + map.get(ip1));
		}
		bufferedReader.close();
		fileReader.close();
		return cmdCount;
	}
	
	
//	public static void main(String[] args) throws ParseException, IOException {
//		Test test = new Test();
//		//test.getTxtTime("mulicast.txt", "2016-02-28 13:32:00", "2016-02-28 13:36:58");
//		//test.getCmdTime("cmd.txt", "2016-02-28 13:32:00", "2016-02-28 13:36:58");
//		test.getCmdCount("cmd.txt", "2016-02-28 13:32:00", "2016-02-28 13:36:58");
//		// int a=89;
//		// System.out.println(a*1.0/90);
////
//		// String t1="2016-02-28 13:32:00";
//		 //String t2="2016-03-03 21:58:30";
//		 //System.out.println(t1.compareTo(t2));
//		// SimpleDateFormat sdf=new SimpleDateFormat("yyyy-MM-dd HH:mm:ss");
//		// Date bt=sdf.parse(t1);
//		// long time=bt.getTime()-15000; //这是毫秒数
//		// Date et=sdf.parse(t2);
//		// System.out.print((bt.getTime()-15000)<(et.getTime()-15000));
//		 //System.out.println(sdf.format(time));
//		 //System.out.println(bt.before(et));
//	}
}
