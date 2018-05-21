package com.hit.heat.util;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;



/* @lhy Lhy
 * @date 2015年11月30日
 * @des  
 */
public class CountData {
	public CountData(String filePath) throws IOException {
		File initFile = new File(filePath);
		if(!initFile.exists()){
			throw new IOException("file is not exists");
		}
		File resultFile = new File("result_data.txt");
		FileReader fileReader = new FileReader(initFile);
		BufferedReader bReader = new BufferedReader(fileReader);
		FileWriter fileWriter = new FileWriter(resultFile);
		String line;
		int count=0;
		int start;
		int stop;
		String ip = null;
		HashMap<String, Integer> map = new HashMap<String, Integer>();
		while((line = bReader.readLine())!= null){
			start =line.indexOf("[")+1;
			stop = line.indexOf("]");
			ip = line.substring(start, stop);
			
			if(map.keySet().contains(ip)){
				map.put(ip, map.get(ip)+1);
			}else {
				map.put(ip, 1);
			}
			count++;
			
		}
		for (String ip1:map.keySet()) {
			fileWriter.write(ip1 + " " + map.get(ip1) +"\r\n");
			
		}
		System.out.println(ip);
		System.out.println(count);
		fileWriter.flush();
		bReader.close();
		fileReader.close();
		fileWriter.close();
	}
//	public static void main(String[] args) {
//		try {
//			new CountData("mulicast.txt");
//		} catch (IOException e) {
//			// TODO 自动生成的 catch 块
//			e.printStackTrace();
//		}
//	}
}
