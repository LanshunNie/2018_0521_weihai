package com.hit.heat.util;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.HashMap;

/**
 * @lhy Lhy
 * @date 2015年11月30日
 * @des
 */
public class InfoExtract {
	public InfoExtract(String filepath) throws IOException{
		File fp = new File(filepath);
		if(!fp.exists()){
			throw new IOException("File not exists");
		}
		File save_fp = new File("result-mulicast.txt");
		FileReader fileReader = new FileReader(fp);
		BufferedReader bufferedReader = new BufferedReader(fileReader);
		FileWriter fileWriter = new FileWriter(save_fp);
		String line;
		int counter = 0;
		int start;
		int stop;
		String ip;
		HashMap<String, Integer> map = new HashMap<String, Integer>();
		while((line = bufferedReader.readLine()) != null){
			start = line.indexOf("[") + 1;
			stop = line.indexOf("]");
			//System.out.println(start+"/"+stop);
			ip = line.substring(start, stop);
			//System.out.println(line.substring(0,start-2));
			if(map.keySet().contains(ip)){
				map.put(ip, map.get(ip) + 1);
			}else{
				map.put(ip, 1);
			}
			counter++;
		}
		//System.out.println(counter);
		for(String ip1 : map.keySet()){
			System.out.println(ip1 + " " + map.get(ip1));
			fileWriter.write(ip1 + " " + map.get(ip1) +"\r\n");
		}
		fileWriter.flush();
		bufferedReader.close();
		fileReader.close();
		fileWriter.close();
	}
	
	public static void WriteToFile(String filepath,Object object) throws IOException{
		File fp = new File(filepath);
		FileWriter fileWriter = new FileWriter(fp,true);
		fileWriter.write(object + "\r\n");
		fileWriter.flush();
		fileWriter.close();
	}
	public static int StringToInt(String src){
		return Integer.valueOf(src, 16);
	}
//	
//	public static void main(String[] args) throws IOException{
//		String filepath = "mulicast.txt";
//		new InfoExtract(filepath);
//	}
}
