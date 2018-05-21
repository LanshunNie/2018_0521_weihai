package com.hit.heat.util;


import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileWriter;
import java.io.IOException;
import java.io.RandomAccessFile;
import java.util.ArrayList;
import java.util.List;


/* @lhy Lhy
 * @date 2015年8月18日
 * @des  
 */
public class WriteDataToFile {
	private BufferedWriter out;
	public WriteDataToFile(String filepath) throws IOException{
		File file = new File(filepath);
		out = new BufferedWriter(new FileWriter(file,true));
	}
	
	public void append(String message) throws IOException{
		out.write(message + "\r\n");//ubuntu下\n
		out.flush();
	}
	
	public void close() throws IOException{
		//out.flush();
		out.close();
	}
	
	public void clearAll(String path) throws IOException{
		BufferedWriter tWriter =  new BufferedWriter(new FileWriter(new File(path),false));
		tWriter.write("");
		tWriter.close();
	}
	
	
	public static List<String> getLine(String fileName,String targetFileName ,int lines) throws FileNotFoundException, IOException {  
        List<String> list = new ArrayList<String>();  
        boolean already = false;  
        RandomAccessFile rf = null;  
        rf = new RandomAccessFile(fileName, "r");  
        long len = rf.length();
        //System.out.println(len);
        long start = rf.getFilePointer(); 
        //System.out.println(start);
        long nextend = start + len - 1;  
        String line;  
        rf.seek(nextend);  
        int c = -1;  
        while (nextend > start) {  
            c = rf.read();  
            if (c == '\n') {  
                line = rf.readLine();  
                if(lines >0){  
                    list.add(line);  
                    lines--;  
                }  
               // nextend--;  
            }  
            nextend--;  
            if(lines == 0)  
                break;  
            rf.seek(nextend);  
            if (nextend == 0) {// 当文件指针退至文件开始处，输出第一行  
                line = rf.readLine();  
                    //if(line.startsWith(startTime)&&lines == 1)  
                    list.add(line);  
            }  
        }  
        rf.close(); 
        WriteDataToFile write = new WriteDataToFile(targetFileName);
        for (int i = list.size()-1; i >= 0; i--) {
            //System.out.println(list.get(i));// 利用get(int index)方法获得指定索引位置的对象
            write.append(list.get(i));
        }
        //cunwenjian !!!!!!!!!!!!!!!!!!! 
        return list;  
    }  
//	public static void main(String[] args) throws IOException {
//		WriteDataToFile file = new WriteDataToFile("data.txt");
//		for(int i= 0;i<100;i++){
//			file.append("hello world" + i);
//		}
//		//file.close();
//		file.clearAll("data.txt");
//		file.append("hello world");
//	}
}
