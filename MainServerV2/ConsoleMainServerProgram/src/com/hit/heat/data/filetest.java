package com.hit.heat.data;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;

public class filetest {
	public static void main(String[] args) throws IOException {
		FileOutputStream fs = new FileOutputStream(new File("text.txt"));
		PrintStream p = new PrintStream(fs);
		//p.println(100);
		
		long start = System.currentTimeMillis();
		for (int i = 0; i < 1000; i++) {
			p.println("111"+"2017-04-27 7:42:11"+"1111");
			
		}
		long end = System.currentTimeMillis();
		long time = end - start;
		System.out.println(time);
		p.close();
	}


}
