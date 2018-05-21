package com.hit.heat.cn;

import java.io.IOException;
import java.nio.ByteBuffer;  

import org.jfree.ui.LengthAdjustmentType;

import com.hit.heat.data.SqlOperate;
import com.hit.heat.model.Command;
import com.hit.heat.util.BitMap;
import com.hit.heat.util.Util;

public class test {
	public static boolean Online_Judge(byte[] bitmap) {
		String datetime = Util.getCurrentDateTime();
		String[] times = datetime.split(":");
		int hour = Integer.parseInt(times[0]);
		int minute = Integer.parseInt(times[1]);
		int count = hour * 6 + minute / 10;
		int index = count / 8 - 1;
		int inbyte = count % 8;
		// System.out.println(hour+","+minute+","+count+","+index+","+inbyte);
		// System.out.println(bitmap[index]);
		// System.out.println(bitmap[index] >> inbyte);
		if ((bitmap[index] >> inbyte) % 2 == 1) {
			// System.out.println("true");
			return true;
		} else
			return false;
	}

	public int StatusJuage(boolean online) {
		String datetime = Util.getCurrentDateTime();
		String[] times = datetime.split(":");
		// int hour = Integer.parseInt(times[0]);
		int minute = Integer.parseInt(times[1]);
		int second = 0;
		int time = second + minute * 60;
		int flag = 0;
		if (online) {
			if (time < 20)
				flag = 1;
			else if (time > 20 && time < 300)
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

	public int commandjiexi(boolean online) {
		int flag;
		byte[] command = new byte[10];
		boolean returntype = false;
		int send_to_net = command[0];
		int has_return = command[1];
		int command_length = command[2];
		byte[] com = new byte[command_length];
		System.arraycopy(command, 3, com, 0, command_length);
		String commands = Util.formatBytesToStr(com);
		return 0;
	}
	public static void change(){
		byte[] bitmap = new byte[3]; 
		byte[] bit = new byte[24];
		bitmap[0] = -1;
		bitmap[1] = 1;
		bitmap[2] = 5;
		int i ,j,t = 0;
		byte bitmap_a = 0;
		for (i = 0;i<3;i++){
			bitmap_a = bitmap[i];
			t = 0;
			for (j = 7;j>=0;j--){
				bit[t] = (byte) (bitmap_a & 1);
				bitmap_a = (byte) (bitmap_a >> 1);
				System.out.println(t +" "+bit[t]);
				t++;
			}
		}
	}
	

	public static byte[] intToByteArray(final long integer) {
	int byteNum = (40 -Long.numberOfLeadingZeros (integer < 0 ? ~integer : integer))/ 8;
	byte[] byteArray = new byte[4];
	
	for (int n = 0; n < byteNum; n++)
	byteArray[3 - n] = (byte) (integer>>> (n * 8));
	
	return (byteArray);
	}
	//byte 数组与 long 的相互转换  
    public static byte[] longToBytes(long x) {  
    	 ByteBuffer buffer = ByteBuffer.allocate(8);
        buffer.putLong(0, x);  
        return buffer.array();  
    } 
	public static void main(String[] args) throws IOException {
		//change();
//		long epoch = System.currentTimeMillis()/1000;
//		System.out.println(epoch);
//		//byte[] BBB = intToByteArray(256);
//		byte[] BBB = longToBytes(epoch);
//		for (int i = 0;i<BBB.length;i++){
//			System.out.println(BBB[i]);
//		}
//	    String stri = "0001";
//	    int j = Integer.parseInt(stri);
//	    System.out.println("j"+j);
	    int com_type = 0x80;
	    System.out.println(Integer.toHexString(com_type));
		//byte[] command = new byte[10];
//		command[0] = 0;
//		command[1] = 1;
//		System.out.println(command.length);
//		int len = command[1];
//		int second = 0;
//		int minute = 0;
//		int time = second + minute * 60;
//		int flag = 0;
//		if (command[0] == 0) {
//			if (time < 20)
//				flag = 1;
//			else if (time > 20 && time < 300)
//				flag = 2;
//		} else if (command[0] == 1) {
//			if (time < 330)
//				flag = 3;
//			else if (time >= 330 && time < 480)
//				flag = 4;
//			else if (time >= 480 && time < 600)
//				flag = 5;
//		}
	}
}
