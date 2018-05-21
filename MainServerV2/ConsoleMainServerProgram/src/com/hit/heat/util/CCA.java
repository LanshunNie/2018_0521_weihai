package com.hit.heat.util;

public class CCA {
//
//	public static void main(String[] args) {
//		// TODO Auto-generated method stub
//
//	StringBuilder sb = new StringBuilder();
//	sb.append("F800000D100000000000000000000000");
//	int seq0 = 0;
//	int seq1 = 0;
//	seq0++;
//	if(seq0 == 256){//FF
//		seq0 = 0;
//		seq1++;
//	}
//	String str0 = Integer.toHexString(seq0).toUpperCase();
//	if(str0.length() == 1){
//		str0 = "0"+ str0;
//	}
//	sb.append(str0);
//	//sb.append(" ");
//	String str1 = Integer.toHexString(seq1).toUpperCase();
//	if(str1.length() == 1){
//		str1 = "0"+ str1;
//	}
//	sb.append(str1);
//	System.out.println(sb);
//	
//	String str = Check(sb);
//	
//	System.out.println(str);
//		
//	}
//	public static String Check(StringBuilder sb){
//		String str = sb.toString();
//		String[] buff = {};
//		int i = 0;
////		System.out.println(str.length());
////		for(i = 0;i<2;i++){
////			buff[i] = str.substring(i*2, i+2);
////		}
////		buff[i] = "sl";
//		int sum = 0;
//		for(i = 0;i<str.length()/2;i++){
//			int temp = Integer.parseInt(str.substring(2*i,2*i+2), 16);
//			sum += temp;
//		}
//		sum = sum % 256;//FF
//		str =  Integer.toHexString(sum).toUpperCase();
//		return str;
//	}

}
