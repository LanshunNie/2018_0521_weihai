package com.hit.heat.util;

public class Ipv6Parse {
	
	public static String formatIpv6ToSimple(String ip) throws IllegalArgumentException{
		return formatIpv6ToSimple(ip,true);
	}
	
	public static String formatIpv6ToSimple(String ip,boolean upperCase)throws IllegalArgumentException{
		if(null == ip || ip.isEmpty()){
			throw new IllegalArgumentException("param is not a ipv6 address");
		}
		//判断是否满足ipv6格式
		
		int index = ip.indexOf("::");
		if(index >= 0){
			return ip;
		}
		String upperStr = null;
		if(upperCase){
			upperStr = ip.toUpperCase();
		}else{
			upperStr = ip.toLowerCase();
		}
		String[] splits = upperStr.split(":");
		boolean simpleFlag = false;
		StringBuilder sb = new StringBuilder();
		int state = 0;
		String part = null;
		for(int i=0;i< splits.length;i++){
			part = splits[i];
			if(simpleFlag){
				sb.append(part + ":");
				continue;
			}
			if(part.equals("0")){
				if(i == splits.length - 1){
					sb.append("::");
					break;
				}
				switch(state){
				case 0:
					state = 1;
					break;
				case 1:
					state = 2;
					break;
				case 2:
					break;
				}
			}else{
				switch (state) {
				case 0:
					sb.append(part +":");
					break;
				case 1:
					sb.append("0:" + part + ":");
					state = 0;
					break;
				case 2:
					sb.append(":" + part + ":");
					simpleFlag = true;
					//state = 0;
					break;
				}
			}
		}
		
		//删除最后多余的:
		sb.deleteCharAt(sb.length() - 1);
		//形如0:0:xxxx的ipv6地址压缩
		if(sb.charAt(0) == ':'){
			return ":" + sb.toString();
		}
		return sb.toString();
	}
	
//	public static void main(String[] args) {
//		final String ipv6 = "ff:0:0:0:ff:0:0:0"; 
//		final String ipv6_1 = "::1";
//		System.out.println(ipv6+" "+Ipv6Parse.formatIpv6ToSimple(ipv6,true));
//	}
}
