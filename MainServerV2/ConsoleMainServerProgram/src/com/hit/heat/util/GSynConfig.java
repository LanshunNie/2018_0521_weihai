package com.hit.heat.util;

import java.io.BufferedInputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;

import org.json.JSONException;
import org.json.JSONObject;

import com.hit.heat.model.SynParameter;
import com.sun.jndi.url.iiopname.iiopnameURLContextFactory;


/* @lhy Lhy
 * @date 2016年5月12日
 * @des  
 */
public class GSynConfig {
	JSONObject json;
	boolean hasException;
	public GSynConfig(String path) throws IOException,JSONException{
		hasException = false;
//		File fp = new File(path);
//		String parentPath = GetPath.getProjectPath();
//		System.out.println((parentPath+"\\"+path));
//		File fp = new File(parentPath+"/"+path); 
		File fp = new File(path); 
		if(!fp.exists()){
			throw new FileNotFoundException("文件" + path +"不存在");
		}
		StringBuilder sb = new StringBuilder();
		FileInputStream fileInputStream = new FileInputStream(fp);
		BufferedInputStream bufferedInputStream = new BufferedInputStream(fileInputStream);
		byte [] buffer = new byte[1024];
		int readLen;
		while((readLen = bufferedInputStream.read(buffer,0,1024)) > 0){
			byte[]read = new byte[readLen];
			System.arraycopy(buffer, 0, read, 0, readLen);
			sb.append(new String(read));
		}
		bufferedInputStream.close();
		fileInputStream.close();
		json = new JSONObject(sb.toString());
}
	public SynParameter getSynParameter(){
		SynParameter synParameter = new SynParameter();
		int value;
		String bitmap;
		boolean flag;
		try {
			value = json.getInt("seqNum");
			if(value!=0){
				synParameter.setSeqNum(value);
			}else{
				synParameter.setSeqNum(0);
				hasException = true;
			}
			
		} catch (JSONException e) {
			// TODO: handle exception
			synParameter.setSeqNum(0);
			hasException = true;
		}
		try {
			value = json.getInt("period");
			if(value!=0){
				synParameter.setPeriod(value);;
			}else{
				synParameter.setPeriod(1);
				hasException = true;
			}
			
		} catch (JSONException e) {
			// TODO: handle exception
			synParameter.setSeqNum(0);
			hasException = true;
		}
		try {
			value = json.getInt("level");
			if(value!=0){
				synParameter.setLevel(value);
			}else{
				synParameter.setLevel(0);
				hasException = true;
			}
			
		} catch (JSONException e) {
			// TODO: handle exception
			synParameter.setLevel(0);
			hasException = true;
		}
		try {
			value = json.getInt("hour");
			if(value!=0){
				synParameter.setHour(value);
			}else{
				synParameter.setHour(0);
				hasException = true;
			}
			
		} catch (JSONException e) {
			// TODO: handle exception
			synParameter.setHour(0);
			hasException = true;
		}
		try {
			value = json.getInt("minute");
			if(value!=0){
				synParameter.setMinute(value);
			}else{
				synParameter.setMinute(0);
				hasException = true;
			}
			
		} catch (JSONException e) {
			// TODO: handle exception
			synParameter.setMinute(0);
			hasException = true;
		}
		try {
			value = json.getInt("second");
			if(value!=0){
				synParameter.setSecond(value);
			}else{
				synParameter.setSecond(0);
				hasException = true;
			}
			
		} catch (JSONException e) {
			// TODO: handle exception
			synParameter.setSecond(0);
			hasException = true;
		}
		try {
			bitmap =json.getString("bitmap");
			if(!bitmap.isEmpty()){
				System.out.println(bitmap);
				String bitmap_temp = bitmap.substring(1,bitmap.length()-1);
				System.out.println(bitmap_temp);
				String[] Bits = bitmap_temp.split(",");
				System.out.println(Bits.length);
				String Bit = "";
				byte[] Bitmap = new byte[18];
				for (int i = 0;i<18;i++){
					Bit = Bits[i];
					int a = Integer.valueOf(Bit);
					Bitmap[i] = (byte) (a & 0xFF);
				}
				//(byte) (a & 0xFF)
				synParameter.setBitmap(Bitmap);
				//synParameter.setBitmap(bitmap.getBytes());
			}else{
				synParameter.setBitmap("000000000000000000".getBytes());
			}
			
		} catch (JSONException e) {
			// TODO 自动生成的 catch 块
			synParameter.setBitmap("000000000000000000".getBytes());
			hasException = true;
		}
		try {
			flag =json.getBoolean("state");
			synParameter.setFlag(flag);
//			System.out.println("state"+flag);
			//flag =json.getString("state");
			//synParameter.setFlag(flag);
		} catch (JSONException e) {
			// TODO 自动生成的 catch 块
			synParameter.setFlag(false);
			hasException = true;
		}
		
		
		
		return synParameter;
	}
}
