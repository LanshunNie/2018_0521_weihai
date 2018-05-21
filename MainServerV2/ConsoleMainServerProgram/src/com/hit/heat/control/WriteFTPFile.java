package com.hit.heat.control;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;

import org.apache.commons.net.ftp.FTPClient;
//import org.apache.commons.net.ftp.FTPFile;
//import org.apache.log4j.Logger;

import com.hit.heat.control.FTPUtil;
import com.hit.heat.util.Util;

public class WriteFTPFile {

//	private Logger logger = Logger.getLogger(WriteFTPFile.class);

	/**
	 * 本地上传文件到FTP服务器
	 * 
	 * @param uploadFile 要上传的本地文件名【绝对路径】
	 *           
	 * @throws IOException
	 */
	public int upload(
			String ftpUserName, 
			String ftpPassword,
			String ftpHost, 
			int    ftpPort, 
			String  uploadFile ) {
		FTPClient ftpClient = null;
		//System.out.println(Util.getCurrentTime()+" 开始上传文件到FTP.");
		try {
			ftpClient = FTPUtil.getFTPClient(ftpHost, ftpPassword,
					ftpUserName, ftpPort);
			// 设置PassiveMode传输
			ftpClient.enterLocalActiveMode();
			// 设置以二进制流的方式传输
			ftpClient.setFileType(FTPClient.BINARY_FILE_TYPE);
			// 对远程目录的处理
			
			String remoteFileName = uploadFile;
			if (uploadFile.contains("/")) {
				remoteFileName = uploadFile
						.substring(uploadFile.lastIndexOf("/") + 1);
			}
			
			// FTPFile[] files = ftpClient.listFiles(new
			// String(remoteFileName));
			// 先把文件写在本地。在上传到FTP上最后在删除
			//System.out.println(Util.getCurrentTime()+" temppath="+uploadFile);
 
			int writeResult=1;
			if (writeResult<2) 
			{
				if(uploadFile.contains("config.json")){	//by mlc
					uploadFile = "config.json";
				}
				File f = new File(uploadFile);
				InputStream in = new FileInputStream(f);
				ftpClient.storeFile(remoteFileName, in);
				in.close();
				System.out.println(Util.getCurrentTime()+" send file " + uploadFile + " to FTP success!");
				//f.delete(); // 删除本地文件	
				return 1;
			} else {
				System.err.println(Util.getCurrentTime()+" send file fail!");
			}
		} catch (Exception e) {
			e.printStackTrace();
			return -1;
		}finally{
			try {
				ftpClient.disconnect();
			} catch (IOException e) {
				e.printStackTrace();
				return -1;
			}
		}
		return 0;
	}

	/**
	 * 本地文件写入
	 * 
	 * @param ftpPath
	 * @param str
	 * @return
	 */
	public boolean write(String fileName, String fileContext,
			String writeTempFilePath) {
		
		try {
			//System.out.println(Util.getCurrentTime()+" 开始写文件");
			System.out.println(Util.getCurrentTime() + writeTempFilePath + "/" + fileName);
			File f = new File(writeTempFilePath + "/" + fileName);
			
			if(!f.exists()){
				if(!f.createNewFile()){
					System.err.println(Util.getCurrentTime()+" file does not exist，creat fail!");
				}
			}
			BufferedWriter bw = new BufferedWriter(new FileWriter(f, true));
			bw.write(fileContext.replaceAll("\n", "\r\n"));
			bw.flush();
			bw.close();
			return true;
		} catch (Exception e) {
			System.err.println(Util.getCurrentTime()+"write file fail");
			e.printStackTrace();
			return true;
			//return false;
		}
	}
}
