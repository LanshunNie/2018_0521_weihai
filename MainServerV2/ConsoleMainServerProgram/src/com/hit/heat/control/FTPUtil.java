 
package com.hit.heat.control;
import java.io.IOException;
//import java.io.InputStream;
import java.net.SocketException;
//import java.util.Properties;

import org.apache.commons.net.ftp.FTPClient;
import org.apache.commons.net.ftp.FTPReply;

import com.hit.heat.util.Util;
//import org.apache.log4j.Logger;

public class FTPUtil {
//	private static Logger logger = Logger.getLogger(FTPUtil.class);

	
	
	/**
	 * 获取FTPClient对象
	 * @param ftpHost FTP主机服务器
	 * @param ftpPassword FTP 登录密码
	 * @param ftpUserName FTP登录用户名
	 * @param ftpPort FTP端口 默认为21
	 * @return
	 */
	public static FTPClient getFTPClient(String ftpHost, String ftpPassword,
			String ftpUserName, int ftpPort) {
		FTPClient ftpClient = null;
		try {
			ftpClient = new FTPClient();
			ftpClient.connect(ftpHost, ftpPort);// 连接FTP服务器
			ftpClient.login(ftpUserName, ftpPassword);// 登陆FTP服务器
			if (!FTPReply.isPositiveCompletion(ftpClient.getReplyCode())) {
				System.err.println(Util.getCurrentTime()+"  did not connect FTP,hoost name or psword wrong");
				ftpClient.disconnect();
			}
		} catch (SocketException e) {
			e.printStackTrace();
			System.err.println(Util.getCurrentTime()+" FTP's IP address may be wrong. Please configure it properly.");
		} catch (IOException e) {
			e.printStackTrace();
			System.err.println(Util.getCurrentTime()+" FTP port error. Please configure it properly.");
		}
		return ftpClient;
	}
}
