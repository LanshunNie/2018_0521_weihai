/*NetParameter.java
 * 2015年7月29日
 * Lhy
*/
package com.hit.heat.model;

/**
 * @lhy Lhy
 *
 */
public class NetParameter {
	//添加id和心跳
	private String id;
	private int HeartIntSec;
	private int AckHeartInt;
	private int MaxAckFail;
	
	private String tcpAddr;
	private int tcpPort;
	private int tcpByWebPort;
	private int tcpRemoteConfigPort;
	private String udpAddr;
	private int udpPort;
	private String rootAddr;
	private int rootPort;
	private String rootRoomId;
	private int rootX;
	private int rootY;
	private String tcpWebServerAddr;
	private int tcpWebServerPort;
	private String remoteAddr;
	private int remotePort;
	private int netPort;
	private int dayLength;
	private int appSendLength;
	private String upperAddr;
	private int upperPort;
	private String ftpuser;
	private String ftphost;   
	private String ftpPwd;
	private int ftpPort;
	private String serverIp;
	public NetParameter(){
	}
	/**
	 * 
	 */
	public NetParameter(String id,int HeartIntSec,int AckHeartInt,int MaxAckFail,String tcpAddr,
			int tcpPort,int tcpByWebPort, int tcpRemoteConfigPort,String udpAddr,int udpPort,
			String rootAddr,int rootPort,String tcpWebServerAddr,int tcpWebServerPort,
			String remoteAddr,int remotePort,int netPort,int dayLength,int appSendLength,String upperAddr,int upperPort,
			String ftpuser,String ftphost,String ftpPwd,int ftpPort,String serverIp) {
		// TODO 自动生成的构造函数存根
		this.id = id;
		this.HeartIntSec = HeartIntSec;
		this.AckHeartInt = AckHeartInt;
		this.MaxAckFail = MaxAckFail;
		
		this.tcpAddr = tcpAddr;
		this.tcpPort = tcpPort;
		this.udpAddr = udpAddr;
		this.udpPort = udpPort;
		this.rootAddr = rootAddr;
		this.rootPort = rootPort;
		this.tcpWebServerPort=tcpWebServerPort;
		this.tcpWebServerAddr=tcpWebServerAddr;
		this.remoteAddr =remoteAddr;
		this.remotePort =remotePort;
		this.tcpByWebPort= tcpByWebPort;
		this.tcpRemoteConfigPort =tcpRemoteConfigPort;
		this.netPort =netPort;
		this.dayLength = dayLength;
		this.appSendLength = appSendLength;
		this.upperAddr = upperAddr;
		this.upperPort = upperPort;
		this.ftpuser = ftpuser;
		this.ftphost = ftphost;   
		this.ftpPwd = ftpPwd;
		this.ftpPort = ftpPort;
		this.serverIp = serverIp;
	}
	
	public String getId() {
		return id;
	}
	public void setId(String id) {
		this.id = id;
	}
	public int getHeartIntSec() {
		return HeartIntSec;
	}
	public void setHeartIntSec(int heartIntSec) {
		HeartIntSec = heartIntSec;
	}
	public int getAckHeartInt() {
		return AckHeartInt;
	}
	public void setAckHeartInt(int ackHeartInt) {
		AckHeartInt = ackHeartInt;
	}
	public int getMaxAckFail() {
		return MaxAckFail;
	}
	public void setMaxAckFail(int maxAckFail) {
		MaxAckFail = maxAckFail;
	}
	
	public int getNetPort() {
		return netPort;
	}
	public void setNetPort(int netPort) {
		this.netPort = netPort;
	}
	public int getTcpRemoteConfigPort() {
		return tcpRemoteConfigPort;
	}
	public void setTcpRemoteConfigPort(int tcpRemoteConfigPort) {
		this.tcpRemoteConfigPort = tcpRemoteConfigPort;
	}
	public int getTcpByWebPort() {
		return tcpByWebPort;
	}
	public void setTcpByWebPort(int tcpByWebPort) {
		this.tcpByWebPort = tcpByWebPort;
	}
	/**
	 * @return tcpWebServerAddr
	 */
	public String getTcpWebServerAddr() {
		return tcpWebServerAddr;
	}
	/**
	 * @param tcpWebServerAddr 要设置的 tcpWebServerAddr
	 */
	public void setTcpWebServerAddr(String tcpWebServerAddr) {
		this.tcpWebServerAddr = tcpWebServerAddr;
	}
	/**
	 * @return tcpWebServerPort
	 */
	public int getTcpWebServerPort() {
		return tcpWebServerPort;
	}
	/**
	 * @param tcpWebServerPort 要设置的 tcpWebServerPort
	 */
	public void setTcpWebServerPort(int tcpWebServerPort) {
		this.tcpWebServerPort = tcpWebServerPort;
	}
	/**
	 * @return rootAddr
	 */
	public String getRootAddr() {
		return rootAddr;
	}
	/**
	 * @param rootAddr 要设置的 rootAddr
	 */
	public void setRootAddr(String rootAddr) {
		this.rootAddr = rootAddr;
	}
	/**
	 * @return rootPort
	 */
	public int getRootPort() {
		return rootPort;
	}
	/**
	 * @param rootPort 要设置的 rootPort
	 */
	public void setRootPort(int rootPort) {
		this.rootPort = rootPort;
	}
	/**
	 * @return tcpAddr
	 */
	public String getTcpAddr() {
		return tcpAddr;
	}
	/**
	 * @param tcpAddr 要设置的 tcpAddr
	 */
	public void setTcpAddr(String tcpAddr) {
		this.tcpAddr = tcpAddr;
	}
	/**
	 * @return tcpPort
	 */
	public int getTcpPort() {
		return tcpPort;
	}
	/**
	 * @param tcpPort 要设置的 tcpPort
	 */
	public void setTcpPort(int tcpPort) {
		this.tcpPort = tcpPort;
	}
	/**
	 * @return udpAddr
	 */
	public String getUdpAddr() {
		return udpAddr;
	}
	/**
	 * @param udpAddr 要设置的 udpAddr
	 */
	public void setUdpAddr(String udpAddr) {
		this.udpAddr = udpAddr;
	}
	/**
	 * @return udpPort
	 */
	public int getUdpPort() {
		return udpPort;
	}
	/**
	 * @param udpPort 要设置的 udpPort
	 */
	public void setUdpPort(int udpPort) {
		this.udpPort = udpPort;
	}
	/**
	 * @return rootRoomId
	 */
	public String getRootRoomId() {
		return rootRoomId;
	}
	/**
	 * @param rootRoomId 要设置的 rootRoomId
	 */
	public void setRootRoomId(String rootRoomId) {
		this.rootRoomId = rootRoomId;
	}
	/**
	 * @return rootX
	 */
	public int getRootX() {
		return rootX;
	}
	/**
	 * @param rootX 要设置的 rootX
	 */
	public void setRootX(int rootX) {
		this.rootX = rootX;
	}
	/**
	 * @return rootY
	 */
	public int getRootY() {
		return rootY;
	}
	/**
	 * @param rootY 要设置的 rootY
	 */
	public void setRootY(int rootY) {
		this.rootY = rootY;
	}
	/**
	 * @return remoteAddr
	 */
	public String getRemoteAddr() {
		return remoteAddr;
	}
	/**
	 * @param remoteAddr 要设置的 remoteAddr
	 */
	public void setRemoteAddr(String remoteAddr) {
		this.remoteAddr = remoteAddr;
	}
	/**
	 * @return remotePort
	 */
	public int getRemotePort() {
		return remotePort;
	}
	/**
	 * @param remotePort 要设置的 remotePort
	 */
	public void setRemotePort(int remotePort) {
		this.remotePort = remotePort;
	}
	public int getdayLength() {
		return dayLength;
	}
	public void setdayLength(int dayLength) {
		this.dayLength = dayLength;
	}
	public int getappSendLength() {
		return appSendLength;
	}
	public void setappSendLength(int appSendLength) {
		this.appSendLength = appSendLength;
	}
	public String getupperAddr() {
		return upperAddr;
	}
	public void setUpperAddr(String upperAddr) {
		this.upperAddr = upperAddr;
	}
	public int getupperPort() {
		return upperPort;
	}
	public void setUpperPort(int upperPort) {
		this.upperPort = upperPort;
	}
	public String getftpuser() {
		return ftpuser;
	}
	public void setftpuser(String ftpuser) {
		this.ftpuser = ftpuser;
	}
	public String getftphost() {
		return ftphost;
	}
	public void setftphost(String ftphost) {
		this.ftphost = ftphost;
	}
	public String getftpPwd() {
		return ftpPwd;
	}
	public void setftpPwd(String ftpPwd) {
		this.ftpPwd = ftpPwd;
	}
	public int getftpPort() {
		return ftpPort;
	}
	public void setftpPort(int ftpPort) {
		this.ftpPort = ftpPort;
	}
	public String getserverIp() {
		return serverIp;
	}
	public void setserverIp(String serverIp) {
		this.serverIp = serverIp;
	}
}
