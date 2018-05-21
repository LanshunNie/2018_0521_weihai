package com.hit.heat.model;

/* @lhy Lhy
 * @date 2016年1月7日
 * @des  
 */
public class SystemParam {

	private boolean sysNodeHeartChFlag;//节点心跳包延迟改变标志
	private int sysNodeHeartDelay;//节点心跳包延迟(s)
	private int sysWaitRetransDelay; //每轮等待重传时间(s)
	private int sysNodeTransDelay;	//节点间传输间隔(s)
	private int sysNodeWaitAckDelay;//节点等待ACK延迟(s)
	private int sysMcastRetransTimes;//多播重传次数
	private int sysUcastRetransTimes;//单播重传次数
	private double sysMcastReliValue;//多播可靠性阀值
	private double sysUcastToMcastValue;//单播比例达到一定范围则转多播
	
	public SystemParam(){
		sysNodeHeartChFlag = false;
		sysNodeHeartDelay = 60;
		sysWaitRetransDelay = 60;
		sysNodeTransDelay = 10;
		sysNodeWaitAckDelay = 150;
		sysMcastRetransTimes = 3;
		sysUcastRetransTimes = 4;
		sysMcastReliValue =1.0D;
		sysUcastToMcastValue =0.7D;
	}
	
	/**
	 * @return sysNodeHeartChFlag
	 */
	public boolean isSysNodeHeartChFlag() {
		return sysNodeHeartChFlag;
	}

	/**
	 * @param sysNodeHeartChFlag 要设置的 sysNodeHeartChFlag
	 */
	public void setSysNodeHeartChFlag(boolean sysNodeHeartChFlag) {
		this.sysNodeHeartChFlag = sysNodeHeartChFlag;
	}



	/**
	 * @return sysNodeHeartDelay
	 */
	public int getSysNodeHeartDelay() {
		return sysNodeHeartDelay;
	}

	/**
	 * @param sysNodeHeartDelay 要设置的 sysNodeHeartDelay
	 */
	public void setSysNodeHeartDelay(int sysNodeHeartDelay) {
		this.sysNodeHeartDelay = sysNodeHeartDelay;
	}


	/**
	 * @return sysWaitRetransDelay
	 */
	public int getSysWaitRetransDelay() {
		return sysWaitRetransDelay;
	}
	/**
	 * @param sysWaitRetransDelay 要设置的 sysWaitRetransDelay
	 */
	public void setSysWaitRetransDelay(int sysWaitRetransDelay) {
		this.sysWaitRetransDelay = sysWaitRetransDelay;
	}
	/**
	 * @return sysNodeTransDelay
	 */
	public int getSysNodeTransDelay() {
		return sysNodeTransDelay;
	}
	/**
	 * @param sysNodeTransDelay 要设置的 sysNodeTransDelay
	 */
	public void setSysNodeTransDelay(int sysNodeTransDelay) {
		this.sysNodeTransDelay = sysNodeTransDelay;
	}
	/**
	 * @return sysNodeWaitAckDelay
	 */
	public int getSysNodeWaitAckDelay() {
		return sysNodeWaitAckDelay;
	}
	/**
	 * @param sysNodeWaitAckDelay 要设置的 sysNodeWaitAckDelay
	 */
	public void setSysNodeWaitAckDelay(int sysNodeWaitAckDelay) {
		this.sysNodeWaitAckDelay = sysNodeWaitAckDelay;
	}
	/**
	 * @return sysMcastRetransTimes
	 */
	public int getSysMcastRetransTimes() {
		return sysMcastRetransTimes;
	}
	/**
	 * @param sysMcastRetransTimes 要设置的 sysMcastRetransTimes
	 */
	public void setSysMcastRetransTimes(int sysMcastRetransTimes) {
		this.sysMcastRetransTimes = sysMcastRetransTimes;
	}
	/**
	 * @return sysUcastRetransTimes
	 */
	public int getSysUcastRetransTimes() {
		return sysUcastRetransTimes;
	}
	/**
	 * @param sysUcastRetransTimes 要设置的 sysUcastRetransTimes
	 */
	public void setSysUcastRetransTimes(int sysUcastRetransTimes) {
		this.sysUcastRetransTimes = sysUcastRetransTimes;
	}
	/**
	 * @return sysMcastReliValue
	 */
	public double getSysMcastReliValue() {
		return sysMcastReliValue;
	}
	/**
	 * @param sysMcastReliValue 要设置的 sysMcastReliValue
	 */
	public void setSysMcastReliValue(double sysMcastReliValue) {
		this.sysMcastReliValue = sysMcastReliValue;
	}
	/**
	 * @return sysUcastToMcastValue
	 */
	public double getSysUcastToMcastValue() {
		return sysUcastToMcastValue;
	}
	/**
	 * @param sysUcastToMcastValue 要设置的 sysUcastToMcastValue
	 */
	public void setSysUcastToMcastValue(double sysUcastToMcastValue) {
		this.sysUcastToMcastValue = sysUcastToMcastValue;
	}
	
	/* （非 Javadoc）
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
		// TODO 自动生成的方法存根
		return "sysNodeHeartDelay:" + sysNodeHeartDelay + " sysWaitRetransDelay:" + sysWaitRetransDelay + 
				" sysNodeTransDelay:" + sysNodeTransDelay + " sysNodeWaitAckDelay:" + sysNodeWaitAckDelay +
				" sysMcastRetransTimes:" + sysMcastRetransTimes + " sysUcastRetransTimes:" + sysUcastRetransTimes +
				" sysMcastReliValue:" + sysMcastReliValue + " sysUcastToMcastValue:" + sysUcastToMcastValue + "\r\n";
	}
}
