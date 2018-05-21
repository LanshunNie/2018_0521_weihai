package com.hit.heat.model;

/* @lhy Lhy
 * @date 2016年5月12日
 * @des  
 */
public class SynParameter {
	private int seqNum;
	private int level;
	private int hour;
	private int minute;
	private int second;
	private int period;
	private byte[] bitmap;
	private String bit;
	private boolean flag;

	public SynParameter() {

	}

	public SynParameter(int seqNum, int level, int hour, int minute, int second,int period, byte[] bitmap, boolean flag,
			String bit) {
		this.seqNum = seqNum;
		this.level = level;
		this.hour = hour;
		this.minute = minute;
		this.second = second;
		this.period = period;
		this.bitmap = bitmap;
		this.flag = flag;
		this.bit = bit;
	}

	public String getBit() {
		return bit;
	}

	public void setBit(String bit) {
		this.bit = bit;
	}

	/**
	 * @return flag
	 */
	public boolean isFlag() {
		return flag;
	}

	/**
	 * @param flag
	 *            要设置的 flag
	 */
	public void setFlag(boolean flag) {
		this.flag = flag;
	}

	/**
	 * @return seqNum
	 */
	public int getSeqNum() {
		return seqNum;
	}

	/**
	 * @param seqNum
	 *            要设置的 seqNum
	 */
	public void setSeqNum(int seqNum) {
		this.seqNum = seqNum;
	}

	/**
	 * @return level
	 */
	public int getLevel() {
		return level;
	}

	/**
	 * @param level
	 *            要设置的 level
	 */
	public void setLevel(int level) {
		this.level = level;
	}

	/**
	 * @return hour
	 */
	public int getHour() {
		return hour;
	}

	/**
	 * @param hour
	 *            要设置的 hour
	 */
	public void setHour(int hour) {
		this.hour = hour;
	}

	/**
	 * @return minute
	 */
	public int getMinute() {
		return minute;
	}

	/**
	 * @param minute
	 *            要设置的 minute
	 */
	public void setMinute(int minute) {
		this.minute = minute;
	}

	/**
	 * @return second
	 */
	public int getSecond() {
		return second;
	}

	/**
	 * @param second
	 *            要设置的 second
	 */
	public void setSecond(int second) {
		this.second = second;
	}

	/**
	 * @return bitmap
	 */
	public byte[] getBitmap() {
		return bitmap;
	}

	/**
	 * @param bitmap
	 *            要设置的 bitmap
	 */
	public void setBitmap(byte[] bitmap) {
		System.out.println("right");
		this.bitmap = bitmap;
	}
	
	/**
	 * @param period
	 *            要设置的 period
	 */
	public int getPeriod() {
		return period;
	}

	/**
	 * @param period
	 *            要设置的 period
	 */
	public void setPeriod(int period) {
		this.period = period;
	}
	



}
