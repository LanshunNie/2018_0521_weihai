package com.hit.heat.model;

/* @lhy Lhy
 * @date 2016年5月13日
 * @des  
 */
public class CurrentTime {
	private int hour;
	private int minute;
	private int second;
	public CurrentTime(){
		
	}
	public CurrentTime(int hour,int minute,int second){
		this.hour=hour;
		this.minute =minute;
		this.second =second;
	}
	/**
	 * @return hour
	 */
	public int getHour() {
		return hour;
	}
	/**
	 * @param hour 要设置的 hour
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
	 * @param minute 要设置的 minute
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
	 * @param second 要设置的 second
	 */
	public void setSecond(int second) {
		this.second = second;
	}
	

}
