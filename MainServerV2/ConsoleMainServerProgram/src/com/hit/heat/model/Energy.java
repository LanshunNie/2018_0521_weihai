package com.hit.heat.model;

import com.hit.heat.util.Util;

public class Energy {
	private String id;// 节点id

	private long CPU;// cpu运行时间
	private long LPM;// lpm
	private long Send_time;// 发送时间
	private long Receive_time;// 接收时间
	private String ParentID;
	private float voltage;
	private int synTime;// 同步时差
	private String beacon;// beacon
	private int num_neighbors;// num_neighbors
	private int rtmetric;
	private int reboot; // 显示的重启计数
	private int r_reboot;// 收到的重启次数
	private int cycleTime;
	private String cycleTimeDirection;
	private String Nodecurrenttime;
	private float Current;
	public int getReboot() {
		return reboot;
	}

	public void setReboot(int reboot) {
		this.reboot = reboot;
	}

	public int getR_reboot() {
		return r_reboot;
	}

	public void setR_reboot(int r_reboot) {
		this.r_reboot = r_reboot;
	}

	public Energy(String id, long cPU, long lPM, long send_time, long receive_time, int voltage, String ParentID,
			int synTime, String beacon, int num_neighbors, int rtmetric, int r_reboot, int reboot, int cycleTime,
			String cycleTimeDirection, String currenttime,float current) {
		super();
		this.id = id;
		this.CPU = cPU;
		this.LPM = lPM;
		this.Send_time = send_time;
		this.Receive_time = receive_time;
		this.voltage = (float) (voltage / 1000.0);
		this.ParentID = ParentID;
		this.synTime = synTime;
		this.beacon = beacon;
		this.num_neighbors = num_neighbors;
		this.rtmetric = rtmetric;
		this.reboot = reboot;
		this.r_reboot = r_reboot;
		this.cycleTime = cycleTime;
		this.setCycleTimeDirection(cycleTimeDirection);
		this.Nodecurrenttime = currenttime;
		this.Current = current;
		
	}

	public Energy() {
		// TODO Auto-generated constructor stub
	}

	public String getBeacon() {
		return beacon;
	}

	public void setBeacon(String beacon) {
		this.beacon = beacon;
	}

	public int getNum_neighbors() {
		return num_neighbors;
	}
	public void setNum_neighbors(int num_neighbors) {
		this.num_neighbors = num_neighbors;
	}

	public int getRtmetric() {
		return rtmetric;
	}

	public void setRtmetric(int rtmetric) {
		this.rtmetric = rtmetric;
	}

	public int getSynTime() {
		return synTime;
	}

	public void setSynTime(int synTime) {
		this.synTime = synTime;
	}

	public float getVoltage() {
		return voltage;
	}

	public String getParentID() {
		return ParentID;
	}

	public void setVoltage(int voltage) {
		this.voltage = (float) (voltage / 1000.0);
	}

	public String getId() {
		return id;
	}

	public void setId(String id) {
		this.id = id;
	}

	public long getCPU() {
		return CPU;
	}

	public void setCPU(long cPU) {
		CPU = cPU;
	}

	public long getLPM() {
		return LPM;
	}

	public void setLPM(long lPM) {
		LPM = lPM;
	}
	
	public float getCurrent() {
		return Current;
	}

	public void setCurrent(float current) {
		Current = current;
	}
	
	public long getSend_time() {
		return Send_time;
	}

	public void setSend_time(long send_time) {
		Send_time = send_time;
	}

	public long getReceive_time() {
		return Receive_time;
	}

	public void setReceive_time(long receive_time) {
		Receive_time = receive_time;
	}

	public void setCycleTime(int cycleTime) {
		this.cycleTime = cycleTime;
	}

	public int getCycleTime() {
		// TODO Auto-generated method stub
		return cycleTime;
	}

	public String getCycleTimeDirection() {
		return cycleTimeDirection;
	}

	public void setCycleTimeDirection(String cycleTimeDirection) {
		this.cycleTimeDirection = cycleTimeDirection;
	}

	public String getNodecurrenttime() {
		String temp = Util.getCurrentTime();
//		System.out.println(temp);
		String DateCurrenttime = temp.substring(0, 11);
//		System.out.println(DateCurrenttime);
		Nodecurrenttime = DateCurrenttime + Nodecurrenttime;
		return Nodecurrenttime;
	}
	
	public String getNodecurrenttime2(String Date) {
		String temp = Util.getCurrentTime();
//		System.out.println(temp);
		//String DateCurrenttime = temp.substring(0, 11);
		
//		System.out.println(DateCurrenttime);
		Nodecurrenttime = Date+ " " + Nodecurrenttime;
		return Nodecurrenttime;
	}

}
