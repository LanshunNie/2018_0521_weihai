package com.hit.heat.util;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map.Entry;

import com.hit.heat.model.Energy;

public class rdc_EF_Control {

	private String nodeAddr;
	private short netDataCount;
	private short appDataCount;
	private float current;// uA
	private int cycle_time;
	private String direction;

	private static HashMap<String, rdc_EF_Control> rdcEFMap = new HashMap<String, rdc_EF_Control>();
	private static Byte rdcStartFlag = 1;
	private static int baseCycleTime = 8192;
	private static Integer current_budget;// nA
	private static Integer current_guard;// nA

	private static final int send_ac = 157540;// 157540;// 44680;
	private static final int receice_ac = 25000;// 25000;
	private static final float lpm_ac = (float) (4.256);
	private static final int cpu_ac = 3280;
	private static final String CYCLE_TIME_INCREASE = "1";
	private static final String CYCLE_TIME_DECREASE = "2";

	public byte getRdcStartFlag() {
		synchronized (rdc_EF_Control.rdcStartFlag) {
			return rdc_EF_Control.rdcStartFlag;

		}
	}

	public void setRdcStartFlag(byte flag) {
		synchronized (rdc_EF_Control.rdcStartFlag) {
			rdc_EF_Control.rdcStartFlag = flag;

		}
	}

	public String getNodeAddr() {
		return nodeAddr;
	}

	public void setNodeAddr(String nodeAddr) {
		this.nodeAddr = nodeAddr;
	}

	public short getNetDataCount() {
		return netDataCount;
	}

	public void setNetDataCount(short netDataCount) {
		this.netDataCount = netDataCount;
	}

	public short getAppDataCount() {
		return appDataCount;
	}

	public void setAppDataCount(short appDataCount) {
		this.appDataCount = appDataCount;
	}

	public float getCurrent() {
		return current;
	}

	public void setCurrent(Float current) {
		this.current = current;
	}

	public int getCycle_time() {
		return cycle_time;
	}

	public void setCycle_time(int cycle_time) {
		this.cycle_time = cycle_time;
	}

	public int getCurrent_budget() {
		synchronized (rdc_EF_Control.current_budget) {
			return rdc_EF_Control.current_budget;

		}
	}

	public void setCurrent_budget(int current_budget) {
		synchronized (rdc_EF_Control.current_budget) {
			rdc_EF_Control.current_budget = current_budget;
		}
	}

	public int getCurrent_guard() {
		synchronized (rdc_EF_Control.current_guard) {
			return rdc_EF_Control.current_guard;
		}
	}

	public void setCurrent_guard(int current_guard) {
		synchronized (rdc_EF_Control.current_guard) {
			rdc_EF_Control.current_guard = current_guard;
		}
	}

	public rdc_EF_Control(String addr) {
		this.nodeAddr = addr;
		this.appDataCount = 1;
		this.netDataCount = 0;
		this.current = 0;
		this.cycle_time = 0;

	}

	public String getDirection() {
		return direction;
	}

	public void setDirection(String direction) {
		this.direction = direction;
	}

	public rdc_EF_Control(String addr, Energy energy) {
		this.nodeAddr = addr;
		this.appDataCount = 0;
		this.netDataCount = 1;
		this.current = calCurrent(energy);
		this.cycle_time = energy.getCycleTime();
		this.direction = energy.getCycleTimeDirection();
	}

	public rdc_EF_Control(int current_budget, int current_guard) {
		rdc_EF_Control.current_budget = current_budget;
		rdc_EF_Control.current_guard = current_guard;
	}

	public static float calCurrent(Energy en) {

		float avg_cur = (en.getCPU() * cpu_ac + en.getLPM() * lpm_ac
				+ en.getReceive_time() * receice_ac + en.getSend_time()
				* send_ac)
				/ (en.getCPU() + en.getLPM());

		return avg_cur;

	}

	public static void updateNetData(String addr, Energy energy) {
		synchronized (rdcEFMap) {

			if (!rdcEFMap.containsKey(addr)) {

				rdc_EF_Control rdcControl = new rdc_EF_Control(addr, energy);
				rdcEFMap.put(addr, rdcControl);

			} else {

				short tempCount = (short) (rdcEFMap.get(addr).getNetDataCount() + 1);
				rdcEFMap.get(addr).setNetDataCount(tempCount);
				rdcEFMap.get(addr).setCurrent(calCurrent(energy));
				rdcEFMap.get(addr).setCycle_time(energy.getCycleTime());
				rdcEFMap.get(addr).setDirection(energy.getCycleTimeDirection());
			}

		}
	}

	public static void updateAppData(String addr) {

		synchronized (rdcEFMap) {
			if (!rdcEFMap.containsKey(addr)) {

				rdc_EF_Control rdcControl = new rdc_EF_Control(addr);
				rdcEFMap.put(addr, rdcControl);

			} else {

				short tempCount = (short) (rdcEFMap.get(addr).getAppDataCount() + 1);
				rdcEFMap.get(addr).setAppDataCount(tempCount);

			}
		}

	}

	public void rdcAnalysis(int appDataRound, int netDataRound) {
		synchronized (rdc_EF_Control.rdcEFMap) {

			int lowPRR = 0;
			int overCount = 0;
			int highCurrentCount = 0;
			int nodeCount = rdcEFMap.size();
			Iterator<Entry<String, rdc_EF_Control>> iterator = rdcEFMap
					.entrySet().iterator();
			while (iterator.hasNext()) {
				Entry<String, rdc_EF_Control> entry = iterator.next();
				if (getRdcStartFlag() == 1) {
					if (entry.getValue().getAppDataCount() > appDataRound / 5
							|| entry.getValue().getNetDataCount() > netDataRound / 5) {
						lowPRR++;
					}

					float ratio = (float) entry.getValue().getCycle_time()
							/ baseCycleTime;
					if (entry.getValue().getDirection()
							.equals(CYCLE_TIME_INCREASE)) {
						if (ratio > (float) 1.8) {
							overCount++;
						}
					}
					ratio = (float) baseCycleTime
							/ entry.getValue().getCycle_time();

					if (entry.getValue().getDirection()
							.equals(CYCLE_TIME_DECREASE)) {
						if (ratio > (float) 1.8) {
							overCount++;
						}
					}

				} else {
					if (entry.getValue().getCurrent() > entry.getValue()
							.getCurrent_budget()
							+ entry.getValue().getCurrent_guard()) {
						highCurrentCount++;
					}
				}

				entry.getValue().setAppDataCount((short) 0);
				entry.getValue().setNetDataCount((short) 0);

			}

			if (getRdcStartFlag() == 1 && lowPRR > nodeCount / 10
					&& overCount > nodeCount / 3) {
				setRdcStartFlag((byte) 0);
			}

			if (getRdcStartFlag() == 0 && highCurrentCount > nodeCount / 5) {
				setRdcStartFlag((byte) 1);
			}
		}
	}
}
