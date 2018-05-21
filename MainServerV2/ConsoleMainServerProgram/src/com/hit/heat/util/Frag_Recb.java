package com.hit.heat.util;

import java.io.IOException;
import java.util.ArrayList;

import java.util.HashMap;
import java.util.Timer;
import java.util.TimerTask;

import com.hit.heat.cn.Main;

public class Frag_Recb {
	private String addr;
	private int Frag_TotalNum;
	private byte seqnum;
	private byte frag_type;
	private Timer timer;
	private int frag_index;
	private boolean frag_End = false;
	private HashMap<String, Integer> orpl_seq_maHashMap;

	private ArrayList<String> fragList;
	public static final int FRAG_TOTAL_NUM = 10;

	public static final int ORPL_SEQ_INDEX = 0;
	public static final int FRAGTYPE_INDEX = 2;
	public static final int FRAG_SEQ_INDEX = 3;
	public static final int DATA_INDEX = 6;
	public static HashMap<String, Frag_Recb> fragHashMap = new HashMap<String, Frag_Recb>();

	public Frag_Recb(String addr, byte[] b) {
		this.addr = addr;
		this.orpl_seq_maHashMap = new HashMap<String, Integer>();
		this.timer = new Timer();
		fragList = new ArrayList<String>();
		for (int i = 0; i < FRAG_TOTAL_NUM; i++) {
			fragList.add(i, "");
		}
		this.setSeqnum(b[FRAG_SEQ_INDEX]);
		timer.schedule(new timerTask(addr), 60*3* 1000);// 60*3* 1000
		//timer.schedule(new timerTask(addr), 60*3* 1000);// 60*3* 1000
	}

	private void insertFrag(String frag, int index) {
		if (index > FRAG_TOTAL_NUM) {
			System.err.println("error");
			return;
		}
		this.fragList.set(index, frag);
	}

	public void getPacket() {
		StringBuilder sb = new StringBuilder();
		// for (String str : this.fragList) {
		for (int i = 0; i < this.Frag_TotalNum; i++) {
			sb.append(this.fragList.get(i));
			this.fragList.set(i, "");
		}
		// System.out.println(this.addr + " " + sb.toString());
		// this.orpl_seq_maHashMap.clear();
		// this.timer.cancel();
		/**********************************************************/

		rdc_EF_Control.updateAppData(this.addr);

		Main.coServer.sent_message(this.addr,
				Util.formatByteStrToByte(sb.toString()));
	}

	private void finishRecb() {
		if (!this.frag_End) {
			// System.out.println("not end ");

			return ;
		}

		for (int i = 0; i < this.Frag_TotalNum; i++) {
			if (this.fragList.get(i).equals("")) {
				return ;
			}
		}
		this.getPacket();

	}

	public byte[] frag_phase(byte[] b) {
		if (orpl_seq_maHashMap.containsKey(this.getOrpl_Seq(b))) {
			return null;
		}
		String messageString = new String();
		orpl_seq_maHashMap.put(this.getOrpl_Seq(b), 1);
		this.setFragType(b[FRAGTYPE_INDEX]);



		if (!this.getIf_frag(this.frag_type)) {
			byte[] bt = new byte[b.length - 3];
			System.err.println("not frag");
			System.arraycopy(b, 3, bt, 0, b.length-3);
			Main.coServer.sent_message(addr, bt);
			return bt;

		}

		byte[] bt = new byte[b.length - DATA_INDEX];
		for (int i = DATA_INDEX; i < b.length; i++) {
			bt[i - DATA_INDEX] = b[i];
		}

		if (this.seqnum != b[FRAG_SEQ_INDEX]) {
			this.setSeqnum(b[FRAG_SEQ_INDEX]);
			for (int i = 0; i < FRAG_TOTAL_NUM; i++) {
				fragList.set(i, "");
			}
			frag_phase(b);
			return null;
		}
		this.setSeqnum(b[FRAG_SEQ_INDEX]);

		this.insertFrag(Util.formatByteToByteStr(bt),
				this.getFragIndex(this.frag_type));

		this.finishRecb();
		return null;
	}

	private int getFragIndex(byte b) {
		this.frag_index = this.bitToint(this.byteToBit(b).substring(2, 7));
		this.setFragEnd(b);
		return this.bitToint(this.byteToBit(b).substring(2, 7));
	}

	private void setFragEnd(byte b) {

		if ((this.bitToint(this.byteToBit(b).substring(7))) == 0) {
			this.setFragTN(this.frag_index + 1);
			this.frag_End = true;

		}
	}

	private boolean getIf_frag(byte b) {
		if (this.byteToBit(b).substring(0, 2).equals("11")) {
			return true;
		} else {
			return false;
		}
	}

	private void setSeqnum(byte b) {
		this.seqnum = b;
	}

	private void setFragType(byte b) {
		this.frag_type = b;
	}

	private void setFragTN(int total) {
		this.Frag_TotalNum = total;
	}

	private String byteToBit(byte b) {
		return "" + (byte) ((b >> 7) & 0x1) + (byte) ((b >> 6) & 0x1)
				+ (byte) ((b >> 5) & 0x1) + (byte) ((b >> 4) & 0x1)
				+ (byte) ((b >> 3) & 0x1) + (byte) ((b >> 2) & 0x1)
				+ (byte) ((b >> 1) & 0x1) + (byte) ((b >> 0) & 0x1);
	}

	private int bitToint(String str) {

		return Integer.parseInt(str, 2);
	}

	class timerTask extends TimerTask {
		private String addr;

		public timerTask(String addr) {
			this.addr = addr;
		}

		@Override
		public void run() {
			// TODO Auto-generated method stub
			if (fragHashMap.containsKey(addr)){
				fragHashMap.get(addr).orpl_seq_maHashMap.clear();

				fragHashMap.remove(addr);
			}

			timer.cancel();
		}

	}

	private String getOrpl_Seq(byte[] b) {
		byte[] bt = new byte[2];
		bt[0] = b[ORPL_SEQ_INDEX];
		bt[1] = b[ORPL_SEQ_INDEX + 1];
		return Util.formatByteToByteStr(bt);
	}

	public static void main(String[] ARG) {

		byte[] fr = new byte[60];
		fr[0] = (byte) 55;
		fr[1] = (byte) 194;
		fr[2] = (byte) 191;
		fr[3] = (byte) 194;
		fr[4] = (byte) 194;
		fr[5] = (byte) 194;
		fr[6] = (byte) (2);
		for (int i = 7; i < 59; i++) {
			fr[i] = (byte) (1);

		}
		fr[59] = (byte) (2);
		String addr = "ssdg";
		Frag_Recb fb = new Frag_Recb(addr, fr);
		fragHashMap.put(addr, fb);
		System.out.println(fb.frag_phase(fr));
		System.out.println("-------------------------------");

		fr[0] = (byte) 54;
		fr[1] = (byte) 194;
		fr[2] = (byte) 191;
		fr[3] = (byte) 194;
		fr[4] = (byte) 194;
		fr[5] = (byte) 194;
		fr[6] = (byte) (2);
		for (int i = 7; i < 59; i++) {
			fr[i] = (byte) (1);

		}
		fr[59] = (byte) (2);
//		System.out.println(fb.frag_phase(fr));
		for(byte r:fb.frag_phase(fr)){
			System.out.print(r);
		}
		System.out.println("-------------------------------");

		fr[0] = (byte) 53;
		fr[1] = (byte) 194;
		fr[2] = (byte) 196;
		fr[3] = (byte) 194;
		fr[4] = (byte) 194;
		fr[5] = (byte) 194;
		fr[6] = (byte) (2);
		for (int i = 7; i < 59; i++) {
			fr[i] = (byte) (1);

		}
		fr[59] = (byte) (2);

		System.out.println(fb.frag_phase(fr));

	}

}
