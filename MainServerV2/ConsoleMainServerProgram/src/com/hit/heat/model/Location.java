package com.hit.heat.model;

import java.awt.Point;

import org.json.JSONException;
import org.json.JSONObject;

/* @lhy Lhy
 * @date 2015年11月13日
 * @des  
 */
public class Location {

	private int id;
	//private int valveId;
	private String addr;
	private String floorName;
	private String roomId;
	private Point pos;
	private int colorIndex;
	
	public Location(int id,/*int valveId,*/String addr,String roomId,Point pos){
		this.id = id;
		///this.valveId = valveId;
		this.addr = addr;
		this.roomId = roomId;
		this.pos = pos;
		colorIndex = 0;
	}

	
	
	/**
	 * @return floorName
	 */
	public String getFloorName() {
		return floorName;
	}

	/**
	 * @param floorName 要设置的 floorName
	 */
	public void setFloorName(String floorName) {
		this.floorName = floorName;
	}

//	/**
//	 * @return valveId
//	 */
//	public int getValveId() {
//		return valveId;
//	}
//
//	/**
//	 * @param valveId 要设置的 valveId
//	 */
//	public void setValveId(int valveId) {
//		this.valveId = valveId;
//	}

	/**
	 * @return id
	 */
	public int getId() {
		return id;
	}

	/**
	 * @param id 要设置的 id
	 */
	public void setId(int id) {
		this.id = id;
	}

	/**
	 * @return addr
	 */
	public String getAddr() {
		return addr;
	}

	/**
	 * @param addr 要设置的 addr
	 */
	public void setAddr(String addr) {
		this.addr = addr;
	}

	/**
	 * @return roomId
	 */
	public String getRoomId() {
		return roomId;
	}

	/**
	 * @param roomId 要设置的 roomId
	 */
	public void setRoomId(String roomId) {
		this.roomId = roomId;
	}

	/**
	 * @return pos
	 */
	public Point getPos() {
		return pos;
	}

	/**
	 * @param pos 要设置的 pos
	 */
	public void setPos(Point pos) {
		this.pos = pos;
	}
	
	/**
	 * @return colorIndex
	 */
	public int getColorIndex() {
		return colorIndex;
	}

	/**
	 * @param colorIndex 要设置的 colorIndex
	 */
	public void setColorIndex(int colorIndex) {
		this.colorIndex = colorIndex;
	}

	public static Location parseFromJsonObject(JSONObject object,int valveId) throws JSONException{
		int id = object.getInt("id");
		String addr = object.getString("addr");
		String roomId = object.getString("roomId");
		return new Location(id, addr, roomId, new Point());
	}
	
	public String toFormatString(){
		return String.format("%d,%s,%s,%d,%d",id,addr,roomId,pos.x,pos.y);
	}
	
	/* （非 Javadoc）
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
		// TODO 自动生成的方法存根
		return "id : " + String.valueOf(id) + " ip地址 : " + addr + " room : "+ String.valueOf(roomId) +" location : (" + String.valueOf(pos.x) +","+ String.valueOf(pos.y)+ ")";
	}
	
	
}
