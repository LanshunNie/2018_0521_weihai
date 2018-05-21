package com.hit.heat.model;

import java.awt.Point;

import org.json.JSONException;
import org.json.JSONObject;

/* @lhy Lhy
 * @date 2015年12月5日
 * @des  
 */
public class RoomInfor {

	private String roomId;
	private Point roomLocation;
	private Point ltPoint;
	private int width;
	private int height;
//	private Point rtPoint;
//	private Point lbPoint;
//	private Point rbPoint;
	public RoomInfor(String roomId,Point roomPoint){
		this.roomId = roomId;
		this.roomLocation = roomPoint;
	}
	
	public RoomInfor setLtPoint(Point ltPoint){
		
		if(this.ltPoint == null){
			this.ltPoint = new Point(ltPoint);
		}else{
			this.ltPoint.x = ltPoint.x;
			this.ltPoint.y = ltPoint.y;
		}
		return this;
	}
	
	public RoomInfor setWidth(int width){
		this.width = width;
		return this;
	}
	
	public RoomInfor setHeight(int height){
		this.height = height;
		return this;
	}
	
	public Point getCenterPoint(){
		return new Point(ltPoint.x + width / 2,ltPoint.y + height/2);
	}
	
//	public RoomInfor setLbPoint(Point lbPoint){
//		this.lbPoint = lbPoint;
//		return this;
//	}
//	public RoomInfor setRtPoint(Point rtPoint){
//		this.rtPoint = rtPoint;
//		return this;
//	}
//	public RoomInfor setRbPoint(Point rbPoint){
//		this.rbPoint = rbPoint;
//		return this;
//	}
	
	
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
	 * @return roomLocation
	 */
	public Point getRoomLocation() {
		return roomLocation;
	}
	/**
	 * @param roomLocation 要设置的 roomLocation
	 */
	public void setRoomLocation(Point roomLocation) {
		if(this.roomLocation == null){
			this.roomLocation = new Point(roomLocation);
		}else{
			this.roomLocation.x = roomLocation.x;
			this.roomLocation.y = roomLocation.y;
		}
		
	}
	
	/**
	 * @return ltPoint
	 */
	public Point getLtPoint() {
		return ltPoint;
	}
	
	public int getRBPointX(){
		return ltPoint.x + width;
	}
	public int getRBPointY(){
		return ltPoint.y + height;
	}
//	/**
//	 * @return rtPoint
//	 */
//	public Point getRtPoint() {
//		return rtPoint;
//	}
//	/**
//	 * @return lbPoint
//	 */
//	public Point getLbPoint() {
//		return lbPoint;
//	}
//	/**
//	 * @return rbPoint
//	 */
//	public Point getRbPoint() {
//		return rbPoint;
//	}
//	
	
	/**
	 * @return width
	 */
	public int getWidth() {
		return width;
	}

	/**
	 * @return height
	 */
	public int getHeight() {
		return height;
	}

	/**
	 * 
	 * @des 从json对象中解析属性，构造 RoomInfor对象
	 * @param object
	 * @return
	 * @throws JSONException
	 */
	public static RoomInfor fromJsonObject(JSONObject object) throws JSONException{
		
		Point ltPoint = new Point(object.getInt("ltx"),object.getInt("lty"));
//		Point lbPoint = new Point(object.getInt("lbx"),object.getInt("lby"));
//		Point rtPoint = new Point(object.getInt("rtx"),object.getInt("rty"));
//		Point rbPoint = new Point(object.getInt("rbx"),object.getInt("rby"));
		return new RoomInfor(object.getString("id"), new Point(object.getInt("x"),object.getInt("y"))).
				setLtPoint(ltPoint).setWidth(object.getInt("width")).setHeight(object.getInt("height"));
	}
	
	/**
	 * 
	 * @des 将对象转换为Json对象 
	 * @return
	 * @throws JSONException
	 */
	public JSONObject toJsonObject() throws JSONException{
		JSONObject object = new JSONObject();
		object.put("id", roomId);
		object.put("x", roomLocation.x);
		object.put("y", roomLocation.y);
		object.put("ltx", ltPoint.x);
		object.put("lty", ltPoint.y);
//		object.put("lbx", lbPoint.x);
//		object.put("lby", lbPoint.y);
//		object.put("rtx", rtPoint.x);
//		object.put("rty", rtPoint.y);
//		object.put("rbx", rbPoint.x);
//		object.put("rby", rbPoint.y);
		object.put("width", width);
		object.put("height", height);
		return object;
	}
	
	@Override
	public String toString(){
		return "roomId :" + roomId + " roomLocation:(" + String.valueOf(roomLocation.x) +"," + String.valueOf(roomLocation.y) +")"
				+ " ltPos:(" + String.valueOf(ltPoint.x) + "," + String.valueOf(ltPoint.y) + ") width:" + String.valueOf(width)
				+ " height:" + String.valueOf(height);
	}
	
//	public static void main(String[] args) throws JSONException {
//		RoomInfor roomInfor = new RoomInfor("101", new Point(1,2)).setLtPoint(new Point(0,0)).setWidth(10).setHeight(10);
//		//System.out.println(roomInfor);
//		JSONObject object = roomInfor.toJsonObject();
//		System.out.println(object.toString());
//		RoomInfor roomInfor2 = RoomInfor.fromJsonObject(object);
//		System.out.println(roomInfor2.toString());
//	}
}
