package com.hit.heat.model;

import java.util.ArrayList;
import java.util.List;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

/* @lhy Lhy
 * @date 2015年12月5日
 * @des  
 */
public class FloorInfor {
	
	private String floorName;
	private String panelImagePath;
	private List<RoomInfor> rooms;
	private List<Location> nodes;
	public FloorInfor(String floorName,String panelImagePath,List<RoomInfor> rooms){
		this.floorName = floorName;
		this.panelImagePath = panelImagePath;
		this.rooms = rooms;
		nodes = new ArrayList<Location>();
	}
	public void addNode(Location location){
		nodes.add(location);
	}
	
	public List<Location> getNodes(){
		return nodes;
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

	/**
	 * @return panelImagePath
	 */
	public String getPanelImagePath() {
		return panelImagePath;
	}
	/**
	 * @param panelImagePath 要设置的 panelImagePath
	 */
	public void setPanelImagePath(String panelImagePath) {
		this.panelImagePath = panelImagePath;
	}
	/**
	 * @return rooms
	 */
	public List<RoomInfor> getRooms() {
		return rooms;
	}
	/**
	 * @param rooms 要设置的 rooms
	 */
	public void setRooms(List<RoomInfor> rooms) {
		this.rooms = rooms;
	}
	
	public JSONObject toJsonObject() throws JSONException{
		JSONObject json = new JSONObject();
		json.put("fName", floorName);
		json.put("imgPath", panelImagePath);
		JSONArray array = new JSONArray();
		for(RoomInfor room : rooms){
			array.put(room.toJsonObject());
		}
		json.put("rooms", array);
		return json;
	}
	
	public static FloorInfor fromJsonObject(JSONObject object) throws JSONException{
		//System.out.println(object.toString());
		List<RoomInfor> roomInfors = new ArrayList<RoomInfor>();
		JSONArray array = object.getJSONArray("rooms");
		for(int i = 0;i<array.length();++i){
			roomInfors.add(RoomInfor.fromJsonObject(array.getJSONObject(i)));
		}
		return new FloorInfor(object.getString("fName"), object.getString("imgPath"), roomInfors);
	}
	
	@Override
	public String toString(){
		StringBuilder sb = new StringBuilder();
		sb.append("floorName :" + floorName + "\r\n");
		sb.append("imgPath :" + panelImagePath + "\r\n");
		for(RoomInfor roomInfor : rooms){
			sb.append(roomInfor.toString() + "\r\n");
		}
		return sb.toString().substring(0, sb.length() - 2);
	}
	
//	public static void main(String[] args) throws JSONException {
//		List<RoomInfor> rooms = new ArrayList<RoomInfor>();
//		rooms.add(new RoomInfor("501", new Point(1,2)).setLtPoint(new Point(0,0)).setWidth(10).setHeight(10));
//		rooms.add(new RoomInfor("502", new Point(1,2)).setLtPoint(new Point(0,0)).setWidth(10).setHeight(10));
//		rooms.add(new RoomInfor("503", new Point(1,2)).setLtPoint(new Point(0,0)).setWidth(10).setHeight(10));
//		FloorInfor floorInfor = new FloorInfor("新技术楼", "images\\location.jpg", rooms);
//		JSONObject json = floorInfor.toJsonObject();
//		FloorInfor floorInfor2 = FloorInfor.fromJsonObject(json);
//		System.out.println(floorInfor.toString());
//		System.out.println(floorInfor2.toString());
//	}
}
