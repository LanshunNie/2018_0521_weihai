package com.hit.heat.model;

import java.awt.Point;

/* @lhy Lhy
 * @date 2015年11月19日
 * @des  
 */
public class Node {
	private String nodeId;// 节点nodeid
	private String parentId;//父节点的nodeid
	private Point point;
	private int level;
	public Node(String nodeId, String parentId) {
		this.nodeId = nodeId;
		this.parentId = parentId;
		level = 0;
	}
	
	/**
	 * @return level
	 */
	public int getLevel() {
		return level;
	}


	/**
	 * @param level 要设置的 level
	 */
	public void setLevel(int level) {
		this.level = level;
	}


	/**
	 * @return nodeId
	 */
	public String getNodeId() {
		return nodeId;
	}

	/**
	 * @param nodeId
	 *            要设置的 nodeId
	 */
	public void setNodeId(String nodeId) {
		this.nodeId = nodeId;
	}

	/**
	 * @return parentId
	 */
	public String getParentId() {
		return parentId;
	}

	/**
	 * @param parentId
	 *            要设置的 parentId
	 */
	public void setParentId(String parentId) {
		this.parentId = parentId;
	}

	/**
	 * @return point
	 */
	public Point getPoint() {
		return point;
	}

	/**
	 * @param point
	 *            要设置的 point
	 */
	public void setPoint(Point point) {
		this.point = point;
	}
	/* （非 Javadoc）
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
		// TODO 自动生成的方法存根
		StringBuilder sb = new StringBuilder();
		sb.append("nodeId=");
		sb.append(nodeId);
		sb.append("\r\nparentId=");
		if(parentId == null){
			sb.append("null");
		}else{
			sb.append(parentId);
		}
		sb.append("\r\npoint=");
		if(point == null){
			sb.append("null");
		}else{
			sb.append(point.toString());
		}
		sb.append("\r\nlevel");
		sb.append(level);
		return sb.toString();
	}
}
