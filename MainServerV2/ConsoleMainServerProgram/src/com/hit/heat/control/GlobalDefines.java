package com.hit.heat.control;

/* @lhy Lhy
 * @date 2015年12月21日
 * @des  全局定义
 */
public class GlobalDefines {

	/**
	 * @lhy Lhy
	 * @date 2015年12月21日
	 * @des 定义全局消息类型
	 */
	public class GlobalCmd{
		//单播 多播 读表指令
		public static final byte G_DEF_READ_DATA = (byte) 0X80;				//多播纯指令
		
		public static final byte G_DEF_CTL_ACK_READ_DATA = (byte)0X82; 		//多播 指令+bitmap
		public static final char G_DEF_UNICAST_ACK_DATA = '3';			//单播指令
		public static final char G_DEF_UNICAST_CONFIG_PERIOD = '4';		//单播节点配置信息
		public static final char G_DEF_MCAST_CONFIG_PERIOD = '5';		//多播上报周期配置
		public static final char G_DEF_HEART = 'h';						//心跳包
		
		public static final byte G_SCHEDULE_CONFIG = (byte)0x42; //配置时间调度标志
		
		//暂定不需要回复
		public static final byte G_DEF_REPORT_SYS = (byte)0x00; // 上报 节点时间+拓扑+能耗+采样电压
		
		public static final byte G_DEF_REPORT_NET = (byte)0x01;//上报 网络参数
		
		//网络配置 回复“h”
		public static final byte G_DEF_RESTART = (byte)0xC0;
		public static final byte G_DEF_REBOOT = (byte)0xC1;
		
		public static final byte G_HEART_CONFIG_PERIOD = (byte)0x41;//配置上报周期
		
		public static final byte G_DEF_SEND_NET = (byte)0x40; //配置网络参数
	}
	
	public class GlobalEditType{
		public static final char G_DEF_Edit_Over = 'a';
		public static final char G_DEF_Edit_Position = 'b';
		public static final char G_DEF_Edit_leftTop = 'c';
		public static final char G_DEF_Edit_RightBot = 'd';
	}
	/**
	 * @lhy Lhy
	 * @date 2016年3月24日
	 * @des 全局面板下标索引
	 */
	public class GlobalTabIndex{
		public static final int G_DEF_LocalConfig = 0;//本地配置
		public static final int G_DEF_PhyLocation = 1;//物理位置
		public static final int G_DEF_ParamConfig = 2;//参数配置
		public static final int G_DEF_NetTest 	  = 3;//网络测试
		public static final int G_DEF_CmdForward  = 4;//指令下发
		public static final int G_DEF_DataQuery   = 5;//数据查询
		public static final int G_DEF_Topology	  = 6;//拓扑结构
		public static final int G_DEF_OutputMsg	  = 7;//输出窗口
	}
	public class GlobalSynLevelConfig{
		public static final int G_SYN_CONFIG_INIT_LEVEL=1;
		public static final int G_SYN_CONFIG_LEVEL =255;
	}
	//节点ip与热表id的对应关系
	public class GlobalIpVsId{
		public static final int G_DEF_IdLocation = 9;
		public static final int G_DEF_IdLength = 4;
	}

}
