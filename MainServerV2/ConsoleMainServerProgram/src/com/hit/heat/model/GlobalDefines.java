package com.hit.heat.model;

public class GlobalDefines {
	public class GlobalCollectView{
		//节点时间的时：分：秒
		public static final int G_DEF_ENERGY_LENGTH0 = 3;
		public static final int G_DEF_ENERGY_LENGTH1 = 6;
		public static final int G_DEF_CRO_TIME_HOUR = 0;
		public static final int G_DEF_CRO_TIME_MINUTE = 1;
		public static final int G_DEF_CRO_TIME_SECOND = 2;
		//父亲节点ID高低位
		public static final int G_DEF_PARENTID_HIGH = 3;
		public static final int G_DEF_PARENTID_LOW = 4;
		//CPU、LPM、发送、接收时间 每个6字节
		public static final int G_DEF_ENERGY_CPU = 5;
		public static final int G_DEF_ENERGY_LPM = 11;
		public static final int G_DEF_ENERGY_SEND = 17;
		public static final int G_DEF_ENERGY_RECEIVE = 23;
		//电压 2字节
		public static final int G_DEF_ENERGY_VOLTAGE = 29;
		//beacon 2字节
		public static final int G_DEF_ENERGY_BEACON = 31;
		//num_neighbors
		public static final int G_DEF_ENERGY_NUM_NEGINBORS = 33;
		//rtimetric
		public static final int G_DEF_ENERGY_RTMERTIC = 35;
		public static final int G_DEF_ENERGY_SYNTIME = 37;
		public static final int G_DEF_ENERGY_REBOOT = 38;
		
		public static final int G_DEF_ENERGY_CYCLETIME = 41;
		public static final int G_DEF_ENERGY_CYCLETIMEDIRECTION = 43;
		public static final int G_DEF_ENERGY_CURRENT = 54;
		
		
//		public static final int send_power = ;
//				cpu	2598
//				lpm	3.9
//				tx	54672
//				rx	22000
//				active_irq	1299.00 
//				inactive_irq	1.2

	}
}
