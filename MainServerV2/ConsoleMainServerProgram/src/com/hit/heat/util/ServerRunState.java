/*ServerRunState.java
 * 2015年7月29日
 * Lhy
*/
package com.hit.heat.util;

/**
 * @lhy Lhy
 *
 */
public class ServerRunState {

	public static final int SRS_INIT = 0;
	public static final int SRS_RUNNING = 1;
	public static final int SRS_STOP = 2;
	public static final int SRS_ERROR = 3;
	public static final String[] SRS_AEEAY = {
		"等待启动",
		"正在运行",
		"停止运行",
		"运行出错"
	};
	
	public static String getRunState(int index){
		if(index < SRS_INIT || index > SRS_ERROR){
			return null;
		}
		return SRS_AEEAY[index];
	}
}
