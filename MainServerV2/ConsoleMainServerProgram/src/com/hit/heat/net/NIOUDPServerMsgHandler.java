package com.hit.heat.net;

/**
 * @lhy Lhy
 * @date 2015年11月17日
 * @des
 */
public interface NIOUDPServerMsgHandler {
	byte[] messageHandler(String addr,byte[] message);
}