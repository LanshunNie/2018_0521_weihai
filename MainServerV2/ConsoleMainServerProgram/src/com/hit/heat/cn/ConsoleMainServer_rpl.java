package com.hit.heat.cn;

//import java.io.BufferedReader;


import java.io.IOException;

//import java.io.InputStreamReader;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.SocketAddress;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Timer;
import java.util.TimerTask;

import org.json.JSONException;
import org.json.JSONObject;

import com.hit.heat.control.GlobalDefines;
import com.hit.heat.model.CurrentTime;
//import com.hit.heat.model.FloorInfor;
import com.hit.heat.model.Location;
import com.hit.heat.model.NetParameter;
import com.hit.heat.model.SynParameter;
import com.hit.heat.model.SystemParam;
import com.hit.heat.net.HeartOffLineHandler;
import com.hit.heat.net.HeartOnLineHandler;
import com.hit.heat.net.HeartProxy;
import com.hit.heat.net.NIOUDPServer;
import com.hit.heat.net.NIOUDPServerMsgHandler;
import com.hit.heat.net.NettyClient;
import com.hit.heat.net.NettyMsgHandler;
import com.hit.heat.net.NettyServer;
import com.hit.heat.net.ParamConfigProxy;
import com.hit.heat.net.ParamConfigResult;
import com.hit.heat.net.ProxyInvoke;
import com.hit.heat.net.UnicastProxy;
import com.hit.heat.util.BitMap;
//import com.hit.heat.util.DESPlus;
import com.hit.heat.util.GConfig;
import com.hit.heat.util.GSynConfig;
import com.hit.heat.util.Util;
import com.hit.heat.util.WriteDataToFile;
import com.sun.org.apache.xml.internal.security.exceptions.Base64DecodingException;
import com.sun.org.apache.xml.internal.security.utils.Base64;

/* @lhy Lhy
 * @date 2016年4月4日
 * @des  
 */
public class ConsoleMainServer_rpl {

	private GConfig config;// 外部文件包括ip地址和端口
	private GSynConfig synConfig;// 外部文件包括同步消息
	private SystemParam g_systemParam;// 外部文件包括重传次数等参数配置
	private NetParameter parameter;// 对应config文件的对象类
	private SynParameter synParameter;
	private WriteDataToFile mulicastDataFile;
	private WriteDataToFile cmdFile;
	private WriteDataToFile topoFile;
//	private WriteDataToFile unicastDataFile;
//	private WriteDataToFile offLineDataFile;

	private NettyClient nettyClient;// netty 客户端有关
	private NettyClient remoteClient;
	private NettyClient configRemoteNettyClient;// 配置信息
	private NettyClient netClient;
	private NettyServer nettyServer;// Netty NIo TCP 服务器 集中器前端

	private NettyServer webNettyServer;// 集中器前台模拟web

	private NIOUDPServer nioUdpServer;// 应用数据nio UDP服务器时
	private NIOUDPServer nioNetDataServer;// 网络参数 只接收心跳数据
	private NIOUDPServer nioSynConfigServer;// 同步
	private NIOUDPServer nioCorrectTime;// 校正时间
	private UnicastProxy unicastProxy;
	private HeartProxy heartProxy;// 心跳包代理
	private ParamConfigProxy paramConfigProxy;// 参数配置 多播代理和单播代理

	private BitMap bitMap;// 判断重传的bitMap队列
//	private Timer timer;// 全局计时器
//	private int cur_retransmition_count = 0;// 当前重传次数

	final int NODE_UNICAST_PORT = 5656; // // 节点接收单播指令

	private ByteBuffer contentByteBuffer;
	private List<String> ipList;// 获取ip列表
	private Map<Integer, Location> locationsMap;//
	private List<Location> locationsList;//
//	private List<FloorInfor> floorInfors;
//	private DESPlus webTokenDesPlus;
//	private DESPlus webDataDesPlus;
	// private String remoteDataToken=null;
	private String webToken = null;
	// private String webDataToken = null;
	private String webDataToken = "12345678";
	private boolean webKeyFlag;
	private String revcMsg = null;
	private int SynLocalPort = 6102;
	private int CorrectTimePort = 1026;// 校正时间端口
	// private String broadcastAddr="FF02::2";
	private int broadcastPort = 6104;// 时间同步端口
//	private int schedule_Port = 1028;
	private boolean synStateFlag;
	private int seqCount;

	private int currect_rate = 60;// 单位秒
	Timer CorrectTimer = new Timer();
	
	private Map<String, String> IpidMap;
//
//	// 集中器接收过上位机发送指令的标记位：初始为没有接收过
//	private boolean receive_frag = false;
//	// 标识是否正在接受指令
//	private boolean receiving_frag = false;

	public ConsoleMainServer_rpl() {
		try {
			unicastProxy = new UnicastProxy();
			cmdFile = new WriteDataToFile("cmd.txt");
			mulicastDataFile = new WriteDataToFile("mulicast.txt");// 存储多播消息
			topoFile = new WriteDataToFile("topo.txt");
			config = new GConfig("config.json");// 读取外部文件的参数
			parameter = config.getNetParameter();
			// remoteToken = "12345";
			webToken = "01234567";
			webKeyFlag = true;
			// remoteKeyFlag = false;
			synStateFlag = false;
			seqCount = 0;
			synConfig = new GSynConfig("GSynConfig.json");
			synParameter = synConfig.getSynParameter();
			IpidMap = new HashMap<String,String>();
			// desPlus = new DESPlus(token);
		} catch (IOException e) {

			// TODO 自动生成的 catch 块
			e.printStackTrace();
		} // 存储客户端下达的指令
		catch (JSONException e) {
			parameter = new NetParameter("00000001", 40, 3, 30, "0.0.0.0", 12300, 12301, 12306, "aaaa::1", 8765,
					"aaaa:0:0:0:12:7400:1:13", 5678, "192.168.1.141", 12303, "192.168.1.141", 12304, 12307,2,3,
					"0.0.0.0,",12400,"xiaoming","139.199.154.37","xiaoming",21,"127.0.0.1");
			synParameter = new SynParameter(0, 0, 0, 0, 0, 0, "0".getBytes(), false, null);
			// TODO 自动生成的 catch 块
			e.printStackTrace();
		}
		try {
			g_systemParam = Util.parseSystemParamFromFile("sysparam.json");
		} catch (Exception e1) {
			// TODO 自动生成的 catch 块
			g_systemParam = new SystemParam();
		}

		try {
			locationsMap = Util.parseLocationsFromFile("location.txt");
		} catch (IOException e) {
			// TODO 自动生成的 catch 块
			e.printStackTrace();
			locationsMap = new HashMap<Integer, Location>(64);
		}
		locationsList = new ArrayList<Location>();
		locationsList.addAll(locationsMap.values());
		ipList = new ArrayList<String>();
		for (Location l : locationsList) {
			ipList.add(l.getAddr());
		}
//		try {
//			floorInfors = Util.readRoomInforsForFile("rooms.json");
//		} catch (Exception ex) {
//			ex.printStackTrace();
//			floorInfors = new ArrayList<FloorInfor>();
//		}
		/******************* TCP远程客户端初始化 *****************************/
		nettyClient = new NettyClient(parameter.getTcpWebServerAddr(), parameter.getTcpWebServerPort());

		remoteClient = new NettyClient(parameter.getRemoteAddr(), parameter.getRemotePort());

		configRemoteNettyClient = new NettyClient(parameter.getRemoteAddr(), parameter.getTcpRemoteConfigPort());
		System.out.println(parameter.getRemoteAddr());
		netClient = new NettyClient(parameter.getRemoteAddr(), parameter.getNetPort());// parameter.getRemotePort()
		System.out.println(parameter.getNetPort());
		try {
			nioSynConfigServer = new NIOUDPServer(parameter.getUdpAddr(), SynLocalPort);
			nioSynConfigServer.registerHandler(new NIOSynMessageHandler());

			nioCorrectTime = new NIOUDPServer(parameter.getUdpAddr(), CorrectTimePort);// 校正时间
			nioCorrectTime.registerHandler(new NIOCorrectTimeHandler());
			try {
				nioCorrectTime.start();
			} catch (IllegalStateException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			} catch (InterruptedException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			} catch (Exception e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
			CorrectTimer.schedule(new TimerTask() {
				@Override
				public void run() {
					// TODO Auto-generated method stub
					try {

						CurrentTime currentTime = Util.getCurrentDateTime(Util.getCurrentDateTime());
						nioCorrectTime.sendto(
								Util.getCorrectTimeMessage(currentTime.getHour(), currentTime.getMinute(),
										currentTime.getSecond()),
								getSocketAddressByName(parameter.getRootAddr(), CorrectTimePort));
						System.out.println("correct time " + currentTime.getSecond());
					} catch (UnknownHostException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
				}
			}, 0, 1000 * currect_rate);
			try {
				nioSynConfigServer.start();
			} catch (IllegalStateException e) {
				// TODO 自动生成的 catch 块
				e.printStackTrace();
			} catch (InterruptedException e) {
				// TODO 自动生成的 catch 块
				e.printStackTrace();
			} catch (Exception e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		} catch (IOException e1) {
			// TODO 自动生成的 catch 块
			e1.printStackTrace();
		}

		/*********** 单播代理设置参数 ************************/
		unicastProxy.registerMethod(new UnicastMethodInvoke());
		unicastProxy.setDelay(g_systemParam.getSysNodeTransDelay() * 1000);
		unicastProxy.setAckWaitDelay(g_systemParam.getSysNodeWaitAckDelay() * 1000);
		unicastProxy.setReTransmitCount(g_systemParam.getSysUcastRetransTimes());
		/*************** 心跳代理 设置参数 ******************/

		heartProxy = new HeartProxy(ipList, g_systemParam.getSysNodeHeartDelay() * 1000);
		heartProxy.setInverval(g_systemParam.getSysNodeHeartDelay() * 1000);
		// heartProxy.setInverval(g_systemParam.getSysNodeHeartDelay() * 1000);
		heartProxy.registerOffLineHandler(new HeartOffLineHandler() {

			@Override
			public void actionPerformed(String addr) {
				System.out.println(addr + "掉线了");
				// TODO 自动生成的方法存根
				// 生成警告信息，并上报给远程客户端和工具
				// nettyClient.asyncWriteAndFlush(formatUcastDataToJsonStr("Warning",addr,"warning"));//formatUcastDataToJsonStr(addr,
				// "warning")
				if (remoteClient.remoteHostIsOnline()) {
					try {
						remoteClient.asyncWriteAndFlush(formatDataToJsonStr("heart_warning", addr, "warning"));
					} catch (Exception e) {
						// TODO 自动生成的 catch 块
						e.printStackTrace();
					}
				}

			}
		});
		heartProxy.registerOnLineHandler(new HeartOnLineHandler() {

			@Override
			public void actionPerformed(String addr) {
				System.out.println(addr + "在线");
				if (remoteClient.remoteHostIsOnline()) {
					try {
						remoteClient.asyncWriteAndFlush(formatDataToJsonStr("heart_succeed", addr, "online"));
					} catch (Exception e) {
						// TODO 自动生成的 catch 块
						e.printStackTrace();
					}
				}
				// TODO 自动生成的方法存根
			}
		});
		// heartProxy.start();
		paramConfigProxy = new ParamConfigProxy(parameter.getRootAddr(), new ProxyInvoke() {

			@Override
			public void invoke(String addr, int port, byte[] message) {
				// TODO 自动生成的方法存根
				try {
					nioUdpServer.sendto(message, getSocketAddressByName(addr, port));
				} catch (UnknownHostException e) {
					// TODO 自动生成的 catch 块
					e.printStackTrace();
				}
			}
		});
		// 配置失败的节点列表
		paramConfigProxy.registerConfigResultHandler(new ParamConfigResult() {

			@Override
			public void actionPerformed(List<String> failIpList) {
				// TODO 自动生成的方法存根
				System.out.println("调用！");
				JSONObject msgJson = null;
				if (failIpList == null) {
					System.out.println(" config succeed");
					msgJson = new JSONObject();
					try {
						msgJson.put("type", "config_succeed");
						msgJson.put("data", "config_succeed");
						// msgJson.put("", "");
					} catch (JSONException e) {
						// TODO 自动生成的 catch 块
						e.printStackTrace();
					}

					if (configRemoteNettyClient.remoteHostIsOnline()) {
						try {
							configRemoteNettyClient.asyncWriteAndFlush(msgJson.toString());
						} catch (Exception e) {
							// TODO 自动生成的 catch 块
							e.printStackTrace();
						}
					}
					return;
				}
				StringBuilder sb = new StringBuilder();
				sb.append("有节点配置失败，配置失败节点ip为\r\n");
				for (String ip : failIpList) {
					sb.append("  " + ip + "\r\n");
					System.out.println("有节点配置失败，配置失败节点ip为:  " + ip);
				}
				try {
					msgJson = new JSONObject();
					msgJson.put("type", "config_fail");
					msgJson.put("data", sb.toString());
				} catch (JSONException e) {
					// TODO 自动生成的 catch 块
					e.printStackTrace();
				}
				if (configRemoteNettyClient.remoteHostIsOnline()) {
					try {
						System.out.println(msgJson.toString());
						configRemoteNettyClient.asyncWriteAndFlush(msgJson.toString());
						// remoteClient.asyncWriteAndFlush(remoteDataDesPlus.encrypt(formatDataToJsonStr("config_fail",)));
					} catch (Exception e) {
						// TODO 自动生成的 catch 块
						e.printStackTrace();
					}
				}
				System.out.println(sb.toString());
				sb = null;
				msgJson = null;
				return;
			}
		});

		nettyServer = new NettyServer(parameter.getTcpAddr(), parameter.getTcpPort(), new NettyMsgHandlerExecutor());
//		webNettyServer = new NettyServer(parameter.getTcpAddr(), parameter.getTcpByWebPort(),
//				new WebNettyMsgHandlerExecutor());
//		try {
//			webNettyServer.start(parameter.getTcpAddr(), parameter.getTcpByWebPort());// 开启接收web前端远程客户端的指令的TCP
//			// server
//		} catch (InterruptedException e1) {
//			// TODO 自动生成的 catch 块
//			e1.printStackTrace();
//		}
		contentByteBuffer = ByteBuffer.allocate(128);
		//

		bitMap = new BitMap();

//		timer = new Timer(false);

		startNIOTcpServer();// 开启接收集中器前端远程客户端的指令的TCP server

		startNIOUdpServer();// 开启接收无线网络节点数据的UDP Server
		try {
			nioNetDataServer = new NIOUDPServer("0.0.0.0", 5688);
			nioNetDataServer.registerHandler(new NIOUdpNetDataHandler());
			nioNetDataServer.start();
			// nioSynConfigServer
		} catch (IOException | IllegalStateException | InterruptedException e) {
			// TODO 自动生成的 catch 块
			e.printStackTrace();
		}
		// nioNetDataServer.stop();
 catch (Exception e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	}

	// UDP 服务器 发送控制命令数据包给根节点 byte
	public void TunSendToRootMessage(byte[] message) throws IOException {
		if (nioUdpServer == null) {
			throw new IOException();
		}
		nioUdpServer.sendto(message, getSocketAddressByName(parameter.getRootAddr(), parameter.getRootPort()));// parameter.getRootPort()
		System.out.println(parameter.getRootPort() + parameter.getRootAddr());
	}

	public void SendToRootSynMsg(byte[] message) throws IOException {
		if (nioSynConfigServer == null) {
			throw new IOException();
		}
		nioSynConfigServer.sendto(message, getSocketAddressByName(parameter.getRootAddr(), broadcastPort));
		System.out.println("send to" + parameter.getRootAddr() + broadcastPort);

	}

	// 开启 Netty 的nio tcp服务
	public void startNIOTcpServer() {
		try {
			nettyServer.start(parameter.getTcpAddr(), parameter.getTcpPort());// new
			// NettyMsgHandlerExecutor()

		} catch (Exception e) {
			// TODO: handle exception
			e.printStackTrace();
		}
	}

	// 开启nio udp服务
	public void startNIOUdpServer() {
		try {
			nioUdpServer = new NIOUDPServer(parameter.getUdpAddr(), parameter.getUdpPort());

			nioUdpServer.registerHandler(new NIOUdpMessageHandler());
			nioUdpServer.start();
		} catch (IOException e) {
			e.printStackTrace();
		} catch (Exception e) {
			// TODO: handle exception
			e.printStackTrace();
		}
	}

	class NIOCorrectTimeHandler implements NIOUDPServerMsgHandler {

		@Override
		public byte[] messageHandler(String addr, byte[] message) {
			// TODO Auto-generated method stub
			return null;
		}

	}
//
//	// 处理来自web客户端发来的指令。
//	class WebNettyMsgHandlerExecutor implements NettyMsgHandler {
//
//		@SuppressWarnings({ "unchecked" })
//		@Override
//		public String messageHandler(String message) {
//			// TODO 自动生成的方法存根
//			JSONObject msgJson = null;
//			// try {
//			// msgJson = new JSONObject(message);
//			// } catch (JSONException e4) {
//			// // TODO 自动生成的 catch 块
//			// e4.printStackTrace();
//			// }
//			// System.out.println(message);
//			if (webKeyFlag) {// web前端认证成功后
//				// System.out.println("receive "+message);
//				try {
//
//					webDataDesPlus = new DESPlus(webDataToken);
//					revcMsg = webDataDesPlus.decrypt(message);
//					msgJson = new JSONObject(revcMsg);
//					System.out.println(revcMsg);
//				} catch (Exception e3) {
//					// TODO 自动生成的 catch 块
//					e3.printStackTrace();
//				}
//				Command command = null;// 命令对象
//				ArrayList<Location> newLocations = null;
//				Object retObject;
//				try {
//					// command = Command.parseCmdFromStream(message);
//					retObject = Util.parseMetaData(revcMsg);
//				} catch (JSONException e1) {
//					// TODO 自动生成的 catch 块
//					e1.printStackTrace();
//					return null;
//				}
//				if (retObject == null) {
//					return null;
//				}
//				if (retObject instanceof Command) {
//					command = (Command) retObject;
//					String addr = parameter.getTcpWebServerAddr();
//					String notes = Util.getCurrentTime() + ":[" + addr + "]" + "下达命令:"
//							+ Util.formatBytesToStr(command.getContent());
//					// 缓存指令内容
//					putCommandToCache(command.getContent());// 获取command中的cmd内容字节数组，并进行缓存
//					System.out.println(notes);
//					// System.out.println(command.getContent());
//					// mainUI.setForwordCmd(new String(command.getContent()));//
//					// 下发命令的文本框改变文字
//
//					if (command.getType() == (byte) 'u') {
//						// List<Integer> sendList = command.getNodeList();
//						List<String> addrList = new ArrayList<String>();
//						for (int id : command.getNodeList()) {
//							addrList.add(locationsMap.get(id).getAddr());
//						}
//						if (g_systemParam.getSysUcastToMcastValue() > 1.0D * addrList.size() / locationsList.size()) {
//							// 单播代理
//							// byte[] data =
//							// packUcastData(command.getContent());
//							byte[] data = packUnicastData(command.getContent());
//
//							unicastProxy.setProxyTask(addrList, NODE_UNICAST_PORT, data);// 以字节数组单播发出去
//							try {
//								unicastProxy.start();
//							} catch (Exception e2) {
//								// TODO: handle exception
//							}
//							return null;
//						}
//
//						// 局部多播
//						bitMap.setPartReUploadList(addrList);
//						try {
//							byte[] buffer = packageReadDataAck(command.getContent(), bitMap.getBitMap());
//							TunSendToRootMessage(buffer);
//							cmdFile.append(notes);// 保存下发指令，存储在cmd.txt里
//							System.out.println("发给root" + new String(buffer));
//							buffer = null;
//						} catch (IOException e) {
//							e.printStackTrace();
//							// TODO: handle exception
//						}
//
//						return null;
//					}
//					// 整体多播
//					bitMap.setTotalNodeSize(locationsList.size());
//					try {
//						byte[] buffer = packageReadData(command.getContent());
//						// System.out.println(command.getContent());
//						TunSendToRootMessage(buffer);
//						cmdFile.append(notes);// 保存下发指令，存储在cmd.txt里
//						System.out.println("发给root2" + new String(buffer));
//						// System.out.println(new String(packageReadData(Util
//						// .formatByteStrToByte("105BFE5916"))));
//						buffer = null;
//					} catch (IOException e) {
//						e.printStackTrace();
//						// TODO: handle exception
//					}
//
//				} else if (retObject instanceof ArrayList) {// 同步节点信息
//					newLocations = (ArrayList<Location>) retObject;
//					locationsList = newLocations;
//					List<String> newipList = new ArrayList<String>();
//					Map<Integer, Location> newLocationsMap = new HashMap<Integer, Location>();
//					for (Location l : newLocations) {
//						newipList.add(l.getAddr());
//						newLocationsMap.put(l.getId(), l);
//					}
//					ipList = newipList;
//					// mainUI.updateIpList(ipList);
//					locationsMap = newLocationsMap;
//					// mainUI.updateLocations(locationsList);
//					newLocations = null;
//					newipList = null;
//					newLocationsMap = null;
//					return null;
//				} else if (retObject instanceof Integer) {// 判断为整型数字，即为重新上报不在线时期数据
//					// sendOffLineDataToRemoteHost("offLineData.txt");
//				}
//			}
//			if (!webKeyFlag) {
//				try {
//					msgJson = new JSONObject(message);
//				} catch (JSONException e1) {
//					// TODO 自动生成的 catch 块
//					e1.printStackTrace();
//				}
//				try {
//					if (!msgJson.getString("name").equals("web")) {
//						return null;
//					}
//					if (msgJson.getString("name").equals("web")) {
//						webTokenDesPlus = new DESPlus(webToken);
//						JSONObject keyJson = null;
//						JSONArray jsonArray = new JSONArray();
//						keyJson = new JSONObject();
//						if (webTokenDesPlus.decrypt(msgJson.getString("value")).equals(webToken)) {
//							System.out.println(webTokenDesPlus.decrypt(msgJson.getString("value")));
//							keyJson.put("type", "succeed");
//							if (nettyClient.remoteHostIsOnline()) {
//								System.out.println(webDataToken);
//								keyJson.put("content", "webDataToken");
//								jsonArray.put(keyJson);
//								// remoteClient.asyncWriteAndFlush(tokenDesPlus.encrypt("认证成功"));
//								nettyClient.asyncWriteAndFlush(webTokenDesPlus.encrypt(jsonArray.toString()));
//								webKeyFlag = true;
//
//							}
//							webDataDesPlus = new DESPlus(webDataToken);
//							keyJson = null;
//							jsonArray = null;
//						} else if (!webTokenDesPlus.decrypt(msgJson.getString("value")).equals(webToken)) {
//							if (nettyClient.remoteHostIsOnline()) {
//								// System.out.println(webTokenDesPlus.decrypt(message));
//								keyJson.put("type", "error");
//								keyJson.put("content", "");
//								jsonArray.put(keyJson);
//								nettyClient.asyncWriteAndFlush(webTokenDesPlus.encrypt(jsonArray.toString()));
//								webKeyFlag = false;
//								keyJson = null;
//								jsonArray = null;
//								// dataDesPlus = new DESPlus(token);
//							}
//						}
//					}
//				} catch (JSONException e) {
//					// TODO 自动生成的 catch 块
//					e.printStackTrace();
//				} catch (Exception e) {
//					// TODO 自动生成的 catch 块
//					e.printStackTrace();
//				}
//			}
//
//			return null;
//		}
//
//	}

	/**
	 * 
	 * @lhy 接收端：接收来自集中器客户端发来的指令，接收发送过来的json形式的字符串，通过与字节数组之间进行变换，以字节数组的格式
	 *      发送给root节点,节点以char形数组接收
	 *
	 */
	class NettyMsgHandlerExecutor implements NettyMsgHandler {

		@SuppressWarnings({ "unchecked" })
		@Override
		public String messageHandler(String message) {
			System.out.println(message);
//			JSONObject msgJson = null;
//			Command command = null;// 命令对象
			ArrayList<Location> newLocations = null;
			Object retObject;
			try {
				// command = Command.parseCmdFromStream(message);
				retObject = Util.parseMetaData(message);
				System.out.println(retObject);
			} catch (JSONException e1) {
				// TODO 自动生成的 catch 块
				e1.printStackTrace();
				return null;
			}
			if (retObject == null) {
				return null;
			} else if (retObject instanceof ArrayList) {// 同步节点信息
				newLocations = (ArrayList<Location>) retObject;
				locationsList = newLocations;
				List<String> newipList = new ArrayList<String>();
				Map<Integer, Location> newLocationsMap = new HashMap<Integer, Location>();
				for (Location l : newLocations) {
					newipList.add(l.getAddr());
					newLocationsMap.put(l.getId(), l);
				}
				ipList = newipList;
				// mainUI.updateIpList(ipList);
				locationsMap = newLocationsMap;
				// mainUI.updateLocations(locationsList);
				newLocations = null;
				newipList = null;
				newLocationsMap = null;
				return null;
			} else if (retObject instanceof Integer) {// 判断为整型数字，即为重新上报不在线时期数据
				// sendOffLineDataToRemoteHost("offLineData.txt");
			} else if (retObject instanceof JSONObject) {// 接收集中器前台的配置参数的命令
				try {
					String type = ((JSONObject) retObject).get("type").toString();
					System.out.println(type);
					byte[] cmd = Util.formatByteStrToByte(((JSONObject) retObject).getString("pama_data"));
					byte[] buffer = null;
					switch (type) {
					case "mcast":
						buffer = Util.packetMcastSend(cmd);
						if(buffer[2] == (byte)0x00){
							System.out.println("收到的指令为 上报能耗+拓扑");
						}else if(buffer[2] == (byte)0x01){
							System.out.println("收到的指令为 上报网络参数");	
						}else if(buffer[2] == (byte)0X80){
							System.out.println("收到的指令为 多播读表指令");
						}else if(buffer[2] == (byte)0x82){
							System.out.println("收到的指令为 初始的多播读表指令");
							for(byte b:buffer){
								System.out.println(b);
							}
						}else{
							System.out.println("出错啦~~");
						}
						try {
							TunSendToRootMessage(buffer);
						} catch (IOException e) {
							// TODO 自动生成的 catch 块
							e.printStackTrace();
						}
						break;
					case "unicast":
						bitMap.setPartReUploadList(parseAddrFromStr(((JSONObject) retObject).getString("addrList")));
						buffer = Util.packageUnicastSend(cmd,bitMap.getBitMap());
						if(buffer[3+bitMap.getBitMap().length] == (byte) 0X80){
							System.out.println("收到的指令为  局部多播读表指令");
						}else if(buffer[3+bitMap.getBitMap().length] == (byte)0x82){
							System.out.println("收到的指令为 初始的局部多播 读表指令");
						}else{
							System.out.println("出错啦~~~");
						}
						try {
							TunSendToRootMessage(buffer);
						} catch (IOException e) {
							// TODO 自动生成的 catch 块
							e.printStackTrace();
						}
						break;
					case "mcast_ack":
						buffer = Util.packetMcastSend(cmd);
						if(buffer[2] == (byte)0x41){
							System.out.println("收到的指令为 周期配置");
						}else if(buffer[2] == (byte)0x40){
							System.out.println("收到的指令为 配置网络参数");	
						}else if(buffer[2] == (byte)0xC0){
							System.out.println("收到的指令为 多播节点初始化指令");
						}else if(buffer[2] == (byte)0xC1){
							System.out.println("收到的指令为 多播节点重启指令");
						}else{
							System.out.println("出错啦~~");
						}
						paramConfigProxy.mcastConfig(buffer, ipList, parameter.getRootPort(), NODE_UNICAST_PORT);
						break;
					case "unicast_ack":
						List<String> unicast_List = parseAddrFromStr(((JSONObject) retObject).getString("addrList"));
						bitMap.setPartReUploadList(parseAddrFromStr(((JSONObject) retObject).getString("addrList")));
						buffer = Util.packageUnicastSend(cmd,bitMap.getBitMap());
						if(buffer[3+bitMap.getBitMap().length] == (byte)0xC0){
							System.out.println("收到的指令为 局部多播节点初始化");
						}else if(buffer[3+bitMap.getBitMap().length] == (byte)0xC1){
							System.out.println("收到的指令为 局部多播节点重启");
						}else{
							System.out.println("出错啦~~~");
						}	
						paramConfigProxy.mcastConfig(buffer, unicast_List, parameter.getRootPort(), NODE_UNICAST_PORT);
						unicast_List.clear();
						break;
					
					
					
					
					
					
//					case "pama_send":// 多播配置节点上报数据周期
//						System.out.println("pama_send");
//						System.out.println(((JSONObject) retObject).get("pama_data").toString());
//						paramConfigProxy.mcastConfig(
//								packMcastConfigData(((JSONObject) retObject).get("pama_data").toString().getBytes()),
//								ipList, parameter.getRootPort(), NODE_UNICAST_PORT);
//						break;
//					// 有用
//					case "cmd_send":// 局部多播下发指令
//						System.out.println("cmd_send");
//						bitMap.setPartReUploadList(parseAddrFromStr(((JSONObject) retObject).getString("addrList")));
//
//						try {
//							buffer = packageReadDataAck(
//									Util.formatByteStrToByte(((JSONObject) retObject).getString("pama_data")),
//									bitMap.getBitMap());
//							TunSendToRootMessage(buffer);
//							// cmdFile.append(notes);//
//							// 保存下发指令，存储在cmd.txt里
//							System.out.println("发给root" + new String(buffer));
//							// buffer = null;
//						} catch (IOException e2) {
//							// TODO: handle exception
//						}
//
//						break;
//					// 有用
//					case "cmd_mcast":// 多播下发指令
//
//						System.out.println("cmd_mcast 多播");
//
//						buffer = packageReadData(
//								Util.formatByteStrToByte(((JSONObject) retObject).getString("pama_data")));
//						try {
//							TunSendToRootMessage(buffer);
//							System.out.println(new String(buffer));
//						} catch (IOException e1) {
//							// TODO 自动生成的 catch 块
//							e1.printStackTrace();
//						}
//
//						buffer = null;
//
//						break;
//					// 有用
//					case "config_heart":// 配置心跳包周期改为配置整体上报周期
//						String heartPeriod = ((JSONObject) retObject).getString(("pama_data"));
//
//						System.out.println(Util.formatByteStrToByte(parseCmdToStr(heartPeriod + "")));
//
//						paramConfigProxy.mcastConfig(packHeartConfigData(heartPeriod.getBytes()), ipList,
//								parameter.getRootPort(), NODE_UNICAST_PORT);
//						// heartProxy.setInverval(Integer.parseInt(heartPeriod)
//						// * 1000);
//
//						break;
					// 有用
					case "schedule":// 广播配置调度
						System.out.println("schedule" + ((JSONObject) retObject).getString(("pama_data")));

						JSONObject synJson = new JSONObject(((JSONObject) retObject).getString(("pama_data")));
						try {
							System.out.println(Util.formatBytesToStr(Base64.decode(synJson.getString("bitmap"))));
							//byte[] bit = Base64.decode(synJson.getString("bitmap"));
						} catch (Base64DecodingException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}

						try {
							synParameter.setBitmap(Base64.decode(synJson.getString("bitmap")));
							synParameter.setBit(synJson.getString("bitmap"));
						} catch (Base64DecodingException e1) {
							// TODO Auto-generated catch block
							e1.printStackTrace();
						}
						synParameter.setFlag(true);
						Util.writeSynConfigParamToFile(synParameter, "GSynConfig.json");
						try {
							// byte[] bit =
							// Base64.decode(synJson.getString("bitmap"));

							// byte[] schedule=packScheduleConfigData(bit);

							paramConfigProxy.mcastConfig(
									packScheduleConfigData(Base64.decode(synJson.getString("bitmap"))), ipList,
									parameter.getRootPort(), NODE_UNICAST_PORT);

							// TunSendToRootMessage(schedule);

						} catch (Base64DecodingException e) {
							// TODO Auto-generated catch block
							e.printStackTrace();
						}
						// TunSendToRootMessage(((JSONObject)
						// retObject).getString(("pama_data")).getBytes());
						synStateFlag = true;
						synJson = null;
						break;
					// 有用
					case "pama_corr":
						try {
							System.out.println("pama_corr1 " + ((JSONObject) retObject).getString(("pama_data")));

							currect_rate = Integer.valueOf(((JSONObject) retObject).getString(("pama_data")));
							CorrectTimer.cancel();
							CorrectTimer = new Timer();
							CorrectTimer.schedule(new TimerTask() {
								@Override
								public void run() {
									// TODO Auto-generated method stub
									try {

										CurrentTime currentTime = Util.getCurrentDateTime(Util.getCurrentDateTime());
										nioCorrectTime.sendto(
												Util.getCorrectTimeMessage(currentTime.getHour(), currentTime.getMinute(),
														currentTime.getSecond()),
												getSocketAddressByName(parameter.getRootAddr(), CorrectTimePort));
										System.out.println("correct time " + currentTime.getSecond());
									} catch (UnknownHostException e) {
										// TODO Auto-generated catch block
										e.printStackTrace();
									}
								}
							}, 0, 1000 * currect_rate);
							System.out.println("pama_corr2 " + currect_rate);

						} catch (Exception e) {
							// TODO: handle exception
						}
						break;
					// 有用
					case "pama_syn":
						try {
							System.out.println("pama_syn" + ((JSONObject) retObject).getString(("pama_data")));

							JSONObject pama_synJson = new JSONObject(((JSONObject) retObject).getString(("pama_data")));
							try {
								System.out.println(
										Util.formatBytesToStr(Base64.decode(pama_synJson.getString("bitmap"))));
								//byte[] bit = Base64.decode(pama_synJson.getString("bitmap"));
							} catch (Base64DecodingException e) {
								// TODO Auto-generated catch block
								e.printStackTrace();
							}

							synParameter.setSeqNum(pama_synJson.getInt("seqNum"));
							synParameter.setLevel(pama_synJson.getInt("level"));
							synParameter.setHour(pama_synJson.getInt("hour"));
							synParameter.setMinute(pama_synJson.getInt("minute"));
							synParameter.setSecond(pama_synJson.getInt("second"));
							synParameter.setPeriod(pama_synJson.getInt("period"));

							// System.out.println("period 2333 "+
							// pama_synJson.getInt("period"));

							try {
								synParameter.setBitmap(Base64.decode(pama_synJson.getString("bitmap")));
								synParameter.setBit(pama_synJson.getString("bitmap"));
							} catch (Base64DecodingException e1) {
								// TODO Auto-generated catch block
								e1.printStackTrace();
							}
							synParameter.setFlag(true);
							Util.writeSynConfigParamToFile(synParameter, "GSynConfig.json");
							try {
								byte[] bit = Base64.decode(pama_synJson.getString("bitmap"));
								CurrentTime currentTime = Util.getCurrentDateTime(Util.getCurrentDateTime());

								SendToRootSynMsg(Util.getSynMessage(pama_synJson.getInt("seqNum"),
										pama_synJson.getInt("level"), currentTime.getHour(), currentTime.getMinute(),
										currentTime.getSecond(), pama_synJson.getInt("period"), bit));
							} catch (Base64DecodingException e) {
								// TODO Auto-generated catch block
								e.printStackTrace();
							}
							synStateFlag = true;
							pama_synJson = null;
						} catch (IOException e) {
							// TODO 自动生成的 catch 块
							e.printStackTrace();
						}
						break;
//					// 添加节点重启，初始化
//					case "reboot_send":
//						List<String> reboot_List = parseAddrFromStr(((JSONObject) retObject).getString("addrList"));
//						bitMap.setPartReUploadList(parseAddrFromStr(((JSONObject) retObject).getString("addrList")));
//						buffer = packageRebootSend(bitMap.getBitMap());
//						paramConfigProxy.mcastConfig(buffer, reboot_List, parameter.getRootPort(), NODE_UNICAST_PORT);
//						buffer = null;
//						reboot_List.clear();
//						break;
//					case "reboot_mcast":
//						paramConfigProxy.mcastConfig(packageRebootMcast(), ipList, parameter.getRootPort(),
//								NODE_UNICAST_PORT);
//						break;
//					case "restart_send":
//						List<String> restart_List = parseAddrFromStr(((JSONObject) retObject).getString("addrList"));
//						bitMap.setPartReUploadList(parseAddrFromStr(((JSONObject) retObject).getString("addrList")));
//						buffer = packageRestartSend(bitMap.getBitMap());
//						paramConfigProxy.mcastConfig(buffer, restart_List, parameter.getRootPort(), NODE_UNICAST_PORT);
//						buffer = null;
//						restart_List.clear();
//						break;
//					case "restart_mcast":
//						paramConfigProxy.mcastConfig(packageRestartMcast(), ipList, parameter.getRootPort(),
//								NODE_UNICAST_PORT);
//						break;
//					// 添加系统监测，网络监测和网络参数配置
//					case "reportSys":
//						System.out.println("reportSys 上报系统监测数据");
//						//System.out.println(((JSONObject) retObject).getString("pama_data").getBytes()[0]);
//						buffer = packageReportSys(((JSONObject) retObject).getString("pama_data").getBytes());
//						System.out.println(buffer[0] + " " + buffer[1] + " " + buffer[2]);
//						try {
//							TunSendToRootMessage(buffer);
//							System.out.println(new String(buffer));
//						} catch (IOException e1) {
//							// TODO 自动生成的 catch 块
//							e1.printStackTrace();
//						}
//
//						buffer = null;
//						break;
//					case "reportNet":
//						System.out.println("reportNet 上报网络监测数据");
//
//						buffer = packageReportNet(((JSONObject) retObject).getString("pama_data").getBytes());
//						System.out.println(((JSONObject) retObject).getString("pama_data").getBytes());
//						try {
//							TunSendToRootMessage(buffer);
//							System.out.println(new String(buffer));
//						} catch (IOException e1) {
//							// TODO 自动生成的 catch 块
//							e1.printStackTrace();
//						}
//						buffer = null;
//						break;
//					case "sendNet":
//						System.out.println("sendNet 配置网络参数");
//						//byte[] test = Util.formatNetStrToByte(((JSONObject) retObject).getString("pama_data"));
//						//System.out.println(test[0] + " " + test[1] + " " + test[2] + " " + test[3] + " " + test[4]+" "+test[5]);
//						buffer = packageSendNet(
//								Util.formatNetStrToByte(((JSONObject) retObject).getString("pama_data")));
//						//System.out.println(buffer + "--" + ipList);
//						paramConfigProxy.mcastConfig(buffer, ipList, parameter.getRootPort(), NODE_UNICAST_PORT);
//
//						break;
					default:
						break;
					}
				} catch (JSONException e) {
					// TODO 自动生成的 catch 块
					e.printStackTrace();
				}
			}

			return null;

		}
	}

	public List<String> parseAddrFromStr(String str) {
		List<String> list = new ArrayList<String>();
		// System.out.println(str.substring(1, str.length()-1));
		String[] addr = str.substring(1, str.length() - 1).split(",");
		for (int i = 0; i < addr.length; i++) {
			list.add(addr[i].substring(1, addr[i].length() - 1));
			// System.out.println(addr[i].substring(1, addr[i].length()-1));
		}
		// System.out.println(list.toString());
		return list;
	}
	
	
	private byte[] packScheduleConfigData(byte[] content) { // 配置调度，类型7
		if (content == null || content.length == 0) {
			return null;
		}
		byte[] cmd = new byte[content.length + 3];
		cmd[0] = (byte) (content.length + 2);
		cmd[1] = (byte) (1); // 1 表示 多播
		cmd[2] = GlobalDefines.GlobalCmd.G_SCHEDULE_CONFIG;
		System.arraycopy(content, 0, cmd, 3, content.length);
		return cmd;
	}
//	private byte[] packHeartConfigData(byte[] content) { // 配置心跳包上报周期，类型6
//		if (content == null || content.length == 0) {
//			return null;
//		}
//		byte[] cmd = new byte[content.length + 3];
//		cmd[0] = (byte) (content.length + 2);
//		cmd[1] = (byte) (1); // 1 表示 多播
//		cmd[2] = GlobalDefines.GlobalCmd.G_HEART_CONFIG_PERIOD;
//		System.arraycopy(content, 0, cmd, 3, content.length);
//		return cmd;
//	}
	//no use
//	private byte[] packMcastConfigData(byte[] content) {// 所有节点多播进行参数配置，类型5
//		if (content == null || content.length == 0) {
//			return null;
//		}
//		byte[] cmd = new byte[content.length + 2];
//		cmd[0] = (byte) (content.length + 1);
//		cmd[1] = GlobalDefines.GlobalCmd.G_DEF_MCAST_CONFIG_PERIOD;
//		System.arraycopy(content, 0, cmd, 2, content.length);
//		return cmd;
//	}
	// no use
//	private byte[] packUnicastConfigData(byte[] content) {// 子节点单播参数配置，类型4
//		if (content == null || content.length == 0) {
//			return null;
//		}
//		int length = content.length;
//		byte[] cmd = new byte[length + 2];
//		cmd[0] = (byte) (length + 1);
//		cmd[1] = GlobalDefines.GlobalCmd.G_DEF_UNICAST_CONFIG_PERIOD;
//		System.arraycopy(content, 0, cmd, 2, length);
//		return cmd;
//	}
	// 下面两个与上位机通讯时使用
	private byte[] packUnicastData(byte[] content) {// 单播发指令,类型3
		if (content == null || content.length == 0) {
			return null;
		}
		byte[] cmd = new byte[content.length + 3];
		cmd[0] = (byte) (content.length + 2);
		cmd[1] = (byte) (1); // 1 表示 多播
		cmd[2] = GlobalDefines.GlobalCmd.G_DEF_READ_DATA;
		System.arraycopy(content, 0, cmd, 3, content.length);
		return cmd;
	}
	
	private byte[] packageReadData(byte[] cmd) {// 多播 纯指令类型1
		if (cmd == null || cmd.length == 0) {
			return null;
		}
		int length = cmd.length;
		byte[] content = new byte[length + 3];
		content[0] = (byte) (length + 2);
		content[1] = (byte) (1); // 1 表示 多播
		content[2] = GlobalDefines.GlobalCmd.G_DEF_READ_DATA;
		System.arraycopy(cmd, 0, content, 3, length);
		return content;
	}

//	// 封装上报系统监测和网络监测指令数据
//	private byte[] packageReportSys(byte[] cmd) {
//		if (cmd == null || cmd.length == 0) {
//			return null;
//		}
//		int length = cmd.length;
//		byte[] content = new byte[length + 3];
//		content[0] = (byte) (length + 2);
//		content[1] = (byte) (1); // 1 表示 多播
//		content[2] = GlobalDefines.GlobalCmd.G_DEF_REPORT_SYS;
//		System.arraycopy(cmd, 0, content, 3, length);
//		System.out.println(content[1]);
//		return content;
//	}
//
//	private byte[] packageReportNet(byte[] cmd) {
//		if (cmd == null || cmd.length == 0) {
//			return null;
//		}
//		int length = cmd.length;
//		byte[] content = new byte[length + 3];
//		content[0] = (byte) (length + 2);
//		content[1] = (byte) (1); // 1 表示 多播
//		content[2] = GlobalDefines.GlobalCmd.G_DEF_REPORT_NET;
//		System.arraycopy(cmd, 0, content, 3, length);
//		return content;
//	}

//	// 封装下发网络参数配置
//	private byte[] packageSendNet(byte[] cmd) {
//		if (cmd == null || cmd.length == 0) {
//			return null;
//		}
//		int length = cmd.length;
//		byte[] content = new byte[length + 3];
//		content[0] = (byte) (length + 2);
//		content[1] = (byte) (1); // 1 表示 多播
//		content[2] = GlobalDefines.GlobalCmd.G_DEF_SEND_NET;
//		System.arraycopy(cmd, 0, content, 3, length);
//		return content;
//	}
//
//	// 封装多播节点重启指令
//	private byte[] packageRebootMcast() {
//		byte[] content = new byte[3];
//		content[0] = (byte) (1);
//		content[1] = (byte) (1); // 1 表示 多播
//		content[2] = GlobalDefines.GlobalCmd.G_DEF_REBOOT;
//		return content;
//	}

//	// 封装多播节点初始化系指令
//	private byte[] packageRestartMcast() {
//		byte[] content = new byte[2];
//		content[0] = (byte) (1);
//		content[1] = (byte) (1);
//		content[2] = GlobalDefines.GlobalCmd.G_DEF_RESTART;
//		return content;
//	}
//
//	// 封装任播节点重启指令
//	private byte[] packageRebootSend(byte[] bitmap) {// 多播重传类型2
//		// 指令加bitmap
//		// 类型2
//		if (bitmap == null || bitmap.length == 0) {
//			return null;
//		}
//		byte[] content = new byte[bitmap.length + 4];
//		content[0] = (byte) (bitmap.length + 3);
//		content[1] = (byte) (2); // 2 表示 局部单播
//		System.arraycopy(bitmap, 0, content, 2, bitmap.length);
//		content[2+bitmap.length] = GlobalDefines.GlobalCmd.G_DEF_REBOOT;
//		content[3+bitmap.length] = (byte) 0;//指令长度 为 零 占位
//		return content;
//	}

	// 封装任播节点初始化指令
//	private byte[] packageRestartSend(byte[] bitmap) {// 多播重传类型2
//		// 指令加bitmap
//		// 类型2
//		if (bitmap == null || bitmap.length == 0) {
//			return null;
//		}
//		byte[] content = new byte[bitmap.length + 4];
//		content[0] = (byte) (bitmap.length + 3);
//		content[1] = (byte) (2);// 2 表示 局部单播
//		System.arraycopy(bitmap, 0, content, 2, bitmap.length);
//		content[2+bitmap.length] = GlobalDefines.GlobalCmd.G_DEF_RESTART;
//		content[3+bitmap.length] = (byte) 0;//指令长度 为 零 占位
//		return content;
//	}
	//与上位机通讯时使用到
	private byte[] packageReadDataAck(byte[] cmd, byte[] bitmap) {// 多播重传类型2
		// 指令加bitmap
		// 类型2
		if (cmd == null || cmd.length == 0) {
			return null;
		}
		if (bitmap == null || bitmap.length == 0) {
			return null;
		}
		byte[] content = new byte[cmd.length + bitmap.length + 4];
		content[0] = (byte) (cmd.length + bitmap.length + 3);
		content[1] = (byte) (2); // 2 表示 局部单播
		System.arraycopy(bitmap, 0, content, 2, bitmap.length);
		content[2+bitmap.length] = GlobalDefines.GlobalCmd.G_DEF_READ_DATA;
		content[3+bitmap.length] = (byte) cmd.length;//指令长度
		System.arraycopy(cmd, 0, content, bitmap.length + 4, cmd.length);
		return content;
	}


	public String formatUcastDataToJsonStr(String addr, String content) {
		JSONObject object = new JSONObject();
		try {
			object.put("addr", addr);
			object.put("data", content);
		} catch (JSONException e) {
			// TODO 自动生成的 catch 块
			e.printStackTrace();
		}
		return "[" + object.toString() + "]";
	}

	/************* 数据组装 ****************/
	public String formatUcastDataToJsonStr(String type, String addr, String content) {
		JSONObject object = new JSONObject();
		try {
			object.put("type", type);
			object.put("addr", addr);
			object.put("data", content);
		} catch (JSONException e) {
			// TODO 自动生成的 catch 块
			e.printStackTrace();
		}
		return "[" + object.toString() + "]";
	}

	
	public  String formatDataToJsonStr(String type, String addr, String content) {
		JSONObject object = new JSONObject();
		try {
			object.put("type", type);
			object.put("addr", addr);
			object.put("data", content);
		} catch (JSONException e) {
			// TODO 自动生成的 catch 块
			e.printStackTrace();
		}
		return object.toString();
	}

	
	class UnicastMethodInvoke implements ProxyInvoke {

		@Override
		public void invoke(String addr, int port, byte[] message) {
			// TODO 自动生成的方法存根
			// 单播出去
			try {
				System.out.println(addr);
				UnicastSendMessage(addr, port, message);// ip地址
				// 端口号，消息
			} catch (IOException e) {
				// TODO 自动生成的 catch 块
				e.printStackTrace();
			}
		}
	}

	// 单播发送
	public void UnicastSendMessage(String addr, int port, byte[] message) throws IOException {
		if (nioUdpServer == null) {
			throw new IOException();
		}
		nioUdpServer.sendto(message, getSocketAddressByName(addr, port));
	}

	// 处理网络参数，设计拓扑结构
	class NIOUdpNetDataHandler implements NIOUDPServerMsgHandler {

		@Override
		public byte[] messageHandler(String addr, byte[] message) {
			//oprl 移3位
			System.out.print("-----------------------------");
//			byte[] orpl = new byte[message.length-3];
//			System.arraycopy(message, 3, orpl, 0, message.length-3);
			System.out.println(addr + "能耗：" + Util.formatBytesToStr(message));
			try {
				topoFile.append(Util.getCurrentTime() + ":[" + addr + "]" + "能耗：" + Util.formatBytesToStr(message));
			} catch (IOException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}
			if (netClient.remoteHostIsOnline()) {
				try {
					// 使用json发送 拓扑 信息
					netClient.asyncWriteAndFlush(formatDataToJsonStr("topo", addr, Util.formatByteToByteStr(message))); // json使用方式
				} catch (Exception e) {
					// TODO 自动生成的 catch 块
					e.printStackTrace();
				}
			}

			return null;
		}
	}

	// 处理同步消息的udp 服务器
	class NIOSynMessageHandler implements NIOUDPServerMsgHandler {
		@Override
		public byte[] messageHandler(String addr, byte[] message) {
			System.out.println(String.valueOf(message));
//			//oprl 移3位
//			byte[] orpl = new byte[message.length-3];
//			System.arraycopy(message, 3, orpl, 0, message.length-3);
//			String noteStr = Util.getCurrentTime() + ":[" + addr + "]" + "上报数据" + Util.formatBytesToStr(orpl);
			System.out.println("hello bitmap" + Util.formatBytesToStr(synParameter.getBitmap()));
//			System.out.println(noteStr);
			// char recvLevel =(char) message[2];
			int recvLevel = Integer.parseInt(String.valueOf(String.format("%02X", message[1])), 16);// String.valueOf(String.format("%02X",
			// message[3]))
			// System.out.println("leve="+Integer.parseInt(recvLevel,16));
			// System.out.println(Integer.valueOf(String.format("0x%02X",
			// message[2])) );
			int hour = Integer.parseInt(String.valueOf(String.format("%02X", message[2])), 16);
			int minute = Integer.parseInt(String.valueOf(String.format("%02X", message[3])), 16);
			int second = Integer.parseInt(String.valueOf(String.format("%02X", message[4])), 16);
			// System.out.println(String.format("0x%02X", message[3]) );
			// System.out.println(String.format("0x%02X", message[4]) );
			// byte[] bitmapBuf = new byte[message.length - 5];
			// System.arraycopy(message, 5, bitmapBuf, 0, bitmapBuf.length);
			System.out.println(recvLevel + "/" + hour + "/" + minute + "/" + second);
			// CurrentTime currentTime
			// =Util.getCurrentDateTime(Util.getCurrentDateTime());

			if (!synStateFlag) {

			} else {
				switch (recvLevel) {
				case GlobalDefines.GlobalSynLevelConfig.G_SYN_CONFIG_LEVEL:// 请求
					// 发送给root

					try {
						seqCount = 0;

						CurrentTime currentTime = Util.getCurrentDateTime(Util.getCurrentDateTime());
						SendToRootSynMsg(Util.getSynMessage(seqCount, 0, currentTime.getHour(), currentTime.getMinute(),
								currentTime.getSecond(), synParameter.getPeriod(), synParameter.getBitmap()));

					} catch (IOException e) {
						// TODO 自动生成的 catch 块
						e.printStackTrace();
					}
					break;
				case GlobalDefines.GlobalSynLevelConfig.G_SYN_CONFIG_INIT_LEVEL:// 请求
					try {
						CurrentTime currentTime = Util.getCurrentDateTime(Util.getCurrentDateTime());
						seqCount = Integer.parseInt(String.valueOf(String.format("%02X", message[0])), 16) + 1;
						SendToRootSynMsg(Util.getSynMessage(seqCount, 0, currentTime.getHour(), currentTime.getMinute(),
								currentTime.getSecond(), synParameter.getPeriod(), synParameter.getBitmap()));
						// seqCount ++;
					} catch (IOException e) {
						// TODO 自动生成的 catch 块
						e.printStackTrace();
					}
					// }
					break;

				default:
					try {
						CurrentTime currentTime = Util.getCurrentDateTime(Util.getCurrentDateTime());
						seqCount = Integer.parseInt(String.valueOf(String.format("%02X", message[0])), 16) + 1;
						SendToRootSynMsg(Util.getSynMessage(seqCount, 0, currentTime.getHour(), currentTime.getMinute(),
								currentTime.getSecond(), synParameter.getPeriod(), synParameter.getBitmap()));
						// seqCount ++;
					} catch (IOException e) {
						// TODO 自动生成的 catch 块
						e.printStackTrace();
					}
					break;
				}
			}
			return null;
		}

	}

	// 处理udp server收到的数据 ，添加到数据库，数据来源于无线网络内的所有节点,将数据直接传给TCP控制中枢
	class NIOUdpMessageHandler implements NIOUDPServerMsgHandler {

		@Override
		public byte[] messageHandler(String addr, byte[] message) {
			// TODO 自动生成的方法存根
			String notes = Util.getCurrentTime() + ":[" + addr + "]" + "上传数据:" + Util.formatBytesToStr(message);
			System.out.println(notes);
			String rootAddr = parameter.getRootAddr();
//			if(addr.equals(rootAddr)&&message[1] == GlobalDefines.GlobalCmd.G_DEF_READ_DATA ){
//				remoteClient.asyncWriteAndFlush(
//						formatDataToJsonStr("appstart", addr, Util.formatByteToByteStr(message)));
//			}
			if (!addr.equals(rootAddr)) {
				if(message.length == 1){
					System.out.println("多播配置心跳参数" + addr);
					paramConfigProxy.setConfigAck(addr);
				}else{
					byte type = message[1];// 0 是 globaltype  1 才是  type
					switch (type) {
					case GlobalDefines.GlobalCmd.G_DEF_READ_DATA:// 多播
					case GlobalDefines.GlobalCmd.G_DEF_CTL_ACK_READ_DATA:// 多播
						
						try {
						
							mulicastDataFile.append(Util.getCurrentTime() + ":[" + addr + "]" + "上传数据:"
									+ Util.formatBytesToStr(message));
							if (remoteClient.remoteHostIsOnline()) {
								try {
									// System.out.println(remoteDataDesPlus.encrypt(formatDataToJsonStr("app",addr,
									// Util.formatByteToByteStr(message))));
									remoteClient.asyncWriteAndFlush(
											formatDataToJsonStr("app", addr, "1"));
								} catch (Exception e) {
									// TODO 自动生成的 catch 块
									e.printStackTrace();
								}
							}
							
							
						if (message.length >= (GlobalDefines.GlobalIpVsId.G_DEF_IdLocation
								+ GlobalDefines.GlobalIpVsId.G_DEF_IdLength)) {
							StringBuilder tempId = new StringBuilder();
							for (int i = 0; i < GlobalDefines.GlobalIpVsId.G_DEF_IdLength; i++) {
								String str = Long.toHexString(message[GlobalDefines.GlobalIpVsId.G_DEF_IdLocation + i]);
								tempId.append(str);
							}
							String Id = tempId.toString();
							String Ip = Util.getIpv6LastByte(addr);
							if (IpidMap.containsKey(Ip)) {
								System.out.println("已经存储过IP和ID的对应关系啦");
								if (IpidMap.get(Ip).equals(Id)) {
									System.out.println("而且对应的ID没有发生改变");
								} else {
									System.out.println("对应的ID发生了改变，重新上报对应关系");
									remoteClient.asyncWriteAndFlush(formatDataToJsonStr("ipidmatchup", addr, Id));
									IpidMap.put(Ip, Id);
								}
							} else {
								System.out.println("之前没有存储过IP和ID的对应关系，上报对应关系");
								remoteClient.asyncWriteAndFlush(formatDataToJsonStr("ipidmatchup", addr, Id));
								IpidMap.put(Ip, Id);
							}
						}
						} catch (IOException e1) {
							// TODO 自动生成的 catch 块
							e1.printStackTrace();
						}
						byte[] buff = new byte[message.length - 1];
						System.arraycopy(message, 1, buff, 0, buff.length);
						if (nettyClient.remoteHostIsOnline() && webKeyFlag) {
							nettyClient.asyncWriteAndFlush(
									formatUcastDataToJsonStr("web_data", addr, Util.formatByteToByteStr(buff)));

						}
						// bitMap.setBit(addr, buff);
						buff = null;
						break;
					case GlobalDefines.GlobalCmd.G_DEF_REPORT_NET:
						System.arraycopy(message, 2, message, 0, message.length - 2);
						if (remoteClient.remoteHostIsOnline()) {
							try {
								remoteClient.asyncWriteAndFlush(formatDataToJsonStr("net", addr,  Util.formatByteToByteStr(message)));
							} catch (Exception e) {
								// TODO 自动生成的 catch 块
								e.printStackTrace();
							}
						}
						break;
					default:
						break;
					}
				}

			}
			return null;

		}

	}
	// 处理udp server收到的数据 ，添加到数据库，数据来源于无线网络内的所有节点,将数据直接传给TCP控制中枢

	public void sent_message(String addr, byte[] message) {
			String notes = Util.getCurrentTime() + ":[" + addr + "]" + "上传数据:" + Util.formatBytesToStr(message);
			System.out.println(notes);
			String rootAddr = parameter.getRootAddr();
//			if(addr.equals(rootAddr)&&message[1] == GlobalDefines.GlobalCmd.G_DEF_READ_DATA ){
//				remoteClient.asyncWriteAndFlush(
//						formatDataToJsonStr("appstart", addr, Util.formatByteToByteStr(message)));
//			}
			if (!addr.equals(rootAddr)) {
				if(message.length == 1){
					System.out.println("多播配置心跳参数" + addr);
					paramConfigProxy.setConfigAck(addr);
				}else{
					byte type = message[1];// 0 是 globaltype  1 才是  type
					switch (type) {
					case GlobalDefines.GlobalCmd.G_DEF_READ_DATA:// 多播
					case GlobalDefines.GlobalCmd.G_DEF_CTL_ACK_READ_DATA:// 多播
						
						try {
						
							mulicastDataFile.append(Util.getCurrentTime() + ":[" + addr + "]" + "上传数据:"
									+ Util.formatBytesToStr(message));
							if (remoteClient.remoteHostIsOnline()) {
								try {
									// System.out.println(remoteDataDesPlus.encrypt(formatDataToJsonStr("app",addr,
									// Util.formatByteToByteStr(message))));
									remoteClient.asyncWriteAndFlush(
											formatDataToJsonStr("app", addr, "1"));
								} catch (Exception e) {
									// TODO 自动生成的 catch 块
									e.printStackTrace();
								}
							}
							
							
						if (message.length >= (GlobalDefines.GlobalIpVsId.G_DEF_IdLocation
								+ GlobalDefines.GlobalIpVsId.G_DEF_IdLength)) {
							StringBuilder tempId = new StringBuilder();
							for (int i = 0; i < GlobalDefines.GlobalIpVsId.G_DEF_IdLength; i++) {
								String str = Long.toHexString(message[GlobalDefines.GlobalIpVsId.G_DEF_IdLocation + i]);
								tempId.append(str);
							}
							String Id = tempId.toString();
							String Ip = Util.getIpv6LastByte(addr);
							if (IpidMap.containsKey(Ip)) {
								System.out.println("已经存储过IP和ID的对应关系啦");
								if (IpidMap.get(Ip).equals(Id)) {
									System.out.println("而且对应的ID没有发生改变");
								} else {
									System.out.println("对应的ID发生了改变，重新上报对应关系");
									remoteClient.asyncWriteAndFlush(formatDataToJsonStr("ipidmatchup", addr, Id));
									IpidMap.put(Ip, Id);
								}
							} else {
								System.out.println("之前没有存储过IP和ID的对应关系，上报对应关系");
								remoteClient.asyncWriteAndFlush(formatDataToJsonStr("ipidmatchup", addr, Id));
								IpidMap.put(Ip, Id);
							}
						}
						} catch (IOException e1) {
							// TODO 自动生成的 catch 块
							e1.printStackTrace();
						}
						byte[] buff = new byte[message.length - 1];
						System.arraycopy(message, 1, buff, 0, buff.length);
						if (nettyClient.remoteHostIsOnline() && webKeyFlag) {
							nettyClient.asyncWriteAndFlush(
									formatUcastDataToJsonStr("web_data", addr, Util.formatByteToByteStr(buff)));

						}
						// bitMap.setBit(addr, buff);
						buff = null;
						break;
					case GlobalDefines.GlobalCmd.G_DEF_REPORT_NET:
						System.arraycopy(message, 2, message, 0, message.length - 2);
						if (remoteClient.remoteHostIsOnline()) {
							try {
								remoteClient.asyncWriteAndFlush(formatDataToJsonStr("net", addr,  Util.formatByteToByteStr(message)));
							} catch (Exception e) {
								// TODO 自动生成的 catch 块
								e.printStackTrace();
							}
						}
						break;
					default:
						break;
					}
				}

			}


	}

	public void putCommandToCache(byte[] content) {
		contentByteBuffer.clear();
		contentByteBuffer.put(content);
	}

	public byte[] getCommandFromCache() {
		int position = contentByteBuffer.position();
		byte[] content = new byte[position];
		contentByteBuffer.get(content);
		contentByteBuffer.position(position);
		System.arraycopy(contentByteBuffer.array(), 0, content, 0, position);
		return content;
	}

	public SocketAddress getSocketAddressByName(String host, int port) throws UnknownHostException {
		return new InetSocketAddress(InetAddress.getByName(host), port);
	}



	public static void main(String[] args) {
				new ConsoleMainServer_rpl();

	}
}
