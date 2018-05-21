#coding:UTF-8

import socket
import json
import os
from app import app
from flask import session
#登录控制
class loginjudge:
	mark = 0
	def __init__(self):
		if "mark" in session:
			if session["mark"] == "true":
				self.mark = 1
	def turntrue(self):
		session["mark"] = "true"
		self.mark = 1
	def turnfalse(self):
		session["mark"] = "false"
		self.mark = 0

	def getPCAPS(self):
		if self.mark == 1:
			return "True"
		else:
			return "False"

#首页信息展示
class Connect:
	def __init__(self):
		self.Config_FILE = os.path.join(app.config['CONFIG_FOLDER'],"config.json")
		f=open(self.Config_FILE,'r')
		self.CONFIG_DICT =json.load(f)
		f.close()
	


	def TCP_send(self, ins):
		serverip = self.CONFIG_DICT['serverIp']
		serverport = int(self.CONFIG_DICT['tcpPort'])
		cli=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
		cli.connect((serverip,serverport))
		cli.send(ins)
		cli.close()

	def UDP_send(self, ins):
		serverip = self.CONFIG_DICT['serverIp']
		serverport = int(self.CONFIG_DICT['udpPort'])
		cli=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
		cli.connect((serverip,serverport))
		cli.send(ins)
		cli.close()

	def all_config_json(self):
		return self.CONFIG_DICT

	def rootaddr(self):
		root =str(self.CONFIG_DICT["rootAddr"].split(":")[-1])
		return root

	def display_config(self):
		display_dict = dict()
		display_dict["id"] = self.CONFIG_DICT['id']
		display_dict["HeartIntSec"] = self.CONFIG_DICT['HeartIntSec']
		display_dict["AckHeartInt"] = self.CONFIG_DICT['AckHeartInt']		
		display_dict["rootAddr"] = self.CONFIG_DICT['rootAddr']
		display_dict["ftpuser"] = self.CONFIG_DICT['ftpuser']
		display_dict["ftphost"] = self.CONFIG_DICT['ftphost']
		display_dict["ftpPwd"] = self.CONFIG_DICT['ftpPwd']
		display_dict["ftpPort"] = self.CONFIG_DICT['ftpPort']
		display_dict["serverIp"] = self.CONFIG_DICT['serverIp']
		return display_dict
	def update_config(self,dicts):
		with open(self.Config_FILE, 'w') as f:
			f.write(dicts)
			f.close()
		f=open(self.Config_FILE,'r')
		self.CONFIG_DICT =json.load(f)
		f.close()

