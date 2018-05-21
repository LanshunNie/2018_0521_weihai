#coding:UTF-8
from app import app
import time
from time import mktime,strptime,strftime
import sqlite3
from db_operate import DBClass
from gxn_topo_analyzer import topo_statistic
from connect import Connect
from gxn_topo_analyzer import topo_traffic_statistic,topo_traffic_analyzer,app_traffic_analyzer,app_traffic_statistic
from num_of_rounds import countrounds

DATABASE = DBClass()

def multipledisplay(time1,time2,dbitems):
    # dbitems = "NodeID,rtimetric,currenttime"
    data_list = list() #形如[ [Date.UTC(1970,  9, 27), 0],[Date.UTC(1970, 10, 10), 0.6 ],...]

    dlist=  list()
    Data_set = DATABASE.my_db_execute(("select NodeID,"+ dbitems +",currenttime from NetMonitor where currenttime >= ? and currenttime <= ?;"),(time1, time2))    
    #print time1,time2,Data_set
    for x in Data_set:
        dicts=dict()
        # print x
        time_ms = time.mktime(time.strptime(x[2],'%Y-%m-%d %H:%M:%S'))*1000
        dicts["name"] = x[0].encode('ascii')
        # print time_ms
        dicts["data"] = [time_ms,x[1]]
        dlist.append(dicts)     
        # {'data': [1493568035000L, 835], 'name': u'0101'}

    dicttemp=dict()
    for x in dlist:
        if x["name"] in dicttemp:
            dicttemp[x["name"]].append(x["data"])
        else:
            dicttemp[x["name"]]=[x["data"]]

    for key,value in dicttemp.items():
        dicts = dict()
        dicts["name"] = key
        dicts["data"] = value
        # print dicts
        data_list.append(dicts)
    timedisplay = ("\""+time1 + ' - ' + time2+"\"").encode('ascii')
    # print data_list
    return data_list,timedisplay

def NetID_list(time1,time2):
    ID_list = list()
    ID_set = DATABASE.my_db_execute("select distinct NodeID from NetMonitor where currenttime >= ? and currenttime <= ?;",(time1, time2))
    for i in range(len(ID_set)):
        if len(ID_set[i][0])==3:
            ID = "0"+ID_set[i][0].upper()
        elif len(ID_set[i][0]) ==2:
            ID = "00"+ID_set[i][0].upper()
        elif len(ID_set[i][0])==1:
            ID = "000"+ID_set[i][0].upper()
        elif len(ID_set[i][0])==4:
            ID = ID_set[i][0].upper()
        else:
            return False
        ID_list.append(ID.encode('ascii'))
    return ID_list

def NetID_all():
    ID_list = list()
    ID_set = DATABASE.my_db_execute("select distinct NodeID from NetMonitor;",None)
    for i in range(len(ID_set)):
        if len(ID_set[i][0])==3:
            ID = "0"+ID_set[i][0].upper()
        elif len(ID_set[i][0]) ==2:
            ID = "00"+ID_set[i][0].upper()
        elif len(ID_set[i][0])==1:
            ID = "000"+ID_set[i][0].upper()
        elif len(ID_set[i][0])==4:
            ID = ID_set[i][0].upper()
        else:
            return False
        ID_list.append(ID.encode('ascii'))
    return ID_list

def AppID_all():
    ID_list = list()
    ID_set = DATABASE.my_db_execute("select distinct NodeID from ApplicationData;",None)
    for i in range(len(ID_set)):
        if len(ID_set[i][0])==3:
            ID = "0"+ID_set[i][0].upper()
        elif len(ID_set[i][0]) ==2:
            ID = "00"+ID_set[i][0].upper()
        elif len(ID_set[i][0])==1:
            ID = "000"+ID_set[i][0].upper()
        elif len(ID_set[i][0])==4:
            ID = ID_set[i][0].upper()
        else:
            return False
        ID_list.append(ID.encode('ascii'))
    return ID_list
    
def restart_display(time1,time2,dbitem):
    ID_list = NetID_list(time1,time2)
    ID_set = set(ID_list)
    data_dict = dict()
    data_dict1 = dict()
    ID_lists = list()
    data_list = list()
    data = DATABASE.my_db_execute(("select "+ dbitem +",NodeID from NetMonitor where currenttime >= ? and currenttime <= ? order by currenttime asc;"),(time1, time2))
    if len(data)!=0:
        counter=len(data)-1
        while len(ID_set)>0 and counter>-1:
            if data[counter][1].upper() in ID_set:
                data_dict[data[counter][1]] = data[counter][0]
                ID_set.remove(data[counter][1].upper())
            counter=counter-1

    ID_list = NetID_list(time1,time2)
    ID_set = set(ID_list)
    data = DATABASE.my_db_execute(("select "+ dbitem +",NodeID from NetMonitor where currenttime >= ? and currenttime <= ? order by currenttime desc;"),(time1, time2))
    if len(data)!=0:
        counter=len(data)-1
        while len(ID_set)>0 and counter > -1:
            if data[counter][1].upper() in ID_set:
                data_dict1[data[counter][1]] = data[counter][0]
                ID_set.remove(data[counter][1].upper())
            counter=counter-1
    # print data_dict
    # print data_dict1

    count=0
    for key, value in data_dict.items():
        if value > data_dict1[key]:
            data_list.append(value-data_dict1[key])
        else:
            data_list.append(value+256-data_dict1[key])
        count+=1
        if count%2!=0:
            key+='      '
        ID_lists.append(key.encode("ascii"))
    # print data_list
    timedisplay = ("\""+time1 + ' - ' + time2+"\"").encode('ascii')
    return ID_lists,data_list,timedisplay

def singledisplay(time1,time2,dbitem):
    ID_list = NetID_list(time1,time2)
    ID_set = set(ID_list)
    data_dict = dict()
    data = DATABASE.my_db_execute(("select "+ dbitem +",NodeID from NetMonitor where currenttime >= ? and currenttime <= ? order by currenttime asc;"),(time1, time2))
    if len(data)!=0:
        counter=len(data)-1
        while len(ID_set)>0 and counter >-1:
            if data[counter][1].upper() in ID_set:
                data_dict[data[counter][1]] = data[counter][0]
                ID_set.remove(data[counter][1].upper())
            counter=counter-1
        data_dict = sorted(data_dict.iteritems(), key=lambda d:d[1], reverse=True)
        ID_list = list()
        data_list = list()
        count=0
        for key, value in data_dict:
            count+=1
            key=key.encode("ascii")
            if count%2!=0:
                key+='      '
            ID_list.append(key)
            data_list.append(value)
    else:
        data_list=[]
    timedisplay = ("\""+time1 + ' - ' + time2+"\"").encode('ascii')
    return ID_list,data_list,timedisplay

def energy_display(time1,time2):
    cpu_list = list()
    lpm_list = list()
    tx_list = list()
    rx_list = list()

    ID_list = NetID_list(time1,time2)
    ID_set = set(ID_list)
    # print ID_set
    energy = DATABASE.my_db_execute("select CPU,LPM,TX,RX,NodeID from NetMonitor where currenttime >= ? and currenttime <= ? order by currenttime asc;",(time1, time2))
    if len(energy)!=0:
        counter=len(energy)-1
        while len(ID_set)>0 and counter > -1:
            if energy[counter][4].upper() in ID_set:
                cpu_list.append(round(float(energy[counter][0])/32768,2))
                lpm_list.append(round(float(energy[counter][1])/32768,2))
                tx_list.append(round(float(energy[counter][2])/32768,2))
                rx_list.append(round(float(energy[counter][3])/32768,2))
                ID_set.remove(energy[counter][4].upper())
            counter=counter-1
    # print ID_set
    timedisplay = ("\""+time1 + ' - ' + time2+"\"").encode('ascii')
    return cpu_list,lpm_list,tx_list,rx_list,timedisplay

def flowdisplay(time1,time2):
    topodata_list = DATABASE.my_db_execute("select * from NetMonitor where currenttime >= ? and currenttime <= ?;",(time1, time2))
    topo_traff_dict=topo_traffic_statistic(topodata_list)
    traffic_key_list = list()
    traffic_value_list = list()
    sum_value = 0
    for key ,value in topo_traff_dict.items():
        traffic_key_list.append(key.encode('ascii'))
        sum_value = sum_value+value
        traffic_value_list.append(sum_value)
    lists=topo_traffic_analyzer(topodata_list)
    templist=[lists[1],lists[2],lists[3],lists[4],lists[5],lists[6],lists[7],lists[8],lists[9]]
    timedisplay = ("\""+time1 + ' - ' + time2+"\"").encode('ascii')
    return lists[0],templist,traffic_key_list,traffic_value_list,timedisplay

def appflowdisplay(time1,time2):
    appdata_list = DATABASE.my_db_execute("select * from ApplicationData where currenttime >= ? and currenttime <= ?;",(time1, time2))
    app_traff_dict=app_traffic_statistic(appdata_list)
    traffic_key_list = list()
    traffic_value_list = list()
    sum_value = 0
    for key ,value in app_traff_dict.items():
        traffic_key_list.append(key.encode('ascii'))
        sum_value = sum_value+value
        traffic_value_list.append(sum_value)
    lists=app_traffic_analyzer(appdata_list)
    templist=[lists[1],lists[2],lists[3],lists[4],lists[5],lists[6],lists[7],lists[8],lists[9]]
    timedisplay = ("\""+time1 + ' - ' + time2+"\"").encode('ascii')
    return lists[0],templist,traffic_key_list,traffic_value_list,timedisplay

def protodisplay(time1,time2):
    num_of_nodes = DATABASE.my_db_execute("select count(distinct NodeID) from NetMonitor;",None)[0][0]#当前NetMonitor表里所有不重复的节点总数
    http_set = selectall(time1,time2,"NetMonitor")
    # 本轮上报个数
    lasttime = DATABASE.my_db_execute("select currenttime from NetMonitor where currenttime >= ? and currenttime <= ? order by currenttime desc LIMIT 1;",(time1, time2))
    if lasttime:
        real_end_time = time.mktime(time.strptime(lasttime[0][0],'%Y-%m-%d %H:%M:%S')) #取选定时间内的最后一个时间，算这个时间与它前十分钟内的数据
        real_start_time = real_end_time - 10 * 60
        rstart_time = strftime("%Y-%m-%d %H:%M:%S", time.localtime(real_start_time))
        rend_time = strftime("%Y-%m-%d %H:%M:%S", time.localtime(real_end_time))
        print rstart_time
        print rend_time
        post = DATABASE.my_db_execute("select count(distinct NodeID) from NetMonitor where currenttime >= ? and currenttime <= ?;",(rstart_time, rend_time))[0][0]
        thispostrate = round((float(post)/len(http_set[0])), 4) * 100
    else:
        post = "?"
        thispostrate = "?"
    # 根据调度计算所选时间段内轮数
    rounds = countrounds(time1,time2)
    print rounds
    if rounds:
        allposts = DATABASE.my_db_execute("select count(*) from NetMonitor where currenttime >= ? and currenttime <= ?;",(time1, time2))[0][0]
        if num_of_nodes:
            postrate = round((float(allposts)/(rounds * num_of_nodes)), 4) * 100
        else:
            postrate = "?"
    else:
        postrate = "?"
    timedisplay = ("\""+time1 + ' - ' + time2+"\"").encode('ascii')
    #print timedisplay
    return num_of_nodes,postrate,post,thispostrate,http_set[0],http_set[1],timedisplay

def nodesearch_display(time1,time2,node):
    time_list_1 = list()
    time_list_2 = list()
    time_list_3 = list()
    time_list_4 = list()
    voltage_list = list()
    current_list = list()
    rtx_list = list()
    beacon_list = list()
    nodeid_list = NetID_all()
    nodeid_list.sort()
    cpu ,lpm ,tx ,rx=[0,0,0,0]

    deploy_info = DATABASE.my_db_execute('select NodeID, MeterID, Place from NodePlace where NodeID == ?;',(node,))
    if deploy_info:
        deploy = list()
        deploy.append(deploy_info[0][0].encode('ascii'))
        deploy.append(deploy_info[0][1].encode('ascii'))
        deploy.append(deploy_info[0][2])
    else:
        deploy = ["no data","no data","no data"]
    voltage = DATABASE.my_db_execute('select currenttime, volage from NetMonitor where currenttime >= ? and currenttime <= ? and NodeID == ?;',(time1, time2, node))
    for i in range(len(voltage)):
        time_list_1.append(voltage[i][0].encode('ascii'))
        voltage_list.append(round(float(voltage[i][1]),2))
    current = DATABASE.my_db_execute('select currenttime, electric from NetMonitor where currenttime >= ? and currenttime <= ? and NodeID == ?;',(time1, time2, node))
    for i in range(len(current)):
        time_list_2.append(current[i][0].encode('ascii'))
        current_list.append(round(float(current[i][1]),2))
    rtx = DATABASE.my_db_execute('select currenttime, rtimetric from NetMonitor where currenttime >= ? and currenttime <= ? and NodeID == ?;',(time1, time2, node))
    for i in range(len(rtx)):
        time_list_3.append(rtx[i][0].encode('ascii'))
        rtx_list.append(rtx[i][1])
    beacon = DATABASE.my_db_execute('select currenttime, beacon from NetMonitor where currenttime >= ? and currenttime <= ? and NodeID == ?;',(time1, time2, node))
    for i in range(len(beacon)):
        time_list_4.append(beacon[i][0].encode('ascii'))
        beacon_list.append(beacon[i][1])

    energycost = DATABASE.my_db_execute('select CPU,LPM,TX,RX from NetMonitor where NodeID==? order by ID desc LIMIT 1',(node,))
    if energycost:
        cpu= round(float(energycost[0][0])/32768,2)
        lpm= round(float(energycost[0][1])/32768,2)
        tx = round(float(energycost[0][2])/32768,2)
        rx = round(float(energycost[0][3])/32768,2)   
    else:
        cpu = 0
        lpm = 0
        tx = 0
        rx = 0        
    timedisplay = ("\""+time1 + ' - ' + time2+"\"").encode('ascii')
    return nodeid_list,str(cpu),str(lpm),str(tx),str(rx),voltage_list,time_list_1,time_list_2,current_list,time_list_3,rtx_list,deploy,timedisplay,time_list_4,beacon_list


def node_time_display(time1,time2,db,node):
    data = DATABASE.my_db_execute("select currenttime from " + db + " where NodeID==? and currenttime >= ? and currenttime <= ?;",(node, time1, time2))
    count = 0
    timelist = list()
    for time in data:
        time_ms = mktime(strptime(time[0],'%Y-%m-%d %H:%M:%S'))*1000
        count += 1
        timelist.append([time_ms,count])
    dicts = dict()
    if node:
        dicts["name"] = node.encode('ascii')
    else:
        dicts["name"] = None
    dicts["data"] = timelist
    lists = list()
    lists.append(dicts)
    timedisplay = ("\""+time1 + ' - ' + time2+"\"").encode('ascii')
    return lists,timedisplay

def selectall(time1,time2,db):
    data = DATABASE.my_db_execute("select * from " + db + " where currenttime >= ? and currenttime <= ?;",(time1, time2))#从名为db的表里取出所有的表项
    data_dict = topo_statistic(data)
    #print data_dict
    data_dict = sorted(data_dict.iteritems(), key=lambda d:d[1], reverse=True)#降序
    #print data_dict
    data_key_list = list()
    data_value_list = list()
    count=0
    for key, value in data_dict:
        count+=1
        if count%2==0:
            data_key_list.append(key.encode('UTF-8'))
        else:
            data_key_list.append(key.encode('UTF-8')+'     ')
        data_value_list.append(value)
    timedisplay = ("\""+time1 + ' - ' + time2+"\"").encode('ascii')
    #print data_key_list #['008e     ', '021c', '007d     ', '0108'] 一个列表，存放节点ID，奇数项后面的空格？？
    #print data_value_list#[1, 1, 1, 1]一个列表，存放对应节点ID出现的次数，即对应节点上报个数`
    #print timedisplay#"2017-10-17 00:00:00 - 2017-10-17 23:59:00"选取的时间段
    return data_key_list,data_value_list,timedisplay

def topo_display(time1,time2):
    getrootaddr = Connect()
    rootID = getrootaddr.rootaddr().upper()
    if len(rootID)==3:
        rootID = "0"+rootID
    elif len(rootID)==2:
        rootID = "00"+rootID
    elif len(rootID)==1:
        rootID = "000"+rootID
    else:
        rootID = "rootIDerror"
    lasttime = DATABASE.my_db_execute("select currenttime from NetMonitor where currenttime >= ? and currenttime <= ? order by currenttime desc LIMIT 1;",(time1, time2))
    if lasttime:
        real_end_time = time.mktime(time.strptime(lasttime[0][0],'%Y-%m-%d %H:%M:%S')) #取选定时间内的最后一个时间，算这个时间与它前十分钟内的数据
        real_start_time = real_end_time - 10 * 60
        # real_start_time = real_end_time - 24*60 * 60
        start_time = strftime("%Y-%m-%d %H:%M:%S", time.localtime(real_start_time))
        end_time = strftime("%Y-%m-%d %H:%M:%S", time.localtime(real_end_time))
    else:
        timedisplay = ("\""+time1 + ' - ' + time2+"\"").encode('ascii')
        return ([],[],timedisplay)
    ID_list = DATABASE.my_db_execute("select NodeID, ParentID from NetMonitor where currenttime >= ? and currenttime <= ?;",(start_time, end_time))
    Parentnode = dict()
    for node in ID_list:
        ID = node[0].upper() # ID
        ParentID = node[1].upper() # parentID
        if ID in Parentnode:
            continue
        else:
            Parentnode[ID] = ParentID
    # print Parentnode
    # 遍历Parentnode的key，绘制散点图；遍历Parentnode的key和value，画箭头

    nodes = list()
    links = list()
    n = dict()
    m = dict()
    all_nodes = set()
    if rootID not in Parentnode.keys():
        rootIDjson = {"category":0, "name":str(rootID.encode('ascii'))}
        nodes.append(rootIDjson)
        for key ,value in Parentnode.items():
            all_nodes.add(key.encode('ascii'))
            all_nodes.add(value.encode('ascii'))
            m = {"source":key.encode('ascii'), "target":value.encode('ascii'), "weight":1}
            links.append(m)
    else:
        for key ,value in Parentnode.items():
            if key==rootID:
                all_nodes.add(key.encode('ascii'))
                all_nodes.add(value.encode('ascii'))
                m = {"source":key.encode('ascii'), "target":value.encode('ascii'), "weight":1}
                links.append(m)
            else:
                all_nodes.add(key.encode('ascii'))
                all_nodes.add(value.encode('ascii'))
                m = {"source":key.encode('ascii'), "target":value.encode('ascii'), "weight":1}
                links.append(m)
    # links.append({"source":"00EB","target":"0015","weight":1})
    # print links
    # print all_nodes
    for no in all_nodes:
        n = {"category":1, "name":no}
        nodes.append(n)
    timedisplay = ("\""+time1 + ' - ' + time2+"\"").encode('ascii')
    return nodes,links,timedisplay
