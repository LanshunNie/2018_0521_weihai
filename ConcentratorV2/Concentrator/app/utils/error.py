#coding:UTF-8
from app import app
from db_operate import DBClass
import sqlite3
import time
from time import strftime

DATABASE = DBClass()
# def data_error(time1,time2):
#     warning_list = list()
#     idata = DATABASE.my_db_execute('select ID, electric, NodeID, currenttime from NetMonitor where currenttime >= ? and currenttime <= ? and electric>600;',(time1, time2))
#     for i in idata:
#         warning_dict = dict()
#         warning_dict["seqnum"] = i[0]
#         warning_dict["warn"] = "current = " + str(i[1]) + "uA"
#         warning_dict["ip_port"] = i[2] #NodeID
#         warning_dict["time"] = i[3] #currenttime
#         warning_list.append(warning_dict)

#     vdata = DATABASE.my_db_execute('select ID, volage, NodeID, currenttime from NetMonitor where currenttime >= ? and currenttime <= ? and volage<3;',(time1, time2))       
#     for v in vdata:
#         warning_dict = dict()
#         warning_dict["seqnum"] = v[0]
#         warning_dict["warn"] = "voltage = " + str(v[1]) + "V"
#         warning_dict["ip_port"] = v[2] #NodeID
#         warning_dict["time"] = v[3] #currenttime
#         warning_list.append(warning_dict)
#         # print warning_dict
#         print warning_list
#     return warning_list

def data_error_new(time1,time2):
    warning_list_v = list()
    warning_list_i = list()    
    vnodeset = set()
    inodeset = set()
    inode_dict = dict()
    vnode_dict = dict()
    # {
    #     "name": "阿里巴巴上市", 
    #     "evolution": [
    #         {
    #             "time": "2014-05-01", 
    #             "value": 14, 
    #             "detail": {
    #                 "link": "http://www.baidu.com", 
    #                 "text": "百度指数", 
    #                 "img": '../asset/ico/favicon.png'
    #             }
    #         }
    #     ]
    # }
    idata = DATABASE.my_db_execute('select electric, NodeID, currenttime from NetMonitor where currenttime >= ? and currenttime <= ? and electric>40;',(time1, time2))
    if idata:
        for i in idata:
            evolution_dict_temp = dict()
            warning_detail = dict()
            if i[1].encode('ascii') in inodeset:                
                evolution_dict_temp["time"] = str(i[2]).encode('ascii')
                evolution_dict_temp["value"] = 50
                warning_detail["text"] = i[1].encode('ascii')+":Current="+str(round(i[0],2))+"uA"
                warning_detail["link"] = "javascript:;"
                evolution_dict_temp["detail"] = warning_detail
                inode_dict[i[1]]["evolution"].append(evolution_dict_temp)
            else:
                warning_dict_temp = dict()
                evolution_list = list()
                inode_dict[str(i[1].encode('ascii'))]=dict()
                inodeset.add(str(i[1].encode('ascii')))
                evolution_dict_temp["time"] = str(i[2]).encode('ascii')
                evolution_dict_temp["value"] = 50
                warning_detail["text"] = i[1].encode('ascii')+":Current="+str(round(i[0],2))+"uA"
                warning_detail["link"] = "javascript:;"
                evolution_dict_temp["detail"] = warning_detail
                evolution_list.append(evolution_dict_temp)
                warning_dict_temp["name"] = i[1].encode('ascii') #NodeID
                warning_dict_temp["evolution"] = evolution_list
                inode_dict[i[1]]=warning_dict_temp

        for key,value in inode_dict.items():
            warning_list_i.append(value)
    else:
        warning_dict_i = dict()
        warning_dict_temp = dict()
        warning_detail = dict()
        warning_dict_temp["value"] = 30
        warning_dict_temp["time"] = time2.encode('ascii')
        # print warning_dict_temp["time"] 
        warning_detail["text"] = "No Current Error"
        warning_detail["link"] = "javascript:;"
        warning_dict_temp["detail"] = warning_detail
        warning_dict_i["name"] = "None"
        warning_dict_i["evolution"] = [warning_dict_temp]
        warning_list_i.append(warning_dict_i)
    # print len(warning_list_i)
    
    vdata = DATABASE.my_db_execute('select volage, NodeID, currenttime from NetMonitor where currenttime >= ? and currenttime <= ? and volage<3;',(time1, time2))
    if vdata:
        for v in vdata:
            evolution_dict_temp = dict()
            warning_detail = dict()
            if v[1].encode('ascii') in vnodeset:                
                evolution_dict_temp["time"] = str(v[2]).encode('ascii')
                evolution_dict_temp["value"] = 50
                warning_detail["text"] = v[1].encode('ascii')+":Voltage="+str(round(v[0],2))+"V"
                warning_detail["link"] = "javascript:;"
                evolution_dict_temp["detail"] = warning_detail
                inode_dict[v[1]]["evolution"].append(evolution_dict_temp)
            else:
                warning_dict_temp = dict()
                evolution_list = list()
                inode_dict[str(v[1].encode('ascii'))]=dict()
                inodeset.add(str(v[1].encode('ascii')))
                evolution_dict_temp["time"] = str(v[2]).encode('ascii')
                evolution_dict_temp["value"] = 50
                warning_detail["text"] = v[1].encode('ascii')+":Voltage="+str(round(v[0],2))+"V"
                warning_detail["link"] = "javascript:;"
                evolution_dict_temp["detail"] = warning_detail
                evolution_list.append(evolution_dict_temp)
                warning_dict_temp["name"] = v[1].encode('ascii') #NodeID
                warning_dict_temp["evolution"] = evolution_list
                vnode_dict[v[1]]=warning_dict_temp

        for key,value in vnode_dict.items():
            warning_list_v.append(value)
    else:
        warning_dict_v = dict()
        warning_dict_temp = dict()
        warning_detail = dict()
        warning_dict_temp["value"] = 30
        warning_dict_temp["time"] = time1.encode('ascii')
        # print warning_dict_temp["time"]  
        warning_detail["text"] = "No Voltage Error"
        warning_detail["link"] = "javascript:;"
        warning_dict_temp["detail"] = warning_detail
        warning_dict_v["name"] = "None"
        warning_dict_v["evolution"] = [warning_dict_temp]
        warning_list_v.append(warning_dict_v)

    nodeinfo = DATABASE.my_db_execute('select * from NodePlace;',None)
    error_set = set()
    for info in nodeinfo:
        # print "====",info[1].encode('ascii')
        if info[1].encode('ascii').lower() in inodeset:
            # print info[1].encode('ascii')
            error_set.add(info)
        if info[1].encode('ascii').lower() in vnodeset:
            error_set.add(info)
    lists = list()
    # print error_set,len(error_set)
    # print inodeset,len(inodeset)
    # print vnodeset
    for x in error_set:
        lists.append([x[1],x[2],x[3]])
    timedisplay = ("\""+time1 + ' - ' + time2+"\"").encode('ascii')
    return warning_list_v,warning_list_i,lists,timedisplay

def syn_error(time1,time2):
    # warning_list= list()    
    # nodeset = set()
    # data = DATABASE.my_db_execute('select syntime, NodeID, currenttime from NetMonitor where currenttime >= ? and currenttime <= ? and (syntime>30 or syntime<-30);',(time1, time2))
    # if data:
    #     nodedict = dict()
    #     for i in data:
    #         warning_dict_temp = dict()
    #         if i[1] in nodedict.keys():
    #             evolution_dict_temp = dict()
    #             warning_detail = dict()
    #             evolution_dict_temp["time"] = str(i[2]).encode('ascii')
    #             evolution_dict_temp["value"] = 50
    #             warning_detail["text"] = i[1].encode('ascii')+":Syntime="+str(i[0])+"ms"
    #             warning_detail["link"] = "javascript:;"
    #             evolution_dict_temp["detail"] = warning_detail
    #             evolution_list = nodedict[i[1]]["evolution"]
    #             evolution_list.append(evolution_dict_temp)
    #             warning_dict_temp["name"] = i[1].encode('ascii')
    #             # print warning_dict_temp["name"]
    #             warning_dict_temp["evolution"] = evolution_list
    #         else:
    #             nodeset.add(i[1])
    #             evolution_dict_temp = dict()
    #             warning_detail = dict()
    #             evolution_list = list()
    #             evolution_dict_temp["time"] = str(i[2]).encode('ascii')
    #             evolution_dict_temp["value"] = 50
    #             warning_detail["text"] = i[1].encode('ascii')+":Syntime="+str(i[0])+"ms"
    #             warning_detail["link"] = "javascript:;"
    #             evolution_dict_temp["detail"] = warning_detail
    #             evolution_list.append(evolution_dict_temp)
    #             warning_dict_temp["name"] = i[1].encode('ascii') #NodeID
    #             warning_dict_temp["evolution"] = evolution_list
    #             nodedict[i[1]]=warning_dict_temp
    #     # print nodedict.keys()
    #         warning_list.append(warning_dict_temp)
    #     # print warning_list
    # else:
    #     warning_dict = dict()
    #     warning_dict_temp = dict()
    #     warning_detail = dict()
    #     warning_dict_temp["value"] = 30
    #     warning_dict_temp["time"] = time2.encode('ascii')
    #     warning_detail["text"] = "No Syntime Error"
    #     warning_detail["link"] = "javascript:;"
    #     warning_dict_temp["detail"] = warning_detail
    #     warning_dict["name"] = "None"
    #     warning_dict["evolution"] = [warning_dict_temp]
    #     warning_list.append(warning_dict)
    # nodeinfo = DATABASE.my_db_execute('select * from NodePlace;',None)
    # lists = list() # nodeplace info
    # for info in nodeinfo:
    #     if info[1] in nodeset:
    #         lists.append([info[1],info[2],info[3]])
    # # print time.time()-timed
    # return warning_list,lists
    data_list = list() #形如[ [Date.UTC(1970,  9, 27), 0],[Date.UTC(1970, 10, 10), 0.6 ],...]
    nodes = set()
    dlist=  list()
    Data_set = DATABASE.my_db_execute(("select NodeID,syntime,currenttime from NetMonitor where currenttime >= ? and currenttime <= ? and (syntime>7 or syntime<-7);"),(time1, time2))    
    for x in Data_set:
        if x[0] not in nodes:
            nodes.add(x[0])
        dicts=dict()
        time_ms = int(time.mktime(time.strptime(x[2],'%Y-%m-%d %H:%M:%S'))*1000)
        dicts["name"] = x[0].encode('ascii')
        dicts["data"] = [str(time_ms),int(x[1])]
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
    nodeinfo = DATABASE.my_db_execute('select * from NodePlace;',None)
    lists = list() # nodeplace info
    for info in nodeinfo:
        if info[1] in nodes:
            lists.append([info[1],info[2],info[3]])
    timedisplay = ("\""+time1 + ' - ' + time2+"\"").encode('ascii')
    return data_list,lists,timedisplay