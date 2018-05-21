import socket,traceback,time,struct
import sys
reload(sys)
sys.setdefaultencoding('utf8')
host=''
port=12400
s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
s.bind((host,port))
while 1: 
    try:
        message,address=s.recvfrom(8192)
        #arry = message.encode('utf-8')
        print(message)
        a,b,c,d,e,f,g,h,i,j,k = struct.unpack('3B1I2B1B1B2I1B',message)
        print(a,b,c,d,e,f,g,h,i,j,k)
        print(address)
        type = raw_input("please input the type:")
        print("the type is:",type)
        if type == "100":
            secs = ""
            reply=struct.pack('5b0s1b',6,16,-16,0,0,secs,0)
            print(secs)
            s.sendto(reply,address)
        elif type == "101":
            secs = ""
            reply=struct.pack('5b0s1b',6,16,-16,0,1,secs,0)
            print(secs)
            s.sendto(reply,address)
        elif type == "102":
            times = time.strftime('%H:%M:%S',time.localtime(time.time()))
            print times
            secs = times+":0:17"
            length = len(secs)
            reply=struct.pack('5b'+length+'s1b',20,16,-96,length,2,secs,0)
            print(secs)
            s.sendto(reply,address)
        elif type == "140":
            secs = ""
            reply=struct.pack('5b0s1b',20,16,-96,0,64,secs,0)
            print(secs)
            s.sendto(reply,address)
        elif type == "141":
            period = raw_input("input the period:")
            length = len(period)
            reply=struct.pack('5b'+length+'s1b',20,16,-96,length,65,period,0)
            print(secs)
            s.sendto(reply,address)
        elif type == "142":
            times = time.strftime('%H:%M:%S',time.localtime(time.time()))
            print times
            schedule = raw_input("input the schedule:")
            secs = times+":"+schedule+":0:17"
            length = len(secs)
            reply=struct.pack('5b'+length+'s1b',length+6,16,-96,length,66,secs,0)
            print(secs)
            s.sendto(reply,address)
        elif type == "180":
            secs='05105BFE5916'
            reply=struct.pack('5b12s1b',18,16,-96,12,-128,secs,0)
            print(secs)
            s.sendto(reply,address)
        elif type == "181":
            times = time.strftime('%H:%M:%S',time.localtime(time.time()))
            print times
            schedule = raw_input("input the schedule:")
            secs = times+":"+schedule+":0:17"
            length = len(secs)
            reply=struct.pack('5b'+length+'s1b',length+6,16,-96,length,-127,secs,0)
            print(secs)
            s.sendto(reply,address)
        elif type == "182":
            secs='105BFE5916'
            reply=struct.pack('5b10s1b',18,16,-96,10,-126,secs,0)
            print(secs)
            s.sendto(reply,address)
        elif type == "1c0":
            secs = ""
            reply=struct.pack('5b0s1b',20,16,-96,0,-64,secs,0)
            print(secs)
            s.sendto(reply,address)
        elif type == "1c1":
            secs = ""
            reply=struct.pack('5b0s1b',20,16,-96,0,-63,secs,0)
            print(secs)
            s.sendto(reply,address)
        elif type == "1c2":
            times = time.strftime('%H:%M:%S',time.localtime(time.time()))
            print times
            schedule = raw_input("input the schedule:")
            period = raw_input("input the period:")
            state = raw_input("input the state:")
            secs = times+":"+schedule+":"+period+":"+state+":"+":0:17"
            length = len(secs)
            reply=struct.pack('5b'+length+'s1b',length+6,16,-96,length,-62,secs,0)
            print(secs)
            s.sendto(reply,address)
        elif type == "1c3":
            secs = ""
            reply=struct.pack('5b0s1b',20,16,-96,0,-61,secs,0)
            print(secs)
            s.sendto(reply,address)
        elif type == "1c4":
            secs = ""
            reply=struct.pack('5b0s1b',20,16,-96,0,-60,secs,0)
            print(secs)
            s.sendto(reply,address)
        elif type == "010":
            begin_time = raw_input("input the begin time:")
            reply=struct.pack('5b19s1b',25,16,112,19,16,begin_time,0)
            print(secs)
            s.sendto(reply,address)
        elif type == "100":
            period = raw_input("input the period:")
            length = len(period)
            reply=struct.pack('5b'+length+'s1b',length+6,16,48,length,0,period,0)
            print(secs)
            s.sendto(reply,address)
        elif type == "103":
            secs = ""
            reply=struct.pack('5b0s1b',6,16,48,0,3,secs,0)
            print(secs)
            s.sendto(reply,address)
        elif type == "104":
            secs = ""
            reply=struct.pack('5b0s1b',6,16,112,0,4,secs,0)
            print(secs)
            s.sendto(reply,address)
        elif type == "105":
            secs = ""
            reply=struct.pack('5b0s1b',6,16,112,0,5,secs,0)
            print(secs)
            s.sendto(reply,address)
        elif type == "107":
            secs = ""
            reply=struct.pack('5b0s1b',6,16,112,0,7,secs,0)
            print(secs)
            s.sendto(reply,address)
        elif type == "10a":
            secs = ""
            reply=struct.pack('5b0s1b',6,16,48,0,10,secs,0)
            print(secs)
            s.sendto(reply,address)
        elif type == "10c":
            secs = ""
            reply=struct.pack('5b0s1b',6,16,48,0,12,secs,0)
            print(secs)
            s.sendto(reply,address)
        elif type == "10d":
            secs = ""
            reply=struct.pack('5b0s1b',6,16,48,0,13,secs,0)
            print(secs)
            s.sendto(reply,address)
        else:
            #message,address=s.recvfrom(8192)
            #arry = message.encode('utf-8')
            print(message)
        time.sleep(15)
    except (KeyboardInterrupt,SystemExit):
        raise
    except:
        traceback.print_exc()



