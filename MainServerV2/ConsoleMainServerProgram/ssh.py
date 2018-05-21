import socket
from datetime import datetime

address = ('139.199.154.37',12400)
max_size =1000
print("Start the client at {}".format(datetime.now()))
client = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
client.connect(address)
client.sendall('wwww')
print("aaaa")
data = client.recv(max_size)
print(data)
client.close()

