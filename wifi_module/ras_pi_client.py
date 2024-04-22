import socket


class WifiNode():
    def __init__(self, real=True, IP="192.168.4.1", PORT=80, BPS=460800):
        self.available = True
        self.byte_len = len('000'.encode('utf-8'))
        if real:
            try:
                self.p = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                # self.p.settimeout(3)
                self.p.connect((IP, int(PORT)))
            except Exception as e:
                print(e)
                self.available = False
        else:
            self.available = False
    
    def _check(self):
        if self.available: # and self.ser.isOpen():
            print(f"Serial successful. a:{self.available}|o:to-do")
            return True
        else:
            print(f"Port not available. a:{self.available}|o:to-do")
            return False

    def write_data(self, data="Hello I am Ras PI", encoding='utf-8'):
        if self._check():
            self.p.send(data.encode(encoding))    #writ a string to port
            self.byte_len = len(data.encode(encoding))
            print("Sent data:", data, "|len:", len(data))
        else:
            print("Port not available, cannot write")
        
    def read_data(self, decoding='utf-8'):
        if self._check():
            response = self.p.recv(self.byte_len)
            print("Received data:", response, "|len:", len(response))
            response = response.decode(decoding)
            print("Received data(decoded):", response, "|len:", len(response))
            return response
        else:
            return "Port not available, cannot read"

server = WifiNode()    
while 1:
    server.write_data("Hello I am Ras PI")
    server.read_data()