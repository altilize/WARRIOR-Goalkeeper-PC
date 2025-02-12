import string,cgi,time
from os import curdir, sep
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
import SocketServer
from SocketServer import ThreadingMixIn
import re
import os
import sys
sys.path.append('/usr/include/opencv')
import cgi
#import cv
import cv2
import numpy as np
import time

hmn=0
smn=0
vmn=0
hmx=0
smx=0
vmx=0
switch = 0

portVideo = 0

lokasi = "/home/r2c/war/kalibrasi/lapangan"

if len(sys.argv)>1:
    portVideo = int(sys.argv[1])
    print "Using webcam",portVideo, "\n"

capture = cv2.VideoCapture(portVideo)
cameraQuality = 75

def readFile():
    global hmn, smn, vmn, hmx, smx, vmx, switch
    print 'Reading file'
    fileName = "bola.txt"
    if switch==0:
        fileName = lokasi+"/../garis.txt"
    elif switch==1:
        fileName = lokasi+"/../lapangan.txt"
    elif switch==2:
        fileName = lokasi+"/../bola.txt"
    elif switch==3:
	fileName = lokasi+"/../gawang.txt"

    if(os.path.isfile(fileName)):
        file = open(fileName,"r")
        hmn = int(file.readline())
        smn = int(file.readline())
        vmn = int(file.readline())
        hmx = int(file.readline())
        smx = int(file.readline())
        vmx = int(file.readline())
        file.close()
    else:
        print "File %s tidak ada" % fileName
        hmn=0
        smn=0
        vmn=0
        hmx=255
        smx=255
        vmx=255
    return

def writeFile():
    global hmn, smn, vmn, hmx, smx, vmx, switch
    print 'Writing file'
    if switch==0:
        file = open(lokasi+"/../garis.txt","w")
    elif switch==1:
        file = open(lokasi+"/../lapangan.txt","w")
    elif switch==2:
        file = open(lokasi+"/../bola.txt", "w");
    elif switch==3:
	file = open(lokasi+"/../gawang.txt", "w");

    file.write("%d\n" %hmn)
    file.write("%d\n" %smn)
    file.write("%d\n" %vmn)
    file.write("%d\n" %hmx)
    file.write("%d\n" %smx)
    file.write("%d\n" %vmx)
    file.close()
    return

def backupFile():
    global hmn, smn, vmn, hmx, smx, vmx, switch
    print 'Writing file'

    directory = lokasi+"/../backup"
    if not os.path.exists(directory):
        os.makedirs(directory)

    fileName = "backup"
    timeNow = time.strftime("%d %b %y %H:%M:%S")
    if switch==0:i
        fileName = "garis"
    elif switch==1:
        fileName = "lapangan"
    elif switch==2:
        fileName = "bola"
    elif switch==3:
	fileName = "gawang"

    file = open(lokasi+"/../backup/backup_%s_%s.txt" %(fileName, timeNow), "w")
    file.write("%d\n" %hmn)
    file.write("%d\n" %smn)
    file.write("%d\n" %vmn)
    file.write("%d\n" %hmx)
    file.write("%d\n" %smx)
    file.write("%d\n" %vmx)
    file.close()
    return

def resetValue():
    global hmn, smn, vmn, hmx, smx, vmx, switch
    print 'Reset value'

    hmn = 0
    smn = 0
    vmn = 0
    hmx = 255
    smx = 255
    vmx = 255

    return

readFile()

class MyRequesthandler(BaseHTTPRequestHandler):
    def info(self):
        readFile()
        self.wfile.write('{')
        if switch==0:
            self.wfile.write('"title":"Kalibrasi garis"')
        elif switch==1:
            self.wfile.write('"title":"Kalibrasi lapangan"')
        elif switch==2:
            self.wfile.write('"title":"Kalibrasi bola"')
	elif switch==3:
	    self.wfile.write('"title":"Kalibrasi Gawang"')i

        self.wfile.write(',"hmn": %d,"hmx": %d,"smn": %d,"smx": %d,"vmn": %d,"vmx": %d}' %(hmn, hmx, smn, smx, vmn, vmx))
    def do_GET(self):
        try:
            print ("HALLO", self.path)
            requestPath=re.sub('[^.a-zA-Z0-9]', "",str(self.path))
            if self.path == 'info' or self.path == '/info':
                self.send_response(200)  # OK
                self.send_header('Content-type', 'text/html')
                self.end_headers()
                self.info()
                return
            elif self.path.endswith(".mjpeg"):
                self.send_response(200)
                self.wfile.write("Content-Type: multipart/x-mixed-replace; boundary=--aaboundary")
                self.wfile.write("\r\n\r\n")
                while True:
                    # raw, img2 = cv2.imread("capture-9.jpg") 
                    raw, img2 = capture.read()
                    if not raw:
                        continue
                    img = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)
                    closing = cv2.inRange(img,np.array([hmn,smn,vmn]),np.array([hmx,smx,vmx]))
                    #bluring = cv2.GaussianBlur(closing,(7,7),0)
                    cv2mat = cv2.imencode(".jpeg",closing,(1,cameraQuality))[1]
                    JpegData=cv2mat.tostring()
                    self.wfile.write("--aaboundary\r\n")
                    self.wfile.write("Content-Type: image/jpeg\r\n")
                    self.wfile.write("Content-length: "+str(len(JpegData))+"\r\n\r\n" )
                    self.wfile.write(JpegData)
                    time.sleep(0.03)
                return
            elif self.path.endswith(".config"):
                self.send_response(200)  # OK
                self.wfile.write("{}")
                return
            else:
                if self.path=="" or self.path==None or self.path[:1]=="." or self.path=="/":
                    f = open(lokasi + sep + 'index.html')
                else:
                    f = open(lokasi + sep + self.path)
                self.send_response(200)
                self.send_header('Content-type', 'text/html')
                self.end_headers()
                self.wfile.write(f.read())
                f.close()
                return
        except IOError:
            self.send_error(404, 'File not found: %s' % self.path)
        return
    def do_POST(self):
        global hmn, smn, vmn, hmx, smx, vmx, switch
        try:
            ctype, pdict = cgi.parse_header(self.headers.getheader('content-type'))
            os.system('clear')
            if ctype == 'multipart/form-data':
                postvars = cgi.parse_multipart(self.rfile, pdict)
            elif ctype == 'application/x-www-form-urlencoded':
                length = int(self.headers.getheader('content-length'))
                postvars = cgi.parse_qs(self.rfile.read(length), keep_blank_values=1)
            else:
                postvars = {}

            self.send_response(200)
            self.send_header('Content-type', 'text/html')
            self.end_headers()
 
            for key in postvars.keys():
                if key=='hmn':
                    hmn = int(postvars[key][0])
                elif key=='smn':
                    smn = int(postvars[key][0])
                elif key=='vmn':
                    vmn = int(postvars[key][0])
                elif key=='hmx':
                    hmx = int(postvars[key][0])
                elif key=='smx':
                    smx = int(postvars[key][0])
                elif key=='vmx':
                    vmx = int(postvars[key][0])
                elif key=='switch':
                    self.wfile.write('{"switch":%d}' % switch)
                    switch=switch+1
                    if switch>3:
                        switch=0
                elif key=='save':
                    self.wfile.write('{"save":1}')
                    writeFile()
		elif key=='exit':
		    self.wfile.write('{"exit":1}')
  		    os.system("ps aux | grep 'server.py' | awk {'print $2'} | xargs kill -9")
                elif key=='backup':
                    self.wfile.write('{"backup":1}')
                    backupFile()
                elif key=='reset' :
					resetValue()
					self.wfile.write('{')
					if switch==0:
						self.wfile.write('"title":"Kalibrasi garis"')
					elif switch==1:
						self.wfile.write('"title":"Kalibrasi lapangan"')
  		                  	elif switch==2:
                        			self.wfile.write('"title":"Kalibrasi bola"')
					elif switch==3:
						self.wfile.write('"title":"Kalibrasi gawang"')

					self.wfile.write(',"hmn": %d,"hmx": %d,"smn": %d,"smx": %d,"vmn": %d,"vmx": %d}' %(hmn, hmx, smn, smx, vmn, vmx))

    	    print ("%d || %d || %d\n" %(hmn,smn,vmn))
    	    print ("%d || %d || %d\n" %(hmx,smx,vmx))
            if switch==0:
                print 'Kalibrasi Garis'
            elif switch==1:
                print 'Kalibrasi Lapangan'
            elif switch==2:
                print 'Kalibrasi Bola'
	    elif switch==3:
		print 'Kalibrasi Gawang'
        except KeyError, e:
            print e, " not found"

        return 0

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """Handle requests in a separate thread."""

def main():
	while True:
    		try:
		        Handler = MyRequesthandler
			print "Kalibrasi Lapangan"
		        print "Starting server, at 0.0.0.0:8000"
		        server = ThreadedHTTPServer(('0.0.0.0', 8000), Handler)
		        server.serve_forever()
		except SocketServer.socket.error as exc:
			print "Port sudah dibuka. Jalankan ulang program"
			os.system("ps aux | grep 'server.py' | awk {'print $2'} | xargs kill -9")
		except KeyboardInterrupt:
		        capture.release()
		        server.socket.close()
		        print 'Closing server'
			os.system("ps aux | grep 'server.py' | awk {'print $2'} | xargs kill -9")

if __name__ == '__main__':
    main()
