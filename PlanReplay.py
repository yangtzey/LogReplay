#!/usr/bin/env python
import getopt
import sys
import re
import math
import os
import numpy as np
import matplotlib.pyplot as plt
import copy
import math


bPress = False

def on_key_press(event):
	if event.key ==str("n"):
		global bPress
		bPress = True

def getPress():
	global bPress
	return bPress

def setPressFalse():
	global bPress
	bPress = False

def DrawSpline(splnex,splney):
	rXList = []
	rYList = []
	for i in range(len(splnex) - 3):
		for u in np.arange(0, 1, 0.01):
			N03=(-u**3 + 3*u**2 - 3*u + 1) / 6;
			N13=(3*u**3 - 6*u**2 + 4) / 6;
			N23=(-3*u**3 + 3*u**2 + 3*u + 1)/6;
			N33=u**3/6;
			rx=N03 * splnex[i] + N13 * splnex[i+1]  + N23 * splnex[i+2]  + N33 * splnex[i+3] 
			ry=N03 * splney[i] + N13 * splney[i+1]  + N23 * splney[i+2]  + N33 * splney[i+3] 
			rXList.append(rx)
			rYList.append(ry)
	return  rXList, rYList

class TracPos:
	def __init__(self):
		self.GobalX = 0
		self.GobalY = 0
		self.GobalHeading = 0
		self.Curvature = 0
		self.Distane = 0
		self.Speedx = 0
		self.Speedy = 0
		self.Accx = 0
		self.Accy = 0

	def setData( self,x,y,heading, curvature,dDistance, speedx,speedy,accx,accy):
		self.GobalX = x
		self.GobalY = y
		self.GobalHeading = heading
		self.Curvature = curvature
		self.Distane = dDistance
		self.Speedx = speedx
		self.Speedy = speedy
		self.Accx = accx
		self.Accy = accy

	def getX(self):
		return self.GobalX

	def getY(self):
		return self.GobalY

	def wirteMatlabData(self,file):
		file.write(str(self.GobalX))
		file.write("  ")
		file.write(str(self.GobalY))
		file.write("  ")
		file.write(str(self.GobalHeading))
		file.write("  ")
		file.write(str(self.Curvature))
		file.write("  ")
		file.write(str(self.Distane))
		file.write("  ")
		file.write(str(self.Speedx))
		file.write("  ")
		file.write(str(self.Speedy))
		file.write("  ")
		file.write(str(self.Accx))
		file.write("  ")
		file.write(str(self.Accy))
		file.write("\r\n")

	def getPrintData(self):
		# return "x:"+str(self.GobalX)+" y:"+str(self.GobalY)+" H:"+str(self.GobalHeading)+" C:"+str(self.Curvature)+"\nSx:"+str(self.Speedx)+" Sy:"+str(self.Speedy)+" Ax:"+str(self.Accx)+" Ay:"+str(self.Accy)
		return "H:"+str(self.GobalHeading)+" C:"+str(self.Curvature)+" Sx:"+str(self.Speedx)+" Sy:"+str(self.Speedy)					

class SplinePos:
	def __init__(self):
		self.x = 0
		self.y = 0

	def set( self,x,y):
		self.x = x
		self.y = y

	def getX(self):
		return self.x

	def getY(self):
		return self.y

	def printPos(self):
		print("x:"+str(self.x)+"y:"+str(self.y))

class VPPos:
	def __init__(self):
		self.x = 0
		self.y = 0
		self.heading = 0

	def set( self,x,y):
		self.x = x
		self.y = y

	def setAll( self,x,y,heading):
		self.x = x
		self.y = y
		self.heading = heading	

	def getX(self):
		return self.x

	def getY(self):
		return self.y

	def getHeading(self):
		return self.heading	

	def printPos(self):
		return "x:"+str(self.x)+"  y:"+str(self.y)+"  heading:"+str(self.heading)

class FrenetPos:
	def __init__(self):
		self.dDistance = 0
		self.dSpeed = 0
		self.dAcceleration = 0
		self.sDistance = 0
		self.sSpeed = 0
		self.sAcceleration = 0
		self.sStopDistance = 0
		self.Width = 0
		self.TargetSpeed = 0

	def set( self,dd,ds,da,sd,ss,sa,stops,widthd,tspeed):
		self.dDistance = dd
		self.dSpeed = ds
		self.dAcceleration = da
		self.sDistance = sd
		self.sSpeed = ss
		self.sAcceleration = sa
		self.sStopDistance = stops
		self.Width = widthd
		self.TargetSpeed = tspeed
	
	def getPrint(self):
		return "Dd:"+str(self.dDistance)+" Ds:"+str(self.dSpeed)+" Da:"+str(self.dAcceleration)+" Sd:"+str(self.sDistance)+" Ss:"+str(self.sSpeed)+" Sa:"+str(self.sAcceleration)+" SStop:"+str(self.sStopDistance)+" DW:"+str(self.Width)+" Ts:"+str(self.TargetSpeed)

class Frame:
	def __init__(self):
		self.VPPos = VPPos()
		self.MatchPos = VPPos()
		self.GobalPos = VPPos()
		self.TList = []
		self.CandidateList = []
		self.FrenetList = []
		self.FrenetPos = FrenetPos()

	def setVP( self, vp):
		self.VPPos = vp

	def getVP(self):
		return self.VPPos

	def setGobalPos( self, vp):
		self.GobalPos = vp

	def getGobalPos(self):
		return self.GobalPos	

	def setMatchPos( self, vp):
		self.MatchPos = vp

	def getMatchPos(self):
		return self.MatchPos

	def setFrenetPos( self, fp):
		self.FrenetPos = fp

	def getFrenetPos(self):
		return self.FrenetPos	

	def getTracPos(self):
		return self.TList

	def appendTracPos(self, tpos):
		self.TList.append(tpos)

	def appendCandidatePos(self, tpos):
		self.CandidateList.append(tpos)	

	def getTXList(self):
		xlist = []
		for tp in self.TList:
			xlist.append(tp.getX())
		return xlist

	def getTYList(self):
		ylist = []
		for tp in self.TList:
			ylist.append(tp.getY())
		return ylist

	def getTYListPrint(self):
		strdata = ""
		for i in range(len(self.TList)):
			strdata = strdata + self.TList[i].getPrintData()+"\n"
		return strdata	

	def getCXList(self):
		xlist = []
		for tp in self.CandidateList:
			xlist.append(tp.getX())
		return xlist

	def getCYList(self):
		ylist = []
		for tp in self.CandidateList:
			ylist.append(tp.getY())
		return ylist

	def printFrame(self):
		self.VPPos.printPos()

show_animation = True
show_trac = True
pltId = 1
regexTracPos = re.compile('\[Planning\]x:(?P<x>[0-9\-\.]+) y:(?P<y>[0-9\-\.]+) heading:(?P<heading>[0-9\-\.]+) curvature:(?P<curvature>[0-9\-\.]+) dDistance:(?P<dDistance>[0-9\-\.]+) speedx:(?P<speedx>[0-9\-\.]+) speedy:(?P<speedy>[0-9\-\.]+) accx:(?P<accx>[0-9\-\.]+) accy:(?P<accy>[0-9\-\.]+)')
regexSplineStart = re.compile('===CubicSpline points start')
regexSplineEnd = re.compile('===CubicSpline points end')
regexSplinePos = re.compile('setControlPoints points: x:(?P<x>[0-9\-\.]+) y:(?P<y>[0-9\-\.]+)')
regexVPLocalPos = re.compile('Local point X:(?P<x>[0-9\-\.]+) Y:(?P<y>[0-9\-\.]+)')
regexCandidatePos = re.compile('getBestCandidatePos List data:(?P<x>[0-9\-\.]+) ,(?P<y>[0-9\-\.]+) ,(?P<heading>[0-9\-\.]+)')
regexFrenetStatusPos = re.compile('FrenetStatus D:(?P<D>[0-9\-\.]+) DS:(?P<DS>[0-9\-\.]+) DA:(?P<DA>[0-9\-\.]+) S:(?P<S>[0-9\-\.]+) SS:(?P<SS>[0-9\-\.]+) SA:(?P<SA>[0-9\-\.]+) SStop:(?P<SStop>[0-9\-\.]+) DWidth:(?P<DWidth>[0-9\-\.]+) TSS:(?P<TSS>[0-9\-\.]+)')
regexMatchedPos = re.compile('getMatchPoint point x:(?P<x>[0-9\-\.]+) y:(?P<y>[0-9\-\.]+) heading:(?P<heading>[0-9\-\.]+)')
regexGobalPos = re.compile('Gobal Wgs getLongitude:(?P<x>[0-9\-\.]+) getLatitude:(?P<y>[0-9\-\.]+)')

if __name__ == '__main__':
	opts, args = getopt.getopt(sys.argv[1:], 'hn:w:', ['name=', 'word=', 'help'])
	color = ['ob', 'og','or','oc','om','oy','ok','ow']
	TracPosList = []
	SplineList = []
	VPPosList  = []
	totalFrame = [] ##list list
	FrameList = []

	bFirst = True
	fm = Frame()
	groupIdx = 0
	if len(sys.argv) < 2:
		print("#please input file#")
	else:

		filenameRead = os.getcwd() + "/"+ sys.argv[1]
		file = open(filenameRead, "r")
		filenameTracPos = os.getcwd() + "/" + "TracPos.txt"
		filenameTracPosW = open(filenameTracPos, "w")
		print ("convet file:"+ file.name)
		line = file.readline()
		print ("resolving.................")
		while line:
			matched = re.search(regexTracPos, line)
			SplineStartmatched = re.search(regexSplineStart, line)
			SplineEndmatched = re.search(regexSplineEnd, line)	
			SplinePosmatched = re.search(regexSplinePos, line)
			VPLocalPosmatched = re.search(regexVPLocalPos, line)
			CandidatePosmatched = re.search(regexCandidatePos, line)
			FrenetStatusPosmatched = re.search(regexFrenetStatusPos, line)
			MatchedPosmatched = re.search(regexMatchedPos, line)
			GobalPosmatched = re.search(regexGobalPos, line)
			if matched:
				ps = TracPos()
				x =  matched.group("x")
				y =  matched.group("y")
				heading =  matched.group("heading")
				curvature =  matched.group("curvature")
				dDistance =  matched.group("dDistance")
				speedx =  matched.group("speedx")
				speedy =  matched.group("speedy")
				accx =  matched.group("accx")
				accy =  matched.group("accy")
				ps.setData(x,y,heading, curvature,dDistance, speedx,speedy,accx,accy)
				fm.appendTracPos(ps)
			if SplineStartmatched:
				if bFirst:
					bFirst = False
				else:
					totalFrame.append(FrameList)
					FrameList = []
				CurrentList = []				
			if SplineEndmatched:
				SplineList.append(CurrentList)
			if GobalPosmatched:
				#print("yue changjiang append adddddddd##################")
				FrameList.append(fm)
				fm = Frame()
				ps = VPPos()
				ps.set(float(GobalPosmatched.group("x")),float(GobalPosmatched.group("y")))
				fm.setGobalPos(ps)	
			if SplinePosmatched:
				ps = SplinePos()
				ps.set(float(SplinePosmatched.group("x")),float(SplinePosmatched.group("y")))
				CurrentList.append(ps)
			if VPLocalPosmatched:
				ps = VPPos()
				ps.set(float(VPLocalPosmatched.group("x")),float(VPLocalPosmatched.group("y")))
				fm.setVP(ps)
			if MatchedPosmatched:
				ps = VPPos()
				ps.setAll(float(MatchedPosmatched.group("x")),float(MatchedPosmatched.group("y")),float(MatchedPosmatched.group("heading")))
				fm.setMatchPos(ps)
			if FrenetStatusPosmatched:
				fp = FrenetPos()
				fp.set(float(FrenetStatusPosmatched.group("D")),float(FrenetStatusPosmatched.group("DS")),float(FrenetStatusPosmatched.group("DA")),float(FrenetStatusPosmatched.group("S")),float(FrenetStatusPosmatched.group("SS")),float(FrenetStatusPosmatched.group("SA")),float(FrenetStatusPosmatched.group("SStop")),float(FrenetStatusPosmatched.group("DWidth")),float(FrenetStatusPosmatched.group("TSS")))
				fm.setFrenetPos(fp)
			if CandidatePosmatched:
				ps = VPPos()
				ps.setAll(float(CandidatePosmatched.group("x")),float(CandidatePosmatched.group("y")),float(CandidatePosmatched.group("heading")))
				fm.appendCandidatePos(ps)
			line = file.readline()
		
		for i in TracPosList:
			i.wirteMatlabData(filenameTracPosW)
		totalFrame.append(FrameList)
		file.close()
		filenameTracPosW.close()

		for sl in SplineList:
			splnex = []
			splney = []
			for i in range(len(sl)):
				#if i != 0 and i != len(sl) -1:
				splnex.append(sl[i].getX())
				splney.append(sl[i].getY())
			print("yue changjiang start ##################")		
			print(splnex)
			print(splney)
			print("yue changjiang end ##################")

			#DrawSpline(splnex, splney)
			tx, ty = DrawSpline(splnex, splney)

			if show_animation:
				plt.figure(pltId)
				pltId = pltId + 1
				plt.title("Global Spline Path")
				plt.plot(splnex, splney, 'b-')
				for i in range(len(splnex)):
					plt.plot(splnex[i], splney[i], color[i%8],label=str(i))					
				plt.plot(tx, ty, '-r')
				plt.axis('equal')				
				plt.legend()
				
				#plt.show()
				#plt.savefig('global_path.png')
				
				xarea = 20
				yarea = 20
				if show_animation:
					plt.figure(pltId)
					plt.gcf().canvas.mpl_connect('key_press_event',on_key_press)
					pltId = pltId + 1
					FrameList = totalFrame[groupIdx]
					for i in range(len(FrameList)):
						if i != 0:
							frameData = FrameList[i]
							frameData.printFrame()
							status = frameData.getFrenetPos().getPrint()
							plt.title("Replay")
							plt.xlabel("Status:"+status+"\nMatchPos:"+frameData.getMatchPos().printPos()+"\nGobalPos:"+frameData.getGobalPos().printPos(),fontsize = 10)
							#plt.ylabel(frameData.getTYListPrint(),fontsize = 10,verticalalignment="bottom",horizontalalignment="right",rotation="horizontal")
							if show_trac:
								plt.ylabel(frameData.getTYListPrint(),fontsize = 10,verticalalignment="center",horizontalalignment="right",rotation="horizontal")
							#plt.xlabel("Status:"+status+"\nGobalPos:"+frameData.getGobalPos().printPos(),fontsize = 10)
							plt.plot(tx, ty, '-r')
							
							plt.plot(frameData.getMatchPos().getX(), frameData.getMatchPos().getY(), "bo",label='Match Pos')
							plt.plot(frameData.getVP().getX(), frameData.getVP().getY(), "rD",label='VP Pos')
							plt.plot(frameData.getTXList(), frameData.getTYList(), "gx",label='Trace Pos')
							plt.plot(frameData.getCXList(), frameData.getCYList(), "y+")
							plt.xlim(frameData.getVP().getX() - xarea, frameData.getVP().getX() + xarea)
							plt.ylim(frameData.getVP().getY() - yarea, frameData.getVP().getY() + yarea)
							plt.gca().set_aspect('equal', adjustable='box')
							plt.legend()
							plt.grid(True)
							if getPress():
								setPressFalse()
								#print("###########Change Next Set###########")
								break;
							else:
								plt.pause(0.0001)	
								plt.cla()		
					print("###########Change Next Set###########")
					groupIdx = groupIdx + 1		
					plt.show()

		print("###########finsh###########")


