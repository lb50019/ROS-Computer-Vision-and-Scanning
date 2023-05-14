#!/usr/bin/env python

import cv2
import numpy as np
import math

img = cv2.imread('/home/luka/catkin_ws/src/boxor/src/mapa.png',1)
global_x = 0
global_y = 0
tocke = []
udalj = []
kutovi = []
def mousecb(event,x,y,flags,param):

	if flags == 1:
		global_x = x
		global_y = y
		cv2.circle(img, (global_x,global_y), 5, (0, 0, 255), -1);
		
	b,g,r = cv2.split(img)
	
	for i in range(0, b.shape[0], 30):
		for j in range(0, b.shape[1], 30):
			if (not(b[i,j]) and not(g[i,j]) and r[i,j]):
				a = (j,i)
				tocke.append(a)
				
	for i in range(len(tocke)):
		cv2.circle(img, tocke[i], 5, (0, 255, 0), -1)
	
	for i in range(1, len(tocke)):
		cv2.line(img, tocke[i-1], tocke[i], (0,255,0), 2);

lower_lim = np.array([0, 255, 255])
upper_lim = np.array([2, 255, 255])
cv2.namedWindow('U nos')
cv2.setMouseCallback('U nos', mousecb)

while(1):
	cv2.imshow('U nos',img)
	k = cv2.waitKey(20)
	if k == ord('q'):
		break
	elif k == ord('d'):
		print(tocke)
		#dtocke = list(dict.fromkeys(tocke))
		dtocke = []
		[dtocke.append(x) for x in tocke if x not in dtocke]
		print(dtocke)

		for i in range(len(dtocke)-1):
			razx = dtocke[i+1][0]-dtocke[i][0]
			razy = dtocke[i+1][1]-dtocke[i][1]
	
			if razx:
				kut = math.atan2(-razy,razx)
			else:
				kut = (-1)*np.sign(razy)*1.57
		
			r = math.sqrt(razx**2+razy**2)
	
			udalj.append(r)
			kutovi.append(kut)
				
		print(udalj)
		print(kutovi)		

cv2.destroyAllWindows()
