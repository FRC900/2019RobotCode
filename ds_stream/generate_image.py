import cv2
import numpy as np

def main():
	imgname = "champs_crosshairs_overlay.png"

	img_height = 980
	img_width = 737
	n_channels = 4
	res = np.zeros((img_height, img_width, n_channels), dtype=np.uint8)

	drawCross(res, 200, 200)

	cv2.imwrite('/home/ubuntu/' + imgname, res)

def drawCross(img, x, y):
     cv2.line(img,(398,362),(328,496),(0,0,255,255),8)
     cv2.line(img,(636,852),(434,888),(0,0,255,255),8)
main()
