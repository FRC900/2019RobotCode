import cv2
import numpy as np

def main():
	imgname = "image.png"

	img_height = 605
	img_width = 1612
	n_channels = 4
	res = np.zeros((img_height, img_width, n_channels), dtype=np.uint8)

	drawCross(res, 200, 200)

	cv2.imwrite('/home/ubuntu/' + imgname, res)

def drawCross(img, x, y):
     cv2.line(img,(103,94),(1380,382),(0,0,255,255),8)

main()
