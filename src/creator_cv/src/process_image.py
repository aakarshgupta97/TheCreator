import numpy as np
import scipy.misc
# ROS Image message
import matplotlib.pyplot as plt
from sklearn.neighbors.kde import KernelDensity
import PIL
# OpenCV2 for saving an image
import cv2

def detect_corners(im):
    
	gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

	gray = np.float32(gray)
	dst = cv2.cornerHarris(gray,2,3,0.04)

	corners = cv2.goodFeaturesToTrack(gray,4,0.01,40)
	corners = np.int0(corners).reshape(4, 2)

	cv2.imwrite('with_corners.png',im)

	get_x = lambda p: p[0]
	get_y = lambda p: p[1]

	p1, p2, p3, p4 = corners

	sort_y = sorted(corners, key=get_y)

	min_y1, min_y2, max_y2, max_y1 = sort_y

	bl = min([max_y1, max_y2], key=get_x)
	br = max([max_y1, max_y2], key=get_x)

	tl = min([min_y1, min_y2], key=get_x)
	tr = max([min_y1, min_y2], key=get_x)

	return np.array([tl, bl, br, tr])


def homography(pts_src, im_src):
	pts_dst = np.array([[0, 0], [0, 200], [240, 200], [240, 0]])

	h, status = cv2.findHomography(pts_src, pts_dst)

	return h


def edge_detect(image, low, high):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    im = PIL.Image.fromarray(gray)
    succ = im.save("1.gray" + ".png")

    kernel_size = 7
    blur_gray = cv2.GaussianBlur(gray, (kernel_size, kernel_size), 0)

    im = PIL.Image.fromarray(blur_gray)
    succ = im.save("1.blur" + ".png")

    edges = cv2.Canny(gray, low, high)

    edges = cv2.GaussianBlur(edges, (9, 9), 0.01)
    im = PIL.Image.fromarray(edges)
    succ = im.save("1.edges" + ".png")
    return edges


def mark_corners(img, corners):
	for p in corners:
		u, v = p
		img[u, v] = [0, 0, 255]


def main():

 	# pts_src = np.array([[256, 118], [256, 293], [473, 295], [470, 117]])
 	pts_src = np.array([[173, 97], [178, 314], [438, 314], [439, 93]])

 	# pts_src = np.array([x[::-1] for x in pts_src])

 	im_src = cv2.imread('image_dump/1.png')

 	depth_src = np.zeros((480, 640))
 	for i in range(5):
 		x = i + 1
 		depth = np.load('depth_arrs/depth' + str(x) + '.npy')
 		depth_src = np.maximum(depth_src, depth)

	im = np.zeros_like(im_src)
	
	im[:,:,:] = im_src[:,:,:]

	zeros = np.ones_like(im_src)
	cv2.fillPoly(zeros, np.array([pts_src]), 0)
	zeros = zeros.astype(bool)
	im[zeros == True] = 0
	im[im > 0] = 255

	im = cv2.GaussianBlur(im, (9, 9), 0.01)

 	corners = detect_corners(im)

 	print(corners)

 	# mark_corners(im, corners)

 	cv2.imwrite('blacked.png', im)

 	h = homography(corners, im_src)

 	im = np.zeros_like(im_src)
	
	im[:,:,:] = im_src[:,:,:]

	im_dst = cv2.warpPerspective(im_src, h, (240, 200))

	# zeros = np.ones_like(im_src)
	# cv2.fillPoly(zeros, np.array([pts_src]), 0)
	# zeros = zeros.astype(bool)
	# im[zeros == True] = 0
	# cv2.imwrite('blacked.png', im)



	depth_warped = cv2.warpPerspective(depth_src, h, (240, 200)) / 10
	dst = np.zeros_like(depth_warped)
	depth_warped = cv2.GaussianBlur(depth_warped, (11, 11), 0.1)

	binwidth = 10
	data = depth_warped.reshape(-1)

	bp_mask = depth_warped > 73

	data = data[(data > 40) & (data < 73)]
	#plt.hist(data, bins=np.arange(min(data), max(data) + binwidth, binwidth))
	
	data = data.reshape(-1, 1)
	
	
	kde = KernelDensity(kernel='gaussian', bandwidth=0.65).fit(data)
	
	#LEAST VIABLE BW = 5.5	

	s = np.linspace(40, 73, 120)
	e = kde.score_samples(s.reshape(-1,1))
	
	from scipy.signal import find_peaks

	peaks, properties = find_peaks(e, prominence=(0.6, None))
	print(properties["prominences"])
	plt.plot(s, np.exp(e))
	plt.plot(peaks, e[peaks], "x")
	plt.show()

	#mi, ma = argrelextrema(e, np.less)[0], argrelextrema(e, np.greater)[0]
	#s[mi] 
	#print("Mins:", s[mi])
	#print("Maxs:", s[ma])
	
	#plt.scatter(s[mi], np.zeros_like(s[mi]), color='red')	
	#plt.scatter(s[ma], np.zeros_like(s[ma]), color='green')
	#plt.plot(s, np.exp(e))

 	xs = np.linspace(0, 200, num=11).astype(np.int32)
 	ys = np.linspace(0, 240, num=13).astype(np.int32)

 	for i in xs[1:-1]:
 		im_dst[i, :] = [255, 0, 255]
 	for j in ys[1:-1]:
 		im_dst[:, j] = [255, 0, 255]

 	grid = np.zeros((10, 12))

 	for i in range(len(xs) - 1):
 		for j in range(len(ys) - 1):
 			u, v = xs[i], xs[i + 1]
 			p, q = ys[j], ys[j + 1]

 			vals = depth_warped[u:v, p:q]
 			vals = vals.reshape(-1)
 			value = np.median(vals[vals > 60])
 			grid[i, j] = value

 		#	if (i, j) == (5, 7) or (i, j) == (5, 6) or (i, j) == (4, 6):
 		#		plt.hist(vals[vals > 600])
    		#	plt.show()


      # if i == 5:
      #   print("nooooooooooooooo")
      #   print(vals)
      #   plt.hist(vals)
      #   plt.show()
	
	im_dst[bp_mask] = [0, 0, 0]
	cv2.imwrite('marked.png', im_dst)

	#depth_warped[depth_warped > 250] = 255
	#cv2.imwrite('depth_transformed.png', depth_warped)

	#plt.imshow(grid)
	#plt.colorbar()

if __name__ == '__main__':
    main()
    plt.show()

