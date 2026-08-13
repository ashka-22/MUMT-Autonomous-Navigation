import numpy as np
import matplotlib.pyplot as plt
import cv2

#image importing, conversion to grayscale and blurring
img = cv2.imread(r"C:\Users\ASHKA\Downloads\drone_img.jpg")
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
gray = cv2.GaussianBlur(gray, (9,9), 0)

#reducing noise
gray = cv2.GaussianBlur(gray, (9, 9), 0)

#Otsu thresholding
_, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

#noise removal using morphology
kernel = np.ones((7,7), np.uint8)

#fill small gaps
binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
#remove isolated noise
binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel)

occupancy = (255 - binary) // 255

#resizing
occupancy_grid = cv2.resize(occupancy, (150,150), interpolation=cv2.INTER_NEAREST)

np.save("occupancy_grid_real.npy", occupancy)

plt.subplot(121)
plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
plt.title("Original Drone Image")
plt.axis("off")

plt.subplot(122)
plt.imshow(occupancy, cmap="gray_r", origin="lower")
plt.title("Occupancy Grid")
plt.axis("off")

plt.tight_layout()
plt.show()