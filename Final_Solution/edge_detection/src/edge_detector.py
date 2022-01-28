import cv2
import numpy as np

def get_image():
        link = input("Enter the link to the image")
        img = cv2.imread(link)
        return img


def get_edges(img):
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_blur = cv2.GaussianBlur(img_gray, (5, 5), 0)
        edges = cv2.Canny(image=img_blur, threshold1=150, threshold2=220)
        return edges

def get_output(edges, img):
        a, b = np.where(edges == 255)
        img[a, b, 1] = 255
        img[a, b, 0] = 0
        img[a, b, 2] = 0
        link = input("Enter the location for the image")
        file_name = input("Please enter the result file name")
        backslash = "\\"
        location = link + backslash + file_name
        cv2.imwrite(str(location), img)

img = get_image()
edges = get_edges(img)
get_output(edges, img)