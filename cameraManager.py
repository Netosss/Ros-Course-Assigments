import cv2
import numpy as np
from tracker import *
import math
import time

time_each = 5
min_time=60
class Camera():
	def __init__(self):  
		################################### COUNT FUNCTION #######################################
		self.curr_count = 0
		#tracker object
		self.tracker = EuclideanDistTracker()

		self.cap = cv2.VideoCapture(0)  #configure!!!
		self.object_detector = cv2.createBackgroundSubtractorMOG2(history=100, varThreshold=30)# need to configure varTreshold in realtime
		self.ret, frame = self.cap.read()
		self.height, width, _ = frame.shape
		self.curr_frame_ids = []
		self.prev_frame_ids = []
		self.counted_ids = []
		self.iteration_num = 0
		self.offset_y = 70
		self.offset_x = 100
		self.y_start = self.height//2 - self.offset_y
		self.y_end = self.height//2 + self.offset_y
		self.x_start = width//2 - self.offset_x - 95
		self.x_end = width//2 + self.offset_x 
		self.id_first_coordinate = {}# key:id, val: coordinate y
		self.id_curr_coordinate = {}
  
	def Counting(self,amount):
		start_time= time.time()
		self.curr_count=0
		drop_frames=10
		estimate_time= max(time_each*amount,min_time)
		while self.curr_count<amount:
			cur_time=time.time()
			ret, frame = self.cap.read()
			# height, width, _ = frame.shape
			while(drop_frames>0):
				ret, frame = self.cap.read()
				drop_frames=drop_frames-1
			cropped = frame[self.y_start:self.y_end, self.x_start:self.x_end] #frame[start_h:end_h , start_w:end_w]

			#detect
			mask = self.object_detector.apply(cropped)
			_, mask = cv2.threshold(mask, 150, 255, cv2.THRESH_BINARY)
			contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
			detections = []
			for cnt in contours:
				#calc Area
				area = cv2.contourArea(cnt)
				if area > 1000:	#200
					# cv2.drawContours(cropped, [cnt], -1, (0,255,0), 2)
					x, y, w, h = cv2.boundingRect(cnt)
					# cv2.rectangle(cropped, (x,y), (x+w, y+h), (0,255,0), 3)
					# cv2.putText(cropped, str(area), (x,y), 1, 1, (0,0,255))
					detections.append([x,y,w,h])

			#track the object
			
			boxes_ids = self.tracker.update(detections) # boxes_ids is list the each object in it is list with coordinates and id : single_obj=[x, y, w, h, id]
			for box in boxes_ids:
				x, y, w, h, box_id = box
				if box_id not in self.id_first_coordinate.keys():
					self.id_first_coordinate[box_id] = y
				self.id_curr_coordinate[box_id] = y
				self.curr_frame_ids.append(box_id)
				cv2.putText(cropped, str(box_id), (x, y-10), cv2.FONT_HERSHEY_PLAIN, 1, (255,0,0), 2) #write the id 10 pixels above the box in blue ,font size 1
				cv2.rectangle(cropped, (x,y), (x+w, y+h), (0,255,0), 3)
			if self.iteration_num > 2 and len(self.prev_frame_ids) > 0:
				prev_id_set = set(self.prev_frame_ids)
				curr_id_set = set(self.curr_frame_ids)
				ids_were_out_this_frame =  prev_id_set.difference(curr_id_set) #ids were in the previous frame but not in the current frame
																										#the set of prev IDs substract curr IDs   
				#the below for loop is for not counting the same screw ID twice
				for id_out in ids_were_out_this_frame:
					
					if id_out not in self.counted_ids:
						self.counted_ids.append(id_out)
						print("Screw {} was out and made a {} pixeles road.\n".format(id_out, abs(self.id_curr_coordinate[box_id]- self.id_first_coordinate[box_id])))
						# print(abs(y_start-y_end)) == 140
						if abs(self.id_curr_coordinate[box_id]- self.id_first_coordinate[box_id]) >= 56:#abs(y_start-y_end)//3:
							self.curr_count += 1


			self.prev_frame_ids = self.curr_frame_ids
			self.curr_frame_ids=[]

			self.iteration_num+=1
			cv2.putText(cropped, "Count : {}".format(str(self.curr_count)), (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
			# cv2.imshow("Frame", frame)
			cv2.imshow("Frame", cropped)
			cv2.waitKey(2)
			if cur_time-start_time > estimate_time:
				print('counting time for screw type amount of {} faild, continue to next counting'.format(amount))
				break

		# self.cap.release()
if __name__ == '__main__':
	camera=Camera()
	camera.Counting(1000)
################################ END COUNT FUNCTION #######################################