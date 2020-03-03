import math
import bezier_conversion

class Coordinate:
	def __init__(self, x, y, rot):
		self.x = x
		self.y = y
		self.rot = rot

	def __repr__(self):
		return(str(self.x) + " " + str(self.y) + " " + str(self.rot))

def create_path(start_x, start_y, start_rot, end_x, end_y, end_rot, handle_influence):
	angle_1 = start_rot
	x1, y1 = (start_x, start_y)

	angle_2 = end_rot
	x2, y2 = (end_x, end_y)

	length = handle_influence
	handle_1_dx = length * math.cos(angle_1)
	handle_1_dy = length * math.sin(angle_1)

	handle_2_dx = length * math.cos(angle_2)
	handle_2_dy = length * math.sin(angle_2)

	P0_x = x1
	P0_y = y1

	P1_x = x1 + (handle_1_dx/2)
	P1_y = y1 + (handle_1_dy/2)

	P2_x = x2 - (handle_2_dx/2)
	P2_y = y2 - (handle_2_dy/2)

	P3_x = x2
	P3_y = y2

	bezier_string = "M" + str(P0_x) + "," + str(P0_y) + "C" + str(P1_x) + "," + str(P1_y) + "," + str(P2_x) + "," + str(P2_y) + "," + str(P3_x) + "," + str(P3_y)

	brick_path = bezier_conversion.Bezier(bezier_string)
	brick_path.length_approximation(150.0)

	dt = 0.01

	spacing = 14
	bricks = int(math.floor(brick_path.length/spacing))

	spacing = brick_path.length/bricks

	#print(spacing)

	brick_poses = []
	#print(brick_path.t_map)

	for i in range(bricks+1):
		try:
			t = brick_path.t_map[round(i*spacing,3)]
		except KeyError:
			t = 1
		
		x = brick_path.B_x(t)
		y = brick_path.B_y(t)
		
		x = (x - 60) / 100
		y = (- y + 120) / 100 - 0.1
		
		dy = brick_path.B_y(t + dt) - brick_path.B_y(t - dt)
		dx = brick_path.B_x(t + dt) - brick_path.B_x(t - dt)
		
		angle = math.atan2(dy,dx)

		#print(x, y)
		
		new_pose = Coordinate(x, y, angle)
		brick_poses.append(new_pose)
		# print(brick_poses, brick_poses[-1].x)

	return brick_poses


