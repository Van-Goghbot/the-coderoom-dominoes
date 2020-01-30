class Node:
	"""a representation of a point within a bezier curve"""
	def __init__(self, x, y):
		self.x = x
		self.y = y

class Bezier:
	"""a representation of a bezier curve, consisting of a series of nodes"""
	def __init__(self, path_command):
		self.path_command = path_command
		self.p = [] # This stores all the nodes
		self.conversion()

	def conversion(self):
		"""Takes the string of coordinates from the SVG
		and splits them into the respective coordinates of the curve"""


		# Removing command letters from the coordinate string
		coordinate_string = ""
		for char in self.path_command:
			if char.isalpha():
				coordinate_string += ","
			elif char == "-":
				coordinate_string += ",-"
			else:
				coordinate_string += char

		if coordinate_string[0] == ",":
			coordinate_string = coordinate_string[1:]
		
		coordinate_string = coordinate_string.split(",")

		coordinate_string = [float(num) for num in coordinate_string]
		print(coordinate_string)

		# Pairs the coordinates (x,y)
		i = 1
		coordinates = []
		for num in coordinate_string:
			if i % 2 == 0:
				coord.append(num)
				coordinates.append(coord)
			else: # This case happens first
				coord = [num]
			i += 1

		for coord in coordinates:
			self.p.append(Node(coord[0], coord[1]))

		

	def B_x(self, t):
		c0 = (1 - t)**3*self.p[0].x
		c1 = 3*(1 - t)**2*t*self.p[1].x
		c2 = 3*(1 - t)*t**2*self.p[2].x
		c3 = t**3*self.p[3].x
		return c0 + c1 + c2 + c3

	def B_y(self, t):
		c0 = (1 - t)**3*self.p[0].y
		c1 = 3*(1 - t)**2*t*self.p[1].y
		c2 = 3*(1 - t)*t**2*self.p[2].y
		c3 = t**3*self.p[3].y
		return c0 + c1 + c2 + c3

test = Bezier("M79.8,80.3C315.2,331.6,345.2-122.6,81,157")

for i in range(11):
	t = i/10
	print(test.B_x(t), test.B_y(t))