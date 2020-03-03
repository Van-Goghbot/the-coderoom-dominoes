import math, numpy

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
		i = 0
		for char in self.path_command:
			if char.isalpha():
				coordinate_string += ","
			elif char == "-" and coordinate_string[i - 1] != ',':
				coordinate_string += ",-"
			else:
				coordinate_string += char
			i += 1

		if coordinate_string[0] == ",":
			coordinate_string = coordinate_string[1:]
		
		coordinate_string = coordinate_string.split(",")

		coordinate_string = [float(num) for num in coordinate_string]
		# print(coordinate_string)

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

		old_coord = (0, 0)
		for coord in coordinates:
			# print(coord, old_coord)
			# print((coord[0] + old_coord[0], coord[1] + old_coord[1]))
			self.p.append(Node(coord[0], coord[1]))
			old_coord = coord

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

	def length_approximation(self, resolution):
			self.length = 0
			old_length = 0
			self.t_map = {}
			old_x = None
			old_y = None
			for i in range(int(resolution)+1):
				t = i/resolution
				x = self.B_x(t)
				y = self.B_y(t)
				if old_y:
					dy = y - old_y
					dx = x - old_x
					old_length = self.length
					self.length += (dy**2 + dx**2) ** 0.5
					map_range = numpy.arange(old_length, self.length, 0.001)
					for index in map_range:
						self.t_map[round(index,3)] = t
				old_y = y
				old_x = x
				
			return self.length