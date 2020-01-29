class Node:
	"""a representation of a point within a bezier curve"""
	def __init__(self, x, y):
		self.x = x
		self.y = y

class Bezier:
	"""a representation of a bezier curve, consisting of a series of nodes"""
	def __init__(self, path_command):
		self.path_command = path_command
		self.nodes = []
		self.conversion()

	def conversion(self):
		coordinate_string = ""
		for char in self.path_command:
			if char.isalpha():
				coordinate_string += ","
			else:
				coordinate_string += char

		if coordinate_string[0] == ",":
			coordinate_string = coordinate_string[1:]
		
		coordinate_string = coordinate_string.split(",")

		print(coordinate_string)

		i = 1
		coordinates = []
		for num in coordinate_string:
			if i % 2 == 0:
				coord.append(num)
				coordinates.append(coord)
			else: # This case happens first
				coord = [num]
			i += 1

		print(coordinates)

		for coord in coordinates:
			print(f'x: {coord[0]} y: {coord[1]}')
			self.nodes.append(Node(coord[0], coord[1]))

test = Bezier("M184.7,207.5C361,31.2,793.9,322,184.7,736.5")