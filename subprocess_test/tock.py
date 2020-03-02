import sys

left = [tuple([float(x) for x in y.split(',')]) for y in str(sys.argv[1])[2:-2].split('), (')]
print(left)
for coord in left:
	print coord