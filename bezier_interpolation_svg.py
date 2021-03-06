import math
import bezier_conversion

rad_conv = 1

# change me
angle_1 = -1.5
x1, y1 = (60, 70)

# change me
angle_2 = -2.33
x2, y2 = (-60, 70)

svg = """<svg version="1.1" baseProfile="basic" id="Layer_1"
	 xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" x="0px" y="0px" width="600px" height="250px"
	 viewBox="0 0 120 50" xml:space="preserve">
  {0}
</svg>"""

brick_1 = f'<rect width="6" height="9" style="fill:rgb(255,106,5);stroke-width:1;stroke:rgb(0,0,0)" x="{x1}" y="{y1}" transform="rotate({angle_1} {x1} {y1}) translate(-3 -4.5)" />'
brick_2 = f'<rect width="6" height="9" style="fill:rgb(255,106,5);stroke-width:1;stroke:rgb(0,0,0)" x="{x2}" y="{y2}" transform="rotate({angle_2} {x2} {y2}) translate(-3 -4.5)" />'

svg = svg.format(brick_1 + '{0}')
svg = svg.format(brick_2 + '{0}')

# change me
length = 100
handle_1_dx = length * math.cos(angle_1 * rad_conv)
handle_1_dy = length * math.sin(angle_1 * rad_conv)

handle_2_dx = length * math.cos(angle_2 * rad_conv)
handle_2_dy = length * math.sin(angle_2 * rad_conv)

P0_x = x1
P0_y = y1

P1_x = x1 + (handle_1_dx/2)
P1_y = y1 + (handle_1_dy/2)

P2_x = x2 - (handle_2_dx/2)
P2_y = y2 - (handle_2_dy/2)

P3_x = x2
P3_y = y2

line_1 = f'<line x1="{x1}" y1="{y1}" x2="{x1 + (handle_1_dx/2)}" y2="{y1 + (handle_1_dy/2)}" stroke="black" />'
line_2 = f'<line x1="{x2 - (handle_2_dx/2)}" y1="{y2 - (handle_2_dy/2)}" x2="{x2}" y2="{y2}" stroke="black" />'

path = f'<path fill="none" stroke="#000000" stroke-width="1.2587" stroke-miterlimit="10" d="M{P0_x},{P0_y}C{P1_x},{P1_y},{P2_x},{P2_y},{P3_x},{P3_y}"/>'

svg = svg.format(line_1 + '{0}')
svg = svg.format(line_2 + '{0}')
svg = svg.format(path+ '{0}')

brick_path = bezier_conversion.Bezier(f"M{P0_x},{P0_y}C{P1_x},{P1_y},{P2_x},{P2_y},{P3_x},{P3_y}")
brick_path.length_approximation(150)

dt = 0.01

spacing = 14
bricks = math.floor(brick_path.length/spacing)

spacing = brick_path.length/bricks

print(spacing)

for i in range(bricks+1):
	try:
		t = brick_path.t_map[round(i*spacing,3)]
	except KeyError:
		t = 1
	x = brick_path.B_x(t)
	y = brick_path.B_y(t)
	conversion = 180 / math.pi
	dy = brick_path.B_y(t + dt) - brick_path.B_y(t - dt)
	dx = brick_path.B_x(t + dt) - brick_path.B_x(t - dt)
	angle = math.atan2(dy,dx) * conversion
	svg = svg.format(f'<rect width="6" height="9" style="fill:rgb(255,106,5);stroke-width:1;stroke:rgb(0,0,0)" x="{x}" y="{y}" transform="rotate({angle} {x} {y}) translate(-3 -4.5)" />' + '{0}')
	svg = svg.format(f'<line x1="{brick_path.B_x(t + dt)}" y1="{brick_path.B_y(t + dt)}" x2="{brick_path.B_x(t - dt)}" y2="{brick_path.B_y(t - dt)}" stroke="black"/>' + '{0}')


with open('interpolation.svg', 'w') as svgfile:
	svgfile.write(svg)
