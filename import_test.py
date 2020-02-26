import bezier_interpolation

brick_poses = bezier_interpolation.create_path(10, 35, 20, 110, 35, 20)

print(brick_poses)
for brick in brick_poses:
	print(brick.x, brick.y, brick.rot)