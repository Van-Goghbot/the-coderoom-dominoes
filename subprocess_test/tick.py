import subprocess
left = [(9, 5, 4), (65, 3, 4), (5, 3, 2), (9, 8, 2)]
rc = subprocess.Popen("python2 tock.py '" + str(left) + "'", shell=True)