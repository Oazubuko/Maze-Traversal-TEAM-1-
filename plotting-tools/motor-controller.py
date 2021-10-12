import re
import matplotlib.pyplot as plt

# Extract output signals from the serial log
with open("pwm.log") as f:
    serial_out = f.read()

left_matches  = re.findall(r'LEFT.+ (\d+\.\d+).* -> (\d+\.\d+)', serial_out)
right_matches = re.findall(r'RIGHT.+ (\d+\.\d+).* -> (\d+\.\d+)', serial_out)

# Pull out target position vs. actual position
min_left_val = float(left_matches[0][1])
left_true_pos    = [float(true_pos) - min_left_val for true_pos, _ in left_matches]
left_target_pos  = [float(target_pos) - min_left_val for _, target_pos in left_matches]

min_right_val = float(right_matches[0][1])
right_true_pos   = [float(true_pos) - min_right_val  for true_pos, _ in right_matches]
right_target_pos = [float(target_pos) - min_right_val for _, target_pos in right_matches]

# Create time axis
dt = 10e-3
t = [dt * i for i in range(len(right_matches))]

plt.subplot(121)
plt.plot(t, left_target_pos, 'b', t, left_true_pos, 'r', marker='o', linestyle='', markersize=1)
plt.xlabel("Time (seconds)")
plt.ylabel("Distance")
plt.legend(["Target Position", "Measured Position"])
plt.title("Left Motor Velocity Controller")

plt.subplot(122)
plt.plot(t, right_target_pos, 'b', t, right_true_pos, 'r', marker='o', linestyle='', markersize=1)
plt.xlabel("Time (seconds)")
plt.ylabel("Distance")
plt.legend(["Target Position", "Measured Position"])
plt.title("Right Motor Velocity Controller")

plt.show()