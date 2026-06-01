import matplotlib.pyplot as plt

# Mock GPS Coordinates based on your perfect square mission
lats = [12.8967989, 12.896850, 12.8966891, 12.896600, 12.8967989]
longs = [77.5217387, 77.521850, 77.5216153, 77.521500, 77.5217387]

plt.figure(figsize=(6,6))
plt.plot(longs, lats, marker='o', color='green', linestyle='-', linewidth=2, label='Flight Path')
plt.scatter(longs[0], lats[0], color='red', s=100, zorder=5, label='Home / Takeoff')

plt.title("Drone Auto Mission - Square Route")
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.grid(True)
plt.legend()
plt.savefig("flight_route_plot.png")
print("GPS Plot generated and saved as flight_route_plot.png")
plt.show()
