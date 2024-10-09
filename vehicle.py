import math

""" Based on Intelligent Driver Model described in Derbel(2013) https://doi.org/10.3311/pp.tr.2012-2.02 """
class Vehicle:
    def __init__(self, x, speed, a_max=1.0, v_max=120.0, s0=5.0, T=1.5, b=2.0, delta=4):
        self.x = x        # Position of the vehicle (meters)
        self.speed = speed  # Current speed (m/s)
        self.a_max = a_max  # Maximum acceleration (m/s^2)
        self.v_max = v_max  # Desired speed (m/s)
        self.s0 = s0       # Minimum desired distance to the car ahead (meters)
        self.T = T         # Safe time headway (seconds)
        self.b = b         # Comfortable braking deceleration (m/s^2)
        self.delta = delta # Acceleration exponent (typically 4)
        self.length = 3
    def calculate_acceleration(self, vehicle_ahead):
        """ Calculate acceleration using the IDM. """
        
        if vehicle_ahead:
            delta_x = vehicle_ahead.x - self.x - vehicle_ahead.length 
            delta_v = self.speed - vehicle_ahead.speed  
        else:
            delta_x = float('inf') 
            delta_v = 0

        # Calculate desired gap based on the IDM
        s_star = self.s0 + max(0, self.speed * self.T + (self.speed * delta_v) / (2 * math.sqrt(self.a_max * self.b)))

        # IDM acceleration formula
        acceleration = self.a_max * (1 - (self.speed / self.v_max)**self.delta - (s_star / delta_x)**2)

        return acceleration

    def update(self, vehicle_ahead, dt):
        """ Update vehicle state based on the calculated acceleration. """
        acc = self.calculate_acceleration(vehicle_ahead)
        self.speed = max(0, self.speed + acc * dt)  # Ensure speed doesn't go below 0
        self.x = self.x + self.speed * dt + 0.5 * acc * dt**2  # Update position

# Example Usage
if __name__ == "__main__":
    vehicle_1 = Vehicle(x=0, speed=110.0)  # Ego vehicle
    vehicle_2 = Vehicle(x=50, speed=123.0)  # Vehicle ahead

    dt = 0.1  # Time step (seconds)
    time_horizon = 20  # Simulate for 20 seconds

    for t in range(int(time_horizon / dt)):
        vehicle_1.update(vehicle_ahead=vehicle_2, dt=dt)  
        vehicle_2.update(vehicle_ahead=None, dt=dt)       

        print(f"Time {t * dt:.1f}s | Vehicle 1 -> Position: {vehicle_1.x:.2f}m, Speed: {vehicle_1.speed:.2f}m/s")
        print(f"         | Vehicle 2 -> Position: {vehicle_2.x:.2f}m, Speed: {vehicle_2.speed:.2f}m/s\n")
