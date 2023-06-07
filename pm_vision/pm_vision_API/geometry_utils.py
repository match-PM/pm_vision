
class circle:
    def __init__(self, ax1, ax2, radius, ax1_suffix, ax2_suffix, unit):
        self.ax1 = ax1
        self.ax2 = ax2
        self.radius = radius
        self.ax1_suffix = ax1_suffix
        self.ax2_suffix = ax2_suffix
        self.unit = unit

    def change_unit_to(self,new_unit):
        if self.unit == "um" and new_unit == 'mm':
            self.ax1 = self.ax1 * 0.001
            self.ax2 = self.ax2 * 0.001
            self.radius = self.radius * 0.001
            self.unit = new_unit
        elif self.unit == "um" and new_unit == 'm':
            self.ax1 = self.ax1 * 0.001 * 0.001
            self.ax2 = self.ax2 * 0.001 * 0.001
            self.radius = self.radius * 0.001 * 0.001
            self.unit = new_unit
        elif self.unit == "mm" and new_unit == 'um':
            self.ax1 = self.ax1 * 1000
            self.ax2 = self.ax2 * 1000
            self.radius = self.radius * 1000
            self.unit = new_unit
        elif self.unit == "mm" and new_unit == 'm':
            self.ax1 = self.ax1 * 0.001
            self.ax2 = self.ax2 * 0.001
            self.radius = self.radius * 0.001
            self.unit = new_unit
        elif self.unit == "m" and new_unit == 'um':
            self.ax1 = self.ax1 * 1000 * 1000
            self.ax2 = self.ax2 * 1000 *1000
            self.radius = self.radius * 1000 *1000
            self.unit = new_unit
        elif self.unit == "m" and new_unit == 'mm':
            self.ax1 = self.ax1 * 1000
            self.ax2 = self.ax2 * 1000
            self.radius = self.radius * 1000
            self.unit = new_unit        
        else:
            print("Invalid unit given!")

    def print_circle_information(self):   
        print(f"{self.ax1_suffix}-Koordinate: {self.ax1}")
        print(f"{self.ax2_suffix}-Koordinate: {self.ax2}")
        print(f"Radius: {self.radius}")
        print(f"Unit: {self.unit}")

if __name__ == "__main__":
    mycircle=circle(5, 5, 1, 'x','y', 'mm')
    mycircle.print_circle_information()
