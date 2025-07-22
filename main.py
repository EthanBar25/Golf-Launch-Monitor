from LaunchCalc import graphLiftTrajectory

PWCD = 0.317
NINECD = 0.319
EIGHTCD = 0.309
SEVENCD = 0.295
SIXCD = 0.296
FIVECD = 0.2815
FOURCD = 0.2776
THREECD = 0.277
HYBRIDCD = 0.267
FIVEWOODCD = 0.2753
THREEWOODCD = 0.271
DRIVERCD = 0.246

class club_data:
    def __init__(self, name, launch_angle, cd, spin, speed, angle_attack):
        self.name = name
        self.launch_angle = launch_angle
        self.cd = cd
        self.spin = spin
        self.speed = speed
        self.angle_attack = angle_attack

pw = club_data("Pitching Wedge", 24.2, PWCD, 9304, 102, -4.7)
nine = club_data("9 Iron", 20.4, NINECD, 8647, 109, -4.3)
eight = club_data("8 Iron", 18.1, EIGHTCD, 7998, 115, -4.2)
seven = club_data("7 Iron", 16.3, SEVENCD, 7097, 120, -3.9)
six = club_data("6 Iron", 14.1, SIXCD, 6231, 127, -3.7)
five = club_data("5 Iron", 12.1, FIVECD, 5361, 132, -3.4)
four = club_data("4 Iron", 11.0, FOURCD, 4836, 137, -2.9)
three = club_data("3 Iron", 10.4, THREECD, 4630, 142, -2.5)
hybrid = club_data("Hybrid", 10.2, HYBRIDCD, 4437, 146, -2.4)
fivewood = club_data("5 Wood", 9.4, FIVEWOODCD, 4350, 152, -2.5)
threewood = club_data("3 Wood", 9.2, THREEWOODCD, 4655, 158, -2.3)
driver = club_data("Driver", 10.9, DRIVERCD, 2686, 167, -0.9)

def main():
    graphLiftTrajectory(driver, -2, 0, "fairway")

if __name__ == "__main__":
    main()