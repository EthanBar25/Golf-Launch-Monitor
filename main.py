from LaunchCalc import graphLiftTrajectory
import DragModel

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
DRIVERCD = 0.2185
CL = 0.25

class club_data:
    def __init__(self, name, launch_angle, cd, cl, spin, speed, angle_attack):
        self.name = name
        self.launch_angle = launch_angle
        self.cd = cd
        self.cl = cl
        self.spin = spin
        self.speed = speed
        self.angle_attack = angle_attack

pw = club_data("Pitching Wedge", 23.7, PWCD, CL, 9304, 104, -4.7)
nine = club_data("9 Iron", 20.0, NINECD, CL, 8647, 112, -4.3)
eight = club_data("8 Iron", 17.8, EIGHTCD, CL, 7998, 118, -4.2)
seven = club_data("7 Iron", 16.1, SEVENCD, CL, 7097, 123, -3.9)
six = club_data("6 Iron", 14.0, SIXCD, CL, 6231, 130, -3.7)
five = club_data("5 Iron", 11.9, FIVECD, CL, 5361, 135, -3.4)
four = club_data("4 Iron", 10.8, FOURCD, CL, 4836, 140, -2.9)
three = club_data("3 Iron", 10.3, THREECD, CL, 4630, 145, -2.5)
hybrid = club_data("Hybrid", 10.2, HYBRIDCD, CL, 4437, 149, -2.4)
fivewood = club_data("5 Wood", 9.7, FIVEWOODCD, CL, 4350, 156, -2.5)
threewood = club_data("3 Wood", 9.3, THREEWOODCD, CL, 4655, 162, -2.3)
driver = club_data("Driver", 10.4, DRIVERCD, CL, 2686, 171, -0.9)

def main():
    club_array = [171, 2545, 10.4, -0.9]
    pred_cd = DragModel.model_cd.predict([club_array])[0]
    pred_cl = DragModel.model_cl.predict([club_array])[0]
    club_test = club_data("Test Club", club_array[2], pred_cd, pred_cl, club_array[1], club_array[0], club_array[3])
    #Should be around 0.215, 0.16 for driver
    graphLiftTrajectory(club_test, 0, 0, "fairway")

if __name__ == "__main__":
    main()