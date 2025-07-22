Golf Ball Flight Simulator (3D) with Spin, Drag, Lift, and Roll
This Python project simulates the full 3D flight of a golf ball using physics-based modeling of drag, lift (Magnus effect), and spin-induced curvature. It also includes post-landing rollout estimation, giving realistic total distances on various surfaces.

The simulation includes:

Club-specific launch conditions

Spin axis modeling from face/path differences

Dynamic loft and spin loft computation

Side spin & lateral curve calculation

Rollout estimation on different surfaces (fairway, rough, wet, etc.)

Interactive 3D trajectory animation using matplotlib

Features
Realistic launch modeling for all golf clubs
Spin axis & side spin-based curvature
Dynamic lift and drag forces
Rollout distance estimation based on spin & impact angle
Adjustable landing surface (fairway, rough, wet, etc.)
3D animated visualization of ball flight

Physics Concepts Used
Drag Force: Based on club-specific drag coefficients

Lift Force (Magnus Effect): Estimated from backspin

Spin Axis: Derived from club face/path relationship

Side Spin: Affects lateral curvature

Rollout Model: Based on descent angle, spin, surface friction

Club Modeling
Each club (from driver to pitching wedge) is modeled with:

Launch angle

Spin RPM

Club speed (mph)

Drag coefficient (Cd)

Angle of attack

These are implemented via the club_data class.

Visual Output
The function graphLiftTrajectory() shows the golf ball's full 3D flight and rollout in matplotlib, including:

Red moving ball during flight

Blue trail path

Final landing point

Optional camera angles (side view / bird's eye)

Installation & Dependencies
Install required Python packages:

Copy code
pip install numpy matplotlib

How to Run
Clone or download the repo.

Run the script:

python golf_simulator.py
To simulate a shot, call:

graphLiftTrajectory(driver, face_angle=-2, path_angle=0, surface="fairway")
Modify driver to other clubs (e.g., seven, pw, hybrid) as needed.

Customize Inputs

face_angle, path_angle: to simulate slices, draws, etc.

surface: "fairway", "rough", "wet", "hard"

Launch parameters: via club definitions (will later implement user input for spin, ball speed, etc...)

File Structure
golf_simulator.py        # Main script
README.md                # This file

Credits
Physics logic by Ethan Bar
Lift/drag modeling inspired by basic aerodynamics
Visualization with matplotlib

Future Improvements
Add wind simulation
Launch monitor input support (GCQuad, TrackMan)
Include clubface loft/lie effects
Export trajectory data for external analysis
