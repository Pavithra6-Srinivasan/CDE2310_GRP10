# **CORE DESIGN FEATURES**
- Tube storing balls wrap around the turtlebot with a slight inclination (~5deg)
    - Reduces the length & width of entire turtlebot, allowing it to traverse tighter spaces
    - Roughly symmertrical design keeps center of gravity near the center of the robot & as low as possible
    - Slight inclination decreases the height required to prevent LiDAR from being blocked
- Balls are gravity fed into the barrel
- Two counterspinng flywheels are mounted at the top of the barrel to launch the ball
    - Counterspinning flywheels will reduce sideway forces, ensuring vertical launch of ball
- Balls are pushed into the flywheels via a solenoid actuator
    - When solenoid actuator is powered, the core is pulled in, allowing a ball to roll into the barrel
    - When the solenoid actuator is unpowered, the spring in the solenoid actuator will extend, pushing the core forward, launching the ball
    - Default state of solenoid actuator is unpowered; core will prevent another ball from entering the barrel
    - When there is a ball in the barrel, another ball will not be able to enter as the current ball will prevent it from doing so
- Solenoid & Flywheels (motors) are controlled via the RPi
- Top two layers are raised using spacers
    - Prevent LiDAR from being blocked by the tube

# **FINER DESIGN FEATURES**
- 1.5cm wide hole along tube
    - Allows for quick trounbleshooting of balls not feeding into barrel/rolling through the tube
    - Allows for quick & easy removal of balls from tube
- RPi & USB2LDS shifted away from original position
    - Make space for other components & cables
- Small notches on tubes
    - Markers for alignment
- Tube support pieces are designed to be slightly lower than required
    - Allows for physical modification due to possible mismatches between physical and virtual design
    - e.g. adding sponge/foam material to prop up tube OR allow tube to be fitted lower (if required)
- Tube inner diammeter is designed to have 5mm clearance
    - Reduces friction, prevents ball from getting stuck
    - Accounts for possible manufacturing tolerances
- Barrel inner diammeter tapers from 5mm clearance to 1mm clearance
    - Ensure precision and reliability when launching as the solenoid actuator will strike the ball in the same area

# **Wheel speed calculations**
Goal: 1.5m launch height  
Radius of flywheels = 2.5cm

v2 = u2 + 2ah  
0 = u2 + 2(-9.81m/s2)(1.5m)  
Launch speed required, u = 5.43m/s

u = rw  
5.43m/s = 0.025m*w  
Angular velocity required, w = 217.2 rad s-1 = 2071 rpm

Max rpm under load: 4500 rpm  
Motor driver required to reduce speed via limiting voltage through pulse width modulation

# **Version Changes**
**Ver 1**:  
- Tube wraps around TurtleBot
- A powerful solenoid actuator (rated 45N) is used to launch the balls into the air  

**Ver 2**:  
- Tube clearance increased from 1mm to 5mm  
- Part dimensions adjustments  
- Added hole along tube  
- Added structural supports for the tube  

**Ver 3**:  
- Added motor & flywheels to launch the balls
    - Solenoid actuator alone has insuffient force to launch the ball to desired height
- Solenoid actuator replaced with a weaker one (rated 5N) to push the balls into flywheels
    - Reduces power consumption, size & weight of launching mechanism
- Slight modification to barrel  
- Shifting of RPi & USB2LDS

