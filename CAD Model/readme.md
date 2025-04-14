///CORE DESIGN FEATURES///
- Tube storing balls wrap around the turtlebot with a slight inclination (~5deg)
    - Reduces the length & width of entire turtlebot, allowing it to traverse tighter spaces
    - Roughly symmertrical design keeps center of gravity near the center of the robot & as low as possible
    - Slight inclination decreases the height required for the tube
- Balls are gravity fed into the barrel
- Two counterspinng flywheels are mounted at the top of the barrel to launch the ball
    - Counterspinning flywheels will reduce sideway forces, ensuring vertical launch of ball
- Balls are pushed into the flywheels via a solenoid
    - When solenoid is powered, the core is pulled in, allowing a ball to roll into the barrel
    - When the solenoid is unpowered, the spring in the solenoid will extend, pushing the core forward, launching the ball
    - Default state of solenoid is unpowered; core will prevent another ball from entering the barrel
    - When there is a ball in the barrel, another ball will not be able to enter as the current ball or solenoid core will prevent it from doing so
- Solenoid & Flywheels (motors) are controlled via the RPi
- Top two layers are raised using spacers
    - Prevent LiDAR from being blocked

///FINER DESIGN FEATURES///
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
- Tube inner diammeter is designed to have 5mm tolerance
- Barrel inner diammeter tapers from 5mm tolerance to 1mm tolerance
    - Ensure preicision when launching

///Version Changes///
Ver 1: Tube wraps around turtlebot, a solenoid is used to launch the balls into the air
Ver 2: Slight modifications to certain parts (tolerances, dimensions, added hole along tube)
Ver 3: Added motor & flywheels to launch the balls, solenoid replaced with a weaker one to push the balls into flywheel, slight changes to dimensions
