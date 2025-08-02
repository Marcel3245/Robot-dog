# Robot-dog
This project is still being developed, but has already some main features like walking mode, pitching and starting position(stand-up). What will be included is to be able to control the robot by gestures, and because of that, there are some extra libraries for Raspberry Pi (openCV, picamera2 etc).

To get 3D model, visit: https://www.thingiverse.com/thing:6733826

# Robot-leg Theory

Taking a cue from quadruped animals, which adjust their stride frequency, reduce contact time, and incorporate lateral swing movements to maintain balance at higher locomotion speeds, I aim to design a leg model that achieves a high-speed gait, minimizes power consumption, and ensures stability.

> It's important to note that this text is based on insights from the article ["Leg Trajectory Planning for Quadruped Robots with High-Speed Trot Gait,"](https://www.mdpi.com/2076-3417/9/9/1890) authored by Xuanqi Zeng, Songyuan Zhang, Hongji Zhang, Xu Li, Haitao Zhou, and Yili Fu. I highly recommend delving into the details of their work to further enrich your understanding. The link to the article is provided above for your reference.

## Introduction

The choice of a legged robot over a wheeled or tracked counterpart is straightforward—it proves more adept in navigating rough terrains and complex environments. However, there are drawbacks to opting for legs, particularly in terms of power consumption. To simplify the physics involved, consider this analogy:

Imagine a bike and a dog with the same mass moving at identical speeds in the same direction. The bike follows a straight trajectory in the Z-axis, maintaining a constant height. In contrast, the dog's mass center fluctuates as it walks, moving up and down. To illustrate, try holding a chalk at the highest point you can reach, walk, and draw a line—you'll notice it's not a straight line but more of a wavy pattern. This variation in height during walking contributes to increased energy consumption compared to the efficiency of riding a bike.

The energy used by a bike is given by `E_bike = E_k + E_p = (m*v²)/2 + m*g*h` (where `h=0`), simplifying to `E_bike = (m*v²)/2`.

On the other hand, the energy used by a dog is represented by `E_dog = E_k + E_p = (m*v²)/2 + m*g*h` (where `h>0`), which further includes potential energy due to height, resulting in `E_dog = (m*v²)/2 + m*g*h`.

As mentioned earlier, both the car and the dog share the same mass and velocity, yet the energy used by the bike is less than that of the dog: `E_bike_used < E_dog_used`.

Hence, I'll endeavor to optimize energy consumption by selecting suitable actuators and refining the movement path. Other factors such as complexity, cost, and more will also be considered, but I'll delve into those details later.

## Types of actuators (very briefly)

### Hydraulic actuators
- Very difficult design.
- High cost.
- Large noise and size.
- Huge carrying capacity and motion ability.
- Heavy

### Electric actuators
- Wide cost range.
- Easier design and control system.
- High acceleration.
- Lighter.

I'll conclude here and proceed by opting for an electric actuator, which aligns better with my DIY projects. It's a cost-effective, lightweight option with sufficient carrying capacity.
> For a deeper understanding of the differences between electric, hydraulic, and pneumatic linear actuator systems, I highly recommend reading the ["Comparative study of a hydraulic, pneumatic, and electric linear actuator system."](https://www.researchgate.net/publication/224213076_Comparative_study_of_a_hydraulic_pneumatic_and_electric_linear_actuator_system)

Furthermore, it's worth mentioning in advance that my plan involves designing a simplified 2-joint structure. This approach aims to streamline the project and reduce overall costs.

In the realm of quadruped robots, the application of leg motors generally follows two primary approaches: distributing motors at each joint or consolidating them solely at the shoulder. The first method results in elastic actuation, favorable for dynamic motions. It facilitates steady and controllable walking; however, the drawback lies in the slower swing speed of the legs, leading to a lower overall walking speed. On the other hand, the second option presents a simpler design with minimal mass center changes. It enables the easier design of high-speed movements without concerns about leg inertia. Now, let's delve into the mathematical analysis to craft an optimal leg design.

## Kinematic analysis of the leg
<p align="center">
  <img width="419" height="441" alt="image" src="https://github.com/user-attachments/assets/df474e90-a52b-490b-865a-eaaf37b0d8a8" />
</p>

### 1. Manipulability Measure
In the context of the Manipulability Measure, let's examine a manipulator equipped with 2-degrees of freedom, where the joint variables are represented as `θ₁` and `θ₂`. When we define the hand position as the manipulation vector 'r' with coordinates `[x, y]ᵀ`, the corresponding Jacobian matrix is expressed as follows:

$$
J(\theta_1, \theta_2) =
\begin{pmatrix}
L_1\cos(\theta_1) + L_2\cos(\theta_1+\theta_2) & L_2\cos(\theta_1+\theta_2) \\
L_1\sin(\theta_1) + L_2\sin(\theta_1+\theta_2) & L_2\sin(\theta_1+\theta_2)
\end{pmatrix}
$$

The manipulability measure `w` is the determinant of the Jacobian:

$$
w = |\det(J(\theta_1, \theta_2))| = L_1 L_2 \sin(\theta_2)
$$

The consequential factor in this scenario is termed the Jacobian determinant. At a specific point, denoted as 'w', it attains its minimum, equivalent to zero. To enhance the manipulability of the leg, a strategic selection of parameters is imperative to maximize the overall output.

Hence, the manipulator attains its optimal position when `θ₂` equals `±90°` (since `sin(90°) = 1`), and the lengths of L₁ and L₂ are equal (when `L₁ + L₂ = const`). To reinforce these findings, consider the daily example of the human arm:

> “A two-joint arm (accounting for the shoulder and elbow as 2-degrees of freedom) that approximately adheres to the relationship L₁=L₂. Moreover, in practical scenarios, when handling objects, the elbow angle tends to hover around the optimum of close to 90°, as highlighted in "Manipulability of Robotic Mechanism" by Tsuneo Yoshikawa.”

### 2. Optimum angle α, between the foot and the ground
Referencing a height `h` from shoulder to foot.
*Cosine rule:* `L₁² = L₂² + h² - 2*h*L₂*cos(β)` - `β` is an angle between L₂ and h
From Figure 2, we know `α + β = 90°`. Therefore:
`α = 90° - arccos((L₂² + h² - L₁²) / (2*L₂*h))`

<p align="center">
  <img width="572" height="461" alt="image" src="https://github.com/user-attachments/assets/573ac185-1e80-44aa-98a8-316ff9be2614" />
</p>

### 3. The distance between shoulder and knee
Using Heron's Formula:
*   Half of the triangle's perimeter: `S = (L₁ + L₂ + h) / 2`
*   Heron's formula: `A = sqrt(S * (S - L₁) * (S - L₂) * (S - h))`
*   Area of triangle: `A = (h * d) / 2`

*Space occupied:* `d = (2 * sqrt(S * (S - L₁) * (S - L₂) * (S - h))) / h`

<p align="center">
  <img width="651" height="514" alt="image" src="https://github.com/user-attachments/assets/a6feb94d-c53e-408c-b757-a015a3113626" />
</p>

## Inverse Kinematic Analysis

To determine the optimal paths for our leg, it's crucial to calculate the positions of two joints: the shoulder and the foot. For this task, we employ Inverse Kinematics (IK), which stands in contrast to Direct or Forward Kinematics. In Forward Kinematics, we start from a fixed base (the shoulder) and work our way towards the end effector position (the foot) using Cartesian coordinates. Inverse Kinematics, on the other hand, takes the end effector (foot) as the base, relying on the process of deducing joint angles from known coordinates of the end effector (x and y coordinates of the foot). This enables us to precisely determine the positions of the shoulder and foot joints in our leg model.

<p align="center">
  <img width="572" height="377" alt="image" src="https://github.com/user-attachments/assets/c1c6b303-5994-4118-83c5-6011c522f06d" />
</p>

- `(x, y)` is our end effector (foot)
- `A` is our base point (shoulder)
- `h` is the height from shoulder to foot

*Pythagoras:* `h² = x² + y²`

<p align="center">
  <img width="549" height="350" alt="image" src="https://github.com/user-attachments/assets/557e52d6-5666-41f6-ba33-5b250dd1d857" />
</p>

*Cosine rule:*
`h² = L₁² + L₂² - 2*L₁*L₂*cos(β)`
`180° = β + θ₂ → β = 180° - θ₂`
Substituting `h² = x² + y²` and the expression for `β`:
`cos(180° - θ₂) = (L₁² + L₂² - x² - y²) / (2 * L₁ * L₂)`
So, the knee angle `θ₂` is:
`θ₂ = arccos((x² + y² - L₁² - L₂²) / (2 * L₁ * L₂))`

<p align="center">
  <img width="621" height="375" alt="image" src="https://github.com/user-attachments/assets/65032fc8-5808-4557-a036-0c2f469f8023" />
</p>
- `α` is the angle between `h` and `L₁` at point A
`α = arctan((L₂*sin(θ₂)) / (L₁ + L₂*cos(θ₂)))`

<p align="center">
  <img width="691" height="426" alt="image" src="https://github.com/user-attachments/assets/b7971ec0-cba9-4ab1-80e3-06f51e46ad84" />
</p>

- `γ` is the angle between `h` and the X-axis
`γ = arctan(y / x)`

The shoulder angle `θ₁` is then:
`θ₁ = γ - α`
`θ₁ = arctan(y/x) - arctan((L₂*sin(θ₂)) / (L₁ + L₂*cos(θ₂)))`

By utilizing these two formulas, we can effortlessly compute the desired position of the foot. In scenarios like a robot arm, such calculations might require additional considerations. For instance, if `θ₂` becomes negative, we might need to adjust the formula by multiplying `θ₂` by -1 and, in `θ₁`, change the minus sign to plus. However, in the case of legs, where the knee only bends in one direction, this complexity is eliminated, and we can proceed without such concerns.

---






# Robot Dog: Assembly and Implementation

## Introduction

As a continuation of my journey with robot dog creation and implementation, the time has come to print the parts and put them all together. It was over 40 hours of continuous printing. Like always, when something has to meet the real world and it’s not only on paper, there are a few things I could make better or change, and I will mention them in this article.

I can’t promise it’s not the last post about this project, and not all my goals will be completed, but we will see what the future will bring.

> I also want to mention that I won’t go so much into detail in parts that I already covered in the other articles, like inverse kinematics, leg design or leg simulation. No more introduction, and enjoy your reading!

## External Parts (Bill of Materials)

### What you need:
*   12x Servos
*   1x Buck converter
*   1x Raspberry Pi 4B
*   1x High-amperage switch
*   1x PCA9685 (to control 12 servos)
*   1x Battery 7.4v, 30c (to not destroy the battery during high ampere draw)
*   Screws:
    *   12x M3x12mm
    *   4x M3x10mm
    *   8x M3x20mm
    *   4x M3x14mm
    *   8x M3x18mm
*   32x M3 nuts
*   20x M3 Washers

### Additional:
*   1x Fuse 20A (to protect buck converter)
*   1x Fuse holder
*   1x Multimeter (I know it can hold up to 10A, but it is only to calibrate the buck converter).

## Design and Printing

If you want to print your robot dog, go and see [my Thingiverse profile](https://www.thingiverse.com/your_profile_link). Let’s start from the place where everything began, which is a 3D model. My main idea was to create a robot which would be:
1.  The size of a small dog.
2.  Effortless in changing broken parts.
3.  Easy to assemble with as few screws as possible.

In the end, I think I accomplished these three points. Overall to put it together you need only 36 screws (without including the screws you get in the kit of servos), where the leg has three dimensions of movement with a reliable rigid body. The robot consists of 21 parts, where one leg is 8 parts and the body is built out of 3 parts (with a possible extension, a cover on the top). For each leg, there are 3 servos. To read about leg design, I highly recommend reading my last post. For my body, I decided to design only 3 parts to make it easy to assemble; it consists of `Back Bumper`, `Front Bumper` and `Body`. To screw these three parts together you need only 8 M3x20mm screws.

During printing, you have to mirror four parts for the left and right sides in your slicing program (`Shoulder-Front`, `Shoulder-Back`, `Thigh-lower` and `Thigh-upper`). Or download ready-to-use files from my Thingiverse profile, given above. For me, it is Cura, which I highly recommend.

### Printing data (quality 0.2 mm, adhesion and support on, 0.4 nozzle):
*   **100% infill:**
    *   `Transition-top1`
    *   `Transition-top2`
    *   `Transition-bottomV2`
*   **20% infill:**
    *   `Thigh-upper Right`
    *   `Thigh-lower`
    *   `Calf Right`
    *   `Foot`
    *   `Shoulder`
    *   `Body`
    *   `Front and Back bumper`

<p align="center">
  <img width="491" height="537" alt="image" src="https://github.com/user-attachments/assets/bc0860e1-bad2-4130-b316-4a78a1481522" />
</p>

## Assembling
<p align="center">
  <img width="432" height="546" alt="image" src="https://github.com/user-attachments/assets/40eeca52-4bdc-401a-b9f7-9e09f7fa5388" />
</p>

Before mounting parts to the servos, you have to calibrate all the servos to their starting position, which means:
*   Servo top (`S1`) - 0 degrees
*   Servo bottom (`S2`) - 0 degrees
*   Shoulder servo (`S3`) - 90 degrees

Using the `servo_calibration.py` script from the repository, you can easily do this. Also, I recommend marking on the servos where their 0 and 180-degree points are and how they rotate. After that, you can start to assemble them.

<p align="center">
  <img width="726" height="300" alt="image" src="https://github.com/user-attachments/assets/b14a31d5-e012-4e7e-87ae-027b3e8b1dd3" />
</p>

1.  First, you have to assemble four legs, two rights and two lefts like in the picture above. I would recommend starting from the bottom: screw the foot to the calf, then the calf to the lower and upper Thigh.
    > **IMPORTANT:** Here, you have to be careful, the bigger holes for nuts should be directed inward! So in the right leg, the bigger holes in the upper part have to be to the left, when the leg is directed to the front (so you see its back, like in the picture above).
2.  Then you can screw the `Transition-top2`.
   
<p align="center">
  <img width="180" height="337" alt="image" src="https://github.com/user-attachments/assets/0e1c843d-59fa-4252-ac27-33ed0a0aef03" />
</p>
  
3.  When four of your legs are done, you can start mounting servos to the shoulder, but you have to remember to put the servo wing in the shoulder part before (you can also glue it to the surface). Servos should be mounted to the part in the given position. The chamfer in the top-right corner indicates the direction in which the shoulder should be facing (to the front).

<p align="center">
  <img width="1036" height="372" alt="image" src="https://github.com/user-attachments/assets/7b8bef74-ff14-4b10-99d4-272f0033a5b0" />
</p>

The entire right side should look like this, and the left side should be the mirror image of the right:

<p align="center">
  <img width="765" height="462" alt="image" src="https://github.com/user-attachments/assets/32ba3b51-40ed-45d7-b1ff-7bc05cb9fc9d" />
</p>

4.  When all four legs are done, you can move to mounting servos which will control the yaw of the leg. For all four servos, the rotating part should be on the bottom.

<p align="center">
  <img width="863" height="471" alt="image" src="https://github.com/user-attachments/assets/e7a443a5-f1d3-48be-be1b-1834a633c9a7" />
</p>

5.  When all sides are completed, you can attach the servo wings to the shoulder servos in the body and screw the front/back bumper to the body. After all these steps, you are now ready to move further, which is hardware design.

## Hardware

Because of my lack of knowledge of electronics, I tried to keep it simple, which in the end became my biggest mistake.

Below is a hardware diagram of my design. But in this section, I also want to mention how I could make it better, so you don’t have to repeat my mistakes.

<p align="center">
  <img width="998" height="509" alt="image" src="https://github.com/user-attachments/assets/4681432d-f64d-4708-8843-9d14818e64f5" />
</p>
  
For many, it is obvious why we should use an external power supply for our servos and a different power source for the Raspberry Pi, but for those who it’s not, I would go into more details. Supplying the PCA9685 module from the Raspberry Pi `5v` pin is not a good idea. The voltage level may be okay (servos operate between 4.8v - 7v), but the problem is the current. My power adapter (AC-DC) gives an output of 5.1V, and the `5V` PIN is almost directly connected to it. My adapter also provides around 3.0A, which seems like a lot.

**BUT**, we have to also include the current draw of the Raspberry Pi, which is around **700mA**. In the end, we end up with only `3000mA - 700mA = 2300mA`.

However, another **BUT** appears when we calculate how much current the servos can need. We can read from the datasheet that one servo in the moving phase can draw up to **1300mA**. Multiplying it by 12 servos (assuming all are moving at once) gives us **15,600mA (15.6A)**. Now, this `2300mA` becomes nothing. Even if in the walking function only 8 servos are moving, it still gives you **10,400mA (10.4A)**. That’s a lot.

Also, we can assume the worst-case scenario when all servos are stalled. The datasheet tells us that a stalled servo can draw **2500mA**, the so-called stall current. That would give **30 amps**. Because of that, we need a good battery that can handle this amount of current draw (minimal `30c`) and a good buck converter that drops the voltage from 7.4v to 6v.

> **My Biggest Mistake:** My main and biggest mistake was not splitting the 12 servos into 4 modules, one for each leg. Of course, I would have to then change my PCA9685 (16-channel) into 4 smaller controllers, but I wouldn’t have to worry about destroying the buck converter, servo controller, or even Raspberry Pi. While splitting the servo controller, the highest possible current per controller would be only 7.5 amps, and the average would be around 2.5 amps, not 10!

> **My Second Problem:** My second problem was buying cheap and bad-quality servos. It happens many times that they stall during movement, causing a high stall current that would burn the servo without my reaction. Also, they have a tendency to skip steps. So maybe that’s a good lesson that cheap projects are not always the best…

A piece of advice from me is to redesign the hardware for this project. When you test some code, switch the power supply for servos back to the `5V` PIN on the Raspberry Pi and put the robot on a pedestal, so that the load on the servo will be minimal. The `2300mA` should be enough to test it without breaking any parts.

## Code

I’m not going to upload the entire code here, but explain the basics and hidden math. If you want to download it, you can go to my [GitHub repository](https://github.com/your-repo-link) and find it there. Also, I’m not going to talk about leg inverse kinematics, which I already described [here](link-to-previous-article).

My idea was to store the position in a Python dictionary to execute the position of the robot relatively at once. We always start from calculations of every servo angle position based on our desired position, then store them in the dictionary. When all calculations are done, the `write_servo` function reads data from our dictionary and writes to the given servo.

> ### Dictionary Abbreviation Explanation:
> *   `FR` - Front leg, Right side
> *   `FL` - Front leg, Left side
> *   `BR` - Back leg, Right side
> *   `BL` - Back leg, Left side
> *   `S1` - Servo on the top
> *   `S2` - Servo on the bottom
> *   `S3` - Servo attached to the shoulder

### Servo Offsets
We also have to remember about the offsets of our servos. 0 degrees for one servo is not the same position in the real world as another. For example, for `FRS1` and `FLS1`, to create a mirror image, we must apply an offset. To do this, we use simple trigonometry.

<p align="center">
  <img width="198" height="329" alt="image" src="https://github.com/user-attachments/assets/cb541817-052f-479e-8c2d-f1852032abd4" />
</p>

If we want to move our 0-degree point to be on the bottom of a semicircle (180 degrees), we have to add `π` (180 degrees). But to also keep the movement area on the correct side, we subtract our desired point from `π`. Our equation will look like `π - desired_point`. When we convert it back to degrees, we create our offset, and the system will know how to make a mirror image.

<p align="center">
  <img width="671" height="376" alt="image" src="https://github.com/user-attachments/assets/173f8fc5-5073-48fb-84cf-1e2209a6e550" />
</p>

For example, let's assume we want a position of 30 degrees for both left and right servos.
*   **Right Servo:** The 0-degree point is already at the bottom, so we just add 30 degrees.
*   **Left Servo:** We use our formula `π - desired_point`. We give `desired_point=30°`, which gives us `180° - 30° = 150°`. Great, we got a mirror image.

<p align="center">
  <img width="778" height="479" alt="image" src="https://github.com/user-attachments/assets/abbf6428-8f4b-453e-a92e-a40aadf4753a" />
</p>

At the end, we just have to find the offset for each servo. This is also a good place to add an adjustment for each servo (in degrees) if the assembly didn't go perfectly.

### Movement Functions
The `starting_position` function is just the starting phase to have some reference. It uses `starting_foot_cords_front` and `starting_foot_cords_back`. The values are `[yaw, forward/backward, height]` in cm. For example, `[0, 0, 15]` means the robot will have a height of 15 cm, and the foot will be inline with the rotating part of the bottom servo (`S2`).

The next function is `pitch`, which controls how much the robot should be skewed to the front. Knowing the robot's length (`c`) and the desired angle (`α`), we can calculate the change in x (`d`) and z (`zB`, `zF`) for the feet.
*   `d = (c/2) * (1 - cos(α))`
*   Back legs height: `zB = h - sin(α) * (robot_length / 2)`
*   Front legs height: `zF = h + sin(α) * (robot_length / 2)`

<p align="center">
  <img width="751" height="497" alt="image" src="https://github.com/user-attachments/assets/658e8e08-d7e3-42e0-a77c-7bc6e9cdf86f" />
</p>

### Walking Gait (Bézier Curve)
The last function is `walking_mode`. This function is the most complicated and uses a **Bézier curve** to make the path of the foot smoother. I won’t explain it in detail, but if you want to read more, I recommend the article: ["Leg Trajectory Planning for Quadruped Robots with High-Speed Trot Gait"](https://www.mdpi.com/2076-3417/9/9/1890).

The Bézier formula helps us to create a smooth path out of a few points.
`B(t) = (1-t)³P₀ + 3(1-t)²tP₁ + 3(1-t)t²P₂ + t³P₃`

<p align="center">
  <img width="782" height="486" alt="image" src="https://github.com/user-attachments/assets/86f0a982-903b-4ac7-8541-8eb592f978e4" />
</p>

Using this formula and creating our path points, we can get a very nice trajectory of the foot based on only 12 points.

<p align="center">
  <img width="823" height="535" alt="image" src="https://github.com/user-attachments/assets/f8ed1923-e0c0-492c-990e-9823d48ee796" />
</p>

The movement of the foot can be split into two parts: the **stance phase** and the **swing phase**. I decided that the stance phase will be linear, while the swing phase will have acceleration and deceleration.

To synchronize the four legs, I found that each leg is delayed by ¾ of a cycle relative to the previous one. We can split our movement into 4 phases. At any time, each leg will be in a different phase.
*   **Phase 1:** ENDING point → MID point
*   **Phase 2:** MID point → STARTING point
*   **Phase 3:** STARTING point → MOVE point (75% of swing)
*   **Phase 4:** MOVE point → ENDING point

The `shift_list_right` function controls which phase each leg is in.
*   **Cycle 1:** `['FR', 'BR', 'FL', 'BL'] = [phase1, phase2, phase3, phase4]`
*   **Cycle 2:** `['BL', 'FR', 'BR', 'FL'] = [phase1, phase2, phase3, phase4]`

The `cords_respect_CoM` (Center of Mass) function is for future development with a different coordinate system.

## Conclusion

<p align="center">
  <img width="871" height="642" alt="image" src="https://github.com/user-attachments/assets/c261261e-7745-4c63-ac2b-7faa1043ba17" />
</p>

To summarize this project, there are some mistakes/faults that could be improved. However, it was a great project where I learned a lot. There is a chance that in the future I will fix my blunders. You can find the code and 3D models in the given links. I highly recommend developing my project and asking questions, as it is an open-source project.
