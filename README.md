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

