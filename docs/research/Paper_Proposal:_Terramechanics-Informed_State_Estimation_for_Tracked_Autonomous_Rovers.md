##  Paper Proposal: Terramechanics-Informed State Estimation for Tracked Autonomous Rovers

**Proposed Title:** Terramechanics-Informed State Estimation for Tracked Autonomous Rovers: Integrating Direct Visual-Inertial Odometry and 23-State Adaptive Fusion in Gazebo
**Target Venue:** *Journal of Field Robotics (JFR)* or *IEEE Robotics and Automation Letters (RA-L)*

### Authors & Affiliations

* **Khalid Bourr (Corresponding Author):** Mechatronics & Robotics Lab, University of Trieste, Italy (*unitsSpaceLab*).
* **Manan Kharwar:** Independent Researcher, Hamilton, ON, Canada.
* **Suyash:** Core Open-Source Maintainer.

---

### Abstract

Autonomous tracked rovers operating in agricultural and forestry environments suffer from severe localization drift due to track-soil slip. Standard wheel odometry fails in deformable terrain (mud, loose soil), and visual-inertial odometry (VIO) degrades under heavy canopy occlusion or low-angle glare. This paper presents a cascaded, terramechanics-aware sensor fusion framework deployed on a tracked agricultural robot ("Sowbot"). By integrating a modernized ROS 2 direct VIO pipeline with a 23-state Unscented Kalman Filter (UKF), the system dynamically down-weights slipping track encoders in favor of visual velocity during soil shearing events. To validate the architecture across the "reality gap," we introduce a comprehensive simulation pipeline using `Forest3D` and `gz-terramechanics`, allowing for the formal verification of the fusion software against genuine, non-linear track-soil interaction models in Gazebo prior to field deployment.

---

### 1. Introduction and Motivation

Tracked vehicles in agriculture require centimeter-level precision for tasks like automated seeding, yet rely on tracks that inherently slip and shear against the topsoil to steer. Traditional navigation stacks treat the ground as a rigid 2D plane. When the Sowbot traverses wet clay, track encoders report forward motion while the vehicle is actually digging into the soil.

This paper solves this reality gap through a three-pillar architecture: a high-fidelity terramechanics simulation engine for validation, a local visual-inertial tracking engine, and a 23-state adaptive filter that cross-references physical slip against optical flow.

---

### 2. System Architecture

The proposed stack bridges high-rate VIO with adaptive state estimation, validated through procedural physics simulations.

| Contributor | Subsystem | Function in the Stack |
| --- | --- | --- |
| **Khalid Bourr** | **Terramechanics Simulation** | Leverages `Forest3D` to build complex DEM-based Gazebo worlds and `gz-terramechanics` to simulate physical sinkage, traction, and resistive torques. Generates the slip data that triggers the filter's adaptive gating. |
| **Suyash** | **Local VIO Engine** | Maintains the modernized ROS 2 ROVIO implementation. Processes high-rate camera and IMU data to produce a clean velocity vector (`/rovio/odometry`) independent of ground friction. |
| **Manan Kharwar** | **23-State UKF Fusion** | Utilizes `FusionCore` to fuse VIO, IMU, and track data. Dynamically isolates the systematic yaw rate bias of the slipping tracks and coasts on VIO/IMU tracking when the physical tracks lose traction. |

---

### 3. Experimental Validation in Gazebo

To prove the framework without risking physical hardware, the system is rigorously tested in a simulated high-slip environment:

1. **Environment Generation:** High-resolution digital elevation models (DEM) of agricultural fields are processed using `Forest3D`. The resulting Gazebo world is populated with procedural vegetation to create realistic visual occlusions.
2. **Physics Injection:** The `gz-terramechanics` plugin is applied to the soil. As the simulated Sowbot executes skid-steer maneuvers, the tracks calculate real-time sinkage and shear, causing the simulated wheel encoders to spike out of sync with actual displacement.
3. **Filter Intercept:** `FusionCore` monitors the divergence between the VIO velocity and the track encoders. As the slip crosses the Mahalanobis chi-squared threshold, the filter drops the encoder weights, relying entirely on the visual map and IMU dead-reckoning to maintain an accurate trajectory.

---

### 4. Bibliography and Prior Work

The foundational frameworks enabling this research are rooted in the authors' prior active developments in state estimation and simulation:

**FusionCore & State Estimation (Manan Kharwar)**

* Kharwar, M. (2026). *FusionCore: A 23-State Unscented Kalman Filter for IMU, Wheel Encoder, GPS, and Visual SLAM Fusion in ROS 2.* arXiv preprint arXiv:2605.25239. [Details the core UKF math, online bias estimation, and adaptive noise covariance used to intercept the VIO data].

**Terramechanics & Environment Generation (Khalid Bourr)**

* Scalera, L., Maset, E., Fasiolo, D. T., Bourr, K., Cottiga, S., De Lorenzo, A., Carabin, G., Alberti, G., Gasparetto, A., Mazzetto, F., & Seriani, S. (2026). *Forest Surveying with Robotics and AI: SLAM-Based Mapping, Terrain-Aware Navigation, and Tree Parameter Estimation.* Machines.
* Bourr, K. (2026). *Forest3D - Terrain and Forest Generation for Gazebo.* unitsSpaceLab, GitHub. [Automated generation of Gazebo environments from DEM files and Blender assets].
* Bourr, K. (2026). *gz-terramechanics.* unitsSpaceLab, GitHub. [Gazebo Sim physics plugin computing real-time wheel/track sinkage, traction forces, and driving torques].

**Visual-Inertial Odometry (Suyash & ETH Zürich)**

* Bloesch, M., Omari, S., Hutter, M., & Siegwart, R. (2015). *Robust visual inertial odometry using a direct EKF-based approach.* IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). [The foundational math modernized into ROS 2 by Suyash].
