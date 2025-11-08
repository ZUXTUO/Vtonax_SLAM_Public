# Vtonax System Design and Implementation

[中文](WIKI.md) | [English](WIKI_EN.md) | [日本語](WIKI_JA.md)

## 1. Introduction

The Vtonax system is an augmented reality application project for the Android platform. The core objective of this system is to implement efficient real-time camera tracking and sparse mapping on mobile devices, including monocular ORB-SLAM2-based simultaneous localization and mapping, stable plane detection and AR object placement, persistable map management, and optional lightweight object detection capabilities (based on the ncnn framework's YOLOv5 algorithm). The system design fully considers the hardware characteristics of mobile devices, achieving a good balance between real-time performance, stability, and ease of use.

## 2. System Architecture

This system adopts a layered architecture design, mainly including the following core components:

### Application Layer (Android)
Responsible for camera drivers, user interface controls, rendering control, user interaction, and bidirectional communication with native libraries (JNI).

### Native Layer (C++)
Integrates the core algorithm based on modified ORB-SLAM2 (including tracking, local mapping, loop closure, map management modules), DBoW2 bag-of-words model, g2o optimization framework, YOLOv5 inference engine (based on ncnn framework), and several utility modules (matrix operations, point cloud rendering, resource reading, plane detection, etc.).

### Resource Management Layer
The system's core dependency resources (including camera configuration, bag-of-words model, detection model parameters, etc.) are embedded into the native library during the build phase and read directly from memory at runtime, effectively reducing external file dependencies and path configuration complexity.

### Rendering Layer
Composed of "background camera frame layer" and "foreground AR object layer" superimposed together, ensuring real-time performance and visual clarity respectively.

## 3. System Operation Flow

The typical operation flow of this system is as follows:

1. The application starts and requests necessary system permissions (camera, storage, etc.).

2. Opens the rear camera, continuously acquires image frames and converts them to the format required for system processing (grayscale and color images).

3. Passes image frames to the native layer through the JNI interface: executes SLAM tracking and mapping tasks; performs asynchronous inference if object detection is enabled; maintains core data such as camera pose, keypoints, map points, and alignment status.

4. Rendering layer processing: background layer draws real-time camera frames; if rendering conditions are met (tracking status is normal, plane detection is valid, alignment strategy is satisfied), the foreground layer draws AR objects.

5. Map management: users can save or load maps at any time, implementing cross-session relocalization and scene recovery functions.

## 4. System Features

This system has the following core features:

### Real-time Monocular SLAM
Implements camera pose estimation, local mapping, loop detection, and graph optimization.

### Plane Detection and AR Placement
Estimates plane parameters based on available poses and map points for AR object placement and coordinate alignment.

### AR Rendering
Draws 3D objects based on model, view, and projection matrices, supporting gesture scaling and basic pose correction.

### Map Persistence
Saves key information of keyframes and map points in custom binary format, supporting cross-session relocalization.

### Object Detection (Optional)
YOLOv5 detection algorithm based on ncnn framework, using asynchronous and downsampling strategies to balance performance.

### Point Cloud Visualization
Displays global map points and current tracking keypoints for observing tracking and alignment quality.

### Reliability Enhancement
Performs "lightweight reset" after long-term tracking loss, retaining map data to accelerate recovery.

### SLAM Switch
Can enable/disable SLAM functionality at runtime; when disabled, pose updates and AR rendering decisions stop.

### Map Statistics Display
Periodically displays keyframe count, map point count, and plane existence status for observing mapping and stability.

## 5. Core Algorithm Implementation

### 5.1 SLAM System Coordination Mechanism

The SLAM system adopts a multi-threaded coordination mechanism to ensure efficient collaboration among modules.

#### Bag-of-Words and Keyframe Database
The system loads pre-trained bag-of-words models to support place recognition and relocalization; the keyframe database is used for fast retrieval of similar frames to improve matching efficiency.

#### Map Data Structure
The system adopts a unified data structure to manage keyframes and map points, supporting statistics, insertion/deletion, and retrieval operations.

#### Three-Thread Collaboration Model

- **Tracking Thread**: Performs pose estimation for each image frame, creating new keyframes when necessary.
- **Local Mapping Thread**: Maintains and optimizes the local map to ensure map quality.
- **Loop Closing Thread**: Identifies loops and performs graph optimization to effectively suppress accumulated drift.

#### Mode Switching Mechanism
The system supports entering "localization-only mode", pausing mapping to improve stability, especially suitable for pure localization scenarios after loading existing maps.

### 5.2 Tracking and Alignment Strategy

#### Pose Estimation
Obtains current camera pose through feature matching and optimization algorithms; for real-time performance, the system adopts reasonable downsampling strategies on the input side.

#### Map Alignment
When a historical map is loaded, the system needs to align the current SLAM coordinate system with the loaded map coordinate system. After successful alignment:

- Global point cloud is projected with aligned poses, providing stable visual reference;
- AR rendering uses aligned view matrices, ensuring consistency of objects and point clouds in the same coordinate system.

#### Rendering Prerequisites
AR objects are drawn only when "tracking status is normal + plane detection is valid + alignment conditions are met (depending on plane source)".

### 5.3 Plane Detection and AR Matrix Calculation

#### Plane Source Classification

- **On-site Detection**: Estimates plane parameters based on current pose and neighboring map points;
- **Map Recovery**: Recovers plane parameters from saved AR auxiliary information (requires attention to map alignment relationship).

#### Matrix Generation Mechanism

- **Model Matrix**: Describes the placement and pose of AR objects relative to the plane;
- **View Matrix**: Calculated based on (aligned when necessary) camera pose;
- **Projection Matrix**: Derived from camera intrinsics and output size, must maintain consistent relationship with tracking resolution.

#### Refresh Strategy
When a plane is detected or recovered from a map, the system immediately updates the model matrix and projection matrix; when alignment status changes, updates the view matrix.

### 5.4 Map Persistence Mechanism

#### Map Saving Objective
The system saves pose and feature information of keyframes, geometric and descriptor information of map points in custom binary format to implement cross-session relocalization.

#### Map Loading Process

1. **Pre-loading Cleanup**: Clears residual map content in memory to ensure a clean coordinate system environment;
2. **Property Recovery**: Recovers key properties of map points and sets reasonable visibility statistics to prevent mistaken deletion;
3. **Cache Reconstruction**: Rebuilds tracker reference cache to improve success rate and speed of subsequent relocalization;
4. **Mode Recovery**: Restores normal SLAM mode to continue refining or extending the map.

#### User Experience Objective
After loading, quickly complete alignment on-site, with green point cloud and AR objects displayed according to historical scene relationships.

### 5.5 Object Detection Mechanism (YOLOv5)

#### Operation Purpose
Overlay detection results on AR frames to enhance scene understanding and demonstration effect.

#### Performance Optimization Strategy

- **Thread Isolation**: Uses separate thread to carry inference tasks, avoiding blocking rendering and SLAM main threads;
- **Downsampling Processing**: Uses downsampled input to reduce inference latency;
- **Result Optimization**: Scales results back to original resolution, and maintains last result when no new result is available to avoid flickering.

#### Compatibility Strategy
The system enables object detection only when device support conditions are good (graphics capability, memory and CPU cores, system version), weak devices default to hiding or disabling this feature.

### 5.6 Rendering Layer and Interaction Design

#### Background Frame Rendering
Draws real-time camera frames in full-screen orthographic mode, prioritizing smoothness and low latency.

#### Foreground AR Object Rendering

- **Conditional Drawing**: Draws when rendering conditions are met;
- **Gesture Interaction**: Supports gesture scaling with fixed pose correction for more natural display;
- **Coordinate Consistency**: Shares the same coordinate system with point cloud display, ensuring visual consistency.

#### Point Cloud Display Mechanism

- **Global Point Display**: Global points displayed after alignment provide spatial reference;
- **Tracking Keypoint Display**: Tracking keypoints reflect current frame matching quality, facilitating system status observation.

#### Interaction Supplement Mechanism
AR object scaling uses cumulative scaling with minimum/maximum boundaries to avoid distortion; supports tap to trigger camera autofocus to improve tracking quality.

### 5.7 Thread Model and Performance Optimization

- **Tracking Thread**: Runs with high priority and high frequency, downsampling input when necessary to improve throughput.
- **Local Mapping and Loop Closing Threads**: Maintain map quality and correct drift errors in the background.
- **Object Detection Thread**: Uses condition triggering and waiting mechanisms to reduce busy waiting and resource contention.
- **Rendering Thread**: Maintains user interface smoothness, avoids heavy computation in main thread, schedules important operations like initialization sequentially to reduce stuttering.

### 5.8 Tracking Pipeline Details

#### Feature Extraction
Extracts robust corner points and generates compact descriptors in the current frame for establishing frame-to-frame and map-to-frame matching relationships.

#### Initial Matching
Prioritizes establishing matching relationships with the previous frame or local reference keyframe to quickly provide initial values.

#### Motion Model and Prior Information
Combines previous frame pose and simple motion assumptions to provide initial pose predictions, improving optimization convergence speed.

#### Geometric Consistency Constraints
Filters incorrect matches through projection geometry, scale, and observation visibility constraints, robustly estimating pose.

#### Pose Optimization
Non-linear optimization with reprojection error as objective, outputs current frame pose and valid observation set after removing outliers.

#### Keyframe Decision
Decides whether to insert new keyframes based on parallax, observation coverage, tracking quality metrics to maintain map density and tracking stability.

### 5.9 Local Mapping Core Responsibilities

#### Map Point Generation and Fusion
Uses parallax and triangulation of new keyframes to generate new map points, fuses with duplicate points from adjacent keyframes to suppress redundancy.

#### Local Optimization
Performs local bundle adjustment on the current local map (current keyframe, covisible keyframes, related map points) to improve geometric consistency.

#### Map Point Culling
Removes unstable points based on visibility rate, matching success rate, and observation angle metrics, maintaining map's "refined and stable" characteristics.

### 5.10 Loop Detection and Global Consistency Optimization

#### Loop Candidate Retrieval
Searches for similar candidates in the keyframe database based on bag-of-words model, preliminarily evaluating similarity.

#### Loop Verification
Verifies candidates through geometric consistency and pose relationships to avoid mismatches caused by pure appearance errors.

#### Graph Optimization
Performs pose graph optimization and necessary global bundle adjustment after introducing loop constraints, globally suppressing drift and reshaping consistent coordinate system.

### 5.11 Map Alignment Mechanism

#### Trigger Condition
When a historical map is loaded, the system attempts to map the current SLAM coordinate system to that map's coordinate system.

#### Alignment Input
Correspondence between observations in the current frame or short time window and known structures (keyframes/map points) in the loaded map.

#### Solution Objective
Estimates a set of rigid transformation parameters to minimize projection error of current observations in the loaded map coordinate system.

#### Usage

- Once alignment is successful, rendering and point cloud display both use aligned camera poses;
- When alignment fails, continues attempting until confidence threshold is met.

### 5.12 Plane Detection Algorithm

#### Input Source
Sparse map points near the current pose and their spatial distribution.

#### Model Assumption
A local approximate plane exists that can explain the spatial positions of a batch of points.

#### Estimation Process
Fits the plane of the maximum consensus set in the 3D point set through robust estimation methods, calculating normal vector and reference point.

#### Stability Strategy
When the map changes significantly or tracking status fluctuates, delays or repeats estimation to obtain more stable plane parameters.

### 5.13 AR Matrix Calculation and Consistency Guarantee

#### Model Matrix
Transforms "object local coordinates" to "plane/world coordinates", embodying position, orientation, and scaling relationships.

#### View Matrix
Converts "world coordinates" to "camera coordinates", determined by current camera pose (preferably using aligned pose).

#### Projection Matrix
Projects "camera coordinates" to "screen coordinates", constructed from intrinsics and viewport parameters, requiring consistent relationship with processing resolution.

#### Consistency Points
Point cloud, AR objects, and camera frames must be based on the same coordinate transformation chain; any inconsistent coordinate system selection will cause misalignment.

### 5.14 YOLO Asynchronous Processing Mechanism

#### Producer-Consumer Model
Rendering/SLAM threads submit downsampled images at a rhythm; detection thread wakes up to perform inference when new tasks arrive.

#### Synchronization and Sharing Mechanism
Uses semaphores/condition variables to avoid busy waiting; shares detection results with timestamps, rendering side retrieves as needed.

#### Result Stability Guarantee
Reuses old results when no new result is available to avoid detection box flickering; can set result expiration policy when necessary to balance timeliness.

#### Resource Control Strategy
Disables detection when device capability is insufficient, or appropriately reduces input size and detection frequency to reduce interference with SLAM.

### 5.15 System Failure Handling and Recovery Mechanism

#### Short-term Tracking Failure
Attempts quick recovery through motion model and local matching; if recovery fails, enters "lost" state timing.

#### Lost Timeout Handling
Executes "lightweight reset", only clears tracking state while retaining map, then continues attempting relocalization based on loaded map.

#### Alignment Instability Handling
When alignment confidence is insufficient, does not use aligned pose for AR rendering to avoid visual misalignment; continues accumulating observations until threshold is met.

#### Plane Instability Handling
Postpones plane updates when map is obviously updated or observations are insufficient, refreshes AR model matrix after status stabilizes.

## 6. System Reliability and Fault Tolerance Design

### Lost Recovery Mechanism
When the system is in a lost state for a long time, executes "lightweight reset" operation, only clears tracking state but retains loaded map for faster recovery.

### Plane Consistency Guarantee

- For "planes recovered from map", strictly depends on map alignment;
- For "on-site detected planes", can be used before alignment, but does not affect aligned logic.

### Map Safety Guarantee
Performs reasonable boundary checks on loaded map scale to avoid memory pressure or system instability.

### Rendering Fault Tolerance Mechanism
Does not draw AR objects when system status or matrices do not meet requirements, avoiding screen flashing and misalignment.

### Reset Detail Handling
When executing "lightweight reset", retains loaded map and planes recovered from map, only clears manually detected planes and temporary AR state to reduce visual mutation.

## 7. System Configuration and Resource Management

### Resource Embedding Mechanism
Camera configuration, bag-of-words, detection models, etc. are packaged in binary form into the native library, read via memory mapping at runtime, avoiding path and permission issues.

### Camera Intrinsic Configuration
Used for projection matrix derivation, must maintain consistent relationship with image processing resolution.

### Digital Watermark Function
Controls whether to overlay through switch, supports adaptive scaling and transparent compositing according to frame size.

## 8. System Usage Boundaries and Prerequisites

### Lighting, Texture, and Motion Conditions
Good texture and stable camera motion can significantly improve tracking stability; excessive motion blur or repetitive textures will reduce effectiveness.

### Device Performance Requirements
Memory, CPU cores, graphics capability, and system version significantly affect real-time performance and whether optional features (such as object detection) are enabled.

### Data Security Requirements
Persistent maps and auxiliary information are local files and must comply with platform permissions and user data security requirements.

## 9. Terminology

- **Pose**: The position and orientation of the camera in space.
- **Keyframe**: Representative frames selected from a time sequence, used for map construction and optimization.
- **Map Points**: A collection of sparse feature points in 3D space.
- **Alignment**: Establishing a stable consistent relationship between the current SLAM coordinate system and the loaded map coordinate system.
- **Projection Matrix**: Calculated from camera intrinsics and viewport parameters, used to transform 3D points to screen plane.

## 10. Conclusion

The Vtonax_SLAM system integrates monocular SLAM, plane detection, AR rendering, and optional object detection on mobile devices. Through clear rendering conditions and alignment strategies, resource embedding and thread parallelism mechanisms, and reliability designs such as lightweight reset and reference cache reconstruction, this system achieves a good balance between real-time performance and stability on ordinary mobile devices, suitable for application scenarios such as AR placement, indoor visual localization, and lightweight perception fusion.

## 13. Mathematics and Optimization Key Points

### 13.1 Camera Model and Projection Transformation

#### Camera Intrinsic Matrix
Contains focal length (fx, fy) and principal point (cx, cy) parameters, used to project 3D space points to 2D pixel coordinates.

#### World to Camera Coordinate Transformation
Transforms world point Xw to camera coordinates Xc through rotation matrix R and translation vector t.

#### Projection Process
First normalizes camera coordinates (Xc/Zc, Yc/Zc), then maps to pixel coordinate system through intrinsic matrix.

#### Monocular Scale Ambiguity
Due to lack of depth information in monocular cameras, scale drift needs to be resolved through keyframe graph optimization and loop closure detection.

### 13.2 Reprojection Error and Robust Optimization

#### Reprojection Residual Calculation
Difference between observed value and predicted projection value, used to measure pose estimation accuracy.

#### Optimization Objective Function
Minimizes weighted sum of reprojection errors of all observation points, using robust loss functions to reduce outlier influence.

#### Pose Optimization Methods

- **PNP Algorithm**: Solves camera pose given known 3D points and corresponding 2D projections
- **LM Algorithm**: Uses damping strategy to improve nonlinear optimization stability
- **BA Optimization**: Simultaneously optimizes camera poses and 3D point coordinates for global optimal solution

### 13.3 Keyframe Insertion and Local Bundle Adjustment

#### Keyframe Insertion Criteria

- **Parallax Threshold**: Current frame and reference keyframe should have sufficient parallax
- **Coverage**: New frame should observe enough local map points
- **Tracking Inlier Rate**: Tracking quality must meet minimum requirements

#### Local Bundle Adjustment (Local BA)

- **Optimization Range**: Subgraph consisting of current keyframe, covisible keyframes, and related map points
- **Optimization Objective**: Minimize all reprojection errors within subgraph
- **Solution Strategy**: Uses sparse incremental solution method to balance computational efficiency and accuracy

### 13.4 Loop Closure Detection and Pose Graph Optimization

#### Loop Closure Detection Flow

- **Candidate Retrieval**: Searches for similar keyframes in keyframe database based on bag-of-words model
- **Geometric Verification**: Confirms true loop closure relationships through geometric consistency checks
- **Constraint Addition**: Adds loop closure constraints to pose graph structure

#### Pose Graph Optimization

- **Optimization Objective**: Minimizes errors of relative transformations between all poses
- **Information Matrix**: Represents confidence of each edge measurement
- **Post-optimization Processing**: Can attach global BA to further optimize map quality

### 13.5 Cross-Session Map Alignment

#### Alignment Trigger Condition
After loading a historical map, the current SLAM coordinate system needs to be aligned with the loaded map coordinate system.

#### Alignment Algorithm Principle

- **Input Data**: Correspondence between current observations and known structures in loaded map
- **Optimization Objective**: Find optimal similarity transformation to minimize projection error of corresponding points in target coordinate system
- **Solution Method**: Uses orthogonal Procrustes problem or Umeyama algorithm, including rotation, translation, and scale transformation

#### Alignment Result Application

- **After Successful Alignment**, rendering and point cloud display both use aligned coordinate system
- **When Alignment Fails**, continues attempting until confidence threshold is met

### 13.6 Plane Estimation Method

#### Plane Representation
Represents plane using normal vector n and distance d from plane to origin, satisfying constraint ||n||=1.

#### Plane Fitting Algorithm

- **RANSAC Algorithm**: Estimates plane model through random sampling of minimum point set
- **Inlier Selection**: Filters points conforming to plane model based on orthogonal distance threshold
- **Optimization Process**: Minimizes sum of squared orthogonal distances for inlier set to obtain optimal plane parameters

#### Plane Parameter Calculation

- **Normal Vector Calculation**: Through eigenvalue decomposition of covariance matrix, normal vector corresponds to eigenvector of smallest eigenvalue
- **Distance Calculation**: Calculates distance from plane to origin through normal vector and plane center point

### 13.7 Projection/View/Model Matrix and Coordinate System Transformation

#### Model Matrix
Describes transformation from object local coordinates to world coordinates, including rotation, translation, and scaling.

#### View Matrix
Converts world coordinates to camera coordinates, calculated based on current camera pose.

#### Projection Matrix
Maps camera coordinates to screen coordinates, constructed from camera intrinsics and viewport parameters.

#### Coordinate Consistency Requirement
Point cloud display, AR object rendering, and camera frames must use the same coordinate transformation chain to ensure visual consistency.

### 13.8 Multi-threaded Inference Modeling for Object Detection

#### Detection Function Model
Maps input image to object set (category, confidence, bounding box).

#### Multi-threaded Architecture

- **Producer**: Rendering/SLAM threads submit downsampled images at fixed beats
- **Consumer**: Detection thread performs inference computation when new tasks arrive
- **Synchronization Mechanism**: Uses semaphores and condition variables to avoid busy waiting, ensuring efficient thread collaboration

#### Result Processing Strategy

- **Stability Guarantee**: Reuses last detection result when no new result available, avoiding detection box flickering
- **Scale Mapping**: Maps detection results from downsampled image scale back to original image scale

### 13.9 System Failure and Recovery Mechanism

#### State Definition
System includes uninitialized, tracking normal, and lost states.

#### Recovery Mechanism

- **Short-term Failure**: Attempts quick recovery through motion model and local matching
- **Long-term Lost**: Executes lightweight reset after threshold time, only clears tracking state while retaining map
- **Relocalization**: Performs relocalization matching based on loaded map

### 13.10 Lie Group Parameterization and Incremental Update

#### Pose Parameterization
Uses Lie group SE(3) or similarity transformation group Sim(3) to represent camera pose.

#### Optimization Process

- **Lie Algebra Representation**: Performs optimization computation in Lie algebra space
- **Exponential Map**: Updates pose through exponential mapping of Lie algebra increment

#### Linearization Processing

- **Residual Approximation**: Approximates residual function through first-order Taylor expansion
- **Normal Equation**: Constructs and solves normal equation based on linearization result

### 13.11 Normal Equation and Schur Complement

#### Weighted Least Squares
Constructs weighted normal equation to handle confidence differences of different observations.

#### Schur Complement Method

- **Matrix Partitioning**: Partitions Hessian matrix into pose and structure parts
- **Elimination Process**: Eliminates structure variables to reduce system dimension
- **Solution Order**: First solves pose increment, then back-substitutes to solve structure increment

#### Optimization Effect
Significantly reduces linear system scale, improving solution efficiency.

### 13.12 Robust M-estimation and Kernel Function

#### Robust Loss Function

- **Weight Function**: Implements robust loss through weight function, reducing outlier influence
- **Huber Kernel Function**: Uses quadratic growth for small residuals, linear growth for large residuals

#### Iteratively Reweighted

- **IRLS Algorithm**: Updates weight matrix based on current residuals in each iteration
- **Convergence**: Repeats iteration until optimization problem converges

### 13.13 RANSAC Inlier Set and Robust Geometric Estimation

#### RANSAC Algorithm Flow

- **Random Sampling**: Randomly selects minimum point set to estimate geometric model
- **Inlier Evaluation**: Evaluates inlier property of all points through cost function
- **Model Selection**: Selects model with most inliers or minimum cost

#### Refinement Optimization

- **Weighted Optimization**: Performs weighted refinement optimization on filtered inlier set
- **Precision Guarantee**: Balances algorithm robustness and estimation accuracy

### 13.14 Similarity Transformation (Sim(3)) Closed-form Estimation

#### Optimization Objective
Finds optimal similarity transformation (including rotation, translation, and scale) to minimize error of corresponding point pairs.

#### Solution Process

- **De-meaning Processing**: Performs de-meaning preprocessing on point sets
- **Covariance Calculation**: Calculates covariance matrix between point sets
- **SVD Decomposition**: Solves optimal rotation matrix through singular value decomposition
- **Parameter Calculation**: Sequentially calculates scale factor and translation vector

#### Degeneracy Handling
Checks condition number and rank for degenerate cases like collinear or coplanar to ensure solution stability.

### 13.15 Constraints, Gauge Handling, and Boundary Processing

#### Monocular System Constraints

- **Gauge Freedom**: Monocular SLAM has gauge freedom in scale and overall pose
- **Constraint Method**: Eliminates null space by fixing first frame or adding prior constraints

#### Map Alignment Handling

- **Coordinate Anchoring**: After map loading alignment, maintains consistency with target coordinate system as anchor
- **Graph Optimization**: Applies strong prior or fixing to anchor nodes in graph optimization

### 13.16 Numerical Stability and Convergence Analysis

#### Stability Strategy

- **Damping Method**: Uses damping strategies like LM to improve stability in highly nonlinear regions
- **Initial Value Selection**: Uses motion model or local matching to provide good initial values to accelerate convergence

#### Degeneracy Identification

- **Monitoring Metrics**: Checks Hessian matrix condition number, eigenvalue spectrum, and inlier ratio
- **Adaptive Adjustment**: Postpones keyframe insertion or adjusts optimization parameters based on monitoring results
