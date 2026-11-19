export const topView = String.raw`
  <h3>Top-view steering model</h3>
  <h4>Coordinates and upright pickups</h4>
  <p>Vehicle X points forward, Y left and Z up. The front axle is at X = 0 and the rear axle at X = −wheelbase. Dimensions are in metres; rack travel and stroke limits are in millimetres. The right upright pickup mirrors the left pickup about Y = 0.</p>
  <p>The upright pickup height follows the line from the centered rack end to the same-side front-view instant center. Both parameter and hardpoint modes use the current FV geometry.</p>
  <p>\[Z_T=Z_R+(Y_T-Y_R)\frac{Z_{IC}-Z_R}{Y_{IC}-Y_R}\]</p>
  <p>A finite instant center and nondegenerate projection are required. The zero-travel FV construction sets the pickup height. Full three-dimensional bump steer still requires verification over suspension travel.</p>
  <h4>Rack motion and fixed-length tie rods</h4>
  <p>The rack is parallel to Y with neutral endpoints (X_R, ±length/2, Z_R). Both endpoints translate by the same signed displacement along Y. Each rod length is fixed from its centered, straight-ahead pose. Wheels rotate about their actual LBJ–UBJ kingpin axes using Rodrigues rotation. The solver retains the assembly branch continuously reachable from zero.</p>
  <p>\[\lVert T(\theta)-R(s)\rVert=\ell_T,\qquad T(\theta)=O+\operatorname{Rot}(\hat k,\theta)(T_0-O)\]</p>
  <p>Positive rack travel moves left. Left and right stops are separate nonnegative magnitudes. Commands outside those stops and mechanically unreachable positions produce an error. Rear steering defaults to disabled with rack displacement fixed at zero. Enable it and apply inputs before moving the rear rack.</p>
  <h4>Wheel angles and Ackermann</h4>
  <p>Wheel steering is the heading of the steered wheel plane projected onto the horizontal plane. Positive headings point left. The result table also records rotation about the kingpin. Each axle evaluates Ackermann with the other axle straight.</p>
  <p>\[A=100\frac{\cot|\delta_{out}|-\cot|\delta_{in}|}{t/L}\]</p>
  <p>Here t is the selected axle track and L the wheelbase. 100% is ideal Ackermann, 0% parallel steering, and a negative value anti-Ackermann. Straight-ahead poses and wheel pairs that do not define a common turn have undefined Ackermann.</p>
  <h4>Camber and local gains</h4>
  <p>Camber is positive top-out. The static suspension pose includes the configured installation camber. Camber change is the steered camber minus that initial value.</p>
  <p>Steering camber gain (deg/deg) is dCamber/dwheel-heading at fixed suspension height. Bump camber gain (deg/mm) holds both current kingpin angles fixed, jounces one wheel by ±0.01 mm and measures camber in the FV transverse contact frame. It matches the FV convention at zero steering. The fixed-angle derivative excludes steering changes caused by tie-rod closure during bump.</p>
  <h4>Charts and LapSim samples</h4>
  <p>Chart curves use rack travel on the horizontal axis, include both stroke limits and zero, and retain gaps for unreachable positions. Current-pose markers follow the slider. Pose CSV and Sweep CSV record both wheels and their gains.</p>
  <p>LapSim imports use steer_FL_deg / steer_FR_deg for the front axle and steer_RL_deg / steer_RR_deg for the rear axle. Rack travel is estimated by fitting both measured angles to the nearest reachable model angle pair. Steering dots retain recorded angles; Ackermann dots use the recorded pair. Camber and gain dots interpolate static model values at the estimated rack position. Missing or unmatched samples are skipped. Body roll, pitch and suspension travel are outside this static TV overlay.</p>
  <h4>Project and STEP</h4>
  <p>Apply TV inputs before downloading project JSON. The same settings round-trip between desktop and browser. Static Full Car displays four applied upright-to-rack tie rods at centered rack position. Its STEP export includes these links as polylines or solid cylinders using the selected tube diameter.</p>
`;
