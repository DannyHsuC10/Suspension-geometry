const frontOptimization = `
  <h4>What Optimize roll center checks</h4>
  <p>The front search tests increasing design heights from the chosen lower bound in 1 mm steps. It rebuilds the geometry and solves every combination of sampled left and right travel. The first accepted height keeps the roll center above the current contact line throughout the checked grid. The check also rejects unreachable travel, parallel arm or contact lines, and determinant sign changes that indicate a singularity between samples.</p>
`;

const rearOptimization = String.raw`
  <h4>How Optimize roll center balances the axles</h4>
  <p>The rear search holds the front design fixed and targets equal transfer of vertical load from the inside to the outside of each axle at zero travel. Let \(h_F,h_R\) be roll center heights above their own contact lines, \(t_F,t_R\) the tracks, and \(w_F,w_R\) the static weight fractions. With \(k=K_{\phi F}/K_{\phi R}\) from <strong>Front / rear roll stiffness ratio</strong>, the fractions of total elastic roll moment are \(q_F=k/(1+k)\) and \(q_R=1/(1+k)\).</p>
  <p>If installed camber puts the front and rear zero-travel contacts at heights \(z_{C,F}\) and \(z_{C,R}\), the contact datum at the CG is their weight-fraction interpolation. The CG above that datum and its lever about the roll axis are</p>
  \[
  h_G^{*}=h_G-w_F z_{C,F}-w_R z_{C,R},
  \qquad \ell=h_G^{*}-w_Fh_F-w_Rh_R.
  \]
  <p>For lateral acceleration \(a_y\), the model adds geometric and elastic transfer at each axle \(a\in\{F,R\}\).</p>
  \[
  \frac{\Delta F_{z,a}}{m a_y}=\frac{w_a h_a+q_a\ell}{t_a}.
  \]
  <p>Set \(\Delta F_{z,F}=\Delta F_{z,R}\) and let \(A=w_F/t_F\), \(B=w_R/t_R\), and \(D=q_F/t_F-q_R/t_R\). Solving for the rear contact-frame height gives</p>
  \[
  h_R=\frac{A h_F+D(h_G^{*}-w_F h_F)}{B+D w_R}.
  \]
  <p>The rear input is the zero-camber <em>design</em> height. Inverting the installed contact equation from step 6 with the rear values gives \(h_0=\frac{b}{F}\bigl[z_C+\frac{x_C-x_I}{x_C}h_R\bigr]\), where \(x_I=x_{I_0}\). The tool rounds it to 1 mm, checks the search limits, then checks both axles over the requested travel grid before applying it. This quasi-static model treats the mass as sprung and omits aero and tire load sensitivity. Its target is a 50/50 axle load-transfer split.</p>
`;

function frontView(axle, trackName, weightFraction, plane, example, optimization) {
  return String.raw`
  <h3>Build the ${axle} FV linkage</h3>
  <p>Use <strong>Vehicle settings</strong> for the shared mass, CG and wheel properties, and the left <strong>${axle} axle design</strong> section for the projection. Select <strong>Apply inputs</strong>, then <strong>Zero pose</strong> to compare the chart with the static construction below. The upper and lower offsets are vertical distances from the wheel center. The chassis <strong>|Y|</strong> entries are lateral distances from the centerline.</p>
  <p>Use metres in the equations. The axle body plane has coordinates \((x,z)\), with \(x\) increasing from the vehicle’s left to right and \(z\) upward. Its origin is the zero-camber ground line at the centerline. In the full-car frame, where \(X\) points forward and \(Y\) points left, \((x,z)\) maps to \((X_a,-x,z)\), with ${plane}. The solver constructs the right side first.</p>
  <ol>
    <li>
      <h4>Place the contact and wheel center</h4>
      <p>Set \(t\) to <strong>${trackName}</strong> and \(b=t/2\). From <strong>Wheelbase</strong> \(L\) and <strong>CG to front axle</strong> \(d_F\), the static front and rear weight fractions are \(w_F=1-d_F/L\) and \(w_R=d_F/L\). For this axle, use ${weightFraction}. With total mass \(m\), gravity \(g\), unloaded tire radius \(r_0\), and vertical stiffness \(k_t\), the loaded radius \(r\) accounts for half of the axle load at each wheel.</p>
      \[
      F_{z,\mathrm{wheel}}=\frac{mgw}{2},
      \qquad r=r_0-\frac{F_{z,\mathrm{wheel}}}{k_t},
      \qquad C_0=(b,0),\quad W_0=(b,r).
      \]
      <p>\(C_0\) is the right construction contact and \(W_0\) the right wheel center. The wheel outline uses the unloaded diameter \(2r_0\) and the section width \(w_t\). Its unrotated corners are \(W_0+(\pm w_t/2,\pm r_0)\). The unloaded outline extends below \(z=0\) by the calculated static deflection \(r_0-r\).</p>
    </li>
    <li>
      <h4>Locate the kingpin and ball joints</h4>
      <p>Enter <strong>Scrub radius</strong> \(s\) and <strong>Kingpin inclination</strong> \(\kappa\). Positive scrub is the outward distance from the right kingpin ground intercept \(K_0\) to the wheel contact. Equivalently, \(x_{C_0}=x_{K_0}+s\). A positive KPI inclines the top of the right kingpin inward.</p>
      \[
      K_0=(b-s,0),\qquad x_{\mathrm{kp}}(z)=b-s-z\tan\kappa.
      \]
      <p>Enter the <strong>Upper joint offset from wheel center</strong> \(A_U\) and the <strong>Lower joint offset from wheel center</strong> \(A_L\). Place the upper joint \(U_0\) and lower joint \(B_0\) on the kingpin at the corresponding heights.</p>
      \[
      z_U=r+A_U,\qquad z_B=r-A_L,
      \qquad U_0=(x_{\mathrm{kp}}(z_U),z_U),
      \qquad B_0=(x_{\mathrm{kp}}(z_B),z_B).
      \]
      <p>The upright is \(B_0U_0\). The kingpin reference line passes through \(K_0,B_0,U_0\).</p>
    </li>
    <li>
      <h4>Construct the design instant center</h4>
      <p>Enter <strong>Roll center height</strong> \(h_0\) and <strong>Front-view swing arm (FVSA)</strong> \(F\). The design roll center \(R_d=(0,h_0)\) fixes the contact-to-center line. Move a horizontal distance \(F\) inward from \(C_0\) along that line to place the instant center \(I_0\).</p>
      \[
      I_0=C_0+\frac{F}{b}(R_d-C_0)
        =\left(b-F,\frac{F}{b}h_0\right).
      \]
      <p>FVSA is a horizontal distance in this projection. An FVSA greater than \(b\) puts the right-side instant center across the vehicle centerline.</p>
    </li>
    <li>
      <h4>Find the inboard pivots and draw the links</h4>
      <p>Enter <strong>Upper arm chassis |Y|</strong> \(m_U\) and <strong>Lower arm chassis |Y|</strong> \(m_B\). Intersect each selected lateral coordinate with the line from its ball joint to \(I_0\). For \(j\in\{U,B\}\) and \(J_j=(x_j,z_j)\), the inboard pivot is</p>
      \[
      P_j=\left(m_j,\ z_j+(m_j-x_j)\frac{z_{I_0}-z_j}{x_{I_0}-x_j}\right).
      \]
      <p>The upper and lower arm segments join \(P_U\) to \(U_0\) and \(P_B\) to \(B_0\). Their extensions \(U_0 I_0\) and \(B_0 I_0\), the contact line \(C_0 I_0\), and the projected body segment \(P_U P_B\) complete the construction. A vertical joint-to-IC line cannot determine a unique height for a specified lateral coordinate.</p>
      <p>These pivots are the two-dimensional linkage points. The full-car model uses the FV values together with the side-view instant center, rim clearance, and the longitudinal mount settings to place each arm’s two physical chassis mounts.</p>
    </li>
    <li>
      <h4>Mirror the left side and recover the design roll center</h4>
      <p>Reflect every right-side point through \(x=0\) using \(M(x,z)=(-x,z)\). The left and right contact-to-IC lines intersect at \((0,h_0)\) when both lines define a unique intersection. The CG projects onto the same centerline at \(G=(0,h_G)\) from <strong>CG height</strong>.</p>
      <p>For any two lines \(P+\lambda u\) and \(Q+\mu v\) in this plane, the finite intersection is</p>
      \[
      P+\frac{[Q-P,v]}{[u,v]}u,
      \qquad [u,v]=u_xv_z-u_zv_x .
      \]
      <p>Use this result for the arm extensions to get each instant center and for the two contact lines to get the roll center. A zero determinant represents parallel or coincident lines, with no unique finite intersection.</p>
    </li>
    <li>
      <h4>Install static camber and establish the road datum</h4>
      <p>Enter <strong>Static camber</strong> \(\gamma\), with positive values tilting the top of each wheel outward. The solver rotates the right wheel outline by \(-\gamma\) about \(W_0\) and mirrors it for the left. The chassis pivots, ball joints, and instant centers keep their zero-camber construction positions. The bottom of the right wheel’s loaded-radius center circle becomes</p>
      \[
      C_\gamma=(b-r\sin\gamma,\ r(1-\cos\gamma)).
      \]
      <p>With \(C_\gamma=(x_C,z_C)\), solve the new mirrored contact-to-IC lines. Their intersection in body coordinates and its height above the installed contacts are</p>
      \[
      R_0=(0,z_C+h_{\mathrm{road}}),
      \qquad h_{\mathrm{road}}=\frac{x_C(z_{I_0}-z_C)}{x_C-x_{I_0}}.
      \]
      <p>At \(\gamma=0\), these reduce to the construction contact and \(h_0\) wherever the intersection is unique. <strong>RC0</strong> marks the zero-travel reference fixed to the body. The plot reports heights above the current contact line, so \(G\) at zero travel has chart height \(h_G-z_C\).</p>
    </li>
  </ol>

  ${example}

  <h4>How the chart moves away from zero travel</h4>
  <p>The lower joint rotates about \(P_B\) with fixed arm length \(\ell_B=\lVert B_0-P_B\rVert\). For lower-arm angle \(\theta\) it is \(B=P_B+\ell_B(\cos\theta,\sin\theta)\). The new upper joint is the circle intersection nearest \(U_0\), using fixed upper arm length \(\ell_U\) and upright length \(\ell_J\).</p>
  \[
  \lVert U-P_U\rVert=\ell_U=\lVert U_0-P_U\rVert,
  \qquad
  \lVert U-B\rVert=\ell_J=\lVert U_0-B_0\rVert.
  \]
  <p>Let \(\delta=\arg(U-B)-\arg(U_0-B_0)\) and \(R_2\) be the counterclockwise planar rotation matrix. The upright carries any original point \(Q\) to \(B+R_2(\delta)(Q-B_0)\), including \(K_0,C_0,W_0\). The right wheel angle is \(\theta_{w,R}=\delta-\gamma\). Its displayed contact is \(W+r(\sin\theta_{w,R},-\cos\theta_{w,R})\) for the current wheel center \(W\). For a travel input \(d\) in mm, the solver finds the linkage angle that moves the carried construction contact \(C_0\) vertically by \(d/1000\) m. The displayed contact then incorporates any installed camber. This computation is performed independently for left and right travel, reflecting the left solution.</p>
  <p>For the resulting contacts \(C_L,C_R\), choose the midpoint \(O=(C_L+C_R)/2\) and unit axes \(\hat e_x=(C_R-C_L)/\lVert C_R-C_L\rVert\) and \(\hat e_y=(-e_{x,z},e_{x,x})\). The chart and its reported roll center use \(x'=(Q-O)\cdot\hat e_x\) and \(y'=(Q-O)\cdot\hat e_y\). Intersect the current arm extensions to update each IC and intersect \(C_LI_L\) with \(C_RI_R\) to update the roll center.</p>
  <p>Roll is the contact line angle \(\rho=\arg(C_R-C_L)\). Heave is the average of the left and right travel inputs. The signed wheel cambers are \(\gamma_L=\theta_{w,L}-\rho\) and \(\gamma_R=-(\theta_{w,R}-\rho)\), converted to degrees. The sweep reports the local change in each wheel’s camber per millimetre of its own travel.</p>

  <h4>Hardpoint mode</h4>
  <p>In <strong>Vehicle settings</strong>, the upper and lower ball joints come directly from the supplied XYZ coordinates. For each arm, let \(H_F,H_R\) be its front and rear chassis mounts and \(J\) its ball joint. The solver intersects the real hinge line with the ball joint’s longitudinal station to define an equivalent pivot \(H\) for the front-view four-bar.</p>
  \[
  H=H_F+\frac{X_J-X_{H_F}}{X_{H_R}-X_{H_F}}(H_R-H_F),
  \qquad P=(-Y_H,Z_H),\qquad J_{\mathrm{FV}}=(-Y_J,Z_J).
  \]
  <p>Intersect the projected arm lines to derive the FV instant center and then the contact lines to recover its roll center. The kingpin comes from the supplied upper and lower ball joints and its intersection with \(Z=0\). Track and loaded radius still place the construction contact and wheel center. The hinge requires a nonzero longitudinal span for this projection.</p>
  ${optimization}
  `;
}

export const frontFV = frontView(
  "Front",
  "Front track",
  String.raw`\(w=w_F\)`,
  String.raw`\(X_F=0\)`,
  String.raw`
    <h4>Baseline front example</h4>
    <p>At \(t_F=1.250\), \(s=0.025\), \(\kappa=12^\circ\), \(F=1.000\), \(h_0=0.035\) and zero installed camber, the baseline right contact is \(C_0=(0.625,0)\), the kingpin intercept is \(K_0=(0.600,0)\), and \(W_0\approx(0.625,0.19573)\). The ball joints are \(U_0\approx(0.54033,0.28073)\) and \(B_0\approx(0.57753,0.10573)\). The IC is \(I_0=(-0.375,0.056)\). The chassis inputs \(m_U=0.255\) and \(m_B=0.210\) give \(P_U\approx(0.255,0.21067)\) and \(P_B\approx(0.210,0.08654)\). The mirrored force lines cross at \(R_d=(0,0.035)\).</p>
  `,
  frontOptimization,
);

export const rearFV = frontView(
  "Rear",
  "Rear track",
  String.raw`\(w=w_R\)`,
  String.raw`\(X_R=-L\)`,
  String.raw`
    <h4>Baseline rear example</h4>
    <p>At \(t_R=1.240\), \(s=0.018\), \(\kappa=13^\circ\), \(F=1.000\), \(h_0=0.040\) and zero installed camber, the baseline right contact is \(C_0=(0.620,0)\), the kingpin intercept is \(K_0=(0.602,0)\), and \(W_0\approx(0.620,0.19612)\). The ball joints are \(U_0\approx(0.53710,0.28112)\) and \(B_0\approx(0.57750,0.10612)\). The IC is \(I_0\approx(-0.380,0.06452)\). The chassis inputs \(m_U=0.250\) and \(m_B=0.205\) give \(P_U\approx(0.250,0.21331)\) and \(P_B\approx(0.205,0.08993)\). The mirrored force lines cross at \(R_d=(0,0.040)\).</p>
  `,
  rearOptimization,
);
