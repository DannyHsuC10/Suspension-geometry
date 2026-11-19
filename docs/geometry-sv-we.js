export const sideView = String.raw`
<h3>Side-view construction</h3>
<p>Select the mechanism chart, enable construction references and set Zero pose to check the static design. Use the front and rear controls to enter each axle's caster, mechanical trail, anti ratio, signed SVSA, and upper and lower joint offsets. Use Vehicle settings for mass, gravity and CG distance to the front axle. Apply the inputs before comparing points.</p>
<ol>
  <li>
    <h4>Place the axles and calculate the loaded wheel radii</h4>
    <p>The body-fixed side view uses \((s,z)\), with \(s\) increasing toward the rear and \(z\) increasing upward. For wheelbase \(L\), the front and rear axle stations are \(s_f=-L\) and \(s_r=0\). The corresponding vehicle coordinates are \(X=-L-s\), \(Y=0\), \(Z=z\). Let \(\ell_f\) be the distance from the front axle to the CG, and let \(h\) be CG height.</p>
    \[
      w_f=\frac{L-\ell_f}{L},\qquad w_r=\frac{\ell_f}{L},\qquad
      R_a=R_{\mathrm{free}}-\frac{Mg\,w_a}{2k_t},\quad a\in\{f,r\}.
    \]
    <p>Here \(M\) is total mass, \(g\) is gravity and \(k_t\) is tire vertical stiffness. Each axle load is split between its two tires. The design contact patch is \(P_{a0}=(s_a,0)\), the wheel center is \(C_{a0}=(s_a,R_a)\), and the CG is \((s_f+\ell_f,h)\). The drawn wheel circle uses \(R_{\mathrm{free}}\) about \(C_{a0}\). Its free-radius outline can extend below the design road.</p>
  </li>
  <li>
    <h4>Use mechanical trail and caster to draw the kingpin</h4>
    <p>Assign \(\sigma_f=+1\) and \(\sigma_r=-1\) so that a positive longitudinal offset points toward the other axle. Let \(\tau_a\) be mechanical trail and \(\chi_a\) be caster. The ground intercept \(G_a\), points \(K_a(z)\) on the kingpin and drawn axis tip \(K_{\mathrm{tip},a}\) are</p>
    \[
      \begin{gathered}
        G_a=(s_a-\sigma_a\tau_a,0),\\
        K_a(z)=(s_a-\sigma_a\tau_a+\sigma_a z\tan\chi_a,\ z),\\
        K_{\mathrm{tip},a}=G_a+0.60\,(\sigma_a\sin\chi_a,\ \cos\chi_a).
      \end{gathered}
    \]
    <p>Positive trail moves the axis ground intercept away from the opposite axle. Positive caster tips the top of the axis toward the opposite axle. With upper offset \(A_{u,a}\) and lower offset \(A_{l,a}\), the upright joins \(U_{a0}=K_a(R_a+A_{u,a})\) and \(B_{a0}=K_a(R_a-A_{l,a})\). The axis point at wheel-center height is \(K_a(R_a)\).</p>
  </li>
  <li>
    <h4>Place the instant centers and draw the force lines</h4>
    <p>Enter the signed side-view swing-arm length \(\ell_{\mathrm{SV},a}\) and the axle anti ratio \(q_a\). A value of \(q_a=0.20\) means a 20% unweighted axle anti ratio. The solver uses the contact patch as the origin of the force line. The signed swing-arm length sets its horizontal run toward the opposite axle, and anti determines the rise per unit run.</p>
    \[
      I_{a0}=P_{a0}+
      \left(\sigma_a\ell_{\mathrm{SV},a},\
        \ell_{\mathrm{SV},a}q_a\frac{h}{L}\right).
    \]
    <p>Draw the contact force line \(P_{a0}I_{a0}\) and the joint reference lines \(U_{a0}I_{a0}\) and \(B_{a0}I_{a0}\). Let \(J(s,z)=(-z,s)\) denote a quarter-turn. The fixed tangent at a joint \(Q_0\) points along \(d_Q=J(I_{a0}-Q_0)/\lVert I_{a0}-Q_0\rVert\). The static tangent reference passes through \(Q_0\) in this direction. The model requires nonzero SVSA and finite tangents.</p>
  </li>
  <li>
    <h4>Intersect the front and rear contact force lines</h4>
    <p>The static pitch center \(PC_0\) is the intersection of \(P_{f0}I_{f0}\) and \(P_{r0}I_{r0}\). In the body-fixed side view, their slopes are \(m_a=\sigma_a q_a h/L\). For distinct slopes, solve</p>
    \[
      s_{PC_0}=\frac{m_f s_f-m_r s_r}{m_f-m_r},\qquad
      z_{PC_0}=m_f(s_{PC_0}-s_f).
    \]
    <p>Parallel or coincident lines do not define a unique pitch center. With the baseline inputs \(L=1.53\,\mathrm m\), \(h\approx0.30165\,\mathrm m\), \(\ell_{\mathrm{SV},f}=\ell_{\mathrm{SV},r}=4\,\mathrm m\) and \(q_f=q_r=0.20\), the design instant centers are approximately \((2.470,0.15772)\,\mathrm m\) and \((-4.000,0.15772)\,\mathrm m\). Their contact force lines cross at \(PC_0\approx(-0.765,0.03016)\,\mathrm m\) in body-fixed coordinates. The zero-pose contact frame displays its longitudinal position as \(x'=0\).</p>
  </li>
</ol>
<h3>Travel, pitch and reported measurements</h3>
<ol>
  <li>
    <h4>Move each upright along the fixed tangents</h4>
    <p>For an axle, write \(\lambda\) for lower-joint motion along \(d_B\), \(u\) for upper-joint motion along \(d_U\), and \(d_0=\lVert U_0-B_0\rVert\) for upright length. The current joints satisfy</p>
    \[
      \begin{gathered}
        B=B_0+\lambda d_B,\qquad U=U_0+u d_U,\\
        u^2+2\,d_U\cdot(U_0-B)\,u+\lVert U_0-B\rVert^2-d_0^2=0.
      \end{gathered}
    \]
    <p>The solver chooses the root closest to the static upper joint. An upright angular change \(\alpha\) is the change in the direction angle of \(U-B\) from \(U_0-B_0\). A point \(Q_0\) carried by the upright moves to \(Q=B+\operatorname{Rot}_2(\alpha)(Q_0-B_0)\). The wheel center, contact reference and both kingpin references follow this transform. For a travel slider value \(t_a\) in millimeters, the solver adjusts \(\lambda\) until the transformed contact satisfies \(P_{a,z}=t_a/1000\). The current instant center is the intersection of the normals \(U+\mu Jd_U\) and \(B+\nu Jd_B\). Repeat for the other axle.</p>
  </li>
  <li>
    <h4>Build the current contact frame</h4>
    <p>The current road is the line through \(P_f\) and \(P_r\). Set \(O=(P_f+P_r)/2\), \(e_x=(P_r-P_f)/\lVert P_r-P_f\rVert\), and \(e_z=Je_x\). A plotted point is \((x',y')=((Q-O)\cdot e_x,(Q-O)\cdot e_z)\). The current contact span is \(\lVert P_r-P_f\rVert\) and pitch is the angle of \(e_x\) from the static horizontal. The live pitch center \(PC\) is the intersection of \(P_fI_f\) and \(P_rI_r\). \(PC_0\) remains a body-fixed reference and is shown in the same contact frame.</p>
    <p>In Heave / Pitch mode the solver holds \((t_f+t_r)/2\) at the requested heave and iterates \(t_r-t_f=1000(P_{r,s}-P_{f,s})\tan(\mathrm{pitch})\) until the current pitch matches the request. The result must fall within the configured bump and droop limits. The displayed axle ride height is \(1000h_b-t_a\), with \(h_b\) the static ground clearance in meters.</p>
  </li>
  <li>
    <h4>Read caster, trail and anti in the current frame</h4>
    <p>Let \(v_a=K_{\mathrm{tip},a}-G_a\) after motion, and let \(G_a^*\) be the intersection of the current kingpin line with the current road. Let \(L'=\lVert P_r-P_f\rVert\) and \(h'=(CG-O)\cdot e_z\). The current metrics use</p>
    \[
      \begin{gathered}
        \chi'_a=\sigma_a\,\operatorname{atan2}(v_a\cdot e_x,\ v_a\cdot e_z),\\
        \tau'_a=\sigma_a(P_a-G_a^*)\cdot e_x,\\
        q'_a=\frac{L'}{h'}\,
              \frac{(I_a-P_a)\cdot e_z}{\sigma_a(I_a-P_a)\cdot e_x}.
      \end{gathered}
    \]
    <p>Angles are shown in degrees and trail in millimeters. If \(b_f\) and \(d_f\) are the front brake and drive force fractions, use \(b_r=1-b_f\) and \(d_r=1-d_f\) at the rear. The displayed anti values are \(100q'_a\), \(100q'_a b_a\), and \(100q'_a d_a\) percent. For example, 20% raw front anti with a front brake share of 0.70 yields 14% front brake-weighted anti. The signed SVSA result uses \(\sigma_a(I_{a,s}-P_{a,s})\), measured in the body-fixed frame. Missing intersections, zero horizontal force-line run and unusable CG height can make a metric undefined.</p>
  </li>
</ol>
<h3>Hardpoint mode</h3>
<p>The input ball joints supply the upright points. For each arm let \(H_f,H_r\) be its two chassis mounts and \(Q\) its outer ball joint, all in vehicle coordinates. The side-view tangent comes from the ball-joint velocity of a rotation about the chassis hinge.</p>
\[
  \hat h=\frac{H_r-H_f}{\lVert H_r-H_f\rVert},\qquad
  v=\hat h\times(Q-H_f),\qquad
  d_{\mathrm{SV}}\ \parallel\ (-v_X,v_Z).
\]
<p>The normalized projection is oriented upward and held fixed for stroke analysis. The normals through the two projected joints now determine the static instant center. Project the line through the true upper and lower ball joints to obtain caster and trail. The wheel-center height still comes from the loaded radius, and the same contact-force and pitch-center formulas apply.</p>
`;

export const wheelEnd = String.raw`
<h3>Build the static 3D wheel end</h3>
<p>Select Front or Rear in the axle control, choose the 3D steering chart and set Zero pose. The isolated wheel-end sweep displays the front-right or rear-right corner. Edit KPI, caster, scrub, mechanical trail and static camber in the matching axle's design group. Track, mass, CG location and tire width are shared through Vehicle settings. The upper and lower offsets come from the same axle design used by FV and SV.</p>
<ol>
  <li>
    <h4>Set the vehicle frame, axle station and wheel center</h4>
    <p>The vehicle uses \(X\) forward, \(Y\) left and \(Z\) up, with the front axle at \(X_f=0\) and the rear axle at \(X_r=-L\). Let \(m=+1\) at a right corner and \(m=-1\) at a left corner. Let \(T_a\) be track and \(R_a\) be the axle's loaded radius, calculated from the free radius and per-wheel static load.</p>
    \[
      w_f=\frac{L-\ell_f}{L},\qquad w_r=\frac{\ell_f}{L},\qquad
      R_a=R_{\mathrm{free}}-\frac{Mg\,w_a}{2k_t},
    \]
    \[
      C_0=(X_a,-mT_a/2,R_a),\qquad
      P_{\mathrm{design}}=(X_a,-mT_a/2,0).
    \]
    <p>\(\ell_f\) is the CG distance from the front axle, \(M\) is vehicle mass, \(g\) is gravity and \(k_t\) is tire vertical stiffness. \(P_{\mathrm{design}}\) is the zero-camber contact used to construct the geometry.</p>
  </li>
  <li>
    <h4>Combine the FV and SV kingpin coordinates at equal heights</h4>
    <p>For scrub \(\rho_a\) and kingpin inclination \(\kappa_a\), the right-positive FV coordinate of the axis at height \(z\) is \(m(T_a/2-\rho_a-z\tan\kappa_a)\). Its sign reverses to become vehicle \(Y\). The SV projection supplies \(X\) using trail \(\tau_a\), caster \(\chi_a\) and axle direction \(\sigma_f=+1,\ \sigma_r=-1\). Joining the projections yields</p>
    \[
      K(z)=\left(
        X_a+\sigma_a\tau_a-\sigma_a z\tan\chi_a,\
        -m(T_a/2-\rho_a-z\tan\kappa_a),\
        z\right).
    \]
    <p>The steering axis passes through \(G=K(0)\). The upper ball joint is \(U=K(R_a+A_{u,a})\) and the lower ball joint is \(B=K(R_a-A_{l,a})\). Their heights come from FV, and the SV line is evaluated at those same heights. The upright joins \(B\) to \(U\). The unit axis is \(k=(U-B)/\lVert U-B\rVert\), which in parameter mode is the normalized vector \((-\sigma_a\tan\chi_a,\ m\tan\kappa_a,\ 1)\).</p>
    <p>For the baseline front-right corner, \(T_f=1.25\,\mathrm m\), \(\rho_f=0.025\,\mathrm m\), \(\tau_f=0.015\,\mathrm m\), \(\kappa_f=12^\circ\), \(\chi_f=5^\circ\), and \(R_f\approx0.19573\,\mathrm m\). These give \(G=(0.015,-0.600,0)\,\mathrm m\) and \(k\approx(-0.08527,0.20715,0.97459)\).</p>
    <p>In hardpoint mode, take \(B\) and \(U\) directly from the corner. Their line intersects the static road at \(G=B-B_Z(U-B)/(U_Z-B_Z)\). The axle station, track and loaded radius still locate \(C_0\).</p>
  </li>
  <li>
    <h4>Install static camber about the vehicle X direction</h4>
    <p>Let \(\gamma\) be static camber and \(A=R_X(m\gamma)\) its installation rotation. Negative camber moves the wheel top inward on either side. \(A\) acts around \(C_0\), so the wheel normal becomes \(n_0=Ae_Y\) while the wheel center and kingpin stay fixed. Here \(e_X,e_Y,e_Z\) are the vehicle unit vectors. The geometric wheel uses the loaded radius for contact and the free radius for its drawn outline.</p>
  </li>
</ol>
<h3>Steer the assembly and evaluate its contact</h3>
<ol>
  <li>
    <h4>Rotate around the upward 3D kingpin axis</h4>
    <p>Apply steering angle \(\theta\) by the right-hand rule about \(k\). In the following Rodrigues formula, \([k]_\times v=k\times v\).</p>
    \[
      R=I+\sin\theta\,[k]_\times+(1-\cos\theta)[k]_\times^2,\qquad
      C=G+R(C_0-G),\qquad n=Rn_0.
    \]
    <p>Every point \(Q_0\) attached to the cambered wheel moves to \(G+R(Q_0-G)\). The drawn kingpin reference runs from \(G-0.08k\) to \(G+0.60k\) in meters. Draw the wheel-axis segment from \(C-wn/2\) to \(C+wn/2\), where \(w\) is tire width. For the three wheel rings, take \(d\) equal to \(-w/2,0,w/2\) and trace</p>
    \[
      W(\phi,d)=C+RA
      (R_{\mathrm{free}}\sin\phi,\ d,\ R_{\mathrm{free}}\cos\phi)^{\mathsf T},
      \qquad 0\leq \phi\leq 2\pi.
    \]
  </li>
  <li>
    <h4>Separate the geometric contact from the material reference</h4>
    <p>The live contact point \(P\) is the lowest point on the loaded-radius wheel center circle. Project vertical \(e_Z\) into the plane normal to \(n\) to obtain \(p=e_Z-(e_Z\cdot n)n\). The material reference \(M_{\mathrm{ref}}\) starts at the installed wheel's bottom reference and stays attached to the same location on the rotating wheel.</p>
    \[
      \begin{gathered}
        P=C-R_a\,\frac{p}{\lVert p\rVert},\qquad
        M_{\mathrm{ref},0}=C_0-R_a Ae_Z,\\
        M_{\mathrm{ref}}=G+R(M_{\mathrm{ref},0}-G),\qquad
        H=G+k\,((P-G)\cdot k).
      \end{gathered}
    \]
    <p>\(H\) is the closest point on the kingpin to \(P\). The contact radius is \(\lVert P-H\rVert\) and the material radius is \(\lVert(M_{\mathrm{ref}}-G)\times k\rVert\). The latter stays constant during a sweep about a fixed axis. Contact height can change through the sweep. When the wheel's center circle is horizontal, it has no unique lowest point.</p>
  </li>
  <li>
    <h4>Read wheel tilt, caster and steering moment arms</h4>
    \[
      \mathrm{tilt}=\operatorname{atan2}\!\left(n_Z,\sqrt{n_X^2+n_Y^2}\right),
      \qquad
      \mathrm{caster}=\operatorname{atan2}(-\sigma_a k_X,k_Z).
    \]
    <p>With \(r=P-G\), a unit contact force along any vehicle axis \(e_i\) produces the signed moment arm \(m_i=k\cdot(r\times e_i)\) about the kingpin. For the Fx, Fy and Fz charts this gives</p>
    \[
      (m_X,m_Y,m_Z)=
      (r_Zk_Y-r_Yk_Z,\ r_Xk_Z-r_Zk_X,\ r_Yk_X-r_Xk_Y).
    \]
    <p>For a force vector \((F_X,F_Y,F_Z)\) in newtons, the steering torque in newton-meters is \(F_Xm_X+F_Ym_Y+F_Zm_Z\) using moment arms in meters. The app displays radii, coordinates and moment arms in millimeters, and angles in degrees.</p>
  </li>
  <li>
    <h4>Use the caster sweep to isolate the axis-angle change</h4>
    <p>The steering slider sweeps from negative to positive Steering sweep. The caster surface also uses Minimum design caster and Maximum design caster. To test an angle \(\chi_{\mathrm{test}}\), the solver keeps \(G\) and \(C_0\) at their design locations. It replaces the longitudinal component of \(D=U-B\) with \(-\sigma_a D_Z\tan\chi_{\mathrm{test}}\), then normalizes \(D\) again. Every surface sample uses that test axis and its own steering angle. Applying a new caster in the design inputs rebuilds the static ball-joint positions.</p>
  </li>
</ol>
`;
