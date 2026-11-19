export const staticVehicle = String.raw`
  <h3>Build the static full-car geometry</h3>
  <p>In <strong>Static Full Car</strong>, enter the vehicle and axle dimensions under <strong>Design inputs</strong>, then choose <strong>Apply inputs</strong>. Use <strong>Vehicle settings</strong> to enter the longitudinal chassis mount positions or to switch to direct hardpoints. The construction below starts from <strong>Design parameters</strong> and builds the four corner assemblies at zero travel and steering.</p>

  <h4>1. Set the vehicle frame and loaded wheel centers</h4>
  <p>The vehicle frame has \(X\) forward, \(Y\) left and \(Z\) up. The road is \(Z=0\), with front and rear axle planes at \(X_{\mathrm F}=0\) and \(X_{\mathrm R}=-L\), where \(L\) is the wheelbase. Use \(s=+1\) on the left and \(s=-1\) on the right. If \(\ell_f\) is the CG-to-front-axle distance and \(h_G\) is the CG height, the CG position is \((-\ell_f,0,h_G)\).</p>
  <p>Mass \(m\), gravity \(g\), unloaded tire radius \(R_0\), and tire vertical stiffness \(K_t\) set the static wheel height. The axle loads \(W_a\) split evenly across the two wheels of each axle. With axle track \(T_a\), the loaded radius \(r_a\), wheel center \(\mathbf C_{a,s}\), and construction contact \(\mathbf P_{a,s}\) are</p>
  \[
    \begin{aligned}
      W_{\mathrm F}&amp;=mg\,\frac{L-\ell_f}{L},
      &amp;W_{\mathrm R}&amp;=mg\,\frac{\ell_f}{L},
      &amp;r_a&amp;=R_0-\frac{W_a}{2K_t},\\
      \mathbf C_{a,s}&amp;=(X_a,sT_a/2,r_a),
      &amp;\mathbf P_{a,s}&amp;=(X_a,sT_a/2,0).
    \end{aligned}
  \]
  <p>The wheel outlines retain the unloaded radius \(R_0\), so their lower edge can extend below the road. <strong>Ground clearance</strong> sets the body-height reference used in the side-view results. The chassis wireframe vertices come from the equivalent front-view arm pickups described below.</p>

  <h4>2. Combine both front views with the side view</h4>
  <p>Repeat the construction for each axle \(a\) using its own scrub radius \(b_a\), kingpin inclination \(\kappa_a\), mechanical trail \(t_a\), caster \(\gamma_a\), and upper and lower joint offsets \(A_{u,a}\) and \(A_{l,a}\). Let \(d_{\mathrm F}=+1\) and \(d_{\mathrm R}=-1\) so positive trail and caster follow each axle's side-view convention. Evaluate the front and side kingpin lines at the same height \(z\) to get the spatial kingpin</p>
  \[
    \mathbf K_{a,s}(z)=
    \left(
      X_a+d_a(t_a-z\tan\gamma_a),
      \ s\bigl(T_a/2-b_a-z\tan\kappa_a\bigr),
      \ z
    \right).
  \]
  <p>Use \(\mathbf K(0)\) for the ground intercept, \(\mathbf K(r_a+A_{u,a})\) for the upper ball joint (UBJ), and \(\mathbf K(r_a-A_{l,a})\) for the lower ball joint (LBJ). The segment joining the ball joints is the upright. The full kingpin reference runs from \(\mathbf K(0)\) to \(\mathbf K(z_{\mathrm{top}})\), with \(z_{\mathrm{top}}=(0.6\,\mathrm m)\cos\gamma_a\) from the fixed side-view reference length.</p>
  <p>The front-view swing arm \(F_a\) and roll center height \(h_{\mathrm{RC},a}\) give the front-view instant center \(\mathbf I_{\mathrm{FV}}\). The signed side-view swing arm \(S_a\) and axle anti ratio \(\eta_a\) give the corresponding center \(\mathbf I_{\mathrm{SV}}\) on the vehicle center plane. The two axle roll centers form the roll axis.</p>
  \[
    \begin{aligned}
      \mathbf I_{\mathrm{FV},a,s}&amp;=(X_a,\ s(T_a/2-F_a),\ 2F_a h_{\mathrm{RC},a}/T_a),\\
      \mathbf I_{\mathrm{SV},a}&amp;=(X_a-d_aS_a,\ 0,\ S_a\eta_a h_G/L),\\
      \mathbf R_a&amp;=(X_a,0,h_{\mathrm{RC},a}).
    \end{aligned}
  \]
  <p>In the right-side front-view coordinates \((u,z)\), \(u\) increases toward the vehicle right. On each arm, evaluate the line through its projected joint \(J^{\mathrm{FV}}=(u_J,z_J)\) and instant center \(I^{\mathrm{FV}}=(u_I,z_I)\) at the input chassis lateral distance \(M\). The equivalent pickup from this construction supplies the arm's chassis lateral plane and the chassis envelope vertex.</p>
  \[
    z_M=z_J+(M-u_J)\frac{z_I-z_J}{u_I-u_J},
    \qquad
    \mathbf Q_{a,s}=(X_a,sM,z_M),\qquad Y_M=sM.
  \]
  <p>Use the individual <strong>Front FV</strong>, <strong>Rear FV</strong>, and <strong>Side View (SV)</strong> notes for the projected linkage and force lines. In the static side-view results, brake-weighted and drive-weighted anti are \(100\eta_a\) times the corresponding axle brake or drive fraction. The rear fraction is one minus the front fraction.</p>

  <h4>3. Build the chassis envelope and eight arm planes</h4>
  <p>The chassis wireframe connects the eight equivalent pickups \(\mathbf Q\), two per corner. Each axle face connects left and right upper and lower pickups. The four longitudinal faces connect matching pickups between the axles, giving 12 edges. These are reference vertices at the axle planes. The actual A-arm legs continue to the separate front and rear chassis mounts calculated below.</p>
  <p>For each upper or lower arm, take the spatial ball joint \(\mathbf J\), its corner's \(\mathbf I_{\mathrm{FV}}\), and its axle's \(\mathbf I_{\mathrm{SV}}\). Write \(\mathbf A=\mathbf I_{\mathrm{FV}}\). Their triangle determines the arm plane, which the clearance calculation extends beyond the triangle.</p>
  \[
    \mathbf N=(\mathbf I_{\mathrm{SV}}-\mathbf A)\times(\mathbf J-\mathbf A),
    \quad
    \mathbf n=\frac{\mathbf N}{\|\mathbf N\|},
    \quad
    \mathbf n\cdot(\mathbf x-\mathbf A)=0.
  \]
  <p>A valid plane requires three noncollinear points. Each ball joint defines its own plane even though both arms at a corner share the same two instant centers.</p>

  <h4>4. Find the rim-clearance intersection points</h4>
  <p>Tire width \(w\) locates the inboard side of each wheel. The rim inner diameter \(D_i\) and radial clearance \(g_r\) shrink the usable circle to radius \(\rho\). With the \(X,Y,Z\) unit vectors denoted by \(\mathbf e_X,\mathbf e_Y,\mathbf e_Z\), the neutral circle is</p>
  \[
    \begin{aligned}
      \rho&amp;=D_i/2-g_r,
      &amp;\mathbf C_i&amp;=\mathbf C_{a,s}-s(w/2)\mathbf e_Y,\\
      \mathbf n_c&amp;=-s\mathbf e_Y,
      &amp;\mathbf Q_i(\theta)&amp;=\mathbf C_i+\rho\bigl(\cos\theta\,\mathbf e_X+\sin\theta\,(\mathbf n_c\times\mathbf e_X)\bigr).
    \end{aligned}
  \]
  <p>For each front wheel, rotate this circle by both positive and negative <strong>Steering angle for clearance</strong> around the upward 3D kingpin axis through \(\mathbf G=\mathbf K(0)\). For steering angle \(\delta\) and unit kingpin axis \(\widehat{\mathbf k}\), let \(\mathsf R_{\widehat{\mathbf k}}(\delta)\) be the right-hand Rodrigues rotation described in the <strong>Wheel End (WE3D)</strong> note. The rotated circle keeps radius \(\rho\), with center and normal</p>
  \[
    \mathbf C_{i,\delta}=\mathbf G+\mathsf R_{\widehat{\mathbf k}}(\delta)(\mathbf C_i-\mathbf G),
    \qquad
    \mathbf n_{c,\delta}=\mathsf R_{\widehat{\mathbf k}}(\delta)\mathbf n_c.
  \]
  <p>The rear mounts use the neutral circle at \(\delta=0\). Let \(\mathbf O\) and \(\mathbf m\) be the selected circle's center and unit normal. Project the arm plane normal \(\mathbf n\) into that circle's plane and solve the chord. The two intersection points, where they exist, are</p>
  \[
    \begin{aligned}
      \mathbf v&amp;=\mathbf n-(\mathbf n\cdot\mathbf m)\mathbf m,
      &amp;\mathbf u&amp;=\mathbf v/\|\mathbf v\|,\\
      q&amp;=\frac{-\mathbf n\cdot(\mathbf O-\mathbf A)}{\|\mathbf v\|},
      &amp;\mathbf B_{\pm}&amp;=\mathbf O+q\mathbf u
        \ \pm\sqrt{\rho^2-q^2}\,(\mathbf m\times\mathbf u).
    \end{aligned}
  \]
  <p>If \(|q|=\rho\), the circle is tangent and contributes a single point. If \(|q|\gt\rho\) there is no intersection. A parallel or coincident circle plane supplies no discrete intersection points. At the front, each steering limit contributes the point with the smallest \(|Y|\) for that arm. At the rear, keep both intersections of the neutral circle when both exist. These positions come from exact circle geometry. The drawn circle segments only visualize the construction.</p>

  <h4>5. Place and check the chassis mounts</h4>
  <p>For each selected clearance point \(\mathbf B\), intersect the line from its ball joint \(\mathbf J\) through \(\mathbf B\) with \(Y=Y_M\). The parameter \(\lambda\) records whether the chassis intersection lies beyond the clearance point, with \(\lambda=1\) at \(\mathbf B\).</p>
  \[
    \lambda=\frac{Y_M-J_Y}{B_Y-J_Y},
    \qquad
    \mathbf H=\mathbf J+\lambda(\mathbf B-\mathbf J).
  \]
  <p>A degenerate or parallel line cannot locate a unique mount. When two positions are available, the larger \(X\) is the front mount and the smaller \(X\) is the rear mount. The upper mounts are UCA_F and UCA_R, and the lower mounts are LCA_F and LCA_R. Draw the two legs from each ball joint to its corresponding chassis mounts to complete the arm.</p>
  <p>Under <strong>Vehicle settings</strong>, each axle has four <strong>Chassis arm mounts</strong> fields in millimetres. The eight entries cover the front and rear mount of each upper and lower arm, with corresponding left and right positions mirrored. Leave a value blank to use its automatic clearance position. For manual placement, choose \(X^*=X_{\mathrm{input,mm}}/1000\) within the two automatic \(X\) limits and keep the rear position strictly behind the front position. Solve the arm plane at the fixed lateral coordinate \(Y_M\).</p>
  \[
    Z^*=A_Z-\frac{n_X(X^*-A_X)+n_Y(Y_M-A_Y)}{n_Z},
    \qquad
    \mathbf H^*=(X^*,Y_M,Z^*).
  \]
  <p>The formula requires a nonzero \(n_Z\). The settings preview shows the automatic pair when the requested positions cannot produce a valid pair. Correct the request or leave it blank to apply the design. The clearance check uses the neutral rear circle and the two front steering limits at zero installation camber. Clearance across intermediate steering, vertical travel, and three-dimensional part surfaces requires a separate check.</p>
  <p><strong>Baseline check.</strong> With the default front settings and automatic mounts, the front-right wheel center is \((0,-0.625,0.195727)\,\mathrm m\), the kingpin ground point is \((0.015,-0.600,0)\,\mathrm m\), and the rim-clearance radius is \(0.1022375\,\mathrm m\). The two upper chassis mounts are approximately</p>
  \[
    \begin{aligned}
      \mathbf H_{\mathrm{front}}&amp;=(0.287561,-0.255,0.209964)\,\mathrm m,\\
      \mathbf H_{\mathrm{rear}}&amp;=(-0.275168,-0.255,0.211324)\,\mathrm m.
    \end{aligned}
  \]

  <h4>6. Install the static camber and choose a projection</h4>
  <p>After the joints, arm mounts and clearance circles are constructed, rotate the wheel outline around the wheel center and vehicle \(X\) axis. For input static camber \(\phi\), the signed rotation at that corner is \(\mathsf R_X(-s\phi)\). Negative camber tips the wheel top inward on either side. The outline joins two circular sidewalls across the width. Each sidewall is given by</p>
  \[
    \mathbf W_{\pm}(\theta)=\mathbf C_{a,s}
      +\mathsf R_X(-s\phi)\bigl(R_0\sin\theta,\ \pm w/2,\ R_0\cos\theta\bigr).
  \]
  <p>For center-plane wheel normal \(\mathbf n_w\), the lowest point of the loaded-radius center circle is</p>
  \[
    \mathbf n_w=\mathsf R_X(-s\phi)\mathbf e_Y,
    \qquad
    \mathbf v_{\downarrow}=\mathbf e_Z-(\mathbf e_Z\cdot\mathbf n_w)\mathbf n_w,
    \qquad
    \mathbf P_{\mathrm{installed}}=\mathbf C_{a,s}
       -r_a\,\frac{\mathbf v_{\downarrow}}{\|\mathbf v_{\downarrow}\|}.
  \]
  <p>The three orthographic charts project \((X,Y,Z)\) to \((-Y,Z)\) in front view, \((X,Z)\) in side view, and \((-Y,X)\) in top view. The 3D chart rotates the same coordinates for display. Toggle the FV and SV planes and construction references to inspect the linkages. Hardpoint CSV and STEP retain the original vehicle-frame coordinates regardless of the camera or visible chart planes. CSV uses metres. STEP uses millimetres.</p>

  <h4>7. Use direct hardpoints for a measured geometry</h4>
  <p>In <strong>Vehicle settings</strong>, choose <strong>Direct hardpoint coordinates</strong>. Enter coordinates in millimetres or start with <strong>Regenerate from design parameters</strong>. Editing a point reflects it to \((X,-Y,Z)\) on the other side. The four arm mounts and two ball joints define each corner's arm triangles and upright. The wheel centers use the shared axle track and loaded radius.</p>
  <p>Use <strong>RACK</strong> for the inboard tie rod end at zero rack travel and <strong>TOE</strong> for the outboard tie rod end on the upright. Enter both points for an axle to use measured tie rod geometry. The static Hardpoints CSV names each point with its corner prefix, such as <code>FL_RACK</code> and <code>FL_TOE</code>.</p>
  <p>For an arm with chassis mounts \(\mathbf H_f,\mathbf H_r\) and ball joint \(\mathbf J\), the hinge direction is \(\mathbf h=\mathbf H_r-\mathbf H_f\). Intersect that hinge with the longitudinal plane \(X=J_X\) to get the equivalent FV pivot \(\mathbf E\). The local tangent to the ball joint path comes from rotation about the normalized hinge direction.</p>
  \[
    \mathbf E=\mathbf H_f+\frac{J_X-H_{f,X}}{h_X}\,\mathbf h,
    \qquad
    \mathbf v_J=\frac{\mathbf h}{\|\mathbf h\|}\times(\mathbf J-\mathbf H_f).
  \]
  <p>Use \((-E_Y,E_Z)\) in FV and the projected tangent \((-v_{J,X},v_{J,Z})\) in the SV solver, whose local horizontal coordinate increases rearward. The actual kingpin direction is \(\mathbf k=\mathrm{UBJ}-\mathrm{LBJ}\). Its ground intercept is \(\mathrm{LBJ}-(\mathrm{LBJ}_Z/k_Z)\mathbf k\), and the reference extends to \(\mathrm{UBJ}+(0.25\,\mathrm m/k_Z)\mathbf k\). A valid corner needs separated chassis mounts with nonzero \(X\) separation, a ball joint off each hinge, and UBJ above LBJ. Static Full Car and WE3D use the supplied 3D coordinates. FV and SV use equivalent planar geometry derived from those points.</p>
`;
