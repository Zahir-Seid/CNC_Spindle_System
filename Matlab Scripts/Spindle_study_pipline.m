%% ========================================================================
%  CNC Spindle Speed-Loop Control Design
%  Section 3.5/3.6 companion script: plant modeling, PID design via pole
%  placement, root locus, Bode stability margins, closed-loop step response.
%
%  Requires: Control System Toolbox
% =========================================================================

clear; clc; close all;

%% ---------------------------------------------------------------------
%  1. PLANT PARAMETERS
%  Mechanical parameters are motor-shaft-referred: the spindle-side shaft,
%  pulley, and tool-holder inertia are reflected through the measured
%  ~1.70:1 belt ratio (spindle:motor) before being lumped into J.
% ------------------------------------------------------------------------
J  = 5.45e-4;   % kg.m^2   total inertia, motor-shaft-referred
B  = 5.00e-4;   % N.m.s/rad  viscous friction (assumed)
R  = 2.0;       % ohm        phase resistance (assumed, typical 1kW AC servo)
L  = 0.008;     % H          phase inductance (assumed, typical 1kW AC servo)
Kt = 0.35;      % N.m/A      torque constant  (assumed, typical 1kW AC servo)
Ke = 0.35;      % V.s/rad    back-EMF constant (= Kt in SI units)

fprintf('=== PLANT PARAMETERS ===\n');
fprintf('J=%.4e kg.m^2, B=%.4e N.m.s/rad\n', J, B);
fprintf('R=%.2f ohm, L=%.4f H, Kt=Ke=%.2f\n\n', R, L, Kt);

%% ---------------------------------------------------------------------
%  2. PLANT TRANSFER FUNCTION  G(s) = Omega(s)/V(s)
%     Combines mechanical: J*w_dot = Tm - TL - B*w,  Tm = Kt*I
%     with electrical:     V = I*R + L*I_dot + Ke*w
%     Eliminating I(s):    G(s) = Kt / [ (Js+B)(Ls+R) + Kt*Ke ]
% ------------------------------------------------------------------------
a2 = J*L;
a1 = J*R + B*L;
a0 = B*R + Kt*Ke;

G = tf(Kt, [a2 a1 a0]);
fprintf('=== PLANT TRANSFER FUNCTION G(s) ===\n');

p_ol = pole(G);
[wn_ol, zeta_ol] = damp(G);
fprintf('Open-loop poles: %.3f %+.3fj , %.3f %+.3fj\n', ...
    real(p_ol(1)), imag(p_ol(1)), real(p_ol(2)), imag(p_ol(2)));
fprintf('Natural frequency wn = %.2f rad/s, damping ratio zeta = %.4f\n', ...
    wn_ol(1), zeta_ol(1));
fprintf('DC gain G(0) = %.4f (rad/s)/V\n\n', dcgain(G));

%% ---------------------------------------------------------------------
%  3. DESIRED CLOSED-LOOP DYNAMICS -> DOMINANT POLE LOCATIONS
%     ts  = 2%% settling time target
%     OS  = overshoot target (fractional, e.g. 0.05 = 5%%)
% ------------------------------------------------------------------------
ts = 0.05;      % seconds
OS = 0.05;      % 5% overshoot

zeta = -log(OS) / sqrt(pi^2 + log(OS)^2);
wn   = 4 / (zeta*ts);

p1 = complex(-zeta*wn,  wn*sqrt(1-zeta^2));
p2 = conj(p1);
p3 = -5*abs(real(p1));   % non-dominant 3rd pole, placed far left

fprintf('=== DESIRED CLOSED-LOOP POLES ===\n');
fprintf('Target zeta = %.4f, wn = %.2f rad/s\n', zeta, wn);
fprintf('Dominant poles: %.2f %+.2fj\n', real(p1), imag(p1));
fprintf('Non-dominant pole: %.2f\n\n', p3);

desired_poly = poly([p1 p2 p3]);   % monic cubic: s^3 + c2 s^2 + c1 s + c0
desired_poly = real(desired_poly);
c2 = desired_poly(2); c1 = desired_poly(3); c0 = desired_poly(4);

%% ---------------------------------------------------------------------
%  4. PID GAINS VIA POLE PLACEMENT
%     Closed-loop char. eq. (unity feedback, C(s) = (Kd s^2+Kp s+Ki)/s):
%     a2 s^3 + (a1+Kt*Kd) s^2 + (a0+Kt*Kp) s + Kt*Ki = 0
%     Normalize by a2 and match coefficients to the desired monic cubic.
% ------------------------------------------------------------------------
Kd = (c2*a2 - a1) / Kt;
Kp = (c1*a2 - a0) / Kt;
Ki = (c0*a2) / Kt;

fprintf('=== PID GAINS (pole placement) ===\n');
fprintf('Kp = %.4f\nKi = %.2f\nKd = %.6f\n\n', Kp, Ki, Kd);

C  = tf([Kd Kp Ki], [1 0]);      % PID controller C(s)
L_ol = series(C, G);              % open-loop C(s)G(s)
T_cl = feedback(L_ol, 1);         % closed-loop, unity feedback

p_cl = pole(T_cl);
fprintf('Closed-loop poles (verification, should match Section 3 target):\n');
disp(p_cl);

%% ---------------------------------------------------------------------
%  5. LEAD/LAG CHARACTER OF THE DESIGNED PID
%     C(s) = Kd*(s^2 + (Kp/Kd)s + (Ki/Kd)) / s
%     The /s term is a pure LAG (integrator) providing zero steady-state
%     error; the numerator's complex zero pair provides LEAD (phase
%     recovery) near crossover to compensate for the ~90 deg of phase the
%     integrator removes. A PID is therefore a combined lag-lead design.
% ------------------------------------------------------------------------
b1 = Kp/Kd; b0 = Ki/Kd;
disc = b1^2 - 4*b0;
fprintf('\n=== PID ZERO ANALYSIS (lead/lag character) ===\n');
if disc < 0
    zr = -b1/2; zi = sqrt(-disc)/2;
    wn_z = sqrt(b0); zeta_z = b1/(2*wn_z);
    fprintf('Complex zero pair: %.2f +/- %.2fj\n', zr, zi);
    fprintf('Zero-pair wn=%.2f rad/s, zeta=%.3f (LEAD action near crossover)\n', wn_z, zeta_z);
else
    zr = roots([1 b1 b0]);
    fprintf('Real zeros: %.2f, %.2f\n', zr(1), zr(2));
end
fprintf('Pole at origin (1/s): pure LAG / integral action (zero steady-state error)\n\n');

%% ---------------------------------------------------------------------
%  6. STABILITY MARGINS
% ------------------------------------------------------------------------
[Gm, Pm, Wcg, Wcp] = margin(L_ol);
fprintf('=== STABILITY MARGINS (open loop C(s)G(s)) ===\n');
if isinf(Gm)
    fprintf('Gain margin: Inf dB\n');
else
    fprintf('Gain margin: %.2f dB at %.1f rad/s\n', 20*log10(Gm), Wcg);
end
fprintf('Phase margin: %.1f deg at %.1f rad/s (crossover)\n\n', Pm, Wcp);

%% ---------------------------------------------------------------------
%  7. ROOT LOCUS  (fixed PID zero/pole SHAPE, varying overall gain K)
%     K = Kd is our actual design point; the locus shows how the closed-
%     loop poles migrate for other gains along the same compensator shape.
% ------------------------------------------------------------------------
shape = tf([1 b1 b0], [1 0]);     % PID shape normalized to unit leading coeff
OL_shape = series(shape, G);       % K * shape * G is the swept open loop
k_vec = [0 logspace(-4, 1, 4000)];   % 0 up to K=10, well beyond K=Kd, in fine steps
fig1 = figure('Name','Root Locus');
[r, k] = rlocus(OL_shape, k_vec);
plot(real(r).', imag(r).', 'b-');
hold on;
plot(real(p_cl), imag(p_cl), 'r*', 'MarkerSize', 14, 'LineWidth', 2);
p_ol_shape = pole(OL_shape);
z_ol_shape = zero(OL_shape);
plot(real(p_ol_shape), imag(p_ol_shape), 'bx', 'MarkerSize', 10);
plot(real(z_ol_shape), imag(z_ol_shape), 'bo', 'MarkerSize', 10);
hold off;
grid on;
xlim([-450 50]);      % zoom to the physically meaningful region
ylim([-150 150]);
xlabel('Real Axis (1/s)'); ylabel('Imaginary Axis (1/s)');
title('Root Locus: PID-shaped open loop C(s)G(s)/K_d vs gain K');
legend({'Root locus branches','Design point (K=K_d)','Open-loop poles','Open-loop zeros'}, ...
    'Location','best');
saveas(fig1, 'root_locus_matlab.png');

%% ---------------------------------------------------------------------
%  8. BODE PLOT WITH MARGINS
% ------------------------------------------------------------------------
fig2 = figure('Name','Bode with Margins');
margin(L_ol);
grid on;
saveas(fig2, 'bode_margins_matlab.png');

%% ---------------------------------------------------------------------
%  9. CLOSED-LOOP STEP RESPONSE  
% ----------------------------
fig3 = figure('Name','Closed-Loop Step Response');
step(T_cl, 0.1);
grid on;
title('Closed-loop speed step response (verification of PID design)');
saveas(fig3, 'step_response_matlab.png');

info = stepinfo(T_cl);
fprintf('=== STEP RESPONSE CHARACTERISTICS ===\n');
fprintf('Rise time:     %.4f s\n', info.RiseTime);
fprintf('Settling time: %.4f s\n', info.SettlingTime);
fprintf('Overshoot:     %.2f %%\n', info.Overshoot);
fprintf('Peak:          %.4f\n', info.Peak);

%% ---------------------------------------------------------------------
% 10. DISTURBANCE REJECTION CHECK
%
%     Step response of the closed-loop speed to a cutting-torque step of
%     magnitude TL_nominal.  The time axis starts at the moment the step
%     is applied, so this is a design-verification curve -- it shows the
%     shape and magnitude of the speed dip and recovery, not the timing
%     of the t = 0.1 s load transient in the full plant simulation.
%
%     Transfer function from TL to Omega at the plant input side:
%       Omega(s)/TL(s) = -(Ls+R) / [(Js+B)(Ls+R)+Kt*Ke]   (open loop)
%     The disturbance enters at the same summing point as V, so the
%     closed-loop path is shaped by the sensitivity function S(s), not
%     by feedback(Gtl, C).
% ------------------------------------------------------------------------
Gtl = tf(-[L R], [a2 a1 a0]);          % TL -> Omega, open-loop plant path
S   = feedback(1, L_ol);                % sensitivity function 1/(1+L_ol)
Gtl_cl = Gtl * S;                       % disturbance shaped by S(s)

% Peak cutting torque (Section 3.1.1.3).  Same value used in Scripts 1
% and 2; named TL_nominal to keep vocabulary consistent across the three
% companion scripts.
TL_nominal = 0.675;    % N.m

t = 0:0.0005:0.15;
fig4 = figure('Name','Disturbance Rejection (torque step)');
step(TL_nominal*Gtl_cl, t);
grid on;
title('Speed dip and recovery under a step cutting-torque disturbance');
xlabel('Time (s) after disturbance onset'); ylabel('\Delta\Omega (rad/s)');
saveas(fig4, 'disturbance_rejection_matlab.png');

fprintf('\nDone. Figures saved: root_locus_matlab.png, bode_margins_matlab.png,\n');
fprintf('step_response_matlab.png, disturbance_rejection_matlab.png\n');