%% ========================================================================
% CNC Milling Spindle - PLC Supervisory Interlock Simulation
%
% Project:
% Design, Analysis, and Modeling of a CNC Milling Spindle System
%
% Purpose:
% Compare tool-change operation:
%   1. Without PLC supervisory interlock
%   2. With PLC supervisory interlock
%
% The existing PID speed controller remains responsible for spindle-speed
% regulation. The PLC/Stateflow logic is represented here as supervisory
% discrete logic that controls the safe sequence for tool release.
% ========================================================================

clear;
clc;
close all;

%% ========================================================================
% 1. SPINDLE AND MOTOR PARAMETERS
% ========================================================================

J  = 5.45e-4;      % kg.m^2, total rotational inertia
B  = 5.00e-4;      % N.m.s/rad, viscous friction
R  = 2.0;          % ohm, phase resistance
L  = 0.008;        % H, phase inductance

Kt = 0.35;         % N.m/A, torque constant
Ke = 0.35;         % V.s/rad, back-EMF constant

%% ========================================================================
% 2. PID CONTROLLER
%    Values obtained from the existing pole-placement design.
%
%    NOTE: the pid() object is not built here because the main loop
%    implements the controller directly (needed for the reference filter,
%    D-on-measurement, and conditional-integration anti-windup).  If you
%    want the Control System Toolbox object for a design check, uncomment
%    the line below.
% ========================================================================

Kp = 0.6118;
Ki = 66.96;
Kd = 0.003850;

% C_PID = pid(Kp, Ki, Kd);   % kept out of the loop, see note above

%% ========================================================================
% 3. SPINDLE PLANT
%
% Electrical:
%       V = R I + L dI/dt + Ke*w
%
% Mechanical:
%       J dw/dt = Kt I - B*w - TL
%
% State vector:
%       x = [I  w]'
%
% Inputs:
%       u = [V  TL]'
%
% Output:
%       y = w
% ========================================================================

A = [-R/L      -Ke/L;
      Kt/J     -B/J];

Bplant = [1/L       0;
          0        -1/J];

Cplant = [0 1];

Dplant = [0 0];

Plant = ss(A, Bplant, Cplant, Dplant);   % kept for reference / design check

%% ========================================================================
% 3b. NOMINAL CUTTING TORQUE
% ========================================================================

% Peak cutting torque used throughout the simulation.  Matches the value
% used in the Simulink model and the pole-placement design report.

TL_nominal = 0.675;    % N.m

%% ========================================================================
% 4. OPERATING CONDITIONS
% ========================================================================

RPM_nominal = 5000;

% Convert RPM to rad/s
omega_nominal = RPM_nominal * 2*pi/60;

% Tool-change request time
t_request = 0.10;       % s

% Simulation duration
t_end = 0.30;           % s

% Time step
dt = 1e-4;              % s

t = (0:dt:t_end)';

%% ========================================================================
% 5. TOOL-CHANGE STOPPING CONDITION
% ========================================================================

% The spindle is considered stationary when its speed falls below this
% threshold.

RPM_stop_threshold = 5;          % rpm
omega_stop_threshold = ...
    RPM_stop_threshold * 2*pi/60;

%% ========================================================================
% 6. TOOL-CHANGE REQUEST
% ========================================================================

Tool_Change_Request = double(t >= t_request);

%% ========================================================================
% 7. INITIAL CONDITIONS
% ========================================================================

% Assume spindle is initially operating at 5000 rpm.

x = zeros(2,1);

% Loaded steady-state current: viscous friction torque + cutting torque
I_initial = (B*omega_nominal + TL_nominal) / Kt;

x(1) = I_initial;
x(2) = omega_nominal;

%% ========================================================================
% 7b. STEADY-STATE FEEDFORWARD VOLTAGE
%
% Voltage required to hold the nominal speed against the nominal cutting
% load.  Supplying this directly at the plant input keeps the PID's
% integrator from having to carry the DC operating point.
% ========================================================================

V_ss = R*I_initial + Ke*omega_nominal;    % ~188.6 V

%% ========================================================================
% 8. PID STATE VARIABLES
% ========================================================================

integral_error  = 0;
omega_previous  = omega_nominal;     % for D-on-measurement

%% ========================================================================
% 8b. REFERENCE LOW-PASS FILTER
%
% A first-order filter is applied to the PLC speed command before it
% reaches the PID.  Without this filter, the step from 5000 rpm to 0 rpm
% at t_request produces a large derivative kick (Kd * d(error)/dt) on the
% voltage command.
%
% The filter time constant is chosen much smaller than the closed-loop
% settling time so that it does not slow the response, but large enough
% to round off the hard step.
% ========================================================================

tau_ref        = 0.005;              % s, reference filter time constant
omega_ref_filt = omega_nominal;      % scalar running filter state

%% ========================================================================
% 9. SIMULATION ARRAYS
% ========================================================================

omega = zeros(size(t));
RPM   = zeros(size(t));

omega_ref_cmd  = zeros(size(t));     % pre-filter reference (rad/s)
omega_ref_used = zeros(size(t));     % post-filter reference (rad/s)

Voltage_command = zeros(size(t));

Cylinder_nonPLC = zeros(size(t));
Cylinder_PLC    = zeros(size(t));

PLC_Stop_Command = zeros(size(t));
PLC_State_signal = zeros(size(t));   % logged every step (see Section 12)

%% ========================================================================
% 10. NON-PLC CASE
%
% In this case the tool-release command is generated directly from the
% tool-change request.
%
% Therefore:
%
% Tool Change Request = 1
%             |
%             v
%     Cylinder Actuate
%
% The cylinder can fire while the spindle is still rotating.
% ========================================================================

Cylinder_nonPLC(t >= t_request) = 1;

%% ========================================================================
% 11. PLC SUPERVISORY LOGIC
%
% State sequence:
%
% NORMAL OPERATION
%       |
%       | Tool Change Request
%       v
% STOPPING
%       |
%       | RPM <= stopping threshold
%       v
% TOOL RELEASE
%       |
%       | Drawbar delay AND RPM still <= stopping threshold
%       v
% NORMAL / TOOL CHANGE COMPLETE
%
% The PLC does NOT replace the PID controller.
% It changes the permitted spindle reference and controls the tool-release
% permission.
%
% NOTE (bug fix carried over from the previous revision):
% The original logic latched the "stopping threshold reached" condition at
% the moment of entering the RELEASE state and then fired the cylinder
% after a fixed delay without re-checking the speed.  Because the PID
% integral action could push the speed back above the threshold during
% that delay, the tool could still be released at an unsafe speed.
%
% The corrected logic re-checks the speed inside the RELEASE state.  If
% the speed has risen above the threshold, the PLC returns to STOPPING
% and the delay timer is reset.
% ========================================================================

PLC_STATE_NORMAL   = 0;
PLC_STATE_STOPPING = 1;
PLC_STATE_RELEASE  = 2;
PLC_STATE_COMPLETE = 3;

PLC_State = PLC_STATE_NORMAL;

release_delay = 0.020;       % s, simulated drawbar actuation delay
release_timer = 0;

%% ========================================================================
% 12. MAIN CLOSED-LOOP SIMULATION
% ========================================================================

V_max = 325;    % V, full-wave-rectified peak of the 230 V single-phase mains (230*sqrt(2))

for k = 1:length(t)

    %% ---------------------------------------------------------------
    % Current spindle speed
    % ---------------------------------------------------------------

    omega(k) = x(2);
    RPM(k)   = omega(k) * 60/(2*pi);

    %% ---------------------------------------------------------------
    % PLC SUPERVISORY LOGIC
    % ---------------------------------------------------------------

    if Tool_Change_Request(k) == 0

        % Normal machining
        PLC_State = PLC_STATE_NORMAL;

    else

        switch PLC_State

            case PLC_STATE_NORMAL

                % Tool change requested
                PLC_State = PLC_STATE_STOPPING;

            case PLC_STATE_STOPPING

                % Command spindle to stop
                PLC_Stop_Command(k) = 1;

                % Wait until spindle reaches stopping threshold
                if abs(RPM(k)) <= RPM_stop_threshold

                    PLC_State = PLC_STATE_RELEASE;
                    release_timer = t(k);
                end

            case PLC_STATE_RELEASE

                PLC_Stop_Command(k) = 1;

                % ---- BUG FIX -----------------------------------------
                % Re-check the speed at every step while waiting for the
                % drawbar delay.  If the spindle has sped back up, we are
                % no longer safe to release the tool.
                % -----------------------------------------------------
                if abs(RPM(k)) > RPM_stop_threshold

                    % Speed is no longer safe: go back to stopping and
                    % restart the release timer.
                    PLC_State = PLC_STATE_STOPPING;
                    release_timer = 0;

                elseif (t(k) - release_timer) >= release_delay

                    % Drawbar delay elapsed AND speed is still safe.
                    Cylinder_PLC(k) = 1;
                    PLC_State = PLC_STATE_COMPLETE;

                end

            case PLC_STATE_COMPLETE

                PLC_Stop_Command(k) = 1;
                Cylinder_PLC(k)     = 1;

        end
    end

    % Log the state every step for Figure 4 (no post-hoc reconstruction).
    PLC_State_signal(k) = PLC_State;

    %% ---------------------------------------------------------------
    % PLC-GENERATED SPEED REFERENCE (rad/s)
    % ---------------------------------------------------------------

    if PLC_State == PLC_STATE_NORMAL
        omega_ref_cmd(k) = omega_nominal;   % run at nominal speed
    else
        omega_ref_cmd(k) = 0;               % tool change -> stop
    end

    %% ---------------------------------------------------------------
    % REFERENCE LOW-PASS FILTER
    %
    % First-order update:
    %       w_f[k+1] = w_f[k] + (dt/tau_ref) * (w_cmd[k] - w_f[k])
    % ---------------------------------------------------------------

    omega_ref_filt = omega_ref_filt + (dt/tau_ref) * ...
                     (omega_ref_cmd(k) - omega_ref_filt);

    omega_ref_used(k) = omega_ref_filt;

    %% ---------------------------------------------------------------
    % PID SPEED CONTROLLER
    %
    %   P  : on speed error
    %   I  : on speed error (with conditional-integration anti-windup)
    %   D  : on measurement only (avoids derivative kick on the step)
    % ---------------------------------------------------------------

    error = omega_ref_used(k) - omega(k);

    % Integral term
    integral_error = integral_error + error*dt;

    % Derivative term on measurement (note the negative sign, because
    % d(error)/dt = -d(omega)/dt when the reference is (locally) constant).
    derivative_meas = -(omega(k) - omega_previous)/dt;
    omega_previous  = omega(k);

    % PID output (unsaturated)
    V_unsat = Kp*error + Ki*integral_error + Kd*derivative_meas;

    % Feedforward + feedback -> total actuator demand
    V_total_unsat = V_unsat + V_ss;

    % Saturation on the total command (physically correct single-saturation
    % architecture: the DC bus limit applies to the sum).
    V = max(min(V_total_unsat, V_max), -V_max);

    % Conditional-integration anti-windup on the total command
    if (V_total_unsat >  V_max && error > 0) || ...
       (V_total_unsat < -V_max && error < 0)
        integral_error = integral_error - error*dt;
    end

    Voltage_command(k) = V;

    %% ---------------------------------------------------------------
    % CUTTING TORQUE
    %
    % Cutting torque is removed when tool-change operation begins.
    % ---------------------------------------------------------------

    if Tool_Change_Request(k) == 0
        TL = TL_nominal;
    else
        TL = 0;
    end

    %% ---------------------------------------------------------------
    % PLANT DYNAMICS
    % ---------------------------------------------------------------

    dx = A*x + Bplant*[V; TL];
    x  = x + dx*dt;

    % ---- One-way bearing physical constraint ------------------------
    % The spindle cannot rotate in reverse.  If numerical integration
    % produces a negative speed, clamp at zero.
    % ----------------------------------------------------------------
    if x(2) < 0
        x(2) = 0;
    end

end

%% ========================================================================
% 13. ENSURE FINAL VALUES ARE RECORDED
% ========================================================================

omega(end) = x(2);
RPM(end)   = omega(end)*60/(2*pi);

%% ========================================================================
% 14. SAFETY CHECK
% ========================================================================

% Find the first instant at which the PLC permits cylinder actuation.

idx_release = find(Cylinder_PLC > 0,1,'first');

if ~isempty(idx_release)

    release_time = t(idx_release);
    release_RPM  = RPM(idx_release);

else

    release_time = NaN;
    release_RPM  = NaN;

end

%% ========================================================================
% 15. UNSAFE RELEASE CHECK
% ========================================================================

idx_nonPLC = find(Cylinder_nonPLC > 0,1,'first');

unsafe_release_time = t(idx_nonPLC);
unsafe_release_RPM  = RPM(idx_nonPLC);

%% ========================================================================
% 16. DISPLAY RESULTS
% ========================================================================

fprintf('\n');
fprintf('============================================================\n');
fprintf(' CNC SPINDLE TOOL-CHANGE INTERLOCK SIMULATION\n');
fprintf('============================================================\n');

fprintf('\nInitial spindle speed: %.1f RPM\n', RPM_nominal);

fprintf('\nWITHOUT PLC INTERLOCK:\n');
fprintf('Cylinder release time: %.4f s\n', unsafe_release_time);
fprintf('Spindle speed at release: %.2f RPM\n', unsafe_release_RPM);

fprintf('\nWITH PLC INTERLOCK:\n');
fprintf('Cylinder release time: %.4f s\n', release_time);
fprintf('Spindle speed at release: %.2f RPM\n', release_RPM);

fprintf('\nStopping threshold: %.2f RPM\n', RPM_stop_threshold);

% Extra explicit safety assertion for the report
if ~isnan(release_RPM)
    if abs(release_RPM) <= RPM_stop_threshold
        fprintf('Safety check: PASS - PLC release speed is within threshold.\n');
    else
        fprintf('Safety check: FAIL - PLC release speed is above threshold.\n');
    end
end

%% ========================================================================
% 17. FIGURE 1 - SPINDLE SPEED AND TOOL-CHANGE REQUEST
% ========================================================================

figure('Name','Spindle Speed During Tool Change');

yyaxis left

plot(t,RPM,'LineWidth',1.5);
hold on;

% Filtered reference (post-LPF) is what the PID actually tracks.
plot(t,omega_ref_used * 60/(2*pi),'--','LineWidth',1.2);

yline(RPM_stop_threshold,':','Stopping Threshold');

ylabel('Spindle Speed (RPM)');

yyaxis right

plot(t,Tool_Change_Request,'LineWidth',1.2);

ylabel('Tool Change Request');

xlabel('Time (s)');
title('CNC Spindle Speed During Tool-Change Sequence');

grid on;

legend('Actual RPM',...
       'PLC Commanded RPM (filtered)',...
       'Stopping Threshold',...
       'Tool Change Request',...
       'Location','best');

%% ========================================================================
% 18. FIGURE 2 - NON-PLC VS PLC CYLINDER ACTUATION
% ========================================================================

figure('Name','PLC Interlock Comparison');

plot(t,RPM/1000,'LineWidth',1.5);
hold on;

plot(t,Cylinder_nonPLC*5,'--','LineWidth',1.5);

plot(t,Cylinder_PLC*5,'-.','LineWidth',1.5);

yline(RPM_stop_threshold/1000,':');

xlabel('Time (s)');
ylabel('Speed / Actuator State');

title('Comparison of Tool-Release Operation');

legend('Spindle Speed (1000 RPM)',...
       'Cylinder - No PLC',...
       'Cylinder - PLC Interlock',...
       'Stopping Threshold',...
       'Location','best');

grid on;

%% ========================================================================
% 19. FIGURE 3 - SAFETY TIMELINE (5 PANELS)
%
% Extra panel added: the PLC Stop Command is now shown explicitly so the
% reader can see the supervisory command separate from the cylinder firing.
% ========================================================================

figure('Name','Tool Change Safety Timeline');

subplot(5,1,1);
plot(t,RPM,'LineWidth',1.5);
hold on;
yline(RPM_stop_threshold,':');
ylabel('RPM');
title('Tool-Change Safety Sequence');
grid on;

subplot(5,1,2);
stairs(t,Tool_Change_Request,'LineWidth',1.5);
ylim([-0.1 1.1]);
ylabel('Request');
grid on;

subplot(5,1,3);
stairs(t,PLC_Stop_Command,'LineWidth',1.5);
ylim([-0.1 1.1]);
ylabel('PLC Stop');
grid on;

subplot(5,1,4);
stairs(t,Cylinder_nonPLC,'LineWidth',1.5);
ylim([-0.1 1.1]);
ylabel('No PLC');
grid on;

subplot(5,1,5);
stairs(t,Cylinder_PLC,'LineWidth',1.5);
ylim([-0.1 1.1]);
ylabel('PLC');
xlabel('Time (s)');
grid on;

%% ========================================================================
% 20. FIGURE 4 - PLC STATE SEQUENCE
%
% The state sequence is now taken directly from PLC_State_signal, which
% is logged during the simulation.  This guarantees the plot reflects the
% actual state machine execution (no post-hoc reconstruction that could
% disagree with the logic).
% ========================================================================

figure('Name','PLC Supervisory State Sequence');

stairs(t,PLC_State_signal,'LineWidth',1.8);

yticks([0 1 2 3]);

yticklabels({'NORMAL',...
             'STOPPING',...
             'RELEASE',...
             'COMPLETE'});

xlabel('Time (s)');
ylabel('PLC State');

title('Supervisory PLC State Sequence');

grid on;

%% ========================================================================
% 21. SAVE FIGURES FOR REPORT
% ========================================================================

saveas(figure(1),'Pictures/spindle_tool_change_speed.png');
saveas(figure(2),'Pictures/plc_interlock_comparison.png');
saveas(figure(3),'Pictures/tool_change_safety_timeline.png');
saveas(figure(4),'Pictures/plc_state_sequence.png');

%% ========================================================================
% END OF SCRIPT
% ========================================================================