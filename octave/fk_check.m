% M0.7 - Verify SO-101 forward kinematics: URDF chain (Octave) vs MuJoCo (MJCF).
%
% Run from the repo root, after generating the inputs:
%   python -m sim.export_fk            % -> outputs/fk_mujoco.csv
%   python -m tools.urdf_chain_to_csv  % -> octave/so101_chain.csv (committed)
%   octave --no-gui octave/fk_check.m
%
% Math (same convention as the URDF spec):
%   T_joint = Trans(xyz) * Rz(yaw) * Ry(pitch) * Rx(roll) * Rot(axis, q)
% Acceptance: every sample within 1 mm of the MuJoCo site position.

1;  % marks this file as a script so the local functions below are allowed

function R = rpy_to_R(r, p, y)
  Rx = [1 0 0; 0 cos(r) -sin(r); 0 sin(r) cos(r)];
  Ry = [cos(p) 0 sin(p); 0 1 0; -sin(p) 0 cos(p)];
  Rz = [cos(y) -sin(y) 0; sin(y) cos(y) 0; 0 0 1];
  R = Rz * Ry * Rx;
end

function R = axis_angle_to_R(k, a)
  k = k(:) / norm(k);
  K = [0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
  R = eye(3) + sin(a) * K + (1 - cos(a)) * (K * K);
end

function T = fk_chain(chain, q)
  % chain rows: is_revolute, x, y, z, roll, pitch, yaw, ax, ay, az
  T = eye(4);
  qi = 1;
  for i = 1:rows(chain)
    c = chain(i, :);
    step = eye(4);
    step(1:3, 1:3) = rpy_to_R(c(5), c(6), c(7));
    step(1:3, 4) = c(2:4)';
    T = T * step;
    if c(1) == 1
      rot = eye(4);
      rot(1:3, 1:3) = axis_angle_to_R(c(8:10), q(qi));
      T = T * rot;
      qi = qi + 1;
    end
  end
end

TOL_MM = 1.0;
chain = dlmread('octave/so101_chain.csv', ',', 1, 0);   % skip header row
data  = dlmread('outputs/fk_mujoco.csv',  ',', 1, 0);   % q1..q5, x, y, z
n_rev = sum(chain(:, 1));
q_all = data(:, 1:n_rev);
p_mj  = data(:, n_rev+1:n_rev+3);

n = rows(data);
p_oct = zeros(n, 3);
for s = 1:n
  T = fk_chain(chain, q_all(s, :));
  p_oct(s, :) = T(1:3, 4)';
end

err_mm = 1000 * sqrt(sum((p_oct - p_mj) .^ 2, 2));
[worst, iworst] = max(err_mm);

printf('Samples           : %d\n', n);
printf('Revolute joints   : %d\n', n_rev);
printf('Error mean / max  : %.4f / %.4f mm\n', mean(err_mm), worst);
printf('Worst sample      : #%d, q = [%s] rad\n', iworst, sprintf(' %+.3f', q_all(iworst, :)));
if worst < TOL_MM
  verdict = 'PASS';
else
  verdict = 'FAIL';
end
printf('Result            : %s (tolerance %.1f mm)\n', verdict, TOL_MM);

% Save a short report and a plot next to the other generated outputs.
fid = fopen('outputs/fk_check_octave.txt', 'w');
fprintf(fid, 'samples=%d mean_mm=%.6f max_mm=%.6f result=%s\n', n, mean(err_mm), worst, verdict);
fclose(fid);

try
  h = figure('visible', 'off');
  subplot(1, 2, 1);
  plot3(p_mj(:,1), p_mj(:,2), p_mj(:,3), 'o', p_oct(:,1), p_oct(:,2), p_oct(:,3), '.');
  grid on; axis equal; xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
  legend('MuJoCo (MJCF)', 'Octave (URDF)'); title('End-effector samples');
  subplot(1, 2, 2);
  hist(err_mm, 20); xlabel('position error [mm]'); ylabel('count'); title('URDF vs MJCF error');
  print(h, 'outputs/fk_check_octave.png', '-dpng', '-r120');
catch err
  printf('Plot skipped (%s); numeric check above is still valid.\n', err.message);
end
printf('Saved outputs/fk_check_octave.txt (and .png if plotting worked)\n');
