function simulateMPC(xinit, K, mu)

  dt = 0.01;
  x = xinit;
  xstore = NaN * zeros(2, 10000);  % Initialize storage for state trajectory
  k = 1;
  u = 1;

  n = 4;  % Horizon length for predictive control
  while (norm(x) > 0.001) && (k < 2000)
    xstore(:, k) = x;

    % Linearization of the system (A and B matrices)
    A = eye(2, 2) + dt * [u * (1 - mu), 1; 1, -u * 4 * (1 - mu)];
    B = dt * [mu + (1 - mu) * x(1); mu - 4 * (1 - mu) * x(2)];

    % Input vector for MPC (control horizon)
    U = [u, u, u, u]';
    H = eye(n, n) * 2;

    % Construct the linearized predictive control matrices
    Aqp = -[A; A * A; A * A * A; A * A * A * A] * x;  % Predictive state vector
    Bqp = [B, zeros(2, n - 1);
           A * B, B, zeros(2, n - 2);
           A * A * B, A * B, B, zeros(2, n - 3);
           A * A * A * B, A * A * B, A * B, B];  % Control matrix over horizon

    % Compute the control input using pseudo-inverse
    U = inv(Bqp' * Bqp) * Bqp' * Aqp;

    if size(K) == 0
      u = U(1);  % Use predictive control if K is empty
    else
      u = -K * x;  % Use the stabilizing feedback gain matrix K
    end

    % Saturate control input to be within bounds
    if (u > 2)
      u = 2;
    elseif (u < -2)
      u = -2;
    end

    % Euler method simulation to update the system state
    x1 = x(1);
    x2 = x(2);
    x(1) = x1 + dt * (x2 + u * (mu + (1 - mu) * x1));
    x(2) = x2 + dt * (x1 + u * (mu - 4 * (1 - mu) * x2));

    k++;
  end

  % Plot the trajectory if the system is stable
  if norm(x) < 0.01
    plot(xstore(1, :), xstore(2, :), '+');
  else
    disp("fail!")  % If the system does not converge to a stable state
  end
endfunction

