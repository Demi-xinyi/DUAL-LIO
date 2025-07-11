function q = euler2quatern(euler)
    % euler2quatern Convert Euler angles to quaternion
    % euler: A matrix of Euler angles [phi, theta, psi]
    % q: The resulting quaternion [q0, q1, q2, q3]

    % Extract the Euler angles
    phi = euler(:, 1);   % Roll
    theta = euler(:, 2); % Pitch
    psi = euler(:, 3);   % Yaw

    % Precompute trigonometric functions
    cosPhi = cos(phi / 2);
    sinPhi = sin(phi / 2);
    cosTheta = cos(theta / 2);
    sinTheta = sin(theta / 2);
    cosPsi = cos(psi / 2);
    sinPsi = sin(psi / 2);

    % Calculate the quaternion components
    q0 = cosPhi .* cosTheta .* cosPsi + sinPhi .* sinTheta .* sinPsi;
    q1 = sinPhi .* cosTheta .* cosPsi - cosPhi .* sinTheta .* sinPsi;
    q2 = cosPhi .* sinTheta .* cosPsi + sinPhi .* cosTheta .* sinPsi;
    q3 = cosPhi .* cosTheta .* sinPsi - sinPhi .* sinTheta .* cosPsi;

    % Combine into a quaternion matrix
    q = [q0, q1, q2, q3];
end
