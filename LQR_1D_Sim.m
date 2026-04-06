function [t, altitude, velocity, acceleration, thrust, mass, fmass] = LQR_1D_Sim(dmass, fm, gravity, int_alt, int_velo, max_time, maxthrust, max_alt_error, max_velo_error, dt)
    t = 0:dt:max_time;
    numSteps = length(t);
    altitude = zeros(numSteps, 1);
    velocity = zeros(numSteps, 1);
    thrust = zeros(numSteps, 1);
    mass = zeros(numSteps, 1);
    fmass = zeros(numSteps, 1);
    acceleration = zeros(numSteps, 1);

    max_Massflow = 0.0625;
    fm0 = fm;
    h = int_alt;
    v = int_velo;
    Q = [1/(max_alt_error^2) 0; 0 1/(max_velo_error^2)];
    R = 1/maxthrust^2;
    Cd = 0.45;
    Pden = 1.225;
    Area = (pi.*0.11.^2)./4;
    c = Pden*Cd*Area;
    x_target = [0;0];

    for i = 1:numSteps
        A = [0 1; 0 -c*v];
        m = dmass + fm;
        
        B = [0; 1/m];
        [K, ~, ~] = lqr(A, B, Q, R);
        x = [h; v];
        error_alt = x(1) - x_target(1);
        error_velo = x(2) - x_target(2);
        u = -(K(1) * error_alt + K(2).* error_velo);
        u_ff = m * gravity;
        DForce = Cd.*Pden.*v.^2.*Area/2;
        controlSignal = u + u_ff;
        throttle = max(0, min(1, controlSignal/maxthrust));
        if fm <= 0
            thrst = 0;
        else
            thrst = max(0, min(maxthrust, controlSignal));
            fm = fm - throttle.*max_Massflow.*dt;
        end
        a = ((thrst + DForce)/m) - gravity;
        
        v = v + a.*dt;
        h = h + v.*dt;
        altitude(i) = h;
        velocity(i) = v;
        thrust(i) = thrst;
        acceleration(i) = a;
        mass(i) = m;
        fmass(i) = (fm./fm0) * 100;
        if h <= 0.5
            h = 0;
            fprintf('Landed at t = %.2f s | Final velocity = %.3f m/s\n', t(i), v);
            altitude(i:end) = h;
            velocity(i:end) = v;
            acceleration(i:end) = a;
            thrust(i:end) = 0;
            mass(i:end) = m;
            fmass(i:end) = (fm./fm0) * 100;
            return
        end
    end
end