% Data for the Wave Runner, taken from service manual
% https://drive.google.com/drive/u/0/folders/1Bj_-HtXJ8rkLbeWXtK07NKwRBSL330Zs
wr.l = 3.350;        % [m]     - Length
wr.w = 1.220;        % [m]     - Width
wr.h = 1.190;        % [m]     - Height
wr.mdry = 348;       % [kg]    - Mass (dry weight)
wr.mdriver = 80;     % [kg]    - Mass driver
wr.mfluids = 75;     % [kg]    - Mass fluids (oil + gas)
wr.m = wr.mdry + wr.mdriver + wr.mfluids; % [kg]    - Mass total

% These are rough estimations and/or guesses - probably incorrect
wr.Al = wr.l * wr.w; % [m^2]   - Lateral Area in contact with water
wr.cr = -1;          % [-]     - Drag coefficient (https://www.boatdesign.net/threads/typical-ship-drag-coefficients.41065/)
wr.mx = 100;         % [kg]    - Added mass due to the inertia of water (should be a function of velocity)
wr.my = 0;         % [kg]    - As above 
% wr.mw = 0.5;       %         - Not the slightest idea what this is (probably not used)
wr.beta0 = 0;        %         - As above
wr.Ao = 0.005;         % [m²]   - Orifice area of the jet

% [kg*m^2] - Moment of inertia, scooter approximated as an ellipsoid
%            with axis through center (Physics handbook)
wr.I = 1/5 * wr.m * (wr.l^2 + wr.h^2); 

% [kg*m^2] - Added moment of inertia due to the inertia of water,
%            obviously should not be zero and should be a function
%            of velocity.
wr.Iz = 0; 


%%% General constants
g = 9.81;            % [m/s^2] - Gravitational constant
water.rho = 1.025;   % [kg/L]  - Sea water density (https://en.wikipedia.org/wiki/Seawater)
water.p = @(h) water.rho * g * h; % [Pa] - Water pressure as a function of depth