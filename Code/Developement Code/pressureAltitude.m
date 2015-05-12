% Jon Wheless

% This script uses numbers and formulas from this page: http://en.wikipedia.org/wiki/Barometric_formula
%% Lookup tables and constants

% Height above sea level at bottom of atmospheric layer (m)
bTbl_h = [0, 11000, 20000, 32000, 47000, 51000, 71000];

% Static pressures (kPa) at bottom of atmospheric layer
bTbl_P = [101.325,22.6321,5.47489,0.86802,0.11091,0.06694,0.00396];

% Average temperature (K) at bottom of atmospheric layer
bTbl_T = [288.15, 216.65, 216.65, 228.65, 270.65, 270.65, 214.65];

% Temperature lapse rate (K/m) inside atmospheric layer
bTbl_L = [-0.0065, 0, 0.001, 0.0028, 0, -0.0028, -0.002];

M = 0.0288; % Molar mass of air (kg/mol)
g_0 = 9.81; % Acceleration of gravity (m/s^2)
R = 8.3145; % Gas constant for air (N*m/mol*K)

% Calculated height above sea level
h = zeros(length(P),1); % Pre-allocate

%% Altitude calculation

% To use:
% Put vector of pressures in variable P in your workspace
% Run the script
% Vector of corresponding altitudes is saved in variable h

% Loop through the input vector of pressures
for i = 1:length(P)
	
	% Find the atmospheric layer of the pressure in question
	b = 1;
	while (P(i) < bTbl_P(b+1))
		b = b + 1;
	end
	
	% Calculate altitude above sea level using the appropriate atmospheric layer properties
	if (bTbl_L(b) == 0)
		% If lapse rate is zero, use this equation
		h(i) = -(R*bTbl_T(b)*(log(P(i)/bTbl_P(b)) - (M*g_0*bTbl_h(b))/(R*bTbl_T(b))))/(M*g_0);
	else
		% if lapse rate is on-zero, use this equation
		h(i) = (bTbl_L(b)*bTbl_h(b) - bTbl_T(b) + bTbl_T(b)/(P(i)/bTbl_P(b))^((bTbl_L(b)*R)/(M*g_0)))/bTbl_L(b);
	end
end