%% display the scale fitting result 

% original data 
[ex, ey] = load_data(); 
plot(ey, abs(ex), 'k+'); 
hold on; 

% best observation 
x = [-30:2:30];
plot(x, abs(x), 'g-');

% scale and translation x' = scale*(x+b)
scale = 2.87309; 
b = 0.64167;
ex2 = scale * ex + b;
plot(ey, abs(ex2), 'b*');

% dynamic scale and translation x' = (atan(|x|-f)+pi/2.)*scale*(x+b)
f = 2.82361;
% scale = 1.0052; 
b = 0.7914
dscale = (atan(abs(ex) - f)+pi/2.);
ex3 = dscale.* (ex + b);
plot(ey, abs(ex3), 'rx');

