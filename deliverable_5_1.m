clear all
close all

Ts = 1/10; % Sample period
car = Car(Ts);
[xs, us] = car.steady_state(120 / 3.6);
sys = car.linearize(xs, us);
[sys_lon, sys_lat] = car.decompose(sys);
tube_mpc_sets(sys_lon, Ts);

% Horizon length in seconds
H_lon = 10;
H_lat = 10;
mpc_lon = MpcControl_lon(sys_lon, Ts, H_lon);
mpc_lat = MpcControl_lat(sys_lat, Ts, H_lat);
mpc = car.merge_lin_controllers(mpc_lon, mpc_lat);

plots();

otherRef = 100 / 3.6;
params = {};
params.Tf = 25;
params.myCar.model = car;
params.myCar.x0 = [0 0 0 100/3.6]'; 
params.myCar.u = @mpc.get_u;
params.myCar.ref =[0 120/3.6]';
params.otherCar.model = car;
params.otherCar.x0 = [15 0 0 otherRef]'; 
params.otherCar.u = car.u_const(otherRef);
result = simulate(params);
visualization(car, result);

%plots of the invariant sets:

load('tube_mpc_data.mat','Xf','E')
figure;
    hold on;

    % Plot the polyhedron E
    E.plot('color', 'b', 'alpha', 0.6);
    % Plot the maximal invariant set Xf
    Xf.plot('color', 'r', 'alpha', 0.6);
    % Add labels and title
    xlabel('$x$', 'Interpreter', 'latex', 'FontSize', 12);
    ylabel('$V$', 'Interpreter', 'latex', 'FontSize', 12);
    title('Robust Positive Invariant Sets', 'Interpreter', 'latex', 'FontSize', 14);

    % Add legend
    legend({'$\mathcal{E}$ (Minimal Set)', '$\mathcal{X}_f$ (Maximal Set)'}, ...
           'Interpreter', 'latex', 'FontSize', 12, 'Location', 'best');

    grid on;
    hold off;

