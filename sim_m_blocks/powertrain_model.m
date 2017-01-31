function [fxf, gear_next] = powertrain_model(throttle, x, gear, param)
    % nonliear_bicycle_model Nonlinear bicycle model dynamics with fiala tire models
    %    Inputs: throttle - Normalized throttle pedal travel distance [0:1]
    %            x        - State vector (s, ey, epsi, vx, vy, r)
    %            gear     - Current gear [-1:5]
    %            param - Powertrain parameter structure
    %                param.wheel_radius      - Road wheel radius [m]
    %                param.gear_ratio        - Gear ratio for all possible gears [-]
    %                param.diff_ratio        - Diferential ratio [-];
    %                param.efficiency        - Overall powetrain efficiency [-]
    %                param.engine_rpm_map    - Torque map's engine speed [rpm]
    %                param.engine_torque_map - Torque map's torque output [Nm]
    %
    %    Outputs: fxf - Powertrain actuated force [N]
    %             gear_next - Next gear to be engaged [-1:5]
    %
    %    Author: Carlos M. Massera
    %    Instituition: University of São Paulo

    wheel_radius = param.wheel_radius;
    gear_ratio = param.gear_ratio(gear + 2);
    diff_ratio = param.diff_ratio;
    efficiency = param.efficiency;

    % Clip throttle
    throttle = max(min(throttle, 1), 0);

    % Perform motor mapping
    engine_rpm = x(4) / wheel_radius * gear_ratio * diff_ratio / efficiency * 60 / 2 / pi;
    torque_max = interp1(param.engine_rpm_map, ...
                         param.engine_torque_map, min(param.engine_rpm_map(end), engine_rpm));
    torque = throttle * torque_max;  % At the engine
    fxf = torque * gear_ratio * diff_ratio * efficiency / wheel_radius;

    % Gearbox logic (forward only)
    if gear == 0
        if throttle > 1e-2
            gear_next = 1;
        else
            gear_next = 0;
        end
    else
        if engine_rpm < 1000
            gear_next = gear - 1;
        else
            if engine_rpm > 4000 && gear < 5
                gear_next = gear + 1;
            else
                gear_next = gear;
            end
        end
    end
end