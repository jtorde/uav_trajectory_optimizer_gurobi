classdef SimplePID
    %SIMPLEPID Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        kp
        ki
        kd
        min
        max
        tau
        
        derivative
        integral
        last_error
    end
    
    methods
        function obj = SimplePID(kp,ki,kd,min,max)
            obj.kp = kp;
            obj.ki = ki;
            obj.kd = kd;
            obj.min = min;
            obj.max = max;
            obj.tau = 0.05;
            obj.derivative = [];
            obj.last_error = [];
            obj.integral = 0;
        end
        
        function [obj, u_sat] = run(obj, err, dt, derivative, pclamp)
            if ~isempty(obj.kp)
                % Proportional error clamp, if specified
                if isempty(pclamp), e = err;
                else, e = clamp(err, pclamp); end
                p_term = obj.kp*e;
            else
                p_term = 0;
            end
            
            if ~isempty(obj.kd)
                if ~isempty(derivative)
                    obj.derivative = derivative;
                else
                    if isempty(obj.last_error)
                        obj.derivative = 0;
                        obj.last_error = 0;
                    else
                        % dirty derivative
                        obj.derivative = (2*obj.tau - dt)/(2*obj.tau + dt)*obj.derivative + 2/(2*obj.tau + dt)*(err - obj.last_error);
                    end
                end
                d_term = obj.kd*obj.derivative;
            else
                d_term = 0;
            end
            
            if ~isempty(obj.ki)
                obj.integral = obj.integral + (dt/2)*(err + obj.last_error);
                i_term = obj.ki*obj.integral;
            else
                i_term = 0;
            end
            
            % combine
            u = p_term + d_term + i_term;
            
            % saturate
            u_sat = u;
            if u < obj.min, u_sat = obj.min; end
            if u > obj.max, u_sat = obj.max; end
            
            % integrator anti-windup
            if ~isempty(obj.ki)
                if abs(p_term + d_term) > abs(u_sat)
                    % PD is already saturating, so set integrator to 0 but
                    % don't let it run backwards
                    obj.integral = 0;
                else
                    % otherwise only let integral term at most take us just
                    % up to saturation
                    obj.integral = (u_sat - p_term - d_term) / obj.ki;
                end
            end
            
            % book keeping
            obj.last_error = err;
        end
    end
end

function v = clamp(v, limit)
if v >  limit, v =  limit; end
if v < -limit, v = -limit; end
end
