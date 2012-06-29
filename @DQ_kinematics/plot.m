function plot(obj,theta,varargin)
    
    robot = obj.robot_RT;
    has_options = 0;
    if nargin < 2
        error('Usage: plot(robot,theta,[options])');
    elseif nargin >= 3
        has_options = 1;
    end
    
    if size(theta,2) == 1
        theta = theta';
    end
    
    if has_options
        plot(robot,theta, varargin{:});
    else
        plot(robot,theta);
    end
    %translating the base if necessary
    rh = findobj('Tag', robot.name);
    rr = get(rh, 'UserData');
    base = translation(obj.base);
    rr.base = transl(base.q(2:4));
    set(rh, 'UserData', rr);
    if has_options
        plot(robot,theta, varargin{:});
    else
        plot(robot,theta);
    end
end