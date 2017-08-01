classdef SPM_Controller < handle
    properties
        v_bus = 0;
        i_a = 0;
        i_b = 0;
        i_c = 0;
        i_d = 0;
        i_q = 0;
        u = 0;
        v = 0;
        w = 0;
        v_d = 0;
        v_q = 0;
        theta = 0;
    end
    methods
        function [d, q] = calc_i_dq(obj, i_a, i_b, i_c, theta)
            abc = @(theta) sqrt(2/3)*[cos(-theta), sin(-theta), 1/sqrt(2);
                cos((2*pi/3)-theta), sin((2*pi/3)-theta), 1/sqrt(2);
                cos((-2*pi/3)-theta), sin((-2*pi/3)-theta), 1/(sqrt(2))];
            dq0 = @(theta) abc(theta)';
            [od, q, o] = dq0(theta)*[i_a; i_b; i_c];
        end
    end
end