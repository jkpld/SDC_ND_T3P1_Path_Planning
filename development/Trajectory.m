classdef Trajectory
    % Simple piecewise polynomial class that can have one or two pieces
    
    properties (SetAccess = private)
        coef1(1,:) double
        coef2(1,:) double
        T(1,1) double
    end
    properties (Hidden, SetAccess = private)
        one_piece
    end
    methods
        function obj = Trajectory(coef, state_at_T, T)
            
            % Note: polynomials will not be stored in matlab's way, but as
            % follows
            % A(x) = a(1) + a(2)*x + a(3)*x^2 + ... = a(i)*x^(i-1)
            
            if nargin == 0
                obj.coef1 = 0;
                obj.T = inf;
                obj.one_piece = true;
            else
                if nargin == 1
                    % Construct a single piece polynomial valid to t=inf.
                    obj.coef1 = coef;
                    obj.T = inf;
                    obj.one_piece = true;
                else
                    % Cunstruct a two piece polynomial with coefficeints
                    % coef for t<=T and use state state_at_T for times t>T

                    % Integrate the state to get the trajectory
                    % T(t) = x(1) + x(2)*t + 0.5*x(3)*t^2
                    c2 = [0.5*state_at_T(3), state_at_T(2), state_at_T(1)];

                    coef = [zeros(1,max(3-numel(coef),0)), coef];
                    obj.coef1 = coef;
                    obj.coef2 = [zeros(1,numel(coef)-3), c2];
                    obj.T = T;
                    obj.one_piece = false;
                end
            end
        end
        
        function val = evaluate(obj, t, n)
            % Evaluate the all derivatives of the piecewise polynomial
            % (including the 0'th derivative, i.e. the polynomial value) at
            % the values t.
            % If input n is specified, then only the n'th derivative is
            % returned. -Note that all derivatives are computed regardless,
            % unless n=0, in which case only the polynomial value is
            % returned. 
            
            if nargin<3
                n = -1;
            end
            
            if n == 0
                if obj.one_piece
                    val = polyval(obj.coef1, t);
                else
                    val = [polyval(obj.coef1, t(t < obj.T)); 
                           polyval(obj.coef2, t(t >= obj.T)-obj.T)];
                end
            else
                if obj.one_piece
                    val = polyeval(flip(obj.coef1), t);
                else
                    val = [polyeval(flip(obj.coef1), t(t < obj.T));
                            polyeval(flip(obj.coef2), t(t >= obj.T)-obj.T)];
                    
                end
            
                if n > 0
                    val = val(:,n+1); % Only return the n'th derivative
                end
            end
        end
        
        function coef = coefs_at(obj, t)
            if obj.one_piece
                [~,coef] = polyeval(flip(obj.coef1), t(1));
            else
                if t(1) < obj.T
                    [~,coef] = polyeval(flip(obj.coef1), t(1));
                else
                    [~,coef] = polyeval(flip(obj.coef2), t(1)-obj.T);
                end
            end
        end
        
        function state = state_at(obj, t)
            state = obj.evaluate(t);
            state = state(:,1:3);
        end
    end
end