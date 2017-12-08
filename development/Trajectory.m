classdef Trajectory
    properties
        pp(1,4) cell
    end
    properties (Hidden)
        coef(1,:) double
        T(1,1) double
    end
    methods
        function obj = Trajectory(coef, state_at_T, T)
            if nargin == 0
                obj.pp{4} = mkpp([-inf,inf], 0);
                obj.pp{1} = mkpp([-inf,inf], 0);
                obj.pp{2} = mkpp([-inf,inf], 0);
                obj.pp{3} = mkpp([-inf,inf], 0);
                return;
            end
            c1 = [zeros(1,max(3-numel(coef),0)), coef];
            c1d = Trajectory.polyder(coef);
            c1dd = Trajectory.polyder(c1d);
            c1ddd = Trajectory.polyder(c1dd);
            
            obj.coef = coef;
            
            if nargin == 1
                % Construct a single piece polynomial valid to t=inf.
                
                obj.pp = cell(1,4);
                
                obj.pp{1} = mkpp([0,inf], c1);
                obj.pp{2} = mkpp([0,inf], c1d);
                obj.pp{3} = mkpp([0,inf], c1dd);
                obj.pp{4} = mkpp([0,inf], c1ddd);
                
                obj.T = inf;
            else
                % A time horizon and the state at the time horizon was
                % given
                state_at_T = [0.5*state_at_T(3), state_at_T(2), state_at_T(1)];
                c2 = [zeros(1,max(numel(coef)-3,0)), state_at_T];
                c2d = Trajectory.polyder(c2);
                c2dd = Trajectory.polyder(c2d);
                c2ddd = Trajectory.polyder(c2dd);

                obj.pp = cell(1,4);

                obj.pp{1} = mkpp([0,T,inf], [c1; c2]);
                obj.pp{2} = mkpp([0,T,inf], [c1d; c2d]);
                obj.pp{3} = mkpp([0,T,inf], [c1dd; c2dd]);
                obj.pp{4} = mkpp([0,T,inf], [c1ddd; c2ddd]);

                
                obj.T = T;
            end
        end
        
        function val = evaluate(obj, t, n)
            if nargin<3
                n = 0;
            end
            
            if n>3
                error('Trajectory:only3rdDeriv','Only up to the 3rd derivative can be returned.')
            end
            
            val = ppval(obj.pp{n+1}, t);
        end
        
        function state = state_at(obj, t)
            state = [obj.evaluate(t,0), obj.evaluate(t,1), obj.evaluate(t,2)];
        end
    end
    
    methods (Access = private, Static)
        function coef = polyder(coef)
            % Same as normal polyder, but the size of the returned array is
            % always one less than the size of the input array. (No
            % trimming high order zeros.)
            n = numel(coef);
            coef = [0,coef(1:n-1).*(n-1:-1:1)];
        end
    end
end