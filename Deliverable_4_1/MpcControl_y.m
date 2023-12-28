classdef MpcControl_y < MpcControlBase
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc, Ts, H)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   X(:,1)       - initial state (estimate)
            %   x_ref, u_ref - reference state/input
            % OUTPUTS
            %   U(:,1)       - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(H/Ts); % Horizon steps
            N = N_segs + 1;      % Last index in 1-based Matlab indexing

            [nx, nu] = size(mpc.B);
            
            % Targets (Ignore this before Todo 3.2)
            x_ref = sdpvar(nx, 1);
            u_ref = sdpvar(nu, 1);
            
            % Predicted state and input trajectories
            X = sdpvar(nx, N, 'full');
            U = sdpvar(nu, N-1, 'full');
            Q = 30*eye(nx);
            R = 0.5*eye(nu);
            alpha_max = deg2rad(10);
            alpha_dif_max = deg2rad(5);
            M = [1;-1];
            m = [alpha_dif_max; alpha_dif_max];

            % Compute LQR controller for unconstrained system
            F = [0 1 0 0; 0 -1 0 0];
            f = [alpha_max; alpha_max];
            [K,Qf,~] = dlqr(mpc.A,mpc.B,Q,R);
            K = -K; 

            obj = X(:,1)'*Q*X(:,1) + U(:,1)'*R*U(:,1);
            con = (X(:,2) == mpc.A*X(:,1) + mpc.B*U(:,1)) + (F*X(:,1) <= f)+ (M*U(:,1)<=m);
            for k = 1:N-1
                obj = obj + X(:,k)'*Q*X(:,k) + U(:,k)'*R*U(:,k);
                con = con + (X(:,k+1) == mpc.A*X(:,k) + mpc.B*U(:,k)) + (F*X(:,k)<= f) + (M*U(:,k)<=m);
            end
            obj = obj + X(:,N)'*Qf*X(:,N);

            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {X(:,1), x_ref, u_ref}, {U(:,1), X, U});
        end
        
        % Design a YALMIP optimizer object that takes a position reference
        % and returns a feasible steady-state state and input (xs, us)
        function target_opti = setup_steady_state_target(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            % OUTPUTS
            %   xs, us - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            nx = size(mpc.A, 1);

            % Steady-state targets
            xs = sdpvar(nx, 1, 'full');
            us = sdpvar;
            
            % Reference position (Ignore this before Todo 3.2)
            ref = sdpvar;
            
            obj = 0;
            con = [xs == 0, us == 0];

            M = [1;-1];
            m = [deg2rad(5); deg2rad(5)];
            
            con = [M*us <= m ,...
                    xs == mpc.A*xs + mpc.B*us,...
                    ref == mpc.C*xs + mpc.D];
            
            
            % Compute the steady-state target
            target_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
        end
    end
end
