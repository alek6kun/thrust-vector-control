classdef MpcControl_x < MpcControlBase
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
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % Define angle constraints
            beta_max = deg2rad(10);
            max_B_dif = deg2rad(5);
            % Define the weight matrices
            Q = 30*eye(nx); Q(1,1) = 20; Q(2,2) = 15;
            R = 0.5*eye(nu); 
            M = [1;-1];
            m = [max_B_dif; max_B_dif];
           % Compute LQR controller for unconstrained system
            F = [0 1 0 0; 0 -1 0 0];
            f = [beta_max; beta_max];
            [K,Qf,~] = dlqr(mpc.A,mpc.B,Q,R);
            K = -K; 

            %Compute maximum invariant set
            Xf = polytope([F;M*K],[f;m]);
            Acl = [mpc.A+mpc.B*K];
            while 1
                prevXf = Xf;
                [T,t] = double(Xf);
                preXf = polytope(T*Acl,t);
                Xf = intersect(Xf, preXf);
                if isequal(prevXf, Xf)
                    break
                end
            end
            [Ff,ff] = double(Xf);

            % Plot maximum invariant set
            figure;
            Xf.projection(1:2).plot();
            xlabel('State 1 : β [rad]');
            ylabel('State 2 : ωy [rad/s]');
            title('Maximum Invariant Set Projection for x (β/ωy)');
            legend('Invariant Set for x (β/ωy)');

            figure;
            Xf.projection(2:3).plot();
            xlabel('State 1 : Vx [m/s]');
            ylabel('State 2 : β [rad] ');
            title('Maximum Invariant Set Projection for x (Vx/β)');
            legend('Invariant Set for x (Vx/β)');

            figure;
            Xf.projection(3:4).plot();
            xlabel('State 1 : x [m]');
            ylabel('State 2 : Vx [m/s] ');
            title('Maximum Invariant Set Projection for x (x/Vx)');
            legend('Invariant Set for x (x/Vx)');

            obj = X(:,1)'*Q*X(:,1) + U(:,1)'*R*U(:,1);
            con = (X(:,2) == mpc.A*X(:,1) + mpc.B*U(:,1)) + (F*X(:,1) <= f)+ (M*U(:,1)<=m);
            for k = 1:N-1
                obj = obj + X(:,k)'*Q*X(:,k) + U(:,k)'*R*U(:,k);
                con = con + (X(:,k+1) == mpc.A*X(:,k) + mpc.B*U(:,k)) + (F*X(:,k)<= f) + (M*U(:,k)<=m);
            end
            con = con + (Ff*X(:,N) <= ff);
            obj = obj + X(:,N)'*Qf*X(:,N);
            % Return YALMIP optimizer object
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
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
            obj = [];
            con = [xs == 0, us == 0];

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Compute the steady-state target
            target_opti = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
        end
    end
end