classdef AcrobotController < DrakeSystem
    properties
        p 
    end
    methods
        function obj = AcrobotController(plant)
          obj = obj@DrakeSystem(0,0,4,1,true,true);
          obj.p = plant;
          obj = obj.setInputFrame(plant.getStateFrame);
          obj = obj.setOutputFrame(plant.getInputFrame);
        end
    
        function u = output(obj,t,~,x)
            q = x(1:2);
            qd = x(3:4);
            % unwrap angles q(1) to [0,2pi] and q(2) to [-pi,pi]
            q(1) = q(1) - 2*pi*floor(q(1)/(2*pi));
            q(2) = q(2) - 2*pi*floor((q(2) + pi)/(2*pi));
            %%%% put your controller here %%%%
            % You might find some of the following functions useful
            % [H,C,B] = obj.p.manipulatorDynamics(q,qd);
            % com_position = obj.p.getCOM(q);
            % mass = obj.p.getMass();
            % gravity = obj.p.gravity;
            % Recall that the kinetic energy for a manipulator given by .5*qd'*H*qd
            x_des = [pi;0;0;0];
            x = [q; qd];
            
            %LQR
            %Manually calculated the dynamics
            %Using p.dynamics wasn't working and had no clue why (see
            %below)
            l1 = 1.1; l2 = 2.1;  
            m1 = 1; m2 = 1;  
            g = -obj.p.gravity(3);
            lc1 = .55; lc2 = 1.05; 
            Ic1 = .083;  Ic2 = .33;
            I1 = Ic1 + m1*lc1^2; I2 = Ic2 + m2*lc2^2;
            H_fixed = [I1+I2+m2*l1^2+2*m2*l1*lc2 I2+m2*l1*lc2; I2+m2*l1*lc2 I2];
            dGdq = [-g*(m1*lc1+m2*l1+m2*lc2) -m2*g*lc2; -m2*g*lc2 -m2*g*lc2];
            A_lin = [zeros(2) eye(2); -inv(H_fixed)*dGdq zeros(2) ];
            B_lin = [0;0;inv(H_fixed)*[0;1]];
            Q = diag([1,1,1,1]);
            R = 1;
            [K,S] = lqr(A_lin,B_lin,Q,R);
            u_lqr = -K*(x-x_des);
            
            %{
            This is the code I tried to get A and B,
            df/dx and df/du at the fixed points should be
            exactly A/B, but for some reason my code wasn't working
            (possibly a problem with drake on my machine?)
            
            [f,df] = obj.p.dynamics(0,x_des,0);
            A_lin = df(1:4,2:5);
            B_lin = df(1:4,6);
            Q = diag([1,1,1,1]);
            R = 1;
            [K,S] = lqr(A_lin,B_lin,Q,R);
            u_lqr = -K*(x-x_des);
            %}            
            
            %Energy shaping
            g = -obj.p.gravity(3);
            l = obj.p.getCOM([pi;0]);
            l = l(2);
            m = obj.p.getMass();
            E_des = m*g*l;
            [H,C,B] = obj.p.manipulatorDynamics(q,qd);
            k1 = 1;
            k2 = 5;
            k3 = 1;
            l = obj.p.getCOM(q);
            l = l(2);
            E = .5*qd'*H*qd + m*g*l;
            u_e = k1*(E_des-E)*qd(2);
            %PFL of desired y
            y = -k2*q(2) - k3*qd(2);
            ddq1 = -(H(1,2)*y + C(1))/H(1,1);
            u_p = H(1,2)*ddq1+H(2,2)*y+C(2);
            u_swing = u_e + u_p;
                        
            %Calculate cost and choose which controller to use
            threshold = 5000;
            cost = (x-x_des)'*S*(x-x_des);
            if (cost < threshold)
                u = u_lqr;
            else
                u = u_swing;
            end
            
            %%%% end of your controller %%%%
            % leave this line below, it limits the control input to [-20,20]
            u = max(min(u,20),-20);
            % This is the end of the function
        end
    end
    methods (Static)
        function [t,x]=run()
            plant = PlanarRigidBodyManipulator('Acrobot.urdf');
            controller = AcrobotController(plant);
            v = plant.constructVisualizer;
            sys_closedloop = feedback(plant,controller);
            x0 = [.1*(rand(4,1) - 1)]; % start near the downward position
            %x0 = [pi - .1*randn;0;0;0];  % start near the upright position
            %x0 = [pi-0.1;0.15;0;0.5];  % use this for problem 3 (b)
            xtraj=simulate(sys_closedloop,[0 5],x0);
            v.axis = [-4 4 -4 4];
            playback(v,xtraj);
            t = xtraj.pp.breaks;
            x = xtraj.eval(t);
            figure(11);
            subplot(1,2,1)
            plot(x(1,:),x(3,:),'r-','LineWidth',2);
            hold on;
            plot(pi,0,'g.','LineWidth',10);
            hold off;
            xlabel('theta 1');
            ylabel('theta 1 dot');
            subplot(1,2,2)
            plot(x(2,:),x(4,:),'r-','LineWidth',2);
            hold on;
            plot(0,0,'g.','LineWidth',10);
            hold off;
            xlabel('theta 2');
            ylabel('theta 2 dot');
        end
    end
end