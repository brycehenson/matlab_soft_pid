%testing the controler against a simulated plant-actuator model using the intuative tank level example
%TODO
%   - build this into a testing environment
%DONE
%   - add a time lag between measurment and response
%     - this gets complicated if this lag is more than the pid_poll_time
%     - will create an actuation queue(FIFO) where the cacluated actuation is stored untill its applied to the
%     system
%   - implement derivative term


%controler parameters
pidstate=[];
pidstate.initalize=1;
pidstate.ctr_output=0;
pidstate.setpt=0;
pidstate.verbose=3;
pidstate.k_int=-800;%-800;
pidstate.k_prop=-50;%-50;
pidstate.k_deriv=0;%-20;%
pidstate.feed_forward.steady_state=[0.3,80,0]; %ff between setpt and ctr out
pidstate.feed_forward.setpt_impulse=[0]; %impulse that should be added to integeral for some change in 
                                                %setpt as a function of the mean of the two setpts
pidstate.outlims=[-1 1]*4e3;
pidstate.aw_thresh_range=0.01; %how far away from the edge AW starts 
pidstate.int_lim=5000;        %limits on the integerator term
pidstate.slew_lim=inf;    %
pidstate.dout_lim=10000;
pidstate.bumpless=true;
pidstate.integrator=0;%dont usualy need to do this

% simulated plant paramters
sim.set.leak_drift=0e-1; %random walk of the load
sim.set.tmax=3; %how long to run the sim for
sim.set.dt=1e-4; %simulate the plant at this rate
sim.set.pid_poll_time=1e-2; %call the controller this often
sim.set.act_lag=3e-2;  %how long it takes the controler to respond
sim.set.load_fun=@(x) polyval([0.5,100,0],x);
sim.set.setpt_sch=[[0.5;10],[2;20]];
sim.set.load_sch=[[5.2;15],[8;-10]];
sim.set.kp_sch=[];%[[0.7;-15],[2;-15]];
sim.run.plant_level=0;
sim.set.flow_strength=1;
sim.set.tank_area=1;
sim.run.plant_drift_load=0;

%%initalize sim loop
kp_sch=sim.set.kp_sch;
setpt_sch=sim.set.setpt_sch;
load_sch=sim.set.load_sch;
iimax=ceil(sim.set.tmax/sim.set.dt);
sim.run.act=pidstate.ctr_output;%actuator value
sim.run.time=0;
sim.run.itt=0;
sim.set.load_walk_rate=0;
pidstate.time=0; %only for simulation for realtime(ish) do not set

sim.history.plant_level=nan(1,iimax);
sim.history.plant_load=nan(1,iimax);
sim.history.time=nan(1,iimax);
sim.history.ctr=nan(1,iimax);
sim.history.int=nan(1,iimax);

%initalize the actuation queue
actuator_queue=nan(2,0); %time,value


fprintf('itt %06i:%06i',iimax,0)
while sim.run.itt<iimax
    %print itteration number
    %if mod(sim.run.itt,100)==0; fprintf('\b\b\b\b\b\b%06i',sim.run.itt);end
    sim.run.time=sim.run.time+sim.set.dt; %increment time
    sim.run.itt=sim.run.itt+1;%increment itteration
    %run the controller if it has been sim.set.pid_poll_time since it has last run
    if sim.run.time>pidstate.time+sim.set.pid_poll_time
        pidstate.meas=sim.run.plant_level;
        pidstate.set_time=sim.run.time;
        pidstate=pid_loop(pidstate);
        actuator_queue=cat(2,actuator_queue,[sim.run.time+sim.set.act_lag;pidstate.ctr_output]);
    end
    
    if size(actuator_queue,2)>0 && sim.run.time>actuator_queue(1,1)
        sim.run.act=actuator_queue(2,1);
        actuator_queue=actuator_queue(:,2:end); %remove from the queue
    end
    
    if size(load_sch,2)>0 && sim.run.time>load_sch(1,1)
        sim.run.plant_drift_load=load_sch(2,1);
        load_sch=load_sch(:,2:end);
    end
    
    if size(setpt_sch,2)>0 && sim.run.time>setpt_sch(1,1)
        pidstate.setpt=setpt_sch(2,1);
        setpt_sch=setpt_sch(:,2:end);
    end
    
    if size(kp_sch,2)>0 && sim.run.time>kp_sch(1,1)
        pidstate.k_prop=kp_sch(2,1);
        kp_sch=kp_sch(:,2:end);
    end
    
    sim.run.plant_drift_load=sim.run.plant_drift_load+sim.set.load_walk_rate*(rand(1)-0.5)*2;
    sim.run.plant_load=sim.run.plant_load_drift+sim.set.leak_height_fun(sim.run.plant_level);
    sim.run.plant_level=sim.run.plant_level+(1/sim.set.tank_area)*sim.set.dt*...
        (sim.set.flow_strength*sim.run.act-sim.run.plant_load);

    sim.history.plant_level(sim.run.itt)=sim.run.plant_level;
    sim.history.plant_load(sim.run.itt)=sim.run.plant_load;
    sim.history.ctr(sim.run.itt)=sim.run.act;
    sim.history.time(sim.run.itt)=sim.run.time;
    sim.history.int(sim.run.itt)=pidstate.integrator;

end
fprintf('\n')


%%
figure(1)
subplot(4,1,1)
plot(sim.history.time,sim.history.plant_level)
ylim([-5,25])
ylabel('plant val')
subplot(4,1,2)
plot(sim.history.time,sim.history.ctr)
ylabel('ctr out')
subplot(4,1,3)
plot(sim.history.time,sim.history.int)
ylabel('ctr int')
set(gcf,'color','w')
subplot(4,1,4)
plot(sim.history.time,sim.history.plant_load)
ylabel('plant load')
set(gcf,'color','w')

%%

fprintf('itt %06i:%06i',iimax,0)
fprintf('\b\b\b\b\b\b%06i',1)
