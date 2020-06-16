function pidstate=pid_loop(pidstate)

%pidstate - A software PID loop with good integerator windup protection and
%slew rate limits.
%
% This function is intended for low bandwidth computer-in-the-loop control.
% If you are aiming for maximum performance I recomend either an anlog
% control loop or a FPGA. If you want better than pid performance look
% elsewhere at some adaptive controllers.
% 
%
% Syntax:  [output1,output2] = function_name(input1,input2,input3)
%
% Inputs:
%   pidstate.meas 
%   pidstate.setpt          - desired setpoint
%   pidstate.k_int=         -integerator gain in [out]/(sec*[pidstate.erroror])
%   pidstate.k_prop=        -proportional gain with unity output scaling
%   pidstate.outlims        -output limits
%   pidstate.aw_thresh_range -how far away from the edge aw starts (full range 0-1)
%   pidstate.int_lim        -integerator limit in multiples of output range
%   pidstate.slew_lim       -output slew limit in multiples of output range/sec

%
% Outputs:
%    pidstate.ctr_output - float,actuator control  between pidstate.outlims
%    pidstate.integrator     - double used to initalize integerator;
% 
% Example: 
% pidstate.ctr_output=0;
% pidstate.setpt=0.65; %max val set pt
% pidstate.k_int=-2e-6;
% pidstate.k_prop=1e-8;
% pidstate.outlims=[-1 1];
% pidstate.aw_thresh_range=0.05; %how far away from the edge AW starts 
% pidstate.int_lim=3;        %limits on the integerator term
% pidstate.slew_lim=1e-5;    %slew rate limit


% Bugs,Improvements,Ideas
% -[x] input external timing
% -[x] bumpless start
% -[ ] optional bumpless start
% -bumpless gains to make output continious with proportional control gain change
%  -[x] prop
%  -[ ] deriv
% - higher order integeral methods
%   -[x] trapz method
%   -[ ] gen high order method  
% - feed forward terms
%   -[x] static polynomial
%   -[x] setpt change
% -[ ] Derivative term
% -[ ] this would probably be better written as a class
% -[ ] slew lim always activates on first loop
% -[ ] write output to log file
% -[ ] implement as a class

% Other m-files required: none
% Subfunctions: none
% MAT-files required: none
% See also: none

% Author: Bryce Henson
% Work address
% email: bryce.henson@live.com
% Last revision: 2019-04-01


%------------- BEGIN CODE --------------
if ~isfield(pidstate,'initalize')
    warning('\ncall pid_loop with the pidstate.initalize=true to set things up')
    pidstate.initalize=true;
end

if pidstate.initalize
    %check the inputs
    %gains
    if ~isfield(pidstate,'outlims')
       error('\nrequires output limits')
    end 
    if ~isfield(pidstate,'feed_forward')
       pidstate.feed_forward.steady_state=nan;
       pidstate.feed_forward.setpt_impulse=nan;
    end 
    if ~isfield(pidstate.feed_forward,'setpt_impulse')
        pidstate.feed_forward.setpt_impulse=nan;
    end
    if ~isfield(pidstate,'verbose')
        pidstate.verbose=0;
    end   
    if ~isfield(pidstate,'ctr_output')
        warning('\nno inital output specified using middle of output limits')
        mean(pidstate.outlims)
    end
    if ~isfield(pidstate,'slew_lim')
        warning('\nno slew rate limit specified setting as inf')
        pidstate.slew_lim=inf;
    end
    if ~isfield(pidstate,'bumpless')
        warning('\nbumpless not specified setting to true')
        pidstate.bumpless=true;
    else
        if ~isnan(pidstate.feed_forward.setpt_impulse)
            warning('impulse ff is not bumpless! set pidstate.feed_forward.setpt_impulse to nan')
        end
    end
    
    if ~isfield(pidstate,'dout_lim')
        warning('\nno output delta specified setting as inf')
        pidstate.dout_lim=inf;
    end
    if ~isfield(pidstate,'aw_thresh_range')
        %default to 1e-3 of the output range
        pidstate.aw_thresh_range=range(pidstate.outlims)*1e-3;
    end
    
    if isfield(pidstate,'set_time') && ~isnan(pidstate.set_time)
        pidstate.time=pidstate.set_time;
    else
        pidstate.time=posixtime(datetime('now'));
    end
    %convert nan to zero
    if isnan(pidstate.feed_forward.setpt_impulse)
        pidstate.feed_forward.setpt_impulse=0;
    end
    pidstate.aw=1;    
    % back calulate the integrator value to match the ctr_out to the comanded value
    pidstate.error=pidstate.meas-pidstate.setpt;
    pidstate.integrator=pidstate.ctr_output-pidstate.k_prop*pidstate.error;
    %pidstate.integrator=pidstate.ctr_output;
    pidstate.ctr_prev=pidstate.ctr_output;
    pidstate.initalize=false;
    pidstate.k_prop_prev=pidstate.k_prop;
    pidstate.setpt_prev=pidstate.setpt;
    pidstate.ff_impulse=0;
end %pidstate.initalize

old_time=pidstate.time;
if isfield(pidstate,'set_time') && ~isnan(pidstate.set_time)
    time_ctr_start=pidstate.set_time;
else
    time_ctr_start=posixtime(datetime('now'));
end
pidstate.loop_time=(time_ctr_start-old_time);
if pidstate.verbose>2,fprintf('feedback bandwidth %.2f\n',1/pidstate.loop_time), end

pidstate.ctr_prev=pidstate.ctr_output;
pidstate.error_hist=pidstate.error;

logist=@(x) 1./(1+exp(-x));%build this in as a initalization 
%scale the anti windup range to a unit output range
scaled_aw_range=pidstate.aw_thresh_range/range(pidstate.outlims);
aw_fun_range=@(x,y) (logist((x-scaled_aw_range)*10/scaled_aw_range))*(y<=0)...
    +(1-logist((x-1+scaled_aw_range)*10/scaled_aw_range))*(y>0);%needs to be defined for di==0
% xvals=linspace(0,1,1e4);
% plot(xvals,aw_fun_range(xvals,1))
% hold on
% plot(xvals,aw_fun_range(xvals,-1))
% hold off


if pidstate.verbose>1
    fprintf('previous control output %.3f\n',pidstate.ctr_prev)
end

pidstate.error=pidstate.meas-pidstate.setpt;
if pidstate.verbose>1,fprintf('error %.3f\n',pidstate.error), end
%the integeral may be calculated using a rectangle,trapezoidal, or higher order approaches
%higher order methods effectively add a lag to the integeral but give back precision
%for the moment a triangle rule is a good compomise
%rectangle rule
%di=pidstate.k_int*pidstate.loop_time*pidstate.error;
%triangle rule
di=pidstate.k_int*pidstate.loop_time*(pidstate.error+pidstate.error_hist)/2;

if abs(pidstate.ff_impulse)>0 && pidstate.loop_time>0
    temp_imp_prev=pidstate.ff_impulse;
    temp_imp_new=temp_imp_prev-pidstate.ctr_prev*pidstate.loop_time;
    if sign(temp_imp_prev)~=temp_imp_new
        pidstate.ff_impulse=0;
    else
         pidstate.ff_impulse=temp_imp_new;
    end
    
end

if  pidstate.setpt_prev~=pidstate.setpt && (numel(pidstate.feed_forward.setpt_impulse)>1 || ~isnan(pidstate.feed_forward.setpt_impulse))
    pidstate.ff_impulse=pidstate.ff_impulse+...
    diff([pidstate.setpt_prev,pidstate.setpt])*...
    polyval(pidstate.feed_forward.setpt_impulse,mean([pidstate.setpt,pidstate.setpt_prev]));
end


%could save this scaled control value to prevent recalc, would come with some issues if outlims are changed
scaled_last_control=(pidstate.ctr_prev-pidstate.outlims(1))/range(pidstate.outlims);
if pidstate.verbose>1,fprintf('scaled last control %.3f\n',scaled_last_control), end

%cancel the integeral accumulation when the impulse ff is applied
pidstate.aw=aw_fun_range(scaled_last_control,di)*(abs(pidstate.ff_impulse)==0);
if pidstate.verbose>1, fprintf('anti windup %.3f\n',pidstate.aw), end

di=di*pidstate.aw; %actuator range anti windup
pidstate.integrator=pidstate.integrator+di;

%calculate the derivative term
%TODO: use a higher order method for deriv calc
pidstate.deriv=pidstate.error-pidstate.error_hist;

if ~isnan(pidstate.feed_forward.steady_state)
    pidstate.ff_static=polyval(pidstate.feed_forward.steady_state,pidstate.setpt);
else
    pidstate.ff_static=0;
end
%if the propoptional gain has changed then back calc a correction to the integral
%correcting the integeral here is not strictly required if the slew limit is also being used
if pidstate.bumpless && pidstate.k_prop_prev~=pidstate.k_prop
    %output with old kp would have been
        %out_old=int+kp_old*err
    %output will now be 
        %out_new=int+kp_new*err+bumpless
    %(kp_old-kp_new)*err=bumpless
    %we will then increment the integeral by this bumpless term
    temp_int_bumpless=(pidstate.k_prop_prev-pidstate.k_prop)*pidstate.error;
    pidstate.integrator=pidstate.integrator+temp_int_bumpless;
    clear('temp_int_bumpless')
end
pidstate.integrator=min(max(-pidstate.int_lim,pidstate.integrator),pidstate.int_lim);
pidstate.ctr_output=pidstate.integrator+...
                    pidstate.k_prop*pidstate.error...
                    +pidstate.k_deriv*pidstate.deriv...
                    +pidstate.ff_static;%-aw_rate*k_aw_rate);

pidstate.delt_out=(pidstate.ctr_prev-pidstate.ctr_output);
pidstate.slew=pidstate.delt_out/(pidstate.loop_time);
if pidstate.verbose>1,fprintf('desired slew %.3f\n',pidstate.slew), end


%combine the delta output step limits and slew limits as an effective slew limit
combined_delt_lim=min(pidstate.slew_lim*pidstate.loop_time,pidstate.dout_lim);
%fprintf('slew before corr %f\n',pidstate.slew)
if abs(pidstate.delt_out)>combined_delt_lim
    %fprintf('pre mod int %f\n',pidstate.integrator)
    pidstate.integrator=pidstate.ctr_prev-combined_delt_lim*sign(pidstate.slew)...
        -pidstate.k_prop*pidstate.error...
        -pidstate.k_deriv*pidstate.deriv...
        -pidstate.ff_static;
    %fprintf('slew mod int %f\n',pidstate.integrator)
    pidstate.aw_slew=1; %not a anti windup should just call slew lim
else
    pidstate.aw_slew=0;
end

%recalc output
%TODO:some cacluations are repeated here
pidstate.ctr_output=pidstate.integrator+...
                    pidstate.k_prop*pidstate.error...
                    +pidstate.k_deriv*pidstate.deriv...
                    +pidstate.ff_static;
if pidstate.loop_time>0 && abs(pidstate.ff_impulse)>0
    pidstate.ctr_output=pidstate.ctr_output+pidstate.ff_impulse/pidstate.loop_time;
end
% if abs(pidstate.ff_impulse)>abs(pidstate.loop_time*pidstate.outlims(1+(pidstate.ff_impulse>0)))
%     pidstate.ctr_output=inf*sign(pidstate.ff_impulse);
% else
%     pidstate.ff_impulse=0;
% end
pidstate.ctr_output=min(max(pidstate.outlims(1),pidstate.ctr_output),pidstate.outlims(2));
pidstate.slew=(pidstate.ctr_prev-pidstate.ctr_output)/(pidstate.loop_time);
if pidstate.verbose>1 && pidstate.aw_slew
    fprintf('clamped slew %.3f\n',pidstate.slew)
end
if pidstate.verbose>1 &&pidstate.aw_slew
    fprintf('output %.3f\n',pidstate.ctr_output)
end
%fprintf('slew after corr %f\n',pidstate.slew)

%set the time that the controler finishes to be the set_time
if isfield(pidstate,'set_time') && ~isnan(pidstate.set_time)
    pidstate.time=pidstate.set_time;
    pidstate.set_time=nan;
else
    pidstate.time=posixtime(datetime('now'));
end

pidstate.setpt_prev=pidstate.setpt;
pidstate.k_prop_prev=pidstate.k_prop;

if pidstate.verbose>1
    fprintf('loop time %.3f\n',pidstate.loop_time)
end
end