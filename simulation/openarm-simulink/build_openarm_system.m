function build_openarm_system()
% Builds openarm_system.slx: one OpenArm v2 arm (rigidBodyTree from the URDF) on seven averaged FOC actuators
% (SteadyWin GIM, openarm_params.m) fed through a Simscape network of the support board and the daisy chain.
%   signal side : computed-torque joint control -> iq command -> dq current loops (PI + back-EMF feedforward, voltage
%                 limit from each board's local bus) -> output torque through gear efficiency and friction -> arm.
%   power side  : source + lead -> BAT+ (C1/C59/C60 lumped) -> hot-swap (variable resistor: current/power limit,
%                 fault timer, LM5069-2 retry) -> LTC4359 ideal diode (switched resistance) -> bank -> primary/backup chopper -> seven hops
%                 (0.2 uH, 10 mOhm) to seven boards (76 uF each), each drawing its inverter DC power as a current.
% Workspace variables used at simulation time: Pm (numeric struct), robot_sim (with payload), robot_free (without),
% ref_ts, scen (see run_openarm.m).
mdl = 'openarm_system';
here = fileparts(mfilename('fullpath'));
if bdIsLoaded(mdl), close_system(mdl,0); end
new_system(mdl);
set_param(mdl,'Solver','ode23t','RelTol','1e-4','AbsTol','auto','MaxStep','50e-6','StopTime','3', ...
    'ReturnWorkspaceOutputs','on','SignalLogging','off');
P = @(b) get_param(b,'PortHandles');
nl = newline;
blk = @(lib,name,pos) add_block(lib,[mdl '/' name],'Position',pos);
ln = @(a,b) add_line(mdl,a,b,'autorouting','on');

%% ---------------- signal side
ref = blk('simulink/Sources/From Workspace','ref',[30 40 110 70]); set_param(ref,'VariableName','ref_ts','Interpolate','on');
iq = blk('simulink/Continuous/Integrator','int_q',[560 40 600 70]); set_param(iq,'InitialCondition','scen.q0');
iqd = blk('simulink/Continuous/Integrator','int_qd',[480 40 520 70]); set_param(iqd,'InitialCondition','zeros(7,1)');
ix = blk('simulink/Continuous/Integrator','int_x',[480 140 520 170]); set_param(ix,'InitialCondition','zeros(35,1)');
mm = blk('robotmaniplib/Joint Space Mass Matrix','M',[660 20 760 60]); set_param(mm,'RigidBodyTree','robot_sim','SimulateUsing','Code generation');
vp = blk('robotmaniplib/Velocity Product Torque','VP',[660 80 760 120]); set_param(vp,'RigidBodyTree','robot_sim','SimulateUsing','Code generation');
gt = blk('robotmaniplib/Gravity Torque','G',[660 140 760 180]); set_param(gt,'RigidBodyTree','robot_sim','SimulateUsing','Code generation');
% the same terms for the arm without its payload (plant after Pm.t_release)
mm0 = blk('robotmaniplib/Joint Space Mass Matrix','M0',[800 20 900 60]); set_param(mm0,'RigidBodyTree','robot_free','SimulateUsing','Code generation');
vp0 = blk('robotmaniplib/Velocity Product Torque','VP0',[800 80 900 120]); set_param(vp0,'RigidBodyTree','robot_free','SimulateUsing','Code generation');
gt0 = blk('robotmaniplib/Gravity Torque','G0',[800 140 900 180]); set_param(gt0,'RigidBodyTree','robot_free','SimulateUsing','Code generation');
clk = blk('simulink/Sources/Clock','clock',[30 240 60 260]);
ctrl = blk('simulink/User-Defined Functions/MATLAB Function','ctrl',[260 100 400 300]);
set_ml(mdl,'ctrl',fileread(fullfile(here,'ctrl_fcn.m')),{'Pm'});
% hot-swap supervisor and chopper logic: discrete 5 us
sup = blk('simulink/User-Defined Functions/MATLAB Function','supervisor',[260 420 400 560]);
set_ml(mdl,'supervisor',fileread(fullfile(here,'supervisor_fcn.m')),{'Pm'});
set_param([mdl '/supervisor'],'SystemSampleTime','Pm.Ts_sup');

%% ---------------- power side (Simscape)
gnd = blk('fl_lib/Electrical/Electrical Elements/Electrical Reference','gnd',[700 760 730 790]);
slv = blk(['nesl_utility/Solver' nl 'Configuration'],'solver',[620 760 660 790]);
ln(P(slv).RConn,P(gnd).LConn);
vsrc = blk('fl_lib/Electrical/Electrical Sources/DC Voltage Source','Vsrc',[640 600 670 640]); set_param(vsrc,'v0','Pm.v_src');
ln(P(vsrc).RConn,P(gnd).LConn);
lead = blk('fl_lib/Electrical/Electrical Elements/Inductor','lead',[700 590 740 610]);
set_param(lead,'l','Pm.l_lead','r','Pm.r_lead + Pm.r_src');
ln(P(vsrc).LConn,P(lead).LConn);
cin = blk('fl_lib/Electrical/Electrical Elements/Capacitor','C_in',[780 640 800 680]);
cap(cin,'80e-6','0.05','Pm.v_src'); ln(P(lead).RConn,P(cin).LConn); ln(P(cin).RConn,P(gnd).LConn);
% source current and hot-swap
isrc = blk('fl_lib/Electrical/Electrical Sensors/Current Sensor','I_src',[820 580 860 620]);
ln(P(lead).RConn,P(isrc).LConn);
hs = blk('fl_lib/Electrical/Electrical Elements/Variable Resistor','hot_swap',[900 580 940 620]); set_param(hs,'Rmin','1e-5');
ln(P(isrc).RConn(2),P(hs).LConn(2));
vds = blk('fl_lib/Electrical/Electrical Sensors/Voltage Sensor','V_ds',[900 500 940 540]);
ln(P(lead).RConn,P(vds).LConn); ln(P(vds).RConn(2),P(hs).RConn);
vbat = blk('fl_lib/Electrical/Electrical Sensors/Voltage Sensor','V_bat',[780 500 820 540]);
ln(P(lead).RConn,P(vbat).LConn); ln(P(vbat).RConn(2),P(gnd).LConn);
% LTC4359 ideal diode as a switched resistance (supervisor output 6): a piecewise-linear diode chattered at zero
% current when regeneration began with the bus within millivolts of BAT+
idio = blk('fl_lib/Electrical/Electrical Elements/Variable Resistor','ideal_diode',[980 590 1020 610]); set_param(idio,'Rmin','1e-5');
ln(P(hs).RConn,P(idio).LConn(2));
bank = blk('fl_lib/Electrical/Electrical Elements/Capacitor','bank',[1060 640 1080 680]);
cap(bank,'Pm.bank_c','Pm.bank_esr','scen.v0'); ln(P(idio).RConn,P(bank).LConn); ln(P(bank).RConn,P(gnd).LConn);
vbus = blk('fl_lib/Electrical/Electrical Sensors/Voltage Sensor','V_bus',[1100 500 1140 540]);
ln(P(idio).RConn,P(vbus).LConn); ln(P(vbus).RConn(2),P(gnd).LConn);
chop = blk('fl_lib/Electrical/Electrical Elements/Variable Resistor','chopper',[1100 700 1140 740]); set_param(chop,'Rmin','1e-3');
ln(P(idio).RConn,P(chop).LConn(2)); ln(P(chop).RConn,P(gnd).LConn);
caux = blk(['fl_lib/Electrical/Electrical Sources/Controlled Current' nl 'Source'],'aux_load',[1160 700 1200 740]);
ln(P(caux).LConn,P(gnd).LConn); ln(P(caux).RConn(2),P(idio).RConn);
% daisy chain
ihop = blk('fl_lib/Electrical/Electrical Sensors/Current Sensor','I_hop1',[1180 580 1220 620]);
ln(P(idio).RConn,P(ihop).LConn);
prev = P(ihop).RConn(2);
vsum = blk('simulink/Signal Routing/Mux','v_mux',[1500 300 1505 420]); set_param(vsum,'Inputs','7');
idem = blk('simulink/Signal Routing/Demux','i_demux',[1500 440 1505 560]); set_param(idem,'Outputs','7');
for k = 1:7
    x0 = 1240 + (k-1)*130;
    h = blk('fl_lib/Electrical/Electrical Elements/Inductor',sprintf('hop%d',k),[x0 590 x0+40 610]);
    set_param(h,'l','Pm.l_hop','r','Pm.r_hop');
    ln(prev,P(h).LConn);
    c = blk('fl_lib/Electrical/Electrical Elements/Capacitor',sprintf('C_local%d',k),[x0+60 640 x0+80 680]);
    cap(c,'Pm.c_local','Pm.esr_local','scen.v0'); ln(P(h).RConn,P(c).LConn); ln(P(c).RConn,P(gnd).LConn);
    cs = blk(['fl_lib/Electrical/Electrical Sources/Controlled Current' nl 'Source'],sprintf('board%d',k),[x0+60 700 x0+100 740]);
    ln(P(cs).LConn,P(gnd).LConn); ln(P(cs).RConn(2),P(h).RConn);
    vs = blk('fl_lib/Electrical/Electrical Sensors/Voltage Sensor',sprintf('V_local%d',k),[x0+60 500 x0+100 540]);
    ln(P(h).RConn,P(vs).LConn); ln(P(vs).RConn(2),P(gnd).LConn);
    a = blk(['nesl_utility/PS-Simulink' nl 'Converter'],sprintf('v_ps%d',k),[x0+60 440 x0+90 460]);
    ln(P(vs).RConn(1),P(a).LConn); ln(P(a).Outport,P(vsum).Inport(k));
    s = blk(['nesl_utility/Simulink-PS' nl 'Converter'],sprintf('i_ps%d',k),[x0+110 700 x0+140 720]);
    set_param(s,'FilteringAndDerivatives','filter','SimscapeFilterOrder','1','InputFilterTimeConstant','5e-6');
    ln(P(idem).Outport(k),P(s).Inport); ln(P(s).RConn,P(cs).RConn(1));
    prev = P(h).RConn;
end
% PS <-> Simulink for the support board
ps = @(name,pos,src) ps_out(mdl,name,pos,src);
v_bat = ps('v_bat_ps',[780 440 810 460],P(vbat).RConn(1));
v_ds = ps('v_ds_ps',[900 440 930 460],P(vds).RConn(1));
i_src = ps('i_src_ps',[820 440 850 460],P(isrc).RConn(1));
v_bus = ps('v_bus_ps',[1100 440 1130 460],P(vbus).RConn(1));
i_hop = ps('i_hop_ps',[1180 440 1210 460],P(ihop).RConn(1));
rhs = blk(['nesl_utility/Simulink-PS' nl 'Converter'],'r_hs_ps',[860 680 890 700]); ln(P(rhs).RConn,P(hs).LConn(1));
rch = blk(['nesl_utility/Simulink-PS' nl 'Converter'],'r_ch_ps',[1060 760 1090 780]); ln(P(rch).RConn,P(chop).LConn(1));
raux = blk(['nesl_utility/Simulink-PS' nl 'Converter'],'i_aux_ps',[1160 760 1190 780]); ln(P(raux).RConn,P(caux).RConn(1));
set_param(rhs,'FilteringAndDerivatives','filter','SimscapeFilterOrder','1','InputFilterTimeConstant','20e-6');
set_param(rch,'FilteringAndDerivatives','filter','SimscapeFilterOrder','1','InputFilterTimeConstant','20e-6');
rid = blk(['nesl_utility/Simulink-PS' nl 'Converter'],'r_id_ps',[960 680 990 700]); ln(P(rid).RConn,P(idio).LConn(1));
set_param(rid,'FilteringAndDerivatives','filter','SimscapeFilterOrder','1','InputFilterTimeConstant','2e-6');

%% ---------------- wiring of the signal side
% ctrl inputs: t, q, qd, ref, M, vp, g, x, vloc, en, M0, vp0, g0   outputs: dx, qdd, idc, log
ln(P(clk).Outport,P(ctrl).Inport(1));
ln(P(iq).Outport,P(ctrl).Inport(2)); ln(P(iqd).Outport,P(ctrl).Inport(3)); ln(P(ref).Outport,P(ctrl).Inport(4));
ln(P(mm).Outport,P(ctrl).Inport(5)); ln(P(vp).Outport,P(ctrl).Inport(6)); ln(P(gt).Outport,P(ctrl).Inport(7));
ln(P(ix).Outport,P(ctrl).Inport(8));
ln(P(mm0).Outport,P(ctrl).Inport(11)); ln(P(vp0).Outport,P(ctrl).Inport(12)); ln(P(gt0).Outport,P(ctrl).Inport(13));
% the boards sample their bus through a 2 us filter (also breaks the Simscape <-> Simulink algebraic loop)
vf = blk('simulink/Continuous/Integrator','v_filt',[1560 330 1590 360]); set_param(vf,'InitialCondition','scen.v0*ones(7,1)');
vfs = blk('simulink/Math Operations/Sum','v_err',[1530 330 1545 360]); set_param(vfs,'Inputs','+-');
vfg = blk('simulink/Math Operations/Gain','v_gain',[1600 380 1630 400]); set_param(vfg,'Gain','1/2e-6');
ln(P(vsum).Outport,P(vfs).Inport(1)); ln(P(vf).Outport,P(vfs).Inport(2)); ln(P(vfs).Outport,P(vfg).Inport);
ln(P(vfg).Outport,P(vf).Inport); ln(P(vf).Outport,P(ctrl).Inport(9));
ln(P(ctrl).Outport(1),P(ix).Inport); ln(P(ctrl).Outport(2),P(iqd).Inport); ln(P(iqd).Outport,P(iq).Inport);
ln(P(ctrl).Outport(3),P(idem).Inport);
% arm dynamics terms at Pm.Ts_dyn, held (compiled rigidBodyTree blocks, MSVC 2022)
zq = blk('simulink/Discrete/Zero-Order Hold','zoh_q',[610 100 630 120]); set_param(zq,'SampleTime','Pm.Ts_dyn');
zqd = blk('simulink/Discrete/Zero-Order Hold','zoh_qd',[610 140 630 160]); set_param(zqd,'SampleTime','Pm.Ts_dyn');
ln(P(iq).Outport,P(zq).Inport); ln(P(iqd).Outport,P(zqd).Inport);
ln(P(zq).Outport,P(mm).Inport); ln(P(zq).Outport,P(vp).Inport(1)); ln(P(zqd).Outport,P(vp).Inport(2)); ln(P(zq).Outport,P(gt).Inport);
ln(P(zq).Outport,P(mm0).Inport); ln(P(zq).Outport,P(vp0).Inport(1)); ln(P(zqd).Outport,P(vp0).Inport(2)); ln(P(zq).Outport,P(gt0).Inport);
% supervisor inputs: t, v_bat, v_ds, i_src, v_bus   outputs: r_hs, r_ch, i_aux, en, log, r_id
% supervisor samples its inputs at 5 us (unit delays break the algebraic loop through the network)
ln(P(clk).Outport,P(sup).Inport(1));
srcs = {v_bat, v_ds, i_src, v_bus};
ics = {'scen.v0','0','0','scen.v0'};
for k = 1:4
    ud = blk('simulink/Discrete/Unit Delay',sprintf('ud%d',k),[200 400+40*k 230 420+40*k]);
    set_param(ud,'SampleTime','Pm.Ts_sup','InitialCondition',ics{k});
    ln(srcs{k},P(ud).Inport); ln(P(ud).Outport,P(sup).Inport(k+1));
end
ln(P(sup).Outport(1),P(rhs).Inport); ln(P(sup).Outport(2),P(rch).Inport); ln(P(sup).Outport(3),P(raux).Inport);
ln(P(sup).Outport(4),P(ctrl).Inport(10)); ln(P(sup).Outport(6),P(rid).Inport);
% logging at 20 us: [t; q; qd; ref q; iq; iq_ref; tau; pdc; vloc; v_bat; v_bus; i_src; i_hop; r_hs; chop; state]
lg = blk('simulink/Signal Routing/Mux','log_mux',[1500 40 1505 220]); set_param(lg,'Inputs','3');
ln(P(ctrl).Outport(4),P(lg).Inport(1)); ln(P(sup).Outport(5),P(lg).Inport(2)); ln(i_hop,P(lg).Inport(3));
tw = blk('simulink/Sinks/To Workspace','log',[1560 110 1620 140]);
set_param(tw,'VariableName','arm_log','SaveFormat','Array','SampleTime','Pm.Ts_log');
ln(P(lg).Outport,P(tw).Inport);
% progress line every 50 ms of simulated time (batch runs otherwise give no sign of how far they are)
pg = blk('simulink/User-Defined Functions/MATLAB Function','progress',[30 300 90 330]);
rt2 = sfroot; pch = rt2.find('-isa','Stateflow.EMChart','Path',[mdl '/progress']);
pch.Script = strjoin({'function progress(t)', 'fprintf(''t = %.2f s\n'', t);', 'end'}, newline);
set_param(pg,'SystemSampleTime','0.05'); ln(P(clk).Outport,P(pg).Inport);
save_system(mdl,fullfile(here,[mdl '.slx']));
fprintf('built %s\n',fullfile(here,[mdl '.slx']));
end

function out = ps_out(mdl,name,pos,src)
b = add_block(sprintf('nesl_utility/PS-Simulink\nConverter'),[mdl '/' name],'Position',pos);
ph = get_param(b,'PortHandles');
add_line(mdl,src,ph.LConn,'autorouting','on');
out = ph.Outport;
end

function cap(b,c,esr,v0)
set_param(b,'c',c,'r',esr,'vc_specify','on','vc_priority','High','vc',v0);
end

function set_ml(mdl,name,code,params)
rt = sfroot; ch = rt.find('-isa','Stateflow.EMChart','Path',[mdl '/' name]);
ch.Script = code;
for p = params
    d = ch.find('-isa','Stateflow.Data','Name',p{1});
    d.Scope = 'Parameter';
end
end
