function [r_hs, r_ch, i_aux, en, lg, r_id] = supervisor(t, v_bat, v_ds, i_src, v_bus, Pm)
% Support-board protection at Pm.Ts_sup (20 us): LM5069 hot-swap (current limit VCL/RS, power limit P/VDS, fault timer, LM5069-2
% retry at 0.5 % duty), primary chopper 57.1/55.0 V while the aux is alive, bus-powered backup 61.13/60.29 V with the
% TPS3700 delays, aux load, the actuator drive-enable used by the scenarios, and the LTC4359 ideal diode: on with
% r_id once its input (BAT+ less the hot-swap drop) exceeds the bus by its 30 mV forward-regulation voltage, off (fast pull-down, < 0.5 us) once the bus
% exceeds that input by 30 mV (LTC4359 Rev. F: dVSD 30 mV regulation, FPD comparator, tOFF 0.3-0.5 us).
persistent rlim tmr off_until pri bak bak_pend bak_t idon
if isempty(rlim)
    rlim = 0; tmr = 0; off_until = -1; pri = 0; bak = 0; bak_pend = 0; bak_t = 0; idon = 1;
end
Ts = Pm.Ts_sup;
aux = double(t < Pm.t_aux_fail);
% hot-swap
i_eff = min(Pm.i_lim, Pm.p_lim/max(v_ds, 0.5));
if t < off_until
    r_hs = 1e4; rlim = 0;
else
    rlim = max(0, rlim + Ts*Pm.k_lim*(i_src - i_eff));
    if rlim > 1e-4
        tmr = tmr + Ts;
    else
        tmr = max(0, tmr - Ts*2.5/85);
    end
    if tmr >= Pm.t_flt
        off_until = t + Pm.t_flt/Pm.retry_duty; tmr = 0; rlim = 0;
    end
    r_hs = Pm.r_hs + rlim;
    if t < off_until
        r_hs = 1e4;
    end
end
% chopper: primary needs the aux; backup needs only the bus
if aux > 0 && Pm.pri_en > 0
    if v_bus > Pm.chop_pri(1), pri = 1; elseif v_bus < Pm.chop_pri(2), pri = 0; end
else
    pri = 0;
end
want = bak;
if Pm.backup > 0
    if v_bus > Pm.chop_bak(1), want = 1; elseif v_bus < Pm.chop_bak(2), want = 0; end
else
    want = 0;
end
if want ~= bak
    if bak_pend == 0
        bak_pend = 1; bak_t = t + Pm.t_bak(2 - want);
    elseif t >= bak_t
        bak = want; bak_pend = 0;
    end
else
    bak_pend = 0;
end
on = max(pri, bak);
r_ch = on*Pm.r_brake + (1 - on)*1e4;
i_aux = aux*Pm.p_aux/max(v_bus, 10);
en = double(t < Pm.t_drv_off);
v_in = v_bat - v_ds;                                         % LTC4359 IN = hot-swap output
% fix 5 Nov 2026: LTC4359 blocks reverse current (off once OUT > IN), back on at 30 mV forward; 30 mV hysteresis kept
if v_bus > v_in, idon = 0; elseif v_in - v_bus > 0.03, idon = 1; end
r_id = idon*Pm.r_id + (1 - idon)*1e4;
lg = [v_bat; v_bus; i_src; v_ds; r_hs; on; pri; bak; aux; double(t < off_until)];
end
