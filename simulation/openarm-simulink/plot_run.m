function plot_run(name, result_dir)
% Saves results/<name>.png from results/<name>.mat: joint tracking, motor currents, bus voltages, source / hop-1
% current, chopper and hot-swap state.
here = fileparts(mfilename('fullpath'));
if nargin < 2, result_dir = fullfile(here,'results'); end
S = load(fullfile(result_dir,name + ".mat")); R = S.R;
f = figure('Visible','off','Position',[0 0 1400 1000]);
tl = tiledlayout(f,3,2,'TileSpacing','compact');
title(tl, sprintf('%s: %.1f kg end load, %s source, bank %.3f mF', name, R.mass, R.P.source, R.P.bank_c*1e3), 'Interpreter','none');
nexttile; plot(R.t, (R.q - R.qr)*180/pi); ylabel('tracking error, deg'); legend(compose('J%d',1:7),'NumColumns',4); grid on
nexttile; plot(R.t, R.q*180/pi); ylabel('joint angle, deg'); grid on
nexttile; plot(R.t, R.iq); ylabel('motor iq, A'); grid on
nexttile; plot(R.t, R.v_bus, 'k', R.t, R.vloc(:,7), 'r', R.t, R.v_bat, 'b'); ylabel('V');
yline([57.1 61.13 80], ':', {'primary','backup','bank rating'}); legend('bank','board 7','BAT+'); grid on
nexttile; plot(R.t, R.i_src, R.t, R.i_hop1); ylabel('A'); legend('source','hop 1 (first XT30)'); grid on
nexttile; plot(R.t, sum(R.pdc,2), R.t, R.chop.*R.v_bus.^2/R.P.r_brake); ylabel('W'); xlabel('s');
legend('boards total (negative = regen)','brake resistor'); grid on
exportgraphics(f, fullfile(result_dir,name + ".png"), 'Resolution', 110);
close(f);
end
