arg_list = argv();
fin = arg_list{1};

data = load(fin);

hax1 = subplot(1, 3, 1);
stairs(data(:, 1), data(:, 2:4));
grid('minor');
xlabel('y [m]'); ylabel('[deg]');
title('torso joints');

hax2 = subplot(1, 3, 2);
stairs(data(:, 1), data(:, 5:11));
grid('minor');
xlabel('y [m]'); ylabel('[deg]');
title('arm joints');

hax3 = subplot(1, 3, 3);
stairs(data(:, 1), data(:, 12:end));
grid('minor');
xlabel('y [m]'); ylabel('[deg]');
title('head joints');

linkaxes([hax1 hax2 hax3], 'x');
