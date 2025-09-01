%%
no_load_data = importfile('no_load.csv');
low_load_data = importfile('low_load.csv');
heavy_load_data = importfile('heavy_load.csv');

%%

% plot(no_load_data(:,4),no_load_data(:,1));
windowSize = 100; 
b = (1/windowSize)*ones(1,windowSize);
a = 1;
velocity_movil_mean = filter(b,a,no_load_data(:,3));

velocity_low_pass = lowpass(no_load_data(:,3),0.01); 
plot(no_load_data(:,4),velocity_low_pass);