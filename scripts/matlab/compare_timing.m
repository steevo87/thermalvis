retrieve_times = load('../log/retrieve_log.txt');
internal_times = load('../log/internal_log.txt');

figure(1) % ROS time diffs
rt2 = 1000*abs(retrieve_times(1:size(internal_times, 1)-1) - retrieve_times(2:size(internal_times,1)));
plot(rt2);
summ_ros = [median(rt2) var(rt2)]

figure(2) % internal time diffs
it2 = 1000*abs(internal_times(1:size(internal_times, 1)-1) - internal_times(2:size(internal_times,1)));
plot(it2);
summ_internal = [median(it2) var(it2)]
xlabel('frame index');
ylabel('diff between adjacent frames (ms');