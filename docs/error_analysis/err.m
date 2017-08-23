error = [8.1161, 8.5572, 4.8979, 5.5006, 6.7667, 4.9637, 66.5887, 35.3022, 2.1859, 1.2276, 95.9361];
m_y = [0.1589081507047702, 0.2550667182261508, 0.3801776573323367, 0.546698870866195, 1.082486590743065, 0.7560710779057358, 5.604304145364201, 3.633041630602545, 0.5178360890690238, 0.01689261267893016, 1.97335561839017];
det = [5.4638e+07, 1.7216e+08, 2.1106e+08, 6.6944e+06, 1.8531e+08, 1.2300e+11, 1.1403e+04, 1.7654e+09, 3.5337e+08, 1.6930e+08, 306.6526];
log_det = log(det);
scatter(m_y, log_det);
labels = num2str(error','%f');
text(m_y, log_det, labels);
title("Average offset")
xlabel('Distribution of both sides: Abs(Mean of y axis of 3D feature points)');
ylabel('Dispersity: Log(Det(Covariance of 3D feature points))');