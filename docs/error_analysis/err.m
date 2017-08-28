error = [4.803313, 11.782807, 4.581539, 2.375941, 6.631727, 3.128704, 4.315196, 1.477921, 112.459351, 72.312232, 36.181538];
m_y = [0.1589081507047702, -0.2550667182261508, -0.3801776573323367, -0.5178360890690238, -0.546698870866195, -1.082486590743065, 0.7560710779057358, 0.01689261267893016, 1.97335561839017, -5.604304145364201, -3.633041630602545];
det_cov = [5.4638e+07, 1.7216e+08, 2.1106e+08, 3.5337e+08, 6.6944e+06, 1.8531e+08, 1.2300e+11, 1.6930e+08, 306.6526, 1.1403e+04, 1.7654e+09];
m_y = abs(m_y);
log_det = log(det_cov);
scatter(m_y, log_det);
labels = num2str(error',3);
text(m_y+0.05, log_det, labels, 'FontSize', 14);
title("Average offset")
xlabel('Center of distribution in horizontal direction: Abs(Mean of y axis of 3D feature points)');
ylabel('Dispersity: Log(Det(Covariance of 3D feature points))');