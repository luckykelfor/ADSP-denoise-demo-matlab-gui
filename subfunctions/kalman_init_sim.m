function kalman_init_sim;

main_data = get(gcf,'UserData');

main_data.signals.x_hat      = 0;
main_data.signals.x_star     = 0;
main_data.signals.x_hat_old  = 0;
main_data.signals.y          = 0;
main_data.signals.y_old      = 0;
main_data.signals.n          = 0;
main_data.signals.n_old      = 0;

main_data.signals.U          = main_data.parameters.sigma_u.^2;
main_data.signals.N          = main_data.parameters.sigma_n.^2;

main_data.signals.star_P     = main_data.signals.P_0;
main_data.signals.star_P_old = main_data.signals.P_0;

main_data.axes_top.index     = 0;
main_data.axes_bottom.index  = 0;

set(gcf,'UserData',main_data);

drawnow;

kalman_plot_init(1);
kalman_plot_init(2);


